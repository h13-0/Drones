#include "ed_debugger.h"

#include <stdarg.h>

#include "esp_log.h"
#include "lwip/sockets.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static int socket_server = -1;
static int socket_connect = -1;
static const char* tag = "ed_debugger";

uint8_t const tail[] = { 0x00, 0x00, 0x80, 0x7f };
float const *__vofa_package_tail__ = (float*)tail;

static bool connected = false;
static int connected_flag_seq_lock = 0;

#define ED_DEBUGGER_FLOAT_NUM       (32)
#define ED_DEBUGGER_RX_BUFFER_SIZE  (65)
static float* float_params[ED_DEBUGGER_FLOAT_NUM] = { NULL };

// sender
#define ED_DEBUGGER_TX_FLOAT_BUFFER_SIZE  (13)
static float float_tx_buffer[ED_DEBUGGER_TX_FLOAT_BUFFER_SIZE] = { 0.0f };
static int float_tx_buffer_nums = 0;
static SemaphoreHandle_t float_tx_buffer_mutex;
// Sender task handle
static TaskHandle_t debugger_sender_task_handle = NULL;
static const UBaseType_t debugger_sender_ready_index = 0;

static void __ed_debugger_set_connected_flag(bool status)
{
    connected_flag_seq_lock ++;
    connected = status;
    connected_flag_seq_lock ++;
}

static bool __ed_debugger_get_connected_flag(void)
{
    // read debugger status with seq lock.
    int connected_flag_seq_lock_old = 0;
    bool debugger_connected_flag = false;

    // waiting to enter the critical zone..
    while(connected_flag_seq_lock %2);

    do {
        if(connected_flag_seq_lock_old != connected_flag_seq_lock)
            connected_flag_seq_lock_old = connected_flag_seq_lock;
        debugger_connected_flag = connected;
    } while(connected_flag_seq_lock_old != connected_flag_seq_lock);
    return debugger_connected_flag;
}

/**
 * @note:
 *          Packet structure:
 *              ${int id in raw}=${float in raw}${tail = { 0x00, 0x00, 0x80, 0x78 }}
 *          Packet length = 13 byte.
 *          such as:
 *              0x01 0x00 0x00 0x00 `=` 0xC3 0xF5 0x48 0x40 0x00 0x00 0x80 0x78
 * @return: Quantity of remaining data(offset).
 */
static int __ed_match_float_param(uint8_t *data, int len)
{
    int id = 0;
    if(len < 13)
        return len; // skip

    // try to find character '='
    int offset = 4;
    while(offset + 8 < len)
    {
        // 1. find the location of character '='
        if(data[offset] != '=')
        {
            offset ++;
            continue;
        }
        
        // 2. try to match package tail(now offset points to characters '=').
        if(memcmp(data + offset + 5, tail, 4))
        {
            offset += 9;
            continue;
        }

        // 3. get and check id.
        memcpy(&id, data + offset - 4, 4);
        if(float_params[id] == NULL)
        {
            ESP_LOGE(tag, "the id: %d is not bound to an element", id);
            offset += 9;
            continue;
        }

        // 4. copy data to corresponding elements.
        memcpy(float_params[id], data + offset + 1, 4);

        // 5. finish
        // 5.1 check if there is a possibility of the next data packet in the subsequent data length.
        if(len - offset - 1 - 8 >= 13)
        {
            // 5.1.1 move offset to the next possible position where the character '=' may appear.
            offset += 13;
            continue;
        } else {
            // 5.1.2 no more packets left, move offset to the position of the first unprocessed element.
            //      note: offset may be equal to len.
            offset += 9;
            break;
        }
    }

    // move remaining data to the head.
#if(ED_DEBUGGER_RX_BUFFER_SIZE >= 24)
    if(len - offset > 0)
        memcpy(data, data + offset, len - offset);
#else
    if(len - offset > 0)
        memmove(data, data + offset, len - offset);
#endif
    return len - offset;
}

static void __ed_debugger_receive(const int sock)
{
    int len;
    static uint8_t rx_buffer[ED_DEBUGGER_RX_BUFFER_SIZE] = { 0 };
    static int offset = 0;
    offset = 0;

    do {
        len = recv(sock, rx_buffer + offset, ED_DEBUGGER_RX_BUFFER_SIZE - offset, 0);
        if (len < 0) {
            ESP_LOGE(tag, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(tag, "Connection closed");
        } else {
            offset = __ed_match_float_param(rx_buffer, len + offset);
        }
    } while (len > 0);

    // clean buffer.
    memset(rx_buffer, 0x00, 64);
    offset = 0;
}


static void __ed_debugger_server_listener_task(void *pvParameters)
{
    int port = (int)pvParameters;
    // create socket
    socket_server = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if(socket_server < 0) 
    {
        ESP_LOGE(tag, "Unable to create tcp socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    // set addr.
    struct sockaddr_in dest_addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(port),
    };

    // bind
    int err = bind(socket_server, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if(err != 0) 
    {
        ESP_LOGE(tag, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(tag, "IPPROTO: %d", AF_INET);
        goto clean_up;
    }
    ESP_LOGI(tag, "Socket bound, port %d", port);

    // listen
    err = listen(socket_server, 1);
    if(err != 0) 
    {
        ESP_LOGE(tag, "Error occurred during listen: errno %d", errno);
        close(socket_server);
    }

    for( ; ; )
    {
        ESP_LOGI(tag, "Socket listening");

        struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
        long unsigned int addr_len = sizeof(source_addr);
        socket_connect = accept(socket_server, (struct sockaddr *)&source_addr, &addr_len);
        if (socket_connect < 0) {
            ESP_LOGE(tag, "Unable to accept connection: errno %d", errno);
            __ed_debugger_set_connected_flag(false);
            break;
        }
        __ed_debugger_set_connected_flag(true);

        // Convert ip address to string
        char addr_str[128] = { 0 };
        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(tag, "Socket accepted ip address: %s", addr_str);

        __ed_debugger_receive(socket_connect);

        // disconnect
        __ed_debugger_set_connected_flag(false);
        shutdown(socket_connect, 0);
        close(socket_connect);
    }

clean_up:
    close(socket_server);
    vTaskDelete(NULL);
}


static void __ed_debugger_server_sender_task(void *pvParameters)
{
    while(1)
    {
        if(__ed_debugger_get_connected_flag())
        {
            static float buffer[ED_DEBUGGER_TX_FLOAT_BUFFER_SIZE] = { 0.0f };
            int nums = 0;
            
            xSemaphoreTake(float_tx_buffer_mutex, portMAX_DELAY);
            if((nums = float_tx_buffer_nums))
                memcpy(buffer, float_tx_buffer, nums * sizeof(float));
            float_tx_buffer_nums = 0;
            xSemaphoreGive(float_tx_buffer_mutex);

            if(nums)
            {
                size_t size = sizeof(float) * nums;
                int ret = send(socket_connect, buffer, size, 0);
                if(ret != size)
                    ESP_LOGW(tag, "%d bytes should be sent, but %d bytes were actually sent.", size, ret);
            }
                
        } else {
            //ESP_LOGD(tag, "debugger not connected");
        }

        // wait signal.
        ulTaskNotifyTakeIndexed(debugger_sender_ready_index, pdTRUE, portMAX_DELAY);
    }

}

/**
 * @brief: Create a debugger, this debugger will be implemented based on TCP
 * @param: The port of the tcp server.
 */
int ed_debugger_create(int port)
{
    float_tx_buffer_mutex = xSemaphoreCreateMutex();
    xTaskCreate(__ed_debugger_server_listener_task, "debugger_server_listener", 4096, (void*)port, 5, NULL);
    xTaskCreate(__ed_debugger_server_sender_task, "debugger_server_sender", 4096, NULL, 5, &debugger_sender_task_handle);

    return 0;
}


static void __ed_debugger_write_to_float_buffer(int nums, float *data)
{
    // write to float_tx_buffer
    xSemaphoreTake(float_tx_buffer_mutex, portMAX_DELAY);
    memcpy(float_tx_buffer, data, nums * sizeof(float));
    float_tx_buffer_nums = nums;
    xSemaphoreGive(float_tx_buffer_mutex);
}


/**
 * @brief: Send several floating-point numbers to the "VOFA+" software.
 * @note: The data engine should be selected as JustFloat, 
            which package structure ends with { 0x00, 0x00, 0x80, 0x78 }.
            This package tail is a type of NAN specified in IEEE 754.
 */
void ed_debugger_send_vofa(int nums, ...)
{
    va_list args;
	va_start(args, nums);

	if(nums)
	{
		float data[nums];

		for(uint8_t index = 0; index < nums; index++)
		{
			data[index] = va_arg(args, double);
		}

		__ed_debugger_write_to_float_buffer(nums, data);
        
        // Send notify to sender
        xTaskNotifyGiveIndexed(debugger_sender_task_handle, debugger_sender_ready_index);
	}

	va_end(args);
}

/**
 * @brief: bind float type to id.
 * @note: the id should be as small as possible
 */
void ed_debugger_bind_float(int id, float* f)
{
    assert(id >= 0);
    assert(id < ED_DEBUGGER_FLOAT_NUM - 1);
    float_params[id] = f;
}
