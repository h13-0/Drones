#include "ed_debugger.h"

#include <stdarg.h>

#include "esp_log.h"
#include "lwip/sockets.h"

#include "freertos/semphr.h"

static int tcp_socket = -1;
static const char* tag = "ed_debugger";

uint8_t const tail[] = { 0x00, 0x00, 0x80, 0x78 };
float const *__vofa_package_tail__ = (float*)tail;

static bool connected = false;
static int connected_flag_seq_lock = 0;

#define ED_DEBUGGER_FLOAT_NUM       (32)
#define ED_DEBUGGER_RX_BUFFER_SIZE  (65)
static float* float_params[ED_DEBUGGER_FLOAT_NUM] = { NULL };

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
    while(connected_flag_seq_lock %2 == 0);

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

    // find package tail.
    int offset = 9;
    while(offset + 4 <= len)
    {
        if(!memcmp(data + offset, tail, 4))
        {
            // tail matched
            if(data[offset - 5] == '=')
            {
                // '=' matched
                memcpy(&id, data + offset - 9, 4);
                if(id < ED_DEBUGGER_FLOAT_NUM)
                {
                    // id matched
                    if(float_params[id] != NULL)
                        memcpy(float_params[id], data + offset - 4, 4);
                }
            }
            offset += 13;
        } else {
            offset ++;
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


static void __ed_debugger_server_task(void *pvParameters)
{
    int port = (int)pvParameters;
    // create socket
    tcp_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if(tcp_socket < 0) 
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
    int err = bind(tcp_socket, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if(err != 0) 
    {
        ESP_LOGE(tag, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(tag, "IPPROTO: %d", AF_INET);
        goto clean_up;
    }
    ESP_LOGI(tag, "Socket bound, port %d", port);

    // listen
    err = listen(tcp_socket, 1);
    if(err != 0) 
    {
        ESP_LOGE(tag, "Error occurred during listen: errno %d", errno);
        close(tcp_socket);
    }
    __ed_debugger_set_connected_flag(true);


    for( ; ; )
    {
        ESP_LOGI(tag, "Socket listening");

        struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
        long unsigned int addr_len = sizeof(source_addr);
        int sock = accept(tcp_socket, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(tag, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Convert ip address to string
        char addr_str[128] = { 0 };
        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(tag, "Socket accepted ip address: %s", addr_str);

        __ed_debugger_receive(sock);

        // disconnect
        __ed_debugger_set_connected_flag(false);
        shutdown(sock, 0);
        close(sock);
    }

clean_up:
    close(tcp_socket);
    vTaskDelete(NULL);
}


/**
 * @brief: Create a debugger, this debugger will be implemented based on TCP
 * @param: The port of the tcp server.
 */
int ed_debugger_create(int port)
{
    xTaskCreate(__ed_debugger_server_task, "debugger_server", 4096, (void*)port, 5, NULL);

    return 0;
}


static void __ed_debugger_send_vofa_frame(int nums, float *data)
{
    if(__ed_debugger_get_connected_flag())
        send(tcp_socket, data, sizeof(float) * nums, 0);
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

		__ed_debugger_send_vofa_frame(nums, data);
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
