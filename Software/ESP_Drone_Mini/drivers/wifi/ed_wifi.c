#include "ed_wifi.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_check.h"
#include <string.h>

static const char* tag = "ed_WiFi";

typedef enum {
    ED_WIFI_DISABLED,
    ED_WIFI_AS_STA,
    ED_WIFI_AS_AP,
} ed_wifi_mode_t;

static ed_wifi_mode_t wifi_mode = ED_WIFI_DISABLED;
static int wifi_mode_seq_lock = 0;
static bool ed_wifi_sta_connected = false;
static int ed_wifi_sta_seq_lock = 0;

static void wifi_event(void *arg, esp_event_base_t eventBase, int32_t eventID, void *eventData) {
    ip_event_got_ip_t *ip = eventData;
    static int retry_times = 0;
    if(eventBase == WIFI_EVENT) {
        switch (eventID) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(tag, "wifi sta start");
                retry_times = 0;
                esp_wifi_connect();

                // change sta status with seq lock.
                ed_wifi_sta_seq_lock ++;
                ed_wifi_sta_connected = true;
                ed_wifi_sta_seq_lock ++;
                
                break;
                
            case WIFI_EVENT_STA_DISCONNECTED:
                retry_times ++;
                
                // change sta status with seq lock.
                ed_wifi_sta_seq_lock ++;
                ed_wifi_sta_connected = false;
                ed_wifi_sta_seq_lock ++;

                if(retry_times <= 5)
                {
                    ESP_LOGE(tag, "sta connection dropped, retry times: %d", retry_times);
                    esp_wifi_connect();
                } else {
                    ESP_LOGE(tag, "connect to the AP fail");
                }
                break;
            default:
                break;
        }
    } else if(eventBase == IP_EVENT) {
        if(eventID == IP_EVENT_STA_GOT_IP){
            ESP_LOGI(tag, "wifi connected, ip addr is: "IPSTR"\n", IP2STR(&ip->ip_info.ip));         
        }
    }
}


static void ip_event(void *arg, esp_event_base_t eventBase, int32_t eventID, void *eventData) {


}



int ed_wifi_sta_connect(const char *ssid, const char *passwd)
{
    esp_err_t esp_ret = ESP_OK;
    int ret = 0;

    if(wifi_mode != ED_WIFI_DISABLED)
    {
        // TODO: switch wifi mode to sta.
        return -1;
    }

    // change sta status with seq lock.
    ed_wifi_sta_seq_lock ++;
    ed_wifi_sta_connected = false;
    ed_wifi_sta_seq_lock ++;

    esp_event_loop_handle_t wifi_handler;
    esp_event_loop_handle_t ip_handler;
    esp_event_loop_create_default();
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        wifi_event, NULL, &wifi_handler);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        ip_event, NULL, &ip_handler);


    // init esp netif
    esp_netif_t *pEsp_wifi_netif;
    esp_netif_init();
    pEsp_wifi_netif = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(pEsp_wifi_netif, "ESP Drone");

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_init_config);

    // config wifi sta
    wifi_config_t wifi_config = { 0 };
    memcpy(wifi_config.sta.ssid, ssid, strlen(ssid));
    memcpy(wifi_config.sta.password, passwd, strlen(passwd));
    ESP_RETURN_ON_FALSE(
        (esp_ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config)) == ESP_OK,
        1,
        tag,
        "esp_wifi_set_config failed with msg: %s", esp_err_to_name(esp_ret));
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    // change wifi_mode with seq lock.
    wifi_mode_seq_lock ++;
    wifi_mode = ED_WIFI_AS_STA;
    wifi_mode_seq_lock ++;

    return ret;
}

int ed_wifi_sta_disconnect(void)
{
    return 0;
}

ed_wifi_mode_t ed_wifi_get_status(void)
{
    // read wifi_mode with seq lock.
    int wifi_mode_seq_lock_old = 0;
    ed_wifi_mode_t current_mode;

    // waiting to enter the critical zone..
    while(wifi_mode_seq_lock %2 == 0);

    do {
        if(wifi_mode_seq_lock_old != wifi_mode_seq_lock)
            wifi_mode_seq_lock_old = wifi_mode_seq_lock;
        current_mode = wifi_mode;
    } while(wifi_mode_seq_lock_old != wifi_mode_seq_lock);
    return current_mode;
}

bool ed_wifi_get_sta_status(void)
{
    // read wifi_mode with seq lock.
    int ed_wifi_sta_seq_lock_old = 0;
    bool sta_status;

    // waiting to enter the critical zone..
    while(ed_wifi_sta_seq_lock %2 == 0);

    do {
        if(ed_wifi_sta_seq_lock_old != ed_wifi_sta_seq_lock)
            ed_wifi_sta_seq_lock_old = ed_wifi_sta_seq_lock;
        sta_status = ed_wifi_sta_connected;
    } while(ed_wifi_sta_seq_lock_old != ed_wifi_sta_seq_lock);
    return sta_status;
}

/**
 * @brief: Check if the current WiFi is available. 
 * @note: It is always true when in AP mode and only true when connected in STA mode.
 */
bool ed_wifi_available(void)
{
    switch(ed_wifi_get_status())
    {
    case ED_WIFI_DISABLED:
        return false;
    case ED_WIFI_AS_AP:
        return true;
    case ED_WIFI_AS_STA:
        return ed_wifi_get_sta_status();
    default:
        return false;
    }
}

