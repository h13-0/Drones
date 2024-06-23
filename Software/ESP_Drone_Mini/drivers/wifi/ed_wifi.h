#ifndef __ED_WIFI_H__
#define __ED_WIFI_H__

#include "esp_err.h"
#include <stdbool.h>

int ed_wifi_sta_connect(const char *ssid, const char *passwd);

int ed_wifi_sta_disconnect(void);


/**
 * @brief: Check if the current WiFi is available. 
 * @note: It is always true when in AP mode and only true when connected in STA mode.
 */
bool ed_wifi_available(void);

//esp_err_t wifi_set_country(const wifi_country_t *country);



// TODO: wifi AP

#endif
