#include "ed_nvs_flash.h"

#include "esp_err.h"
#include "nvs_flash.h"

int ed_nvs_flash_init(void)
{
    esp_err_t err = nvs_flash_init();
    if(err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND){
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if(err != ESP_OK)
        return 1;
    return 0;
}

