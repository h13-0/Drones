#include "ed_idf_i2c.h"

#include "driver/i2c.h"
#include "esp_check.h"


static const char* tag = "idf_i2c";

int ed_idf_i2c_init(i2c_port_t i2c_num, gpio_num_t sda_io_num, gpio_num_t scl_io_num)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_io_num,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl_io_num,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
        // .clk_flags = 0,
    };
    // set i2c param.
    ESP_RETURN_ON_FALSE(
        i2c_param_config(i2c_num, &conf) == ESP_OK,
        -1,
        tag, 
        "set i2c param failed."
    );
    // install i2c driver.
    ESP_RETURN_ON_FALSE(
        i2c_driver_install(i2c_num, conf.mode, 0, 0, 0) == ESP_OK,
        -2,
        tag,
        "install i2c driver failed."
    );
    return 0;
}

int ed_idf_i2c_read_reg(i2c_port_t i2c_num, uint8_t address, uint8_t reg, uint8_t len, uint8_t *buf)
{
    esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    assert (handle != NULL);
    // send start signal.
	if((ret = i2c_master_start(handle)) != ESP_OK)
    {
        ESP_LOGE(tag, "i2c_master_start failed with ret: %d", ret);
        goto end;
    }
    // write address.
   	if((ret = i2c_master_write_byte(handle, (address << 1) | I2C_MASTER_WRITE, true)) != ESP_OK)
    {
        ESP_LOGE(tag, "i2c_master_write_byte failed with ret: %d", ret);
        goto end;
    }
    // select reg address.
    if((ret = i2c_master_write_byte(handle, reg, true)) != ESP_OK)
    {
        ESP_LOGE(tag, "i2c_master_write_byte failed with ret: %d", ret);
        goto end;
    }
    // re-send start signal.
	if((ret = i2c_master_start(handle)) != ESP_OK)
    {
        ESP_LOGE(tag, "i2c_master_start failed with ret: %d", ret);
        goto end;
    }
    // set read address
    if((ret = i2c_master_write_byte(handle, (address << 1) | I2C_MASTER_READ, true)) != ESP_OK)
    {
        ESP_LOGE(tag, "i2c_master_write_byte failed with ret: %d", ret);
        goto end;
    }
    // read bytes.
    if((ret = i2c_master_read(handle, buf, len, I2C_MASTER_LAST_NACK)) != ESP_OK)
    {
        ESP_LOGE(tag, "i2c_master_read failed with ret: %d", ret);
        goto end;
    }
    // send stop signal.
    if((ret = i2c_master_stop(handle)) != ESP_OK)
    {
        ESP_LOGE(tag, "i2c_master_stop failed with ret: %d", ret);
        goto end;
    }
    // exec.
    if((ret = i2c_master_cmd_begin(i2c_num, handle, 500 / portTICK_PERIOD_MS) != ESP_OK))
    {
        ESP_LOGE(tag, "i2c_master_cmd_begin failed with ret: %d", ret);
        goto end;
    }
end:
    i2c_cmd_link_delete(handle);
    return ret;
}

int ed_idf_i2c_write_reg(i2c_port_t i2c_num, uint8_t address, uint8_t reg, uint8_t len, uint8_t *buf)
{
    esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    assert (handle != NULL);
    // send start signal.
	if((ret = i2c_master_start(handle)) != ESP_OK)
    {
        ESP_LOGE(tag, "i2c_master_start failed with ret: %d", ret);
        goto end;
    } 
    // write address.
   	if((ret = i2c_master_write_byte(handle, (address << 1) | I2C_MASTER_WRITE, true) != ESP_OK))
    {
        ESP_LOGE(tag, "i2c_master_write_byte failed with ret: %d", ret);
        goto end;
    } 
    // select reg address.
    if((ret = i2c_master_write_byte(handle, reg, true)) != ESP_OK)
    {
        ESP_LOGE(tag, "i2c_master_write_byte failed with ret: %d", ret);
        goto end;
    } 
    // write bytes.
   	if((ret = i2c_master_write(handle, buf, len, true)) != ESP_OK)
    {
        ESP_LOGE(tag, "i2c_master_write failed with ret: %d", ret);
        goto end;
    } 
   	// send stop signal.
    if((ret = i2c_master_stop(handle)) != ESP_OK)
    {
        ESP_LOGE(tag, "i2c_master_stop failed with ret: %d", ret);
        goto end;
    }
    // exec.
    if((ret = i2c_master_cmd_begin(i2c_num, handle, 500 / portTICK_PERIOD_MS) != ESP_OK))
    {
        ESP_LOGE(tag, "i2c_master_cmd_begin failed with ret: %d", ret);
        goto end;
    }
end:
    i2c_cmd_link_delete(handle);
    return ret;
}

