#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "task_air_sensor.h"
#include "task_bme280.h"
#include "task_oled.h"
#include "driver/i2c.h"
#include "u8g2_esp32_hal.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
#include "i2c_config.h"

static const char *TAG = "main";
extern     u8g2_t u8g2; // a structure which will contain all the data for one display

void initDisplay(void);


void app_main() 
{
    esp_log_level_set("*", ESP_LOG_VERBOSE);        // set all components to ERROR level

    // ESP_ERROR_CHECK(i2c_master_init());
    // ESP_LOGI(TAG, "I2C Master initialized successfully.");
    initDisplay();


    // xTaskCreate(&task_air_sensor, "task_air_sensor", 2048, NULL, 5, NULL);
    // xTaskCreate(&task_bme280, "task_bme280", 2048, NULL, 5, NULL);
    xTaskCreate(&task_oled, "task_oled", 2048, NULL, 5, NULL);
}

void initDisplay(void) 
{
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.sda   = I2C_MASTER_SDA_IO;
	u8g2_esp32_hal.scl  = I2C_MASTER_SCL_IO;
	u8g2_esp32_hal_init(u8g2_esp32_hal);

    u8g2_Setup_sh1107_i2c_128x128_f(
        &u8g2,
        U8G2_R2, 
        u8g2_esp32_i2c_byte_cb, 
        u8g2_esp32_gpio_and_delay_cb);

    u8x8_SetI2CAddress(&u8g2.u8x8,(0x3D << 1));
	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2, 0);
}