#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "task_air_sensor.h"
#include "task_bme280.h"
#include "task_oled.h"
#include "common_i2c.h"

static const char *TAG = "main";


void app_main()
{
    esp_err_t err;

    err = i2c_master_init();
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "I2C Master initialized successfully.");

    xTaskCreate(&task_air_sensor, "task_air_sensor", 2048, NULL, 5, NULL);
    // xTaskCreate(&task_bme280, "task_bme280", 2048, NULL, 5, NULL);
    // xTaskCreate(&task_oled, "task_oled", 2048, NULL, 5, NULL);
}
