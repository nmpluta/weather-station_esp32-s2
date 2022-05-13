#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "task_air_sensor.h"
#include "task_bme280.h"
#include "task_oled.h"

void app_main() 
{
    xTaskCreate(&task_air_sensor, "task_air_sensor", 2048, NULL, 5, NULL);
    xTaskCreate(&task_bme280, "task_bme280", 2048, NULL, 5, NULL);
    xTaskCreate(&task_oled, "task_oled", 2048, NULL, 5, NULL);
}