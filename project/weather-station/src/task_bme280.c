#include "task_bme280.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void task_bme280(void *pvParameter)
{
    printf("Start of Task BME280\n");
    while(1)
    {
        printf("App in Task BME280\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}