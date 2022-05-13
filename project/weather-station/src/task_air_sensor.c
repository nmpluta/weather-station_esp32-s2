#include "task_air_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void task_air_sensor(void *pvParameter)
{
    printf("Start of Task Air Sensor\n");

    while(1)
    {
        printf("App in Task Air Sensor\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
