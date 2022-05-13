#include "task_oled.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void task_oled(void *pvParameter)
{
    printf("Start of Task Oled\n");
    while(1)
    {
        printf("App in Task Oled\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}