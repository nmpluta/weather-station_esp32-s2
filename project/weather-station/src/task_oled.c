#include "task_oled.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#include <clib/u8g2.h>
#include "u8g2_esp32_hal.h"
#include "i2c_config.h"

u8g2_t u8g2; // a structure which will contain all the data for one display

static const char *TAG = "sh1107";

void task_oled(void *pvParameter)
{
    printf("Start of Task Oled\n");

    // u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    // u8g2_esp32_hal.sda   = I2C_MASTER_SDA_IO;
	// u8g2_esp32_hal.scl  = I2C_MASTER_SCL_IO;
	// u8g2_esp32_hal_init(u8g2_esp32_hal);

    // u8g2_Setup_sh1107_i2c_128x128_f(
    //     &u8g2,
    //     U8G2_R2, 
    //     u8g2_esp32_i2c_byte_cb, 
    //     u8g2_esp32_gpio_and_delay_cb);

    // u8x8_SetI2CAddress(&u8g2.u8x8,(0x3D << 1));

    // ESP_LOGI(TAG, "u8g2_InitDisplay");
	// u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this

    // ESP_LOGI(TAG, "u8g2_SetPowerSave");
	// u8g2_SetPowerSave(&u8g2, 0); // wake up display


	ESP_LOGI(TAG, "u8g2_ClearBuffer");
	u8g2_ClearBuffer(&u8g2);
	ESP_LOGI(TAG, "u8g2_DrawBox");
	u8g2_DrawBox(&u8g2, 0, 26, 80,6);
	u8g2_DrawFrame(&u8g2, 0,26,100,6);

    u8g2_DrawFrame(&u8g2, 0,0,128,128);

	ESP_LOGI(TAG, "u8g2_SetFont");
    u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
	ESP_LOGI(TAG, "u8g2_DrawStr");
    u8g2_DrawStr(&u8g2, 2,17,"Hi nkolban!");
	ESP_LOGI(TAG, "u8g2_SendBuffer");
	u8g2_SendBuffer(&u8g2);

	ESP_LOGI(TAG, "All done!");


    while(1)
    {
        printf("App in Task Oled\n");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}


void setup_oled(void)
{
    
}

