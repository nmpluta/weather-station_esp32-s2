#include "task_air_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/i2c.h"
#include "common_i2c.h"

static const char *TAG = "i2c-simple-example";

// defines for master
#define I2C_MASTER_SCL_IO 2         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 1         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 40000    /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */

// defines for ccs_811
#define STATUS_REG 0x00
#define MEAS_MODE_REG 0x01
#define ALG_RESULT_DATA 0x02
#define ENV_DATA 0x05
#define NTC_REG 0x06
#define THRESHOLDS 0x10
#define BASELINE 0x11
#define HW_ID_REG 0x20
#define ERROR_ID_REG 0xE0
#define APP_START_REG 0xF4
#define SW_RESET 0xFF
#define CCS_811_ADDRESS 0x5B
#define CCS_811_WHO_AM_I 0x81
#define GPIO_WAKE 0x5
#define DRIVE_MODE_IDLE 0x0         /* Drive mode */
#define DRIVE_MODE_1SEC 0x10        /* Drive mode */
#define DRIVE_MODE_10SEC 0x20       /* Drive mode */
#define DRIVE_MODE_60SEC 0x30       /* Drive mode */
#define INTERRUPT_DRIVEN 0x8
#define THRESHOLDS_ENABLED 0x4

#define APP_VALID 0x10      /* Status Register fied */
#define DATA_READY 0x8      /* Status Register fied */
#define ERROR 0x1           /* Status Register fied */
#define DATA_LENGTH 4

#define ERROR_NOT_A_CCS811 -10
#define ERROR_NO_VALID_APP -9

typedef struct
{
    uint16_t eco2;
    uint16_t tvoc;
    uint8_t status;
    uint8_t error_id;
    uint16_t raw_data;
} ccs811_measurement_t;

ccs811_measurement_t current_data;

static esp_err_t i2c_sensor_init(void);
static esp_err_t i2c_sensor_read_status(void);
static esp_err_t i2c_sensor_read_errors(void);
static esp_err_t i2c_sensor_read_data(void);
static esp_err_t i2c_sensor_check(void);
static esp_err_t i2c_sensor_start_app(void);

uint8_t i2c_buff[8];
bool wake_gpio_enabled = true;

void task_air_sensor(void *pvParameter)
{
    esp_err_t err;
    printf("Start of Task Air Sensor.\n");

    err = i2c_sensor_check();
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "Sensor successfully connected to I2C bus.");

    err = i2c_sensor_init();
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "I2C Sensor initialized successfully.");

    err = i2c_sensor_start_app();
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "I2C Sensor app started successfully.");

    while (1)
    {
        printf("App in Task Air Sensor\n");

        err = i2c_sensor_read_status();
        ESP_ERROR_CHECK(err);
        ESP_LOGI(TAG, "I2C Status register read successfully.");
        printf("Status register = %d\n", current_data.status);

        if(current_data.status & ERROR)
        {
            err = i2c_sensor_read_errors();
            ESP_ERROR_CHECK(err);
            printf("error id register = %d\n", current_data.error_id);
        }

        if (current_data.status & DATA_READY)
        {
            err = i2c_sensor_read_data();
            ESP_ERROR_CHECK(err);
            printf("eco2 = %d\n", current_data.eco2);
            printf("tvoc = %d\n", current_data.tvoc);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

/**
 * @brief test code to operate on CCS811 air quality sensor
 *
 * 1. set drive mode in Measure Mode Register
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
static esp_err_t i2c_sensor_init(void)
{
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    esp_err_t err;
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();

    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, CCS_811_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd_handle, MEAS_MODE_REG, ACK_CHECK_EN);
    i2c_master_write_byte(cmd_handle, DRIVE_MODE_1SEC, ACK_CHECK_EN);
    i2c_master_stop(cmd_handle);
    err = i2c_master_cmd_begin(i2c_num, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);

    if (err != ESP_OK)
    {
        ESP_LOGI(TAG, "Error during setting drive mode.\n");
        return err;
    }

    return err;
}

static esp_err_t i2c_sensor_check(void)
{
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    esp_err_t err;
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();

    cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, CCS_811_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd_handle, HW_ID_REG, ACK_CHECK_EN);

    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, CCS_811_ADDRESS << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd_handle, &i2c_buff[0], NACK_VAL);
    i2c_master_stop(cmd_handle);
    err = i2c_master_cmd_begin(i2c_num, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);

    if (i2c_buff[0] != CCS_811_WHO_AM_I)
    {
        printf("i2c_buff[0] = %d \n", i2c_buff[0]);
        ESP_LOGI(TAG, "Error during reading from HW_ID_REG.\n");
        return ERROR_NOT_A_CCS811;
    }
    return err;
}

static esp_err_t i2c_sensor_read_status(void)
{
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    esp_err_t err;
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();

    cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, CCS_811_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd_handle, STATUS_REG, ACK_CHECK_EN);

    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, CCS_811_ADDRESS << 1 | READ_BIT, ACK_VAL);
    i2c_master_read_byte(cmd_handle, &current_data.status, NACK_VAL);
    i2c_master_stop(cmd_handle);
    err = i2c_master_cmd_begin(i2c_num, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
    if (err != ESP_OK)
    {
        ESP_LOGI(TAG, "Error during reading from STATUS_REG.\n");
        return err;
    }

    return err;
}

static esp_err_t i2c_sensor_read_errors(void)
{
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    esp_err_t err;
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();

    cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, CCS_811_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd_handle, ERROR_ID_REG, ACK_CHECK_EN);

    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, CCS_811_ADDRESS << 1 | READ_BIT, ACK_VAL);
    i2c_master_read_byte(cmd_handle, &current_data.error_id, NACK_VAL);
    i2c_master_stop(cmd_handle);
    err = i2c_master_cmd_begin(i2c_num, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
    if (err != ESP_OK)
    {
        ESP_LOGI(TAG, "Error during reading from ERROR_ID_REG.\n");
        return err;
    }

    return err;
}

static esp_err_t i2c_sensor_start_app(void)
{
    ESP_ERROR_CHECK(i2c_sensor_read_status());

    if (!(current_data.status & APP_VALID))
    {
        return ERROR_NO_VALID_APP;
    }

    i2c_port_t i2c_num = I2C_MASTER_NUM;
    esp_err_t err;
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();

    cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, CCS_811_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd_handle, APP_START_REG, ACK_CHECK_EN);
    i2c_master_stop(cmd_handle);
    err = i2c_master_cmd_begin(i2c_num, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
    if (err != ESP_OK)
    {
        ESP_LOGI(TAG, "Error during writing to APP_START_REG.\n");
        return err;
    }

    return err;
}

static esp_err_t i2c_sensor_read_data(void)
{
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    esp_err_t err;
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    uint8_t temp_buff[DATA_LENGTH];

    cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, CCS_811_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd_handle, ALG_RESULT_DATA, ACK_CHECK_EN);
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, CCS_811_ADDRESS << 1 | READ_BIT, ACK_VAL);
    i2c_master_read(cmd_handle, temp_buff, DATA_LENGTH - 1, ACK_VAL);
    i2c_master_read_byte(cmd_handle, &temp_buff[DATA_LENGTH - 1], NACK_VAL);
    i2c_master_stop(cmd_handle);
    err = i2c_master_cmd_begin(i2c_num, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
    if (err != ESP_OK)
    {
        ESP_LOGI(TAG, "Error during reading from ALG_RESULT_DATA.\n");
        return err;
    }

    current_data.eco2 = (temp_buff[0]) << 8 | temp_buff[1];
    current_data.tvoc = (temp_buff[2]) << 8 | temp_buff[3];

    return err;
}
