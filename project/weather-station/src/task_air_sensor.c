#include "task_air_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "i2c-simple-example";

// defines for master
#define I2C_MASTER_SCL_IO           2                      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           1                      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */

// defines for ccs_811
#define STATUS_REG                  0x00
#define MEAS_MODE_REG               0x01
#define ALG_RESULT_DATA             0x02
#define ENV_DATA                    0x05
#define NTC_REG                     0x06
#define THRESHOLDS                  0x10
#define BASELINE                    0x11
#define HW_ID_REG                   0x20
#define ERROR_ID_REG                0xE0
#define APP_START_REG               0xF4
#define SW_RESET                    0xFF
#define CCS_811_ADDRESS             0x5A
#define CCS_811_WHO_AM_I_REG_ADDR   0x81
#define GPIO_WAKE                   0x5
#define DRIVE_MODE_IDLE             0x0             /*!< Drive mode */
#define DRIVE_MODE_1SEC             0x10            /*!< Drive mode */
#define DRIVE_MODE_10SEC            0x20            /*!< Drive mode */
#define DRIVE_MODE_60SEC            0x30            /*!< Drive mode */
#define INTERRUPT_DRIVEN            0x8
#define THRESHOLDS_ENABLED          0x4

#define ERROR_NOT_A_CCS811          -1

typedef struct 
{
    uint16_t eco2;
    uint16_t tvoc;
    uint8_t status;
    uint8_t error_id;
    uint16_t raw_data;
} ccs811_measurement_t;

// static esp_err_t mpu9250_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
// static esp_err_t mpu9250_register_write_byte(uint8_t reg_addr, uint8_t data);
static esp_err_t i2c_master_init(void);
static esp_err_t i2c_ccs811_init(i2c_port_t i2c_num);

uint8_t i2c_buff[8];
bool wake_gpio_enabled = true;
// void i2c_write(uint8_t address, uint8_t register, uint8_t *tx_data_ptr, uint8_t length);
// void i2c_read(uint8_t address, uint8_t *rx_data_ptr, uint8_t length);
// void gpio_write(uint8_t gpio_id, uint8_t level);

/*
    Task handling reading data from Air Quality Sensor: 

*/

void task_air_sensor(void *pvParameter)
{
    printf("Start of Task Air Sensor\n");

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    while(1)
    {
        printf("App in Task Air Sensor\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

// /**
//  * @brief Read a sequence of bytes from a MPU9250 sensor registers
//  */
// static esp_err_t mpu9250_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
// {
//     return i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
// }

// /**
//  * @brief Write a byte to a MPU9250 sensor register
//  */
// static esp_err_t mpu9250_register_write_byte(uint8_t reg_addr, uint8_t data)
// {
//     int ret;
//     uint8_t write_buf[2] = {reg_addr, data};

//     ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

//     return ret;
// }

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief i2c slave CCS811 air quality sensor initialization
 */
 static esp_err_t i2c_ccs811_init(i2c_port_t i2c_num)
 {
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CCS_811_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, HW_ID_REG, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) 
    {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CCS_811_ADDRESS << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &i2c_buff[0], NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if(i2c_buff[0] != 0x81)
    {
        return ERROR_NOT_A_CCS811;
    }
    return ret;
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
static esp_err_t i2c_master_sensor_test(i2c_port_t i2c_num, uint8_t *data_h, uint8_t *data_l)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CCS_811_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MEAS_MODE_REG, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, DRIVE_MODE_1SEC, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) 
    {
        return ret;
    }
    vTaskDelay(30 / portTICK_PERIOD_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CCS_811_ADDRESS << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_h, ACK_VAL);
    i2c_master_read_byte(cmd, data_l, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}