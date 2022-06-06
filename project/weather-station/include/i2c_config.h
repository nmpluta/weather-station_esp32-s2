#ifndef I2C_CONFIG_H
#define I2C_CONFIG_H

// defines for master
#define I2C_MASTER_SCL_IO           2                      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           1                      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          10000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000
#define ACK_CHECK_EN   0x1                 //  I2C master will check ack from slave
#define ACK_CHECK_DIS  0x0                 //  I2C master will not check ack from slave


#endif // I2C_CONFIG_H