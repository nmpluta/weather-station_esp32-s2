#ifndef TASK_AIR_SENSOR_H
#define TASK_AIR_SENSOR_H

#include "esp_err.h"

void task_air_sensor(void *pvParameter);
esp_err_t i2c_master_init(void);

#endif // TASK_AIR_SENSOR_H