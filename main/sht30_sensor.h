#ifndef SHT30_SENSOR_H
#define SHT30_SENSOR_H

#include "esp_err.h"

esp_err_t sht30_sensor_init(void);
esp_err_t sht30_sensor_read(float *temperature_c, float *humidity_percent);

#endif
