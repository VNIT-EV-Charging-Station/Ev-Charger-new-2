#ifndef PT1000_MAX31865_H
#define PT1000_MAX31865_H

#include "esp_err.h"

esp_err_t pt1000_max31865_init(void);
esp_err_t pt1000_max31865_read_temperature_c(float *temperature_c);

#endif
