#ifndef CURRENT_SENSOR_SCT013_H
#define CURRENT_SENSOR_SCT013_H

#include "esp_err.h"

esp_err_t current_sensor_sct013_init(void);
void current_sensor_sct013_pause(void);
void current_sensor_sct013_resume(void);

#endif
