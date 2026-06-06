#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <stdbool.h>

typedef struct {
    float p_loss_w;
    float predicted_delta_t_c;
    float predicted_temperature_c;
    int thermal_samples_collected;
    bool thermal_model_ready;
    bool valid;
} algorithm_result_t;

algorithm_result_t algorithm_update_from_power(float power_est_w,
                                               bool power_valid,
                                               float ambient_temperature_c,
                                               bool ambient_valid);

#endif
