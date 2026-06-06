#include "algorithm.h"

#include <math.h>

#include "thermal_model.h"

#define P_LOSS_POWER_FRACTION 0.01f
#define P_LOSS_POWER_NOISE_FLOOR_W 10.0f

static float algorithm_calculate_p_loss(float power_est_w, bool power_valid)
{
    if (!power_valid || power_est_w < P_LOSS_POWER_NOISE_FLOOR_W) {
        return 0.0f;
    }

    return power_est_w * P_LOSS_POWER_FRACTION;
}

algorithm_result_t algorithm_update_from_power(float power_est_w,
                                               bool power_valid,
                                               float ambient_temperature_c,
                                               bool ambient_valid)
{
    algorithm_result_t result = {
        .p_loss_w = algorithm_calculate_p_loss(power_est_w, power_valid),
        .predicted_delta_t_c = 0.0f,
        .predicted_temperature_c = 0.0f,
        .thermal_samples_collected = 0,
        .thermal_model_ready = false,
        .valid = power_valid,
    };

    float model_delta_t_c = 0.0f;
    result.thermal_model_ready = thermal_model_update(result.p_loss_w, &model_delta_t_c);
    result.thermal_samples_collected = thermal_model_samples_collected();

    if (result.thermal_model_ready && ambient_valid) {
        result.predicted_delta_t_c = model_delta_t_c;
        result.predicted_temperature_c = fabsf(model_delta_t_c + ambient_temperature_c);
    }
    return result;
}
