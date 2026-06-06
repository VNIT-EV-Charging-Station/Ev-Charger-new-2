#ifndef THERMAL_MODEL_H
#define THERMAL_MODEL_H

#include <stdbool.h>

#define THERMAL_MODEL_WINDOW_SIZE 126
#define THERMAL_MODEL_PLOSS_MIN_W 0.0f
#define THERMAL_MODEL_PLOSS_MAX_W 23.004281f
#define THERMAL_MODEL_DELTA_T_MIN_C 0.0f
#define THERMAL_MODEL_DELTA_T_MAX_C 22.7781093f

void thermal_model_reset(void);
bool thermal_model_is_ready(void);
int thermal_model_samples_collected(void);
bool thermal_model_update(float p_loss_w, float *predicted_delta_t_c);
float thermal_model_predict_window(const float p_loss_window_w[THERMAL_MODEL_WINDOW_SIZE]);

#endif
