#ifndef INMP441_FFT_H
#define INMP441_FFT_H

#include "esp_err.h"

esp_err_t inmp441_fft_init(void);
esp_err_t inmp441_fft_measure_frequency_hz(float *frequency_hz);
void inmp441_fft_pause_for_current_measurement(void);
void inmp441_fft_resume_after_current_measurement(void);

#endif
