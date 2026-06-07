#pragma once

#include <stdint.h>

#include "esp_err.h"

#define NRF24_FLAG_PT1000_VALID       0x01
#define NRF24_FLAG_SHT30_VALID        0x02
#define NRF24_FLAG_MIC_VALID          0x04
#define NRF24_FLAG_CURRENT_VALID      0x08
#define NRF24_FLAG_ALGORITHM_VALID    0x10
#define NRF24_FLAG_THERMAL_READY      0x20

esp_err_t nrf24_sender_init(void);
esp_err_t nrf24_sender_send_charger_data(float pt1000_temperature_c,
                                         float sht30_temperature_c,
                                         float sht30_humidity_percent,
                                         float mic_frequency_hz,
                                         float current_rms_a,
                                         float current_raw_a,
                                         float current_power_w,
                                         float current_adc_voltage_v,
                                         float current_bias_voltage_v,
                                         float current_adc_rms_voltage_v,
                                         float current_signal_rms_voltage_v,
                                         float p_loss_w,
                                         float thermal_temperature_c,
                                         uint8_t flags);
