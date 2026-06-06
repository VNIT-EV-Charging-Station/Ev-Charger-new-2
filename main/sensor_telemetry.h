#ifndef SENSOR_TELEMETRY_H
#define SENSOR_TELEMETRY_H

#include <stdbool.h>

typedef struct {
    float pt1000_temperature_c;
    bool pt1000_valid;
    float sht30_temperature_c;
    float sht30_humidity_percent;
    bool sht30_valid;
    float mic_frequency_hz;
    float mic_level_dbfs;
    bool mic_valid;
    float current_adc_voltage_v;
    float current_bias_voltage_v;
    float current_rms_a;
    float current_power_w;
    float current_adc_rms_voltage_v;
    float current_signal_rms_voltage_v;
    float current_raw_a;
    bool current_valid;
    float p_loss_w;
    float thermal_delta_t_c;
    float thermal_temperature_c;
    int thermal_samples_collected;
    bool thermal_model_ready;
    bool algorithm_valid;
} sensor_telemetry_t;

void sensor_telemetry_set_pt1000(float temperature_c, bool valid);
void sensor_telemetry_set_sht30(float temperature_c, float humidity_percent, bool valid);
void sensor_telemetry_set_mic(float frequency_hz, float level_dbfs, bool valid);
void sensor_telemetry_set_current(float adc_voltage_v, float bias_voltage_v, float current_rms_a, float power_w, bool valid);
void sensor_telemetry_set_current_details(float adc_rms_voltage_v, float signal_rms_voltage_v, float raw_current_a);
void sensor_telemetry_set_algorithm(float p_loss_w,
                                    float thermal_delta_t_c,
                                    float thermal_temperature_c,
                                    int thermal_samples_collected,
                                    bool thermal_model_ready,
                                    bool valid);
void sensor_telemetry_get(sensor_telemetry_t *telemetry);

#endif
