#include "sensor_telemetry.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static portMUX_TYPE s_telemetry_lock = portMUX_INITIALIZER_UNLOCKED;
static sensor_telemetry_t s_telemetry = {0};

void sensor_telemetry_set_pt1000(float temperature_c, bool valid)
{
    portENTER_CRITICAL(&s_telemetry_lock);
    s_telemetry.pt1000_temperature_c = temperature_c;
    s_telemetry.pt1000_valid = valid;
    portEXIT_CRITICAL(&s_telemetry_lock);
}

void sensor_telemetry_set_sht30(float temperature_c, float humidity_percent, bool valid)
{
    portENTER_CRITICAL(&s_telemetry_lock);
    s_telemetry.sht30_temperature_c = temperature_c;
    s_telemetry.sht30_humidity_percent = humidity_percent;
    s_telemetry.sht30_valid = valid;
    portEXIT_CRITICAL(&s_telemetry_lock);
}

void sensor_telemetry_set_mic(float frequency_hz, float level_dbfs, bool valid)
{
    portENTER_CRITICAL(&s_telemetry_lock);
    s_telemetry.mic_frequency_hz = frequency_hz;
    s_telemetry.mic_level_dbfs = level_dbfs;
    s_telemetry.mic_valid = valid;
    portEXIT_CRITICAL(&s_telemetry_lock);
}

void sensor_telemetry_set_current(float adc_voltage_v, float bias_voltage_v, float current_rms_a, float power_w, bool valid)
{
    portENTER_CRITICAL(&s_telemetry_lock);
    s_telemetry.current_adc_voltage_v = adc_voltage_v;
    s_telemetry.current_bias_voltage_v = bias_voltage_v;
    s_telemetry.current_rms_a = current_rms_a;
    s_telemetry.current_power_w = power_w;
    s_telemetry.current_valid = valid;
    portEXIT_CRITICAL(&s_telemetry_lock);
}

void sensor_telemetry_set_current_details(float adc_rms_voltage_v, float signal_rms_voltage_v, float raw_current_a)
{
    portENTER_CRITICAL(&s_telemetry_lock);
    s_telemetry.current_adc_rms_voltage_v = adc_rms_voltage_v;
    s_telemetry.current_signal_rms_voltage_v = signal_rms_voltage_v;
    s_telemetry.current_raw_a = raw_current_a;
    portEXIT_CRITICAL(&s_telemetry_lock);
}

void sensor_telemetry_set_algorithm(float p_loss_w,
                                    float thermal_delta_t_c,
                                    float thermal_temperature_c,
                                    int thermal_samples_collected,
                                    bool thermal_model_ready,
                                    bool valid)
{
    portENTER_CRITICAL(&s_telemetry_lock);
    s_telemetry.p_loss_w = p_loss_w;
    s_telemetry.thermal_delta_t_c = thermal_delta_t_c;
    s_telemetry.thermal_temperature_c = thermal_temperature_c;
    s_telemetry.thermal_samples_collected = thermal_samples_collected;
    s_telemetry.thermal_model_ready = thermal_model_ready;
    s_telemetry.algorithm_valid = valid;
    portEXIT_CRITICAL(&s_telemetry_lock);
}

void sensor_telemetry_get(sensor_telemetry_t *telemetry)
{
    if (telemetry == NULL) {
        return;
    }

    portENTER_CRITICAL(&s_telemetry_lock);
    *telemetry = s_telemetry;
    portEXIT_CRITICAL(&s_telemetry_lock);
}
