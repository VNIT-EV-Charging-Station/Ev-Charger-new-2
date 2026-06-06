#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "algorithm.h"
#include "current_sensor_sct013.h"
#include "sensor_telemetry.h"

static const char *TAG = "POWER";

/* ADC config */
#define SCT_ADC_UNIT            ADC_UNIT_1
#define SCT_ADC_CHANNEL         ADC_CHANNEL_3   // GPIO4 = ADC1_CH3 on ESP32-S3 DevKit
#define SCT_ATTEN               ADC_ATTEN_DB_11

/* SCT calibration */
#define BURDEN_RESISTOR         47.0f
#define TURNS_RATIO             2000.0f
#define MAINS_VOLTAGE_RMS       230.0f
#define POWER_FACTOR_ASSUMED    1.0f
/* Tuned against a nominal 100 W resistive load on 230 V mains. */
#define CURRENT_CALIBRATION     2.12f

/* Sampling */
#define NUM_CYCLES              6
#define SAMPLES_PER_CYCLE       200
#define TOTAL_SAMPLES           (NUM_CYCLES * SAMPLES_PER_CYCLE)

/* Noise filtering */
#define CURRENT_NOISE_FLOOR_A   0.05f
#define CURRENT_WAKEUP_A        0.10f
#define POWER_NOISE_FLOOR_W     10.0f
#define NOISE_MARGIN_FACTOR     1.20f
#define CURRENT_WAKEUP_CONFIRM_SAMPLES 3
#define CURRENT_SLEEP_CONFIRM_SAMPLES  2

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle = NULL;
static float bias_voltage = 1.65f;
static float noise_rms_voltage = 0.0f;
static volatile bool s_pause_requested;
static volatile bool s_paused;

typedef struct {
    float mean_voltage;
    float adc_rms_voltage;
    float signal_rms_voltage;
    float raw_current;
} current_measurement_t;

static float read_voltage(void)
{
    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, SCT_ADC_CHANNEL, &raw));

    if (cali_handle != NULL) {
        int mv = 0;
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, raw, &mv));
        return mv / 1000.0f;
    }

    return (raw / 4095.0f) * 3.3f;
}

static void init_adc(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = SCT_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = SCT_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, SCT_ADC_CHANNEL, &chan_cfg));

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = SCT_ADC_UNIT,
        .chan = SCT_ADC_CHANNEL,
        .atten = SCT_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };

    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration OK");
    } else {
        cali_handle = NULL;
        ESP_LOGW(TAG, "ADC calibration unavailable");
    }
}

static void calibrate_current_channel(void)
{
    double sum = 0.0;
    double sum_sq = 0.0;

    ESP_LOGI(TAG, "Calibrating no-load current noise...");
    ESP_LOGI(TAG, "Keep the SCT clamp empty during startup.");

    for (int i = 0; i < 200; i++) {
        read_voltage();
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    for (int i = 0; i < TOTAL_SAMPLES; i++) {
        float sample = read_voltage();
        sum += sample;
        sum_sq += (double)(sample * sample);
        esp_rom_delay_us(100);
    }

    bias_voltage = (float)(sum / TOTAL_SAMPLES);

    float variance = (float)(sum_sq / TOTAL_SAMPLES) - (bias_voltage * bias_voltage);
    if (variance < 0.0f) {
        variance = 0.0f;
    }

    noise_rms_voltage = sqrtf(variance);

    ESP_LOGI(TAG, "Bias voltage  : %.5f V", bias_voltage);
    ESP_LOGI(TAG, "Noise floor   : %.6f V RMS", noise_rms_voltage);
}

static current_measurement_t read_current_measurement(void)
{
    double sum = 0.0;
    double sum_sq = 0.0;

    for (int i = 0; i < TOTAL_SAMPLES; i++) {
        float sample = read_voltage();
        sum += sample;
        sum_sq += (double)(sample * sample);
        esp_rom_delay_us(100);
    }

    float mean = (float)(sum / TOTAL_SAMPLES);
    float variance = (float)(sum_sq / TOTAL_SAMPLES) - (mean * mean);
    if (variance < 0.0f) {
        variance = 0.0f;
    }

    bias_voltage = mean;

    float adc_rms = sqrtf(variance);
    float baseline_sq = noise_rms_voltage * NOISE_MARGIN_FACTOR;
    baseline_sq *= baseline_sq;

    float signal_sq = (adc_rms * adc_rms) - baseline_sq;
    if (signal_sq < 0.0f) {
        signal_sq = 0.0f;
    }

    float signal_rms = sqrtf(signal_sq);
    float burden_current = signal_rms / BURDEN_RESISTOR;
    float line_current = burden_current * TURNS_RATIO * CURRENT_CALIBRATION;

    if (line_current < CURRENT_NOISE_FLOOR_A) {
        line_current = 0.0f;
    }

    current_measurement_t measurement = {
        .mean_voltage = mean,
        .adc_rms_voltage = adc_rms,
        .signal_rms_voltage = signal_rms,
        .raw_current = line_current,
    };
    return measurement;
}

#define FILTER_SIZE 4
static float apply_filter(float new_value)
{
    static float history[FILTER_SIZE] = {0};
    static int index = 0;
    static int count = 0;

    history[index] = new_value;
    index = (index + 1) % FILTER_SIZE;
    if (count < FILTER_SIZE) {
        count++;
    }

    float sum = 0.0f;
    for (int i = 0; i < count; i++) {
        sum += history[i];
    }
    return sum / count;
}

static void reset_current_filter(void)
{
    apply_filter(0.0f);
    apply_filter(0.0f);
    apply_filter(0.0f);
    apply_filter(0.0f);
}

static float apply_power_filter(float new_value)
{
    static float history[5] = {0};
    static int index = 0;
    static int count = 0;

    history[index] = new_value;
    index = (index + 1) % 5;
    if (count < 5) {
        count++;
    }

    float sum = 0.0f;
    for (int i = 0; i < count; i++) {
        sum += history[i];
    }
    return sum / count;
}

static void reset_power_filter(void)
{
    apply_power_filter(0.0f);
    apply_power_filter(0.0f);
    apply_power_filter(0.0f);
    apply_power_filter(0.0f);
    apply_power_filter(0.0f);
}

static void sct_task(void *arg)
{
    (void)arg;
    static int output_active = 0;
    static int wakeup_count = 0;
    static int sleep_count = 0;

    while (1) {
        s_paused = true;
        while (s_pause_requested) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        s_paused = false;

        current_measurement_t measurement = read_current_measurement();
        if (measurement.raw_current <= CURRENT_NOISE_FLOOR_A) {
            output_active = 0;
            wakeup_count = 0;
            sleep_count = 0;
            reset_current_filter();
            reset_power_filter();
        }

        float current = apply_filter(measurement.raw_current);
        float estimated_power = current * MAINS_VOLTAGE_RMS * POWER_FACTOR_ASSUMED;
        estimated_power = apply_power_filter(estimated_power);
        float adc_voltage = read_voltage();

        if (!output_active) {
            sleep_count = 0;
            if (current >= CURRENT_WAKEUP_A) {
                wakeup_count++;
                if (wakeup_count >= CURRENT_WAKEUP_CONFIRM_SAMPLES) {
                    output_active = 1;
                    wakeup_count = 0;
                }
            } else {
                wakeup_count = 0;
            }
        } else if (current <= CURRENT_NOISE_FLOOR_A) {
            wakeup_count = 0;
            sleep_count++;
            if (sleep_count >= CURRENT_SLEEP_CONFIRM_SAMPLES) {
                output_active = 0;
                sleep_count = 0;
                reset_current_filter();
                reset_power_filter();
            }
        } else {
            sleep_count = 0;
        }

        if (!output_active) {
            current = 0.0f;
            estimated_power = 0.0f;
        }

        if (estimated_power < POWER_NOISE_FLOOR_W) {
            estimated_power = 0.0f;
        }

        sensor_telemetry_set_current(adc_voltage, bias_voltage, current, estimated_power, true);
        sensor_telemetry_set_current_details(measurement.adc_rms_voltage, measurement.signal_rms_voltage, measurement.raw_current);

        sensor_telemetry_t telemetry;
        sensor_telemetry_get(&telemetry);

        algorithm_result_t algorithm = algorithm_update_from_power(estimated_power,
                                                                  true,
                                                                  telemetry.sht30_temperature_c,
                                                                  telemetry.sht30_valid);
        sensor_telemetry_set_algorithm(algorithm.p_loss_w,
                                       algorithm.predicted_delta_t_c,
                                       algorithm.predicted_temperature_c,
                                       algorithm.thermal_samples_collected,
                                       algorithm.thermal_model_ready,
                                       algorithm.valid);

        printf("================================\n");
        printf("POWER DATA\n");
        printf("ADC pin      : %.3f V\n", adc_voltage);
        printf("Bias ref     : %.5f V\n", bias_voltage);
        printf("ADC RMS      : %.5f V\n", measurement.adc_rms_voltage);
        printf("Signal RMS   : %.5f V\n", measurement.signal_rms_voltage);
        printf("Raw Current  : %.3f A\n", measurement.raw_current);
        printf("Filt Current : %.3f A\n", current);
        printf("Est. Power   : %.2f W\n", estimated_power);
        printf("P_loss       : %.2f W\n", algorithm.p_loss_w);
        if (algorithm.thermal_model_ready) {
            printf("Pred Temp    : %.2f C\n", algorithm.predicted_temperature_c);
        } else {
            printf("Pred Temp    : collecting P_loss %d/126\n", algorithm.thermal_samples_collected);
        }
        printf("Assumed mains: %.0f V, PF %.1f\n", MAINS_VOLTAGE_RMS, POWER_FACTOR_ASSUMED);
        printf("================================\n");

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void current_sensor_sct013_pause(void)
{
    s_pause_requested = true;

    TickType_t start = xTaskGetTickCount();
    while (!s_paused && (xTaskGetTickCount() - start) < pdMS_TO_TICKS(500)) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void current_sensor_sct013_resume(void)
{
    s_pause_requested = false;
}

esp_err_t current_sensor_sct013_init(void)
{
    ESP_LOGI(TAG, "------------------------------");
    ESP_LOGI(TAG, "Current and thermal monitor");
    ESP_LOGI(TAG, "ESP32-S3 DevKit GPIO4 (ADC1_CH3)");
    ESP_LOGI(TAG, "230 V assumed for power estimate");
    ESP_LOGI(TAG, "------------------------------");

    init_adc();
    calibrate_current_channel();

    xTaskCreate(sct_task, "sct_task", 8192, NULL, 5, NULL);
    return ESP_OK;
}
