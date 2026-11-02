#include <stdio.h>

#include "current_sensor_sct013.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "inmp441_fft.h"
#include "lora_sender.h"
#include "pt1000_max31865.h"
#include "sensor_telemetry.h"
#include "sht30_sensor.h"

#define MIC_MEASURE_INTERVAL_MS 5000

void app_main(void)
{
    esp_err_t current_init_ret = current_sensor_sct013_init();
    if (current_init_ret == ESP_OK) {
        printf("SCT013 Initialized\n");
    } else {
        printf("SCT013 init failed: %s\n", esp_err_to_name(current_init_ret));
    }

    ESP_ERROR_CHECK(pt1000_max31865_init());
    printf("PT1000 Initialized\n");

    esp_err_t sht30_init_ret = sht30_sensor_init();
    bool sht30_ready = (sht30_init_ret == ESP_OK);
    if (sht30_ready) {
        printf("SHT30 Initialized\n");
    } else {
        printf("SHT30 init failed: %s\n", esp_err_to_name(sht30_init_ret));
    }

    esp_err_t lora_init_ret = lora_sender_init();
    bool lora_ready = (lora_init_ret == ESP_OK);
    if (lora_init_ret == ESP_OK) {
        printf("LoRa Initialized\n");
    } else {
        printf("LoRa init failed: %s\n", esp_err_to_name(lora_init_ret));
    }

    TickType_t last_mic_measurement = 0;
    TickType_t last_lora_tx = 0;

    while (1) {
        float pt1000_temperature = 0.0f;
        esp_err_t pt1000_ret = pt1000_max31865_read_temperature_c(&pt1000_temperature);

        if (pt1000_ret == ESP_OK) {
            printf("PT1000 Temperature: %.2f C\n", pt1000_temperature);
        } else {
            printf("Failed to read PT1000 temperature: %s\n", esp_err_to_name(pt1000_ret));
        }

        if (sht30_ready) {
            float sht30_temperature = 0.0f;
            float sht30_humidity = 0.0f;
            esp_err_t sht30_ret = sht30_sensor_read(&sht30_temperature, &sht30_humidity);

            if (sht30_ret == ESP_OK) {
                printf("SHT30 Temperature: %.2f C, Humidity: %.2f %%\n", sht30_temperature, sht30_humidity);
            } else {
                printf("Failed to read SHT30 sensor: %s\n", esp_err_to_name(sht30_ret));
            }
        }

        TickType_t now = xTaskGetTickCount();
        if (last_mic_measurement == 0 ||
            (now - last_mic_measurement) >= pdMS_TO_TICKS(MIC_MEASURE_INTERVAL_MS)) {
            float mic_frequency_hz = 0.0f;

            current_sensor_sct013_pause();
            esp_err_t mic_ret = inmp441_fft_measure_frequency_hz(&mic_frequency_hz);
            current_sensor_sct013_resume();

            if (mic_ret != ESP_OK) {
                printf("Failed to read INMP441 frequency: %s\n", esp_err_to_name(mic_ret));
            }

            last_mic_measurement = now;
        }

        if (lora_ready && (last_lora_tx == 0 || (xTaskGetTickCount() - last_lora_tx) >= pdMS_TO_TICKS(6000))) {
            last_lora_tx = xTaskGetTickCount();
            sensor_telemetry_t telemetry;
            sensor_telemetry_get(&telemetry);

            uint8_t lora_flags = 0;
            if (telemetry.pt1000_valid) {
                lora_flags |= LORA_FLAG_PT1000_VALID;
            }
            if (telemetry.sht30_valid) {
                lora_flags |= LORA_FLAG_SHT30_VALID;
            }
            if (telemetry.mic_valid) {
                lora_flags |= LORA_FLAG_MIC_VALID;
            }
            if (telemetry.current_valid) {
                lora_flags |= LORA_FLAG_CURRENT_VALID;
            }
            if (telemetry.algorithm_valid) {
                lora_flags |= LORA_FLAG_ALGORITHM_VALID;
            }
            if (telemetry.thermal_model_ready) {
                lora_flags |= LORA_FLAG_THERMAL_READY;
            }

            current_sensor_sct013_pause();
            esp_err_t send_ret = lora_sender_send_charger_data(
                telemetry.pt1000_valid ? telemetry.pt1000_temperature_c : 0.0f,
                telemetry.sht30_valid ? telemetry.sht30_temperature_c : 0.0f,
                telemetry.sht30_valid ? telemetry.sht30_humidity_percent : 0.0f,
                telemetry.mic_valid ? telemetry.mic_frequency_hz : 0.0f,
                telemetry.current_valid ? telemetry.current_rms_a : 0.0f,
                telemetry.current_valid ? telemetry.current_raw_a : 0.0f,
                telemetry.current_valid ? telemetry.current_power_w : 0.0f,
                telemetry.current_valid ? telemetry.current_adc_voltage_v : 0.0f,
                telemetry.current_valid ? telemetry.current_bias_voltage_v : 0.0f,
                telemetry.current_valid ? telemetry.current_adc_rms_voltage_v : 0.0f,
                telemetry.current_valid ? telemetry.current_signal_rms_voltage_v : 0.0f,
                telemetry.algorithm_valid ? telemetry.p_loss_w : 0.0f,
                (telemetry.algorithm_valid && telemetry.thermal_model_ready)
                    ? telemetry.thermal_temperature_c
                    : 0.0f,
                lora_flags);
            vTaskDelay(pdMS_TO_TICKS(50));
            current_sensor_sct013_resume();

            if (send_ret != ESP_OK) {
                printf("LoRa send set failed: %s\n", esp_err_to_name(send_ret));
            } else {
                printf("LoRa charger packet TX OK\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
