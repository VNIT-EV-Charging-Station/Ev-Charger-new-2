#include "inmp441_fft.h"

#include <stdbool.h>
#include <math.h>
#include <stdio.h>

#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_dsp.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensor_telemetry.h"

#define TAG "MIC_FFT"

#define MIC_WS_PIN  18
#define MIC_SCK_PIN 17
#define MIC_SD_PIN  16

#define SAMPLE_RATE       16000
#define FFT_SIZE          1024
#define ADC_MAX_VALUE     8388608.0f
#define LOG_INTERVAL_MS   1000

#define EMA_ALPHA_FREQ    0.15f
#define EMA_ALPHA_DB      0.20f

#define NOISE_DB_THRESHOLD -90.0f
#define FFT_MAG_THRESHOLD   2.0f

static __attribute__((aligned(16))) float fft_buf[FFT_SIZE * 2];
static __attribute__((aligned(16))) float window[FFT_SIZE];
static int32_t raw_buf[FFT_SIZE];

static i2s_chan_handle_t rx_chan = NULL;
static bool s_inmp441_initialized;
static bool s_fft_initialized;
static bool s_window_built;
static volatile bool s_pause_requested;
static volatile bool s_paused;
static bool s_i2s_enabled;

static void i2s_init(void)
{
    if (rx_chan != NULL) {
        if (!s_i2s_enabled) {
            ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
            s_i2s_enabled = true;
        }
        return;
    }

    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT,
            I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = MIC_SCK_PIN,
            .ws = MIC_WS_PIN,
            .dout = I2S_GPIO_UNUSED,
            .din = MIC_SD_PIN,
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    s_i2s_enabled = true;

    printf("I2S Initialized\n");
}

static void i2s_disable_if_enabled(void)
{
    if (rx_chan != NULL && s_i2s_enabled) {
        ESP_ERROR_CHECK(i2s_channel_disable(rx_chan));
        s_i2s_enabled = false;
    }
}

static void handle_pause_request(void)
{
    if (!s_pause_requested) {
        return;
    }

    if (s_i2s_enabled) {
        ESP_ERROR_CHECK(i2s_channel_disable(rx_chan));
        s_i2s_enabled = false;
    }

    s_paused = true;
    while (s_pause_requested) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    s_paused = false;

    if (!s_i2s_enabled) {
        ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
        s_i2s_enabled = true;
    }
}

static int mic_read(void)
{
    size_t bytes = 0;

    if (i2s_channel_read(rx_chan, raw_buf, sizeof(raw_buf), &bytes, portMAX_DELAY) != ESP_OK) {
        return -1;
    }

    int n = (int)(bytes / sizeof(int32_t));
    for (int i = 0; i < n; ++i) {
        raw_buf[i] >>= 8;
    }

    return n;
}

static void build_window(void)
{
    if (s_window_built) {
        return;
    }

    for (int i = 0; i < FFT_SIZE; ++i) {
        window[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (FFT_SIZE - 1)));
    }

    s_window_built = true;
}

static void load_fft(void)
{
    for (int i = 0; i < FFT_SIZE; ++i) {
        float v = (float)raw_buf[i] / ADC_MAX_VALUE;
        v *= 1.5f;
        fft_buf[2 * i] = v * window[i];
        fft_buf[2 * i + 1] = 0.0f;
    }
}

static float get_db(void)
{
    double sum = 0.0;

    for (int i = 0; i < FFT_SIZE; ++i) {
        double v = (double)raw_buf[i] / ADC_MAX_VALUE;
        sum += v * v;
    }

    double rms = sqrt(sum / FFT_SIZE);
    if (rms < 1e-10) {
        return -120.0f;
    }

    return (float)(20.0 * log10(rms));
}

static float get_freq(void)
{
    float bin_hz = (float)SAMPLE_RATE / FFT_SIZE;

    ESP_ERROR_CHECK(dsps_fft2r_fc32(fft_buf, FFT_SIZE));
    ESP_ERROR_CHECK(dsps_bit_rev2r_fc32(fft_buf, FFT_SIZE));
    dsps_cplx2reC_fc32(fft_buf, FFT_SIZE);

    float max_mag = 0.0f;
    int max_bin = -1;

    for (int i = 4; i < 80; ++i) {
        if (fft_buf[i] > max_mag && fft_buf[i] > FFT_MAG_THRESHOLD) {
            max_mag = fft_buf[i];
            max_bin = i;
        }
    }

    return (max_bin > 0) ? (max_bin * bin_hz) : 0.0f;
}

static void mic_task(void *arg)
{
    if (!s_fft_initialized) {
        ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, FFT_SIZE));
        s_fft_initialized = true;
    }
    build_window();

    float ema_freq = 0.0f;
    float ema_db = -120.0f;
    int first = 1;

    TickType_t last_print = 0;
    const TickType_t print_interval = pdMS_TO_TICKS(LOG_INTERVAL_MS);

    while (1) {
        handle_pause_request();

        if (mic_read() != FFT_SIZE) {
            continue;
        }

        float raw_db = get_db();
        load_fft();
        float raw_freq = get_freq();

        if (first) {
            ema_freq = raw_freq;
            ema_db = raw_db;
            first = 0;
        } else {
            if (raw_freq > 0.0f) {
                ema_freq = EMA_ALPHA_FREQ * raw_freq + (1.0f - EMA_ALPHA_FREQ) * ema_freq;
            }
            ema_db = EMA_ALPHA_DB * raw_db + (1.0f - EMA_ALPHA_DB) * ema_db;
        }

        TickType_t now = xTaskGetTickCount();
        if (now - last_print >= print_interval) {
            last_print = now;
            sensor_telemetry_set_mic(ema_freq, ema_db, ema_db > NOISE_DB_THRESHOLD);

            if (ema_db > NOISE_DB_THRESHOLD) {
                if (ema_freq > 0.0f) {
                    printf("Freq: %.1f Hz | Level: %.1f dBFS\n", ema_freq, ema_db);
                } else {
                    printf("Freq: --- | Level: %.1f dBFS\n", ema_db);
                }
            } else {
                printf("Freq: --- | Level: %.1f dBFS\n", ema_db);
            }
        }
    }
}

esp_err_t inmp441_fft_measure_frequency_hz(float *frequency_hz)
{
    if (frequency_hz == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_fft_initialized) {
        ESP_RETURN_ON_ERROR(dsps_fft2r_init_fc32(NULL, FFT_SIZE), TAG, "FFT init failed");
        s_fft_initialized = true;
    }

    build_window();
    i2s_init();

    if (mic_read() != FFT_SIZE) {
        i2s_disable_if_enabled();
        return ESP_FAIL;
    }

    float raw_db = get_db();
    load_fft();
    float raw_freq = get_freq();

    i2s_disable_if_enabled();

    *frequency_hz = raw_freq;
    sensor_telemetry_set_mic(raw_freq, raw_db, raw_freq > 0.0f);
    printf("Mic Frequency: %.2f Hz\n", raw_freq);
    return ESP_OK;
}

void inmp441_fft_pause_for_current_measurement(void)
{
    if (!s_inmp441_initialized) {
        return;
    }

    s_pause_requested = true;

    TickType_t start = xTaskGetTickCount();
    while (!s_paused && (xTaskGetTickCount() - start) < pdMS_TO_TICKS(200)) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void inmp441_fft_resume_after_current_measurement(void)
{
    s_pause_requested = false;
}

esp_err_t inmp441_fft_init(void)
{
    if (s_inmp441_initialized) {
        return ESP_OK;
    }

    printf("Starting microphone FFT monitor\n");
    i2s_init();
    ESP_RETURN_ON_FALSE(xTaskCreate(mic_task, "mic_task", 8192, NULL, 3, NULL) == pdPASS, ESP_FAIL, TAG, "Failed to create mic task");

    s_inmp441_initialized = true;
    return ESP_OK;
}
