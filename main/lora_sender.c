#include "lora_sender.h"

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LORA_SCK_GPIO 39
#define LORA_MOSI_GPIO 40
#define LORA_MISO_GPIO 41
#define LORA_NSS_GPIO 45
#define LORA_RESET_GPIO 42
#define LORA_DIO0_GPIO 47
#define LORA_SPI_CLOCK_HZ (5 * 1000 * 1000)
#define LORA_FREQUENCY_HZ 433920000UL
#define LORA_PACKET_LEN 32
#define LORA_SYNC_WORD 0x12
#define LORA_MODEM_CONFIG_1 0x78 /* BW 125 kHz, CR 4/8, explicit header */
#define LORA_MODEM_CONFIG_2 0xA4 /* SF10, payload CRC enabled */
#define LORA_MODEM_CONFIG_3 0x04 /* AGC; low data-rate optimization disabled */
#define LORA_TX_POWER_CONFIG 0x87 /* PA_BOOST, approx. +9 dBm conducted */
#define LORA_TX_MIN_INTERVAL_MS 6000

#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_OCP 0x0B
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_FIFO_TX_BASE_ADDR 0x0E
#define REG_IRQ_FLAGS 0x12
#define REG_PAYLOAD_LENGTH 0x22
#define REG_MODEM_CONFIG_1 0x1D
#define REG_MODEM_CONFIG_2 0x1E
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_MODEM_CONFIG_3 0x26
#define REG_SYNC_WORD 0x39
#define REG_DIO_MAPPING_1 0x40
#define REG_VERSION 0x42

#define MODE_LONG_RANGE 0x80
#define MODE_SLEEP 0x00
#define MODE_STANDBY 0x01
#define MODE_TX 0x03
#define IRQ_TX_DONE BIT(3)
#define PACKET_TYPE_CHARGER 0x21

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint8_t flags;
    uint16_t sequence;
    int16_t pt1000_temp_x100;
    int16_t sht30_temp_x100;
    uint16_t sht30_humidity_x100;
    uint16_t mic_hz_x10;
    int16_t current_ma;
    int16_t current_raw_ma;
    int32_t power_est_x100;
    uint16_t current_adc_mv;
    uint16_t current_bias_mv;
    uint16_t current_adc_rms_uv;
    uint16_t current_signal_rms_uv;
    int16_t p_loss_x100;
    int16_t pred_temp_x100;
} charger_packet_t;

_Static_assert(sizeof(charger_packet_t) == LORA_PACKET_LEN, "charger telemetry wire format must remain unchanged");

static const char *TAG = "LORA_CHARGER_TX";
static spi_device_handle_t s_spi;
static uint16_t s_sequence;
static TickType_t s_last_tx_tick;

static esp_err_t lora_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_transmit(s_spi, &t);
}

static uint8_t lora_read_reg(uint8_t reg)
{
    uint8_t tx[2] = {reg & 0x7F, 0};
    uint8_t rx[2] = {0};
    (void)lora_transfer(tx, rx, sizeof(tx));
    return rx[1];
}

static esp_err_t lora_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg | 0x80, value};
    return lora_transfer(tx, NULL, sizeof(tx));
}

static esp_err_t lora_set_frequency(uint32_t frequency_hz)
{
    uint64_t frf = ((uint64_t)frequency_hz << 19) / 32000000ULL;
    esp_err_t ret = lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    if (ret == ESP_OK) ret = lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
    if (ret == ESP_OK) ret = lora_write_reg(REG_FRF_LSB, (uint8_t)frf);
    return ret;
}

static esp_err_t send_payload(const void *payload, size_t len)
{
    if (payload == NULL || len == 0 || len > LORA_PACKET_LEN) return ESP_ERR_INVALID_ARG;
    esp_err_t ret = lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE | MODE_STANDBY);
    if (ret == ESP_OK) ret = lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x00);
    if (ret == ESP_OK) ret = lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);
    if (ret != ESP_OK) return ret;

    uint8_t tx[LORA_PACKET_LEN + 1] = {REG_FIFO | 0x80};
    memcpy(&tx[1], payload, len);
    ret = lora_transfer(tx, NULL, len + 1);
    if (ret == ESP_OK) ret = lora_write_reg(REG_PAYLOAD_LENGTH, (uint8_t)len);
    if (ret == ESP_OK) ret = lora_write_reg(REG_IRQ_FLAGS, 0xFF);
    if (ret == ESP_OK) ret = lora_write_reg(REG_DIO_MAPPING_1, 0x40); /* DIO0 = TxDone */
    if (ret == ESP_OK) ret = lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE | MODE_TX);
    if (ret != ESP_OK) return ret;

    const TickType_t timeout = pdMS_TO_TICKS(10000);
    TickType_t started = xTaskGetTickCount();
    while ((xTaskGetTickCount() - started) < timeout) {
        if (gpio_get_level(LORA_DIO0_GPIO) != 0 && (lora_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE)) {
            lora_write_reg(REG_IRQ_FLAGS, 0xFF);
            lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE | MODE_STANDBY);
            s_last_tx_tick = xTaskGetTickCount();
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE | MODE_STANDBY);
    return ESP_ERR_TIMEOUT;
}

static int16_t clamp_i16(float value)
{
    if (value > 32767.0f) {
        return 32767;
    }
    if (value < -32768.0f) {
        return -32768;
    }
    return (int16_t)lroundf(value);
}

static uint16_t clamp_u16(float value)
{
    if (value > 65535.0f) {
        return 65535;
    }
    if (value < 0.0f) {
        return 0;
    }
    return (uint16_t)lroundf(value);
}

static int32_t clamp_i32(float value)
{
    if (value > 2147483647.0f) {
        return 2147483647;
    }
    if (value < -2147483648.0f) {
        return (int32_t)-2147483647 - 1;
    }
    return (int32_t)lroundf(value);
}

esp_err_t lora_sender_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = LORA_MOSI_GPIO,
        .miso_io_num = LORA_MISO_GPIO,
        .sclk_io_num = LORA_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LORA_PACKET_LEN + 1,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = LORA_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = LORA_NSS_GPIO,
        .queue_size = 1,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    esp_err_t ret_add = spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi);
    if (ret_add != ESP_OK) return ret_add;

    gpio_config_t reset_cfg = {
        .pin_bit_mask = 1ULL << LORA_RESET_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ret = gpio_config(&reset_cfg);
    if (ret != ESP_OK) return ret;
    gpio_config_t dio_cfg = {
        .pin_bit_mask = 1ULL << LORA_DIO0_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&dio_cfg);
    if (ret != ESP_OK) return ret;

    gpio_set_level(LORA_RESET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_RESET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t version = lora_read_reg(REG_VERSION);
    if (version != 0x12) {
        ESP_LOGE(TAG, "SX1278 not detected (RegVersion=0x%02X)", version);
        return ESP_FAIL;
    }

    ret = lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE | MODE_SLEEP);
    if (ret == ESP_OK) ret = lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE | MODE_STANDBY);
    if (ret == ESP_OK) ret = lora_set_frequency(LORA_FREQUENCY_HZ);
    if (ret == ESP_OK) ret = lora_write_reg(REG_MODEM_CONFIG_1, LORA_MODEM_CONFIG_1);
    if (ret == ESP_OK) ret = lora_write_reg(REG_MODEM_CONFIG_2, LORA_MODEM_CONFIG_2);
    if (ret == ESP_OK) ret = lora_write_reg(REG_MODEM_CONFIG_3, LORA_MODEM_CONFIG_3);
    if (ret == ESP_OK) ret = lora_write_reg(REG_PREAMBLE_MSB, 0x00);
    if (ret == ESP_OK) ret = lora_write_reg(REG_PREAMBLE_LSB, 0x08);
    if (ret == ESP_OK) ret = lora_write_reg(REG_SYNC_WORD, LORA_SYNC_WORD);
    if (ret == ESP_OK) ret = lora_write_reg(REG_PA_CONFIG, LORA_TX_POWER_CONFIG);
    if (ret == ESP_OK) ret = lora_write_reg(REG_OCP, 0x2B);
    if (ret == ESP_OK) ret = lora_write_reg(REG_DIO_MAPPING_1, 0x40);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "TX %lu Hz, SF10/BW125/CR4-8, unchanged 32-byte packet, PA config 0x%02X",
             (unsigned long)LORA_FREQUENCY_HZ, LORA_TX_POWER_CONFIG);
    return ESP_OK;
}

esp_err_t lora_sender_send_charger_data(float pt1000_temperature_c,
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
                                         uint8_t flags)
{
    charger_packet_t packet = {
        .type = PACKET_TYPE_CHARGER,
        .flags = flags,
        .sequence = ++s_sequence,
        .pt1000_temp_x100 = clamp_i16(pt1000_temperature_c * 100.0f),
        .sht30_temp_x100 = clamp_i16(sht30_temperature_c * 100.0f),
        .sht30_humidity_x100 = clamp_u16(sht30_humidity_percent * 100.0f),
        .mic_hz_x10 = clamp_u16(mic_frequency_hz * 10.0f),
        .current_ma = clamp_i16(current_rms_a * 1000.0f),
        .current_raw_ma = clamp_i16(current_raw_a * 1000.0f),
        .power_est_x100 = clamp_i32(current_power_w * 100.0f),
        .current_adc_mv = clamp_u16(current_adc_voltage_v * 1000.0f),
        .current_bias_mv = clamp_u16(current_bias_voltage_v * 1000.0f),
        .current_adc_rms_uv = clamp_u16(current_adc_rms_voltage_v * 1000000.0f),
        .current_signal_rms_uv = clamp_u16(current_signal_rms_voltage_v * 1000000.0f),
        .p_loss_x100 = clamp_i16(p_loss_w * 100.0f),
        .pred_temp_x100 = clamp_i16(thermal_temperature_c * 100.0f),
    };

    TickType_t now = xTaskGetTickCount();
    if (s_last_tx_tick != 0 &&
        (now - s_last_tx_tick) < pdMS_TO_TICKS(LORA_TX_MIN_INTERVAL_MS)) {
        return ESP_ERR_NOT_FINISHED;
    }
    return send_payload(&packet, sizeof(packet));
}
