#include "nrf24_sender.h"

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define NRF_CE_GPIO 42
#define NRF_CSN_GPIO 45
#define NRF_SCK_GPIO 39
#define NRF_MOSI_GPIO 40
#define NRF_MISO_GPIO 41

#define NRF_CHANNEL 76
#define NRF_PAYLOAD_LEN 32
#define NRF_SPI_CLOCK_HZ (500 * 1000)
#define NRF_RF_SETUP_250KBPS_REDUCED_PA 0x22
#define NRF_SETUP_RETR_DISABLED 0x00
#define TOTAL_CHARGER_PACKETS 13

#define CMD_R_REGISTER 0x00
#define CMD_W_REGISTER 0x20
#define CMD_W_TX_PAYLOAD 0xA0
#define CMD_FLUSH_TX 0xE1
#define CMD_NOP 0xFF

#define REG_CONFIG 0x00
#define REG_EN_AA 0x01
#define REG_EN_RXADDR 0x02
#define REG_SETUP_AW 0x03
#define REG_SETUP_RETR 0x04
#define REG_RF_CH 0x05
#define REG_RF_SETUP 0x06
#define REG_STATUS 0x07
#define REG_RX_ADDR_P0 0x0A
#define REG_TX_ADDR 0x10
#define REG_RX_PW_P0 0x11
#define REG_DYNPD 0x1C
#define REG_FEATURE 0x1D

#define STATUS_RX_DR BIT(6)
#define STATUS_TX_DS BIT(5)
#define STATUS_MAX_RT BIT(4)
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

_Static_assert(sizeof(charger_packet_t) == NRF_PAYLOAD_LEN, "charger packet must be exactly one NRF24 payload");

static const char *TAG = "NRF24_CHARGER_TX";
static const uint8_t BESS_ADDR[5] = {'B', 'E', 'S', 'S', '1'};
static spi_device_handle_t s_spi;
static uint16_t s_sequence;

static esp_err_t nrf_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_transmit(s_spi, &t);
}

static uint8_t nrf_read_reg(uint8_t reg)
{
    uint8_t tx[2] = {CMD_R_REGISTER | (reg & 0x1F), CMD_NOP};
    uint8_t rx[2] = {0};
    (void)nrf_transfer(tx, rx, sizeof(tx));
    return rx[1];
}

static void nrf_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {CMD_W_REGISTER | (reg & 0x1F), value};
    (void)nrf_transfer(tx, NULL, sizeof(tx));
}

static void nrf_write_buf(uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t tx[1 + 5] = {0};
    tx[0] = CMD_W_REGISTER | (reg & 0x1F);
    memcpy(&tx[1], data, len);
    (void)nrf_transfer(tx, NULL, len + 1);
}

static void nrf_command(uint8_t cmd)
{
    uint8_t tx = cmd;
    (void)nrf_transfer(&tx, NULL, 1);
}

static esp_err_t send_payload(const void *payload, size_t len)
{
    uint8_t tx[1 + NRF_PAYLOAD_LEN] = {CMD_W_TX_PAYLOAD};
    memcpy(&tx[1], payload, len);

    nrf_write_reg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
    nrf_command(CMD_FLUSH_TX);
    ESP_RETURN_ON_ERROR(nrf_transfer(tx, NULL, sizeof(tx)), TAG, "payload write failed");

    gpio_set_level(NRF_CE_GPIO, 1);
    esp_rom_delay_us(20);
    gpio_set_level(NRF_CE_GPIO, 0);

    for (int i = 0; i < 120; i++) {
        uint8_t status = nrf_read_reg(REG_STATUS);
        if ((status & STATUS_TX_DS) != 0) {
            nrf_write_reg(REG_STATUS, STATUS_TX_DS);
            return ESP_OK;
        }
        if ((status & STATUS_MAX_RT) != 0) {
            nrf_write_reg(REG_STATUS, STATUS_MAX_RT);
            nrf_command(CMD_FLUSH_TX);
            return ESP_FAIL;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    nrf_command(CMD_FLUSH_TX);
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

esp_err_t nrf24_sender_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = NRF_MOSI_GPIO,
        .miso_io_num = NRF_MISO_GPIO,
        .sclk_io_num = NRF_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = NRF_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = NRF_CSN_GPIO,
        .queue_size = 1,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi), TAG, "add device failed");

    gpio_config_t ce_cfg = {
        .pin_bit_mask = 1ULL << NRF_CE_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&ce_cfg));
    gpio_set_level(NRF_CE_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(100));

    nrf_write_reg(REG_CONFIG, 0x0C);
    vTaskDelay(pdMS_TO_TICKS(5));
    nrf_write_reg(REG_EN_AA, 0x00);
    nrf_write_reg(REG_EN_RXADDR, 0x01);
    nrf_write_reg(REG_SETUP_AW, 0x03);
    nrf_write_reg(REG_SETUP_RETR, NRF_SETUP_RETR_DISABLED);
    nrf_write_reg(REG_RF_CH, NRF_CHANNEL);
    nrf_write_reg(REG_RF_SETUP, NRF_RF_SETUP_250KBPS_REDUCED_PA);
    nrf_write_buf(REG_TX_ADDR, BESS_ADDR, sizeof(BESS_ADDR));
    nrf_write_buf(REG_RX_ADDR_P0, BESS_ADDR, sizeof(BESS_ADDR));
    nrf_write_reg(REG_RX_PW_P0, NRF_PAYLOAD_LEN);
    nrf_write_reg(REG_EN_AA, 0x00);
    nrf_write_reg(REG_FEATURE, 0x00);
    nrf_write_reg(REG_DYNPD, 0x00);
    nrf_write_reg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
    nrf_command(CMD_FLUSH_TX);
    nrf_write_reg(REG_CONFIG, 0x0E);
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t config = nrf_read_reg(REG_CONFIG);
    uint8_t channel = nrf_read_reg(REG_RF_CH);
    uint8_t rf_setup = nrf_read_reg(REG_RF_SETUP);
    if (config == 0x00 || config == 0xFF || (config & 0x0E) != 0x0E ||
        channel != NRF_CHANNEL || rf_setup != NRF_RF_SETUP_250KBPS_REDUCED_PA) {
        ESP_LOGE(TAG, "NRF24 bad readback CONFIG=0x%02X CH=%u RF=0x%02X",
                 config, channel, rf_setup);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "TX channel %u, 250kbps, shared address, no ACK, SPI=%uHz",
             NRF_CHANNEL, NRF_SPI_CLOCK_HZ);
    return ESP_OK;
}

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

    esp_err_t ret = send_payload(&packet, sizeof(packet));
    vTaskDelay(pdMS_TO_TICKS(35));
    esp_err_t ret_repeat = send_payload(&packet, sizeof(packet));
    return ret == ESP_OK ? ret_repeat : ret;
}
