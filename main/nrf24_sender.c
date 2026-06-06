#include "nrf24_sender.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define NRF_CE_PIN GPIO_NUM_42
#define NRF_CSN_PIN GPIO_NUM_45
#define NRF_SCK_PIN GPIO_NUM_39
#define NRF_MOSI_PIN GPIO_NUM_40
#define NRF_MISO_PIN GPIO_NUM_41

#define NRF_SPI_HOST SPI3_HOST
#define NRF_SPI_CLOCK_HZ (1000 * 1000)

#define NRF_RF_CHANNEL 76
#define NRF_RF_SETUP_250KBPS_LOW_POWER 0x26
#define NRF_SETUP_RETR_4000US_15_RETRY 0xFF

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

#define CMD_W_REGISTER 0x20
#define CMD_ACTIVATE 0x50
#define CMD_W_TX_PAYLOAD 0xA0
#define CMD_FLUSH_TX 0xE1
#define CMD_FLUSH_RX 0xE2
#define CMD_NOP 0xFF

#define CONFIG_EN_CRC (1U << 3)
#define CONFIG_CRCO (1U << 2)
#define CONFIG_PWR_UP (1U << 1)

#define STATUS_RX_DR (1U << 6)
#define STATUS_TX_DS (1U << 5)
#define STATUS_MAX_RT (1U << 4)
#define STATUS_CLEAR_MASK (STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT)

#define TX_TIMEOUT_MS 250

static const char *TAG = "NRF24_TX";
static const uint8_t pipe_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

static spi_device_handle_t s_nrf;
static bool s_initialized;
static uint16_t s_sequence;

static void ce(bool level)
{
    gpio_set_level(NRF_CE_PIN, level ? 1 : 0);
}

static esp_err_t transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t transaction = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_polling_transmit(s_nrf, &transaction);
}

static esp_err_t command(uint8_t cmd)
{
    uint8_t tx[1] = {cmd};
    uint8_t rx[1] = {0};
    return transfer(tx, rx, sizeof(tx));
}

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(CMD_W_REGISTER | (reg & 0x1F)), value};
    uint8_t rx[2] = {0};
    return transfer(tx, rx, sizeof(tx));
}

static esp_err_t write_reg_bytes(uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t tx[6] = {0};
    uint8_t rx[6] = {0};

    tx[0] = CMD_W_REGISTER | (reg & 0x1F);
    memcpy(&tx[1], data, len);
    return transfer(tx, rx, len + 1);
}

static esp_err_t read_status(uint8_t *status)
{
    uint8_t tx[1] = {CMD_NOP};
    uint8_t rx[1] = {0};

    ESP_RETURN_ON_ERROR(transfer(tx, rx, sizeof(tx)), TAG, "status read failed");
    *status = rx[0];
    return ESP_OK;
}

static esp_err_t clear_status(void)
{
    return write_reg(REG_STATUS, STATUS_CLEAR_MASK);
}

static esp_err_t activate_features(void)
{
    uint8_t tx[2] = {CMD_ACTIVATE, 0x73};
    uint8_t rx[2] = {0};
    return transfer(tx, rx, sizeof(tx));
}

static uint16_t clamp_u16(float value)
{
    if (value <= 0.0f) {
        return 0;
    }
    if (value >= 65535.0f) {
        return 65535;
    }
    return (uint16_t)(value + 0.5f);
}

static uint32_t clamp_u32(float value)
{
    if (value <= 0.0f) {
        return 0;
    }
    if (value >= 4294967295.0f) {
        return 4294967295U;
    }
    return (uint32_t)(value + 0.5f);
}

static int16_t clamp_i16(float value)
{
    if (value <= -32768.0f) {
        return -32768;
    }
    if (value >= 32767.0f) {
        return 32767;
    }
    return (int16_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static esp_err_t write_tx_payload(const nrf24_charger_packet_t *packet)
{
    uint8_t tx[1 + NRF_PAYLOAD_SIZE] = {0};
    uint8_t rx[1 + NRF_PAYLOAD_SIZE] = {0};

    tx[0] = CMD_W_TX_PAYLOAD;
    memcpy(&tx[1], packet, sizeof(*packet));
    return transfer(tx, rx, sizeof(tx));
}

esp_err_t nrf24_sender_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    gpio_config_t ce_config = {
        .pin_bit_mask = 1ULL << NRF_CE_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&ce_config), TAG, "CE config failed");
    ce(false);

    spi_bus_config_t bus_config = {
        .mosi_io_num = NRF_MOSI_PIN,
        .miso_io_num = NRF_MISO_PIN,
        .sclk_io_num = NRF_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1 + NRF_PAYLOAD_SIZE,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(NRF_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO), TAG, "SPI bus failed");

    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = NRF_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = NRF_CSN_PIN,
        .queue_size = 1,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(NRF_SPI_HOST, &dev_config, &s_nrf), TAG, "SPI device failed");

    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_RETURN_ON_ERROR(command(CMD_FLUSH_TX), TAG, "flush tx failed");
    ESP_RETURN_ON_ERROR(command(CMD_FLUSH_RX), TAG, "flush rx failed");
    ESP_RETURN_ON_ERROR(clear_status(), TAG, "clear status failed");

    ESP_RETURN_ON_ERROR(write_reg(REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO | CONFIG_PWR_UP), TAG, "config failed");
    ESP_RETURN_ON_ERROR(write_reg(REG_EN_AA, 0x01), TAG, "ack failed");
    ESP_RETURN_ON_ERROR(write_reg(REG_EN_RXADDR, 0x01), TAG, "rxaddr failed");
    ESP_RETURN_ON_ERROR(write_reg(REG_SETUP_AW, 0x03), TAG, "addr width failed");
    ESP_RETURN_ON_ERROR(write_reg(REG_SETUP_RETR, NRF_SETUP_RETR_4000US_15_RETRY), TAG, "retry failed");
    ESP_RETURN_ON_ERROR(write_reg(REG_RF_CH, NRF_RF_CHANNEL), TAG, "channel failed");
    ESP_RETURN_ON_ERROR(write_reg(REG_RF_SETUP, NRF_RF_SETUP_250KBPS_LOW_POWER), TAG, "rf setup failed");
    ESP_ERROR_CHECK_WITHOUT_ABORT(activate_features());
    ESP_RETURN_ON_ERROR(write_reg(REG_DYNPD, 0x01), TAG, "dynpd failed");
    ESP_RETURN_ON_ERROR(write_reg(REG_FEATURE, 0x04), TAG, "feature failed");
    ESP_RETURN_ON_ERROR(write_reg_bytes(REG_TX_ADDR, pipe_addr, sizeof(pipe_addr)), TAG, "tx addr failed");
    ESP_RETURN_ON_ERROR(write_reg_bytes(REG_RX_ADDR_P0, pipe_addr, sizeof(pipe_addr)), TAG, "rx p0 addr failed");

    vTaskDelay(pdMS_TO_TICKS(5));

    s_initialized = true;
    ESP_LOGI(TAG, "NRF24 sender ready: one 32-byte packet, CH76, RF_SETUP 0x26, SPI=1MHz, DPL");
    return ESP_OK;
}

esp_err_t nrf24_sender_send_charger_data(float pt1000_temp,
                                         float sht30_temp,
                                         float sht30_humidity,
                                         float mic_hz,
                                         float current,
                                         float current_raw,
                                         float power_est,
                                         float current_adc,
                                         float current_bias,
                                         float current_adc_rms,
                                         float current_signal_rms,
                                         float p_loss,
                                         float pred_temp,
                                         uint8_t flags)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    nrf24_charger_packet_t packet = {
        .type = NRF24_PACKET_TYPE_CHARGER,
        .flags = flags,
        .sequence = s_sequence++,
        .pt1000_temp_x100 = clamp_i16(pt1000_temp * 100.0f),
        .sht30_temp_x100 = clamp_i16(sht30_temp * 100.0f),
        .sht30_humidity_x100 = clamp_u16(sht30_humidity * 100.0f),
        .mic_hz_x10 = clamp_u16(mic_hz * 10.0f),
        .current_ma = clamp_u16(current * 1000.0f),
        .current_raw_ma = clamp_u16(current_raw * 1000.0f),
        .power_est_x100 = clamp_u32(power_est * 100.0f),
        .current_adc_mv = clamp_u16(current_adc * 1000.0f),
        .current_bias_mv = clamp_u16(current_bias * 1000.0f),
        .current_adc_rms_uv = clamp_u16(current_adc_rms * 1000000.0f),
        .current_signal_rms_uv = clamp_u16(current_signal_rms * 1000000.0f),
        .p_loss_x100 = clamp_u16(p_loss * 100.0f),
        .pred_temp_x100 = clamp_i16(pred_temp * 100.0f),
    };

    ESP_RETURN_ON_ERROR(command(CMD_FLUSH_TX), TAG, "flush tx failed");
    ESP_RETURN_ON_ERROR(clear_status(), TAG, "clear status failed");
    ESP_RETURN_ON_ERROR(write_tx_payload(&packet), TAG, "payload write failed");

    ce(true);
    esp_rom_delay_us(20);
    ce(false);

    for (int i = 0; i < TX_TIMEOUT_MS; i++) {
        uint8_t status = 0;
        ESP_RETURN_ON_ERROR(read_status(&status), TAG, "status read failed");

        if (status & STATUS_TX_DS) {
            clear_status();
            ESP_LOGI(TAG,
                     "TX seq=%u PT=%.2f SHT=%.2f H=%.2f Mic=%.1fHz I=%.3fA P=%.2fW Ploss=%.2f Pred=%.2f",
                     packet.sequence,
                     packet.pt1000_temp_x100 / 100.0f,
                     packet.sht30_temp_x100 / 100.0f,
                     packet.sht30_humidity_x100 / 100.0f,
                     packet.mic_hz_x10 / 10.0f,
                     packet.current_ma / 1000.0f,
                     packet.power_est_x100 / 100.0f,
                     packet.p_loss_x100 / 100.0f,
                     packet.pred_temp_x100 / 100.0f);
            return ESP_OK;
        }

        if (status & STATUS_MAX_RT) {
            clear_status();
            command(CMD_FLUSH_TX);
            ESP_LOGW(TAG, "TX failed MAX_RT seq=%u", packet.sequence);
            return ESP_FAIL;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    command(CMD_FLUSH_TX);
    ESP_LOGW(TAG, "TX timeout seq=%u", packet.sequence);
    return ESP_ERR_TIMEOUT;
}
