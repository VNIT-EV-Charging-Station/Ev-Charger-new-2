#include "pt1000_max31865.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/spi_master.h"

#include "sensor_telemetry.h"

#define PT1000_PIN_NUM_MISO 13
#define PT1000_PIN_NUM_MOSI 11
#define PT1000_PIN_NUM_CLK  12
#define PT1000_PIN_NUM_CS   10

#define MAX31865_RREF     4300.0f
#define PT1000_RNOMINAL   1000.0f
#define MAX31865_CONFIG_REG 0x00
#define MAX31865_RTD_MSB_REG 0x01
#define MAX31865_WRITE_BIT 0x80
#define MAX31865_ENABLE_FLAGS 0xD0

static spi_device_handle_t s_pt1000_spi;
static bool s_pt1000_initialized;

static esp_err_t max31865_write_register(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = { (uint8_t)(reg | MAX31865_WRITE_BIT), data };
    spi_transaction_t transaction = {
        .length = 16,
        .tx_buffer = tx,
    };

    return spi_device_transmit(s_pt1000_spi, &transaction);
}

static esp_err_t max31865_read_register16(uint8_t reg, uint16_t *data)
{
    uint8_t tx[3] = { (uint8_t)(reg & 0x7F), 0x00, 0x00 };
    uint8_t rx[3] = { 0 };
    spi_transaction_t transaction = {
        .length = 24,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t ret = spi_device_transmit(s_pt1000_spi, &transaction);
    if (ret != ESP_OK) {
        return ret;
    }

    *data = ((uint16_t)rx[1] << 8) | rx[2];
    return ESP_OK;
}

static float max31865_rtd_to_temperature(uint16_t rtd)
{
    rtd >>= 1;

    float resistance = ((float)rtd / 32768.0f) * MAX31865_RREF;
    return (resistance - PT1000_RNOMINAL) / (0.00385f * PT1000_RNOMINAL);
}

esp_err_t pt1000_max31865_init(void)
{
    if (s_pt1000_initialized) {
        return ESP_OK;
    }

    spi_bus_config_t buscfg = {
        .mosi_io_num = PT1000_PIN_NUM_MOSI,
        .miso_io_num = PT1000_PIN_NUM_MISO,
        .sclk_io_num = PT1000_PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 1,
        .spics_io_num = PT1000_PIN_NUM_CS,
        .queue_size = 1,
    };

    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = spi_bus_add_device(SPI3_HOST, &devcfg, &s_pt1000_spi);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = max31865_write_register(MAX31865_CONFIG_REG, MAX31865_ENABLE_FLAGS);
    if (ret != ESP_OK) {
        return ret;
    }

    s_pt1000_initialized = true;
    return ESP_OK;
}

esp_err_t pt1000_max31865_read_temperature_c(float *temperature_c)
{
    if (temperature_c == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_pt1000_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t rtd = 0;
    esp_err_t ret = max31865_read_register16(MAX31865_RTD_MSB_REG, &rtd);
    if (ret != ESP_OK) {
        return ret;
    }

    *temperature_c = max31865_rtd_to_temperature(rtd);
    sensor_telemetry_set_pt1000(*temperature_c, true);
    return ESP_OK;
}
