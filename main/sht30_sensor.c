#include "sht30_sensor.h"

#include <stdbool.h>

#include "i2cdev.h"
#include "sht3x.h"

#include "sensor_telemetry.h"

#define SHT30_I2C_PORT 0
#define SHT30_SDA_GPIO 8
#define SHT30_SCL_GPIO 9
#define SHT30_I2C_ADDR SHT3X_I2C_ADDR_GND

static sht3x_t s_sht30_dev;
static bool s_sht30_initialized;

esp_err_t sht30_sensor_init(void)
{
    if (s_sht30_initialized) {
        return ESP_OK;
    }

    i2cdev_init();

    esp_err_t ret = sht3x_init_desc(&s_sht30_dev, SHT30_I2C_ADDR, SHT30_I2C_PORT, SHT30_SDA_GPIO, SHT30_SCL_GPIO);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = sht3x_init(&s_sht30_dev);
    if (ret != ESP_OK) {
        return ret;
    }

    s_sht30_initialized = true;
    return ESP_OK;
}

esp_err_t sht30_sensor_read(float *temperature_c, float *humidity_percent)
{
    if (temperature_c == NULL || humidity_percent == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_sht30_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = sht3x_measure(&s_sht30_dev, temperature_c, humidity_percent);
    if (ret == ESP_OK) {
        sensor_telemetry_set_sht30(*temperature_c, *humidity_percent, true);
    }

    return ret;
}
