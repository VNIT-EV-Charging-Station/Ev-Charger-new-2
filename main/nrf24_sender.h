#ifndef NRF24_SENDER_H
#define NRF24_SENDER_H

#include <stdint.h>

#include "esp_err.h"

#define NRF24_PACKET_TYPE_CHARGER 0x21

#define NRF24_FLAG_PT1000_VALID (1U << 0)
#define NRF24_FLAG_SHT30_VALID (1U << 1)
#define NRF24_FLAG_MIC_VALID (1U << 2)
#define NRF24_FLAG_CURRENT_VALID (1U << 3)
#define NRF24_FLAG_ALGORITHM_VALID (1U << 4)
#define NRF24_FLAG_THERMAL_READY (1U << 5)

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint8_t flags;
    uint16_t sequence;
    int16_t pt1000_temp_x100;
    int16_t sht30_temp_x100;
    uint16_t sht30_humidity_x100;
    uint16_t mic_hz_x10;
    uint16_t current_ma;
    uint16_t current_raw_ma;
    uint32_t power_est_x100;
    uint16_t current_adc_mv;
    uint16_t current_bias_mv;
    uint16_t current_adc_rms_uv;
    uint16_t current_signal_rms_uv;
    uint16_t p_loss_x100;
    int16_t pred_temp_x100;
} nrf24_charger_packet_t;

#define NRF_PAYLOAD_SIZE ((uint8_t)sizeof(nrf24_charger_packet_t))

_Static_assert(sizeof(nrf24_charger_packet_t) == 32, "NRF24 charger packet must be 32 bytes");

esp_err_t nrf24_sender_init(void);
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
                                         uint8_t flags);

#endif
