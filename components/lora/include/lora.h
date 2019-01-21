#ifndef __LORA_H__
#define __LORA_H__

#include "driver/spi_master.h"
#include "driver/gpio.h"

#if defined(__cplusplus)
extern "C" {
#endif

#ifndef ESP32_HAL_UNDEFINED
#define ESP32_HAL_UNDEFINED (-1)
#endif

#define LORA_ESP32_ERR_RESET_TIMEOUT 0x9000

#define LORA_ESP32_PARAM_DEFAULT {\
HSPI_HOST, \
(gpio_num_t)ESP32_HAL_UNDEFINED, \
(gpio_num_t)ESP32_HAL_UNDEFINED, \
(gpio_num_t)ESP32_HAL_UNDEFINED, \
(gpio_num_t)ESP32_HAL_UNDEFINED, \
(gpio_num_t)ESP32_HAL_UNDEFINED, \
false, \
false, \
true, \
E_LORA_POWER_LEVEL_MIN, \
1024 \
};

#define LORA_ESP32_PARAM_USING_SYSTEM_SPI {\
HSPI_HOST, \
(gpio_num_t)ESP32_HAL_UNDEFINED, \
(gpio_num_t)ESP32_HAL_UNDEFINED, \
(gpio_num_t)ESP32_HAL_UNDEFINED, \
(gpio_num_t)ESP32_HAL_UNDEFINED, \
(gpio_num_t)ESP32_HAL_UNDEFINED, \
true, \
false, \
true, \
E_LORA_POWER_LEVEL_MIN, \
1024 \
};

typedef enum {
    E_LORA_POWER_LEVEL_MIN = 0,
    E_LORA_POWER_LEVEL_0 = 0,
    E_LORA_POWER_LEVEL_1,
    E_LORA_POWER_LEVEL_2,
    E_LORA_POWER_LEVEL_3,
    E_LORA_POWER_LEVEL_4,
    E_LORA_POWER_LEVEL_5,
    E_LORA_POWER_LEVEL_6,
    E_LORA_POWER_LEVEL_7,
    E_LORA_POWER_LEVEL_8,
    E_LORA_POWER_LEVEL_9,
    E_LORA_POWER_LEVEL_10,
    E_LORA_POWER_LEVEL_11,
    E_LORA_POWER_LEVEL_12,
    E_LORA_POWER_LEVEL_13,
    E_LORA_POWER_LEVEL_14,
    E_LORA_POWER_LEVEL_MAX,
} lora_esp32_power_level;

typedef enum {
    E_LORA_BANDWIDTH_7_8_KHZ = 0,
    E_LORA_BANDWIDTH_10_4_KHZ,
    E_LORA_BANDWIDTH_15_6_KHZ,
    E_LORA_BANDWIDTH_20_8_KHZ,
    E_LORA_BANDWIDTH_31_25_KHZ,
    E_LORA_BANDWIDTH_41_7_KHZ,
    E_LORA_BANDWIDTH_62_5_KHZ,
    E_LORA_BANDWIDTH_125_KHZ,
    E_LORA_BANDWIDTH_250_KHZ,
    E_LORA_BANDWIDTH_MAX,
} lora_esp32_bandwidth;

typedef enum {
    E_LORA_SPREADING_FACTOR_MIN = 6,
    E_LORA_SPREADING_FACTOR_6 = 6,
    E_LORA_SPREADING_FACTOR_7,
    E_LORA_SPREADING_FACTOR_8,
    E_LORA_SPREADING_FACTOR_9,
    E_LORA_SPREADING_FACTOR_10,
    E_LORA_SPREADING_FACTOR_11,
    E_LORA_SPREADING_FACTOR_MAX,
} lora_esp32_spreading_factor;

typedef enum {
    E_LORA_CODING_RATE_MIN = 5,
    E_LORA_CODING_RATE_5 = 5,
    E_LORA_CODING_RATE_6,
    E_LORA_CODING_RATE_7,
    E_LORA_CODING_RATE_MAX,
} lora_esp32_coding_rate;

typedef struct {
    spi_host_device_t spi_host_device;
    gpio_num_t cs;
    gpio_num_t miso;
    gpio_num_t mosi;
    gpio_num_t sck;
    gpio_num_t rst;
    bool using_preconfigured_spi_bus;
    bool using_preconfigured_spi_device;
    bool crc_enabled;
    lora_esp32_power_level tx_power;
    size_t rx_buffer_size;
} lora_esp32_param_t;

esp_err_t lora_init(lora_esp32_param_t params);
void lora_reset();
esp_err_t lora_explicit_header_mode();
esp_err_t lora_implicit_header_mode(uint8_t size);
esp_err_t lora_idle();
esp_err_t lora_sleep();
esp_err_t lora_receive();
esp_err_t lora_set_tx_power(lora_esp32_power_level level, bool use_boost);
esp_err_t lora_set_frequency(long frequency);
esp_err_t lora_set_spreading_factor(lora_esp32_spreading_factor sf);
esp_err_t lora_set_bandwidth(lora_esp32_bandwidth sbw);
esp_err_t lora_set_coding_rate(lora_esp32_coding_rate denominator);
esp_err_t lora_set_preamble_length(long length);
esp_err_t lora_set_sync_word(int sw);
esp_err_t lora_enable_crc();
esp_err_t lora_disable_crc();
esp_err_t lora_send_packet(const void *buf, unsigned size);
esp_err_t lora_receive_packet(void *buf, unsigned size, unsigned *received);
esp_err_t lora_received(bool *has_data);
esp_err_t lora_packet_rssi(int *rssi);
esp_err_t lora_packet_snr(double *snr);
esp_err_t lora_close();
void lora_dump_registers();

#if defined(__cplusplus)
}
#endif

#endif // __LORA_H__
