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
ESP32_HAL_UNDEFINED, \
ESP32_HAL_UNDEFINED, \
ESP32_HAL_UNDEFINED, \
ESP32_HAL_UNDEFINED, \
false, \
true, \
E_LORA_POWER_LEVEL_MIN, \
1024 \
};

#define LORA_ESP32_PARAM_USING_SYSTEM_SPI {\
HSPI_HOST, \
ESP32_HAL_UNDEFINED, \
ESP32_HAL_UNDEFINED, \
ESP32_HAL_UNDEFINED, \
ESP32_HAL_UNDEFINED, \
false, \
true, \
E_LORA_POWER_LEVEL_MIN, \
1024 \
};

typedef enum {
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
    spi_device_handle_t spi_device_handle;
    gpio_num_t cs;
    gpio_num_t miso;
    gpio_num_t mosi;
    gpio_num_t sck;
    gpio_num_t rst;
    int spi_dma_chan;
    bool using_preconfigured_spi_bus;
    bool using_preconfigured_spi_device;
    bool crc_enabled;
    lora_esp32_power_level tx_power;
    size_t rx_buffer_size;
} lora_esp32_param_t;

esp_err_t lora_init(lora_esp32_param_t params);
void lora_reset();
void lora_explicit_header_mode();
void lora_implicit_header_mode(uint8_t size);
void lora_idle();
void lora_sleep();
void lora_receive();
void lora_set_tx_power(lora_esp32_power_level level);
void lora_set_frequency(long frequency);
void lora_set_spreading_factor(lora_esp32_spreading_factor sf);
void lora_set_bandwidth(lora_esp32_bandwidth sbw);
void lora_set_coding_rate(lora_esp32_coding_rate denominator);
void lora_set_preamble_length(long length);
void lora_set_sync_word(int sw);
void lora_enable_crc();
void lora_disable_crc();
void lora_send_packet(uint8_t *buf, int size);
int lora_receive_packet(uint8_t *buf, int size);
int lora_received();
int lora_packet_rssi();
double lora_packet_snr();
void lora_close();
int lora_initialized();
void lora_dump_registers();

#if defined(__cplusplus)
}
#endif

#endif // __LORA_H__
