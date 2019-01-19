
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "lora.h"


static spi_device_handle_t spi_device_handle;
static lora_esp32_param_t esp32_param;
static const char* TAG = "lora-esp32";

typedef enum {
    E_LORA_REG_FIFO = 0x00,
    E_LORA_REG_OP_MODE = 0x01,
    E_LORA_REG_FRF_MSB = 0x06,
    E_LORA_REG_FRF_MID = 0x07,
    E_LORA_REG_FRF_LSB = 0x08,
    E_LORA_REG_PA_CONFIG = 0x09,
    E_LORA_REG_LNA = 0x0c,
    E_LORA_REG_FIFO_ADDR_PTR = 0x0d,
    E_LORA_REG_FIFO_TX_BASE_ADDR = 0x0e,
    E_LORA_REG_FIFO_RX_BASE_ADDR = 0x0f,
    E_LORA_REG_FIFO_RX_CURRENT_ADDR = 0x10,
    E_LORA_REG_IRQ_FLAGS = 0x12,
    E_LORA_REG_RX_NB_BYTES = 0x13,
    E_LORA_REG_PKT_SNR_VALUE = 0x19,
    E_LORA_REG_PKT_RSSI_VALUE = 0x1a,
    E_LORA_REG_MODEM_CONFIG_1 = 0x1d,
    E_LORA_REG_MODEM_CONFIG_2 = 0x1e,
    E_LORA_REG_PREAMBLE_MSB = 0x20,
    E_LORA_REG_PREAMBLE_LSB = 0x21,
    E_LORA_REG_PAYLOAD_LENGTH = 0x22,
    E_LORA_REG_MODEM_CONFIG_3 = 0x26,
    E_LORA_REG_RSSI_WIDEBAND = 0x2c,
    E_LORA_REG_DETECTION_OPTIMIZE = 0x31,
    E_LORA_REG_DETECTION_THRESHOLD = 0x37,
    E_LORA_REG_SYNC_WORD = 0x39,
    E_LORA_REG_DIO_MAPPING_1 = 0x40,
    E_LORA_REG_VERSION = 0x42,
} lora_esp32_register;

typedef enum {
    E_LORA_MODE_LONG_RANGE_MODE = 0x80,
    E_LORA_MODE_SLEEP = 0x00,
    E_LORA_MODE_STDBY = 0x01,
    E_LORA_MODE_TX = 0x03,
    E_LORA_MODE_RX_CONTINUOUS = 0x05,
    E_LORA_MODE_RX_SINGLE = 0x06,
} lora_esp32_transceiver_mode;

typedef enum {
E_LORA_IRQ_TX_DONE_MASK = 0x08,
E_LORA_IRQ_PAYLOAD_CRC_ERROR_MASK = 0x20,
E_LORA_IRQ_RX_DONE_MASK = 0x40,
} lora_esp32_irq_mask;

/*
 * PA configuration
 */
#define PA_BOOST                       0x80


#define PA_OUTPUT_RFO_PIN              0
#define PA_OUTPUT_PA_BOOST_PIN         1

#define TIMEOUT_RESET                  100

static int __implicit;
static long __frequency;

/**
 * Write a value to a register.
 * @param reg Register index.
 * @param val Value to write.
 */
void lora_write_reg(lora_esp32_register reg, uint8_t val) {
    uint8_t out[2] = { (uint8_t)(0x80 | reg), val };
    uint8_t in[2];

    spi_transaction_t t = {
            .flags = 0,
            .length = 16,
            .tx_buffer = out,
            .rx_buffer = in
    };

    gpio_set_level(esp32_param.cs, 0);
    spi_device_transmit(spi_device_handle, &t);
    gpio_set_level(esp32_param.cs, 1);
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 */
uint8_t lora_read_reg(lora_esp32_register reg) {
    uint8_t out[2] = { reg, 0xff };
    uint8_t in[2];

    spi_transaction_t t = {
            .flags = 0,
            .length = 16,
            .tx_buffer = out,
            .rx_buffer = in
    };

    gpio_set_level(esp32_param.cs, 0);
    spi_device_transmit(spi_device_handle, &t);
    gpio_set_level(esp32_param.cs, 1);
    return in[1];
}

/**
 * Perform physical reset on the Lora chip
 */
void lora_reset(void) {
    if(esp32_param.rst != ESP32_HAL_UNDEFINED) {
        gpio_set_level(esp32_param.rst, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(esp32_param.rst, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
void lora_explicit_header_mode(void) {
    __implicit = 0;
    lora_write_reg(E_LORA_REG_MODEM_CONFIG_1, lora_read_reg(E_LORA_REG_MODEM_CONFIG_1) & (uint8_t)(0xfe));
}

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
void lora_implicit_header_mode(uint8_t size) {
    __implicit = 1;
    lora_write_reg(E_LORA_REG_MODEM_CONFIG_1, lora_read_reg(E_LORA_REG_MODEM_CONFIG_1) | (uint8_t)(0x01));
    lora_write_reg(E_LORA_REG_PAYLOAD_LENGTH, (uint8_t)(size));
}

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
void lora_idle(void) {
    lora_write_reg(E_LORA_REG_OP_MODE, E_LORA_MODE_LONG_RANGE_MODE | E_LORA_MODE_STDBY);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
void lora_sleep(void) {
    lora_write_reg(E_LORA_REG_OP_MODE, E_LORA_MODE_LONG_RANGE_MODE | E_LORA_MODE_SLEEP);
}

/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
void lora_receive(void) {
    lora_write_reg(E_LORA_REG_OP_MODE, E_LORA_MODE_LONG_RANGE_MODE | E_LORA_MODE_RX_CONTINUOUS);
}

/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
void lora_set_tx_power(lora_esp32_power_level level) {
    // RF9x module uses PA_BOOST pin
    lora_write_reg(E_LORA_REG_PA_CONFIG, (uint8_t)(PA_BOOST | level));
}

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
void lora_set_frequency(long frequency) {
    __frequency = frequency;

    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

    lora_write_reg(E_LORA_REG_FRF_MSB, (uint8_t)(frf >> 16));
    lora_write_reg(E_LORA_REG_FRF_MID, (uint8_t)(frf >> 8));
    lora_write_reg(E_LORA_REG_FRF_LSB, (uint8_t)(frf >> 0));
}

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
void lora_set_spreading_factor(lora_esp32_spreading_factor sf) {
    if (sf < 6) sf = 6;
    else if (sf > 12) sf = 12;

    if (sf == E_LORA_SPREADING_FACTOR_MIN) {
        lora_write_reg(E_LORA_REG_DETECTION_OPTIMIZE, 0xc5);
        lora_write_reg(E_LORA_REG_DETECTION_THRESHOLD, 0x0c);
    } else {
        lora_write_reg(E_LORA_REG_DETECTION_OPTIMIZE, 0xc3);
        lora_write_reg(E_LORA_REG_DETECTION_THRESHOLD, 0x0a);
    }

    lora_write_reg(E_LORA_REG_MODEM_CONFIG_2, (lora_read_reg(E_LORA_REG_MODEM_CONFIG_2) & (uint8_t)0x0f) | (uint8_t)((sf << 4) & 0xf0));
}

/**
 * Set bandwidth (bit rate)   * @param sbw Bandwidth in Hz (up to 500000)   */
void lora_set_bandwidth(lora_esp32_bandwidth sbw) {
    lora_write_reg(E_LORA_REG_MODEM_CONFIG_1, (lora_read_reg(E_LORA_REG_MODEM_CONFIG_1) & 0x0f) | (sbw << 4));
}

/**
 * Set coding rate
 * @param denominator 5-8, Denominator for the coding rate 4/x
 */
void lora_set_coding_rate(lora_esp32_coding_rate denominator) {
    int cr = denominator - 4;
    lora_write_reg(E_LORA_REG_MODEM_CONFIG_1, (lora_read_reg(E_LORA_REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
void lora_set_preamble_length(long length) {
    lora_write_reg(E_LORA_REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
    lora_write_reg(E_LORA_REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
void lora_set_sync_word(int sw) {
    lora_write_reg(E_LORA_REG_SYNC_WORD, sw);
}

/**
 * Enable appending/verifying packet CRC.
 */
void lora_enable_crc(void) {
    lora_write_reg(E_LORA_REG_MODEM_CONFIG_2, lora_read_reg(E_LORA_REG_MODEM_CONFIG_2) | 0x04);
}

/**
 * Disable appending/verifying packet CRC.
 */
void lora_disable_crc(void) {
    lora_write_reg(E_LORA_REG_MODEM_CONFIG_2, lora_read_reg(E_LORA_REG_MODEM_CONFIG_2) & 0xfb);
}

/**
 * Perform hardware initialization.
 */
esp_err_t lora_init(lora_esp32_param_t params) {
    esp32_param = params;
    esp_err_t ret;

    if(esp32_param.cs == ESP32_HAL_UNDEFINED) {
        ESP_LOGE(TAG, "CS PIN is NOT configured");
        return ESP_ERR_INVALID_ARG;
    }

    if(esp32_param.rst == ESP32_HAL_UNDEFINED) {
        ESP_LOGW(TAG, "RESET pin is not configured. reset function is disabled");
        return ESP_ERR_INVALID_ARG;
    }

    if(!esp32_param.using_preconfigured_spi_bus) {
        if (esp32_param.sck == ESP32_HAL_UNDEFINED ||
            esp32_param.mosi == ESP32_HAL_UNDEFINED ||
            esp32_param.miso == ESP32_HAL_UNDEFINED) {
            ESP_LOGE(TAG, "SPI PIN is NOT configured");
            return ESP_ERR_INVALID_ARG;
        }
        spi_bus_config_t bus = {
                .miso_io_num = esp32_param.miso,
                .mosi_io_num = esp32_param.mosi,
                .sclk_io_num = esp32_param.sck,
                .quadwp_io_num = -1,
                .quadhd_io_num = -1,
                .max_transfer_sz = 0
        };

        if ((ret = spi_bus_initialize(esp32_param.spi_host_device, &bus, 0)) != ESP_OK) {
            ESP_LOGE(TAG, "spi_bus_initialize failed: %s(0x%04X)", esp_err_to_name(ret), ret);
            return ret;
        }
    }
    if(!esp32_param.using_preconfigured_spi_device) {
        spi_device_interface_config_t dev = {
                .clock_speed_hz = 9000000,
                .mode = 0,
                .spics_io_num = -1,
                .queue_size = 1,
                .flags = 0,
                .pre_cb = NULL
        };
        if ((ret = spi_bus_add_device(esp32_param.spi_host_device, &dev, &spi_device_handle)) != ESP_OK) {
            ESP_LOGE(TAG, "spi_bus_add_device failed: %s(0x%04X)", esp_err_to_name(ret), ret);
            return ret;
        }
    }

    /*
     * Configure CPU hardware to communicate with the radio chip
     */
    gpio_pad_select_gpio(esp32_param.rst);
    gpio_set_direction(esp32_param.rst, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(esp32_param.cs);
    gpio_set_direction(esp32_param.cs, GPIO_MODE_OUTPUT);

    /*
     * Perform hardware reset.
     */
    lora_reset();

    /*
     * Check version.
     */
    uint8_t version;
    uint8_t i = 0;
    while(i++ < TIMEOUT_RESET) {
        version = lora_read_reg(E_LORA_REG_VERSION);
        if(version == 0x12) break;
        vTaskDelay(2);
    }

    if(i >= TIMEOUT_RESET) {
        ESP_LOGE(TAG, "RESET TIME OUT.");
        return LORA_ESP32_ERR_RESET_TIMEOUT;
    }

    /*
     * Default configuration.
     */
    lora_sleep();
    lora_write_reg(E_LORA_REG_FIFO_RX_BASE_ADDR, 0);
    lora_write_reg(E_LORA_REG_FIFO_TX_BASE_ADDR, 0);
    lora_write_reg(E_LORA_REG_LNA, lora_read_reg(E_LORA_REG_LNA) | (uint8_t)0x03);
    lora_write_reg(E_LORA_REG_MODEM_CONFIG_3, 0x04);
    lora_set_tx_power(17);

    lora_idle();
    return ESP_OK;
}

/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 */
void lora_send_packet(uint8_t *buf, int size) {
    /*
     * Transfer data to radio.
     */
    lora_idle();
    lora_write_reg(E_LORA_REG_FIFO_ADDR_PTR, 0);

    for(int i=0; i<size; i++)          lora_write_reg(E_LORA_REG_FIFO, *buf++);

    lora_write_reg(E_LORA_REG_PAYLOAD_LENGTH, size);

    /*
     * Start transmission and wait for conclusion.
     */
    lora_write_reg(E_LORA_REG_OP_MODE, E_LORA_MODE_LONG_RANGE_MODE | E_LORA_MODE_TX);
    while((lora_read_reg(E_LORA_REG_IRQ_FLAGS) & E_LORA_IRQ_TX_DONE_MASK) == 0)          vTaskDelay(2);

    lora_write_reg(E_LORA_REG_IRQ_FLAGS, E_LORA_IRQ_TX_DONE_MASK);
}

/**
 * Read a received packet.
 * @param buf Buffer for the data.
 * @param size Available size in buffer (bytes).
 * @return Number of bytes received (zero if no packet available).
 */
int lora_receive_packet(uint8_t *buf, int size) {
    int len = 0;

    /*
     * Check interrupts.
     */
    int irq = lora_read_reg(E_LORA_REG_IRQ_FLAGS);
    lora_write_reg(E_LORA_REG_IRQ_FLAGS, irq);
    if((irq & E_LORA_IRQ_RX_DONE_MASK) == 0) return 0;
    if(irq & E_LORA_IRQ_PAYLOAD_CRC_ERROR_MASK) return 0;

    /*
     * Find packet size.
     */
    if (__implicit) len = lora_read_reg(E_LORA_REG_PAYLOAD_LENGTH);
    else len = lora_read_reg(E_LORA_REG_RX_NB_BYTES);

    /*
     * Transfer data from radio.
     */
    lora_idle();
    lora_write_reg(E_LORA_REG_FIFO_ADDR_PTR, lora_read_reg(E_LORA_REG_FIFO_RX_CURRENT_ADDR));
    if(len > size) len = size;
    for(int i=0; i<len; i++)          *buf++ = lora_read_reg(E_LORA_REG_FIFO);

    return len;
}

/**
 * Returns non-zero if there is data to read (packet received).
 */
int lora_received(void) {
    if(lora_read_reg(E_LORA_REG_IRQ_FLAGS) & E_LORA_IRQ_RX_DONE_MASK) return 1;
    return 0;
}

/**
 * Return last packet's RSSI.
 */
int lora_packet_rssi(void) {
    return (lora_read_reg(E_LORA_REG_PKT_RSSI_VALUE) - (__frequency < 868E6 ? 164 : 157));
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
double lora_packet_snr(void) {
    return (double)(lora_read_reg(E_LORA_REG_PKT_SNR_VALUE)) * 0.25;
}

/**
 * Shutdown hardware.
 */
void lora_close(void) {
    lora_sleep();
//   close(__spi);  FIXME: end hardware features after lora_close
//   close(__cs);
//   close(__rst);
//   __spi = -1;
//   __cs = -1;
//   __rst = -1;
}

void lora_dump_registers(void) {
    int i;
    printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
    for(i=0; i<0x40; i++) {
        printf("%02X ", lora_read_reg(i));
        if((i & 0x0f) == 0x0f) printf("\n");
    }
    printf("\n");
}

