
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/event_groups.h>
#include "esp_system.h"
#include "esp_sleep.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "lora.h"

static spi_device_handle_t spi_device_handle;
static lora_esp32_param_t esp32_param;
static EventGroupHandle_t wait_event_group;
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
    E_LORA_REG_IRQ_MASK = 0x11,
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
    E_LORA_REG_DIO_MAPPING_2 = 0x41,
    E_LORA_REG_VERSION = 0x42,
} lora_esp32_register;

typedef enum {
    E_LORA_MODE_LONG_RANGE_MODE = 0x80,
    E_LORA_MODE_SLEEP           = 0x00,
    E_LORA_MODE_STDBY           = 0x01,
    E_LORA_MODE_TX              = 0x03,
    E_LORA_MODE_RX_CONTINUOUS   = 0x05,
    E_LORA_MODE_RX_SINGLE       = 0x06,
} lora_esp32_transceiver_mode;

typedef enum {
    E_LORA_IRQ_TX_DONE_MASK           = 0x08,
    E_LORA_IRQ_VALID_HEADER_MASK      = 0x10,
    E_LORA_IRQ_PAYLOAD_CRC_ERROR_MASK = 0x20,
    E_LORA_IRQ_RX_DONE_MASK           = 0x40,
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

volatile static struct isr_flags_t {
    bool crc_error;
    bool rx_finished;
    bool tx_finished;
} irq_flags;

/**
 * Write a value to a register.
 * @param reg Register index.
 * @param val Value to write.
 */
static
esp_err_t lora_write_reg(lora_esp32_register reg, uint8_t val) {
    uint8_t out[2] = { (uint8_t)(0x80 | reg), val };
    uint8_t in[2];

    spi_transaction_t t = {
        .flags = 0,
        .length = 2 * CHAR_BIT,
        .tx_buffer = out,
        .rx_buffer = in
    };

    gpio_set_level(esp32_param.cs, 0);
    esp_err_t res = spi_device_transmit(spi_device_handle, &t);
    gpio_set_level(esp32_param.cs, 1);
    return res;
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @param val[out] Value of the register.
 * @return return code.
 */
static
esp_err_t lora_read_reg(lora_esp32_register reg, uint8_t *val) {
    uint8_t out[2] = { reg, 0xff };
    uint8_t in[2];

    spi_transaction_t t = {
            .flags = 0,
            .length = 2 * CHAR_BIT,
            .tx_buffer = out,
            .rx_buffer = in
    };

    gpio_set_level(esp32_param.cs, 0);
    esp_err_t res = spi_device_transmit(spi_device_handle, &t);
    gpio_set_level(esp32_param.cs, 1);
    *val = in[1];
    return res;
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    (void)arg;
    ESP_ERROR_CHECK(gpio_intr_disable(esp32_param.intr));

    BaseType_t xHigherPriorityTaskWoken, xResult;
    xHigherPriorityTaskWoken = pdFALSE;
    xResult = xEventGroupSetBitsFromISR(wait_event_group, BIT0, &xHigherPriorityTaskWoken);
    if (xResult == pdPASS) {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t lora_set_dio(bool is_rx)
{
    uint8_t reg1 = is_rx ? 0 : 0x55; // b01010101
    uint8_t reg2 = is_rx ? 0 : 0x50; // b01010000
    esp_err_t res = lora_write_reg(E_LORA_REG_DIO_MAPPING_1, reg1);
    return res ?: lora_write_reg(E_LORA_REG_DIO_MAPPING_2, reg2);
}

static esp_err_t lora_fetch_irq_status()
{
    uint8_t irq;
    esp_err_t res = lora_read_reg(E_LORA_REG_IRQ_FLAGS, &irq);
    res = res ?: lora_write_reg(E_LORA_REG_IRQ_FLAGS, irq);

    irq_flags.crc_error = !! (irq & E_LORA_IRQ_PAYLOAD_CRC_ERROR_MASK);
    irq_flags.rx_finished = !! (irq & E_LORA_IRQ_RX_DONE_MASK);
    irq_flags.tx_finished = !! (irq & E_LORA_IRQ_TX_DONE_MASK);
    return res;
}

static esp_err_t lora_wait_operation(bool is_read, uint64_t timeout_us)
{
    esp_err_t res = ESP_OK;
    EventBits_t bits;
    esp_sleep_wakeup_cause_t cause;
    volatile bool *txrx_finish = is_read ? &irq_flags.rx_finished : &irq_flags.tx_finished;

    irq_flags.crc_error = false;
    irq_flags.rx_finished = false;
    irq_flags.tx_finished = false;
    res = lora_set_dio(is_read);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set DIO mapping:%d", res);
        return res;
    }

    if (timeout_us == LORA_TIMEOUT_NOWAIT) {
        return lora_fetch_irq_status();
    }

    if (esp32_param.wait == E_LORA_WAIT_IDLE) {
        xEventGroupClearBits(wait_event_group, BIT0);
        ESP_ERROR_CHECK(gpio_intr_enable(esp32_param.intr));
    }

    while (!(*txrx_finish) && res == ESP_OK) {
        if (is_read) {
            res = res ?: lora_write_reg(E_LORA_REG_FIFO_ADDR_PTR, 0);
        }

        switch (esp32_param.wait) {
        case E_LORA_WAIT_LIGHT_SLEEP:
            if (timeout_us != LORA_TIMEOUT_INFINITY) {
                ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(timeout_us));
            }

            ESP_ERROR_CHECK(esp_light_sleep_start());
            cause = esp_sleep_get_wakeup_cause();
            if (cause == ESP_SLEEP_WAKEUP_TIMER) {
                res = ESP_ERR_TIMEOUT;
                goto out;
            }
            if (cause != ESP_SLEEP_WAKEUP_GPIO) {
                ESP_LOGE(TAG, "Invalid wake up reason:%d", cause);
                res = ESP_FAIL;
                goto out;
            }
            break;
        case E_LORA_WAIT_IDLE:
            bits = xEventGroupWaitBits(wait_event_group,
                BIT0,
                true,  // clear on exit
                false, // wait all
                (timeout_us == LORA_TIMEOUT_INFINITY ? (TickType_t) - 1 : pdMS_TO_TICKS(timeout_us / 1000)));
            if (bits == 0) {
                res = ESP_ERR_TIMEOUT;
                goto out;
            }
            break;
        case E_LORA_WAIT_POLL:
            vTaskDelay(pdMS_TO_TICKS(10));
            if (timeout_us != LORA_TIMEOUT_INFINITY && timeout_us >= 10000) {
                timeout_us -= 10000;
            } else if (timeout_us < 10000) {
                res = ESP_ERR_TIMEOUT;
                goto out;
            }
        }

        res = lora_fetch_irq_status();
    }

out:
    if (esp32_param.wait == E_LORA_WAIT_IDLE) {
        ESP_ERROR_CHECK(gpio_intr_disable(esp32_param.intr));
    }

    return res;
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
esp_err_t lora_explicit_header_mode(void) {
    __implicit = 0;
    uint8_t val;
    esp_err_t res = lora_read_reg(E_LORA_REG_MODEM_CONFIG_1, &val);
    return res ? res : lora_write_reg(E_LORA_REG_MODEM_CONFIG_1, val & (uint8_t)(0xfe));
}

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
esp_err_t lora_implicit_header_mode(uint8_t size) {
    __implicit = 1;
    uint8_t val;
    esp_err_t res = lora_read_reg(E_LORA_REG_MODEM_CONFIG_1, &val);
    res = res ?: lora_write_reg(E_LORA_REG_MODEM_CONFIG_1, val | (uint8_t)(0x01));
    return res ? res : lora_write_reg(E_LORA_REG_PAYLOAD_LENGTH, (uint8_t)(size));
}

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
esp_err_t lora_idle(void) {
    return lora_write_reg(E_LORA_REG_OP_MODE, E_LORA_MODE_LONG_RANGE_MODE | E_LORA_MODE_STDBY);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
esp_err_t lora_sleep(void) {
    return lora_write_reg(E_LORA_REG_OP_MODE, E_LORA_MODE_LONG_RANGE_MODE | E_LORA_MODE_SLEEP);
}

/**
 * Configure power level for transmission
 * @param level 0-15, from least to most power
 */
esp_err_t lora_set_tx_power(lora_esp32_power_level level, bool use_boost) {
    // RF9x module uses PA_BOOST pin
    const uint8_t boost = use_boost ? PA_BOOST : 0;
    return lora_write_reg(E_LORA_REG_PA_CONFIG, (uint8_t)(boost | level));
}

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
esp_err_t lora_set_frequency(long frequency) {
    __frequency = frequency;

    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
    esp_err_t res;
    res = lora_write_reg(E_LORA_REG_FRF_MSB, (uint8_t)(frf >> 16));
    res = res ?: lora_write_reg(E_LORA_REG_FRF_MID, (uint8_t)(frf >> 8));
    res = res ?: lora_write_reg(E_LORA_REG_FRF_LSB, (uint8_t)(frf >> 0));

    return res;
}

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
esp_err_t lora_set_spreading_factor(lora_esp32_spreading_factor sf) {
    if (sf < 6) sf = 6;
    else if (sf > 12) sf = 12;
    esp_err_t res;

    if (sf == E_LORA_SPREADING_FACTOR_MIN) {
        res = lora_write_reg(E_LORA_REG_DETECTION_OPTIMIZE, 0xc5);
        res = res ?: lora_write_reg(E_LORA_REG_DETECTION_THRESHOLD, 0x0c);
    } else {
        res = lora_write_reg(E_LORA_REG_DETECTION_OPTIMIZE, 0xc3);
        res = res ?: lora_write_reg(E_LORA_REG_DETECTION_THRESHOLD, 0x0a);
    }

    uint8_t val;
    res = res ?: lora_read_reg(E_LORA_REG_MODEM_CONFIG_2, &val);
    return res ? res : lora_write_reg(E_LORA_REG_MODEM_CONFIG_2,
                                     (val & (uint8_t)0x0f) | (uint8_t)((sf << 4) & 0xf0));
}

/**
 * Set bandwidth (bit rate)   * @param sbw Bandwidth in Hz (up to 500000)   */
esp_err_t lora_set_bandwidth(lora_esp32_bandwidth sbw) {
    uint8_t val;
    esp_err_t res = lora_read_reg(E_LORA_REG_MODEM_CONFIG_1, &val);
    return res ? res : lora_write_reg(E_LORA_REG_MODEM_CONFIG_1, (val & 0x0f) | (sbw << 4));
}

/**
 * Set coding rate
 * @param denominator 5-8, Denominator for the coding rate 4/x
 */
esp_err_t lora_set_coding_rate(lora_esp32_coding_rate denominator) {
    int cr = denominator - 4;
    uint8_t val;
    esp_err_t res = lora_read_reg(E_LORA_REG_MODEM_CONFIG_1, &val);
    return res ? res : lora_write_reg(E_LORA_REG_MODEM_CONFIG_1, (val & 0xf1) | (cr << 1));
}

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
esp_err_t lora_set_preamble_length(long length) {
    esp_err_t res = lora_write_reg(E_LORA_REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
    return res ? res : lora_write_reg(E_LORA_REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
esp_err_t lora_set_sync_word(int sw) {
    return lora_write_reg(E_LORA_REG_SYNC_WORD, sw);
}

/**
 * Enable appending/verifying packet CRC.
 */
esp_err_t lora_enable_crc(void) {
    uint8_t val;
    esp_err_t res = lora_read_reg(E_LORA_REG_MODEM_CONFIG_2, &val);
    return res ? res : lora_write_reg(E_LORA_REG_MODEM_CONFIG_2, val | 0x04);
}

/**
 * Disable appending/verifying packet CRC.
 */
esp_err_t lora_disable_crc(void) {
    uint8_t val;
    esp_err_t res = lora_read_reg(E_LORA_REG_MODEM_CONFIG_2, &val);
    return res ? res : lora_write_reg(E_LORA_REG_MODEM_CONFIG_2, val & 0xfb);
}

/**
 * Perform hardware initialization.
 */
esp_err_t lora_init(lora_esp32_param_t params) {
    esp32_param = params;
    esp_err_t ret = ESP_OK;

    if(esp32_param.cs == ESP32_HAL_UNDEFINED) {
        ESP_LOGE(TAG, "CS PIN is NOT configured");
        return ESP_ERR_INVALID_ARG;
    }

    if(esp32_param.rst == ESP32_HAL_UNDEFINED) {
        ESP_LOGE(TAG, "RESET pin is not configured. reset function is disabled");
        return ESP_ERR_INVALID_ARG;
    }

    if (esp32_param.intr == ESP32_HAL_UNDEFINED && esp32_param.wait != E_LORA_WAIT_POLL) {
        ESP_LOGE(TAG, "Interrupt pin configured incorrectly");
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
    ESP_ERROR_CHECK(gpio_set_direction(esp32_param.rst, GPIO_MODE_OUTPUT));
    gpio_pad_select_gpio(esp32_param.cs);
    ESP_ERROR_CHECK(gpio_set_direction(esp32_param.cs, GPIO_MODE_OUTPUT));

    gpio_config_t io_conf;
    io_conf.intr_type = (gpio_int_type_t)GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = 1 << esp32_param.intr;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Configure interrupt handling.
    if (esp32_param.intr != ESP32_HAL_UNDEFINED) {
        switch (esp32_param.wait) {
        case E_LORA_WAIT_LIGHT_SLEEP:
            ESP_ERROR_CHECK(gpio_wakeup_enable(esp32_param.intr, GPIO_INTR_HIGH_LEVEL));
            ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
            break;
        case E_LORA_WAIT_IDLE:
            wait_event_group = xEventGroupCreate();
            ESP_ERROR_CHECK(gpio_intr_disable(esp32_param.intr));
            ESP_ERROR_CHECK(gpio_set_intr_type(esp32_param.intr, GPIO_INTR_HIGH_LEVEL));
            ESP_ERROR_CHECK(gpio_isr_handler_add(esp32_param.intr, gpio_isr_handler, 0));
            break;
        case E_LORA_WAIT_POLL:
            break;
        }

    }

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
        ret = lora_read_reg(E_LORA_REG_VERSION, &version);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "FAILED TO READ VERSION: %d", ret);
            break;
        }
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
    ret = ret ?: lora_sleep();
    ret = ret ?: lora_write_reg(E_LORA_REG_FIFO_RX_BASE_ADDR, 0);
    ret = ret ?: lora_write_reg(E_LORA_REG_FIFO_TX_BASE_ADDR, 0);
    uint8_t val;
    ret = ret ?: lora_read_reg(E_LORA_REG_LNA, &val);
    ret = ret ?: lora_write_reg(E_LORA_REG_LNA, val | (uint8_t)0x03);
    ret = ret ?: lora_write_reg(E_LORA_REG_MODEM_CONFIG_3, 0x04);
    ret = ret ?: lora_set_tx_power(params.tx_power, false);
    ret = ret ?: esp32_param.crc_enabled ? lora_enable_crc() : lora_disable_crc();
    ret = ret ?: lora_write_reg(E_LORA_REG_IRQ_MASK, E_LORA_IRQ_VALID_HEADER_MASK);
    ret = ret ?: lora_idle();

    return ret;
}

/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 */
esp_err_t lora_send_packet(const void *buf, unsigned size, uint64_t timeout_us) {
    const uint8_t *buf_ptr = buf;
    esp_err_t res = lora_idle();

    res = res ?: lora_write_reg(E_LORA_REG_FIFO_ADDR_PTR, 0);
    res = res ?: lora_write_reg(E_LORA_REG_PAYLOAD_LENGTH, 0);

    for(int i=0; res == ESP_OK && i < size; i++)
        res = lora_write_reg(E_LORA_REG_FIFO, *buf_ptr++);

    res = res ?: lora_write_reg(E_LORA_REG_PAYLOAD_LENGTH, size);
    res = res ?: lora_write_reg(E_LORA_REG_OP_MODE, E_LORA_MODE_LONG_RANGE_MODE | E_LORA_MODE_TX);
    res = res ?: lora_wait_operation(/*is_read*/ false, timeout_us);
    return res;
}

/**
 * Read a received packet.
 * @param buf Buffer for the data.
 * @param size Available size in buffer (bytes).
 * @param received[out] Number of bytes received (zero if no packet available).
 * @return error code.
 */
esp_err_t lora_receive_packet(void *buf, unsigned size, unsigned *received, uint64_t timeout_us) {
    esp_err_t res;
    uint8_t *buf_ptr = buf;
    *received = 0;

    res = lora_idle();
    res = res ?: lora_write_reg(E_LORA_REG_OP_MODE, E_LORA_MODE_LONG_RANGE_MODE | E_LORA_MODE_RX_CONTINUOUS);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set receive mode: 0x%x", res);
        return res;
    }

    do {
        res = lora_wait_operation(/*is_read*/ true, timeout_us);
        if (res == ESP_ERR_TIMEOUT || !irq_flags.rx_finished) {
            return res;
        }
    } while (irq_flags.crc_error);

    lora_esp32_register reg = __implicit ? E_LORA_REG_PAYLOAD_LENGTH : E_LORA_REG_RX_NB_BYTES;
    uint8_t llen;
    res = res ?: lora_read_reg(reg, &llen);
    *received = (llen > size ? size : llen);

    res = res ?: lora_idle();
    uint8_t val;
    res = res ?: lora_read_reg(E_LORA_REG_FIFO_RX_CURRENT_ADDR, &val);
    res = res ?: lora_write_reg(E_LORA_REG_FIFO_ADDR_PTR, val);

    for(int i = 0; res == ESP_OK && i < *received; i++)
        res = lora_read_reg(E_LORA_REG_FIFO, buf_ptr++);

    return res;
}

/**
 * Return last packet's RSSI.
 */
esp_err_t lora_packet_rssi(int *rssi) {
    uint8_t val = 0;
    esp_err_t res = lora_read_reg(E_LORA_REG_PKT_RSSI_VALUE, &val);
    *rssi =  (val - (__frequency < 868E6 ? 164 : 157));
    return res;
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
esp_err_t lora_packet_snr(double *snr) {
    uint8_t val;
    esp_err_t res = lora_read_reg(E_LORA_REG_PKT_SNR_VALUE, &val);
    *snr = (double)(val) * 0.25;
    return res;
}

/**
 * Shutdown hardware.
 */
esp_err_t lora_close(void) {
    esp_err_t res = lora_sleep();
    if (esp32_param.wait == E_LORA_WAIT_IDLE) {
        ESP_ERROR_CHECK(gpio_set_intr_type(esp32_param.intr, GPIO_INTR_DISABLE));
        ESP_ERROR_CHECK(gpio_isr_handler_remove(esp32_param.intr));
        ESP_ERROR_CHECK(gpio_intr_disable(esp32_param.intr));
        vEventGroupDelete(wait_event_group);
    }

    return res;
//   close(__spi);  FIXME: end hardware features after lora_close
//   close(__cs);
//   close(__rst);
}

void lora_dump_registers(void) {
    int i;
    printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
    for(i=0; i<0x40; i++) {
        uint8_t reg;
        lora_read_reg(i, &reg);
        printf("%02X ", reg);
        if((i & 0x0f) == 0x0f) printf("\n");
    }
    printf("\n");
}
