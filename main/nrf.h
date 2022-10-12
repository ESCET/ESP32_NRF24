
#ifndef _NRF_H_
#define _NRF_H_
#include "driver/spi_master.h"
#define HOST_ID SPI3_HOST

static const uint32_t NRF_TRANSMTTER_MODE = BIT0;
static const uint32_t NRF_RECEIVER_MODE = BIT1;
static const uint32_t NRF_RECEIVER_CONFIRMATION_MODE = BIT2;
static const uint32_t NRF_CONFIG_LIGHT = BIT3;
static const uint32_t NRF_CONFIG_PANIC_BUTTON = BIT4;
static const uint32_t NRF_CONFIG_MASK = 0x07;
static const uint32_t PAYLOAD_LENGTH = 32;

// esps3
#define CONFIG_MISO_GPIO 13 // esp32s3 16 // 31 io19
#define CONFIG_MOSI_GPIO 11 // esp32s3 8 // 37 io23
#define CONFIG_SCLK_GPIO 12 // esp32s3 3 // 30 io18
#define CONFIG_CE_GPIO 9    // esp32s3 9 // 24 io2
#define CONFIG_CSN_GPIO 3

#define TX_MODE 1
#define RX_MODE 2

/* Memory Map */
#define CONFIG 0x00
#define EN_AA 0x01
#define EN_RXADDR 0x02
#define SETUP_AW 0x03
#define SETUP_RETR 0x04
#define RF_CH 0x05
#define RF_SETUP 0x06
#define STATUS 0x07
#define OBSERVE_TX 0x08
#define CD 0x09
#define RX_ADDR_P0 0x0A
#define RX_ADDR_P1 0x0B
#define RX_ADDR_P2 0x0C
#define RX_ADDR_P3 0x0D
#define RX_ADDR_P4 0x0E
#define RX_ADDR_P5 0x0F
#define TX_ADDR 0x10
#define RX_PW_P0 0x11
#define RX_PW_P1 0x12
#define RX_PW_P2 0x13
#define RX_PW_P3 0x14
#define RX_PW_P4 0x15
#define RX_PW_P5 0x16
#define FIFO_STATUS 0x17
#define DYNPD 0x1C
#define FEATURE 0x1D
#define STATUS_IRQ (uint8_t)0x70 // Mask for all IRQ bits in STATUS register

/* Instruction Mnemonics */
#define R_REGISTER 0x00
#define W_REGISTER 0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE 0x50
#define R_RX_PL_WID 0x60
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define REUSE_TX_PL 0xE3
#define NOP 0xFF

#define NRF_BIT_0 0x01 /**< The value of bit 0 */
#define NRF_BIT_1 0x02 /**< The value of bit 1 */
#define NRF_BIT_2 0x04 /**< The value of bit 2 */
#define NRF_BIT_3 0x08 /**< The value of bit 3 */
#define NRF_BIT_4 0x10 /**< The value of bit 4 */
#define NRF_BIT_5 0x20 /**< The value of bit 5 */
#define NRF_BIT_6 0x40 /**< The value of bit 6 */
#define NRF_BIT_7 0x80 /**< The value of bit 7 */

#define EN_CRC 3
#define CRCO 2
typedef enum
{
    NRF_CRC_OFF,      /**< CRC check disabled */
    NRF_CRC_8BIT = 2, /**< CRC check set to 8-bit */
    NRF_CRC_16BIT     /**< CRC check set to 16-bit */
} nrf_crc_mode_t;

uint8_t nrf24_read_reg(uint8_t Reg);
void nrf24_read_reg_multi(uint8_t Reg, uint8_t *data, int size);
void nrf24_set_crc_mode(nrf_crc_mode_t crc_mode);
void nrf24_init(void);
void nrf24_tx_mode(uint8_t *Address, uint8_t channel);
uint8_t nrf24_transmit(uint8_t *data);
void nrf24_rx_mode(uint8_t *Address, uint8_t channel);
void Nrf24_spi_init();
uint8_t is_data_available(int pipenum);
void nrf24_receive(uint8_t *data);
void nrf24_read_all(uint8_t *data);
void nrf24_clear_iqr(void);
void nrf24_write_reg_multi(uint8_t Reg, uint8_t *data, int size);
void nrf24_write_reg(uint8_t Reg, uint8_t Data);
void nrf24_Init_e_shockburst(void);
void nrf24_send_cmd(uint8_t cmd);
uint8_t nrf24_get_status();
uint8_t nrf_send_command_to_devices(uint8_t command, uint8_t device_id[5], uint8_t channel, uint8_t user_parameter, uint8_t host_address[5]);
 
uint8_t nrf_read_rx(uint8_t ch, uint8_t address[5]);
#endif