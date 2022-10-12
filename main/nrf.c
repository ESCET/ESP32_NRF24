#include "nrf.h"
#include "esp_log.h"
#include <driver/gpio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <time.h>

#define TAG "NRF .. "
#define COMMAND_SET_CHANNEL BIT0

static const int SPI_Frequency = 4000000; // Stable even with a long jumper cable

spi_device_handle_t SPIHandle;
uint8_t PTX;
uint8_t mode;
uint8_t channel;
bool spi_init = false;

bool is_light_TX_address = true;

void CS_Select(void)
{
    gpio_set_level(CONFIG_CSN_GPIO, 0);
}

void CS_UnSelect(void)
{
    gpio_set_level(CONFIG_CSN_GPIO, 1);
}

void CE_Enable(void)
{
    gpio_set_level(CONFIG_CE_GPIO, 1);
}

void CE_Disable(void)
{
    gpio_set_level(CONFIG_CE_GPIO, 0);
}

void nrf24_reset(uint8_t REG)
{
    if (REG == STATUS)
    {
        nrf24_write_reg(STATUS, 0x00);
    }

    else if (REG == FIFO_STATUS)
    {
        nrf24_write_reg(FIFO_STATUS, 0x11);
    }

    else
    {
        nrf24_write_reg(CONFIG, 0x08);
        nrf24_write_reg(EN_AA, 0x3F);
        nrf24_write_reg(EN_RXADDR, 0x0F);
        nrf24_write_reg(SETUP_AW, 0x03);
        nrf24_write_reg(SETUP_RETR, 0x03);
        nrf24_write_reg(RF_CH, 0x02);
        nrf24_write_reg(RF_SETUP, 0x0E);
        nrf24_write_reg(STATUS, 0x00);
        nrf24_write_reg(OBSERVE_TX, 0x00);
        nrf24_write_reg(CD, 0x00);
        uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
        nrf24_write_reg_multi(RX_ADDR_P0, rx_addr_p0_def, 5);
        uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
        nrf24_write_reg_multi(RX_ADDR_P1, rx_addr_p1_def, 5);
        nrf24_write_reg(RX_ADDR_P2, 0xC3);
        nrf24_write_reg(RX_ADDR_P3, 0xC4);
        nrf24_write_reg(RX_ADDR_P4, 0xC5);
        nrf24_write_reg(RX_ADDR_P5, 0xC6);
        uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
        nrf24_write_reg_multi(TX_ADDR, tx_addr_def, 5);
        nrf24_write_reg(RX_PW_P0, 0x20);
        nrf24_write_reg(RX_PW_P1, 0x20);
        nrf24_write_reg(RX_PW_P2, 0x20);
        nrf24_write_reg(RX_PW_P3, 0);
        nrf24_write_reg(RX_PW_P4, 0);
        nrf24_write_reg(RX_PW_P5, 0);
        nrf24_write_reg(FIFO_STATUS, 0x11);
        nrf24_write_reg(DYNPD, 0);
        nrf24_write_reg(FEATURE, 0);
    }
}
void nrf24_rx_mode(uint8_t *Address, uint8_t channel)
{
    // disable the chip before configuring the device
    CE_Disable();

    nrf24_reset(STATUS);
    uint8_t STATUS_reg = nrf24_read_reg(STATUS);
    STATUS_reg = STATUS_reg;
    nrf24_write_reg(RF_CH, channel); // select the channel

    // select data pipe 2
    uint8_t en_rxaddr = nrf24_read_reg(EN_RXADDR);
    en_rxaddr = en_rxaddr | (1 << 2);
    nrf24_write_reg(EN_RXADDR, en_rxaddr);
    //	nrf24_write_reg_multi(TX_ADDR, Address, 5);  // Set RX_ADDR_P0 equal to TX_ADDR address to handle automatic acknowledge if this is a PTX device with Enhanced ShockBurst™ enabled

    /* We must write the address for Data Pipe 1, if we want to use any pipe from 2 to 5
     * The Address from DATA Pipe 2 to Data Pipe 5 differs only in the LSB
     * Their 4 MSB Bytes will still be same as Data Pipe 1
     *
     * For Eg->
     * Pipe 1 ADDR = 0xAABBCCDD11
     * Pipe 2 ADDR = 0xAABBCCDD22
     * Pipe 3 ADDR = 0xAABBCCDD33
     *
     */
    nrf24_write_reg_multi(RX_ADDR_P1, Address, 5); // Write the Pipe1 address
    nrf24_write_reg(RX_ADDR_P2, 0xEE);             // Write the Pipe2 LSB address
    uint8_t Addressp0[5];
    uint8_t Addressp1[5];

    nrf24_write_reg(RX_PW_P2, 0x20); // 32 bit payload size for pipe 2
    nrf24_write_reg(RX_PW_P1, 0x20); // 32 bit payload size for pipe 2
    nrf24_read_reg_multi(RX_ADDR_P0, Addressp0, 5);
    nrf24_read_reg_multi(RX_ADDR_P1, Addressp1, 5);

    uint8_t Addressp2 = nrf24_read_reg(RX_ADDR_P2);
    nrf24_write_reg(EN_AA, 0xF);

    // power up the device in Rx mode
    uint8_t config = nrf24_read_reg(CONFIG);
    config = config | (1 << 1) | (1 << 0) | (1 << EN_CRC) | (0 << CRCO);
    nrf24_write_reg(CONFIG, config);
    uint8_t config3 = nrf24_read_reg(CONFIG);
    config3 = 0;
    // Enable the chip after configuring the device
    CE_Enable();
}

// Read all the Register data
void nrf24_read_all(uint8_t *data)
{
    for (int i = 0; i < 10; i++)
    {
        *(data + i) = nrf24_read_reg(i);
    }

    nrf24_read_reg_multi(RX_ADDR_P0, (data + 10), 5);

    nrf24_read_reg_multi(RX_ADDR_P1, (data + 15), 5);

    *(data + 20) = nrf24_read_reg(RX_ADDR_P2);
    *(data + 21) = nrf24_read_reg(RX_ADDR_P3);
    *(data + 22) = nrf24_read_reg(RX_ADDR_P4);
    *(data + 23) = nrf24_read_reg(RX_ADDR_P5);

    nrf24_read_reg_multi(RX_ADDR_P0, (data + 24), 5);

    for (int i = 29; i < 38; i++)
    {
        *(data + i) = nrf24_read_reg(i - 12);
    }
}

void nrf24_receive(uint8_t *data)
{
    uint8_t cmdtosend[1];
    cmdtosend[0] = R_RX_PAYLOAD;
    spi_transaction_t t = {
        .tx_buffer = cmdtosend,
        .rx_buffer = data,

        .length = 8 * 32,
    };

    // select the device
    CS_Select();
    spi_device_transmit(SPIHandle, &t);
    // spi_device_transmit(SPIHandle, &t2);

    // Receive the payload

    // Unselect the device
    CS_UnSelect();

    nrf24_send_cmd(FLUSH_RX);
}
uint8_t is_data_available(int pipenum)
{
    uint8_t status = nrf24_read_reg(STATUS);

    if ((status & (1 << 6)) && (status & (pipenum << 1)))
    {

        nrf24_write_reg(STATUS, (1 << 6));

        return 1;
    }
    else if ((status & (pipenum << 1)))
    {
        // flush RX if RX_DR is not set and there are data in the selected pipe
        nrf24_send_cmd(FLUSH_RX);
    }
    if ((status & (1 << 3)))
    {
        nrf24_write_reg(STATUS, (1 << 3));
    }
    return 0;
}

void nrf24_Init_e_shockburst(void)
{

    if (spi_init == false)
    {
        Nrf24_spi_init();
        spi_init = true;
    }
    // disable the chip before configuring the device
    CE_Disable();

    // reset everything
    nrf24_reset(0);
    nrf24_write_reg(CONFIG, 0x08);
    nrf24_write_reg(EN_AA, 0x3F);
    nrf24_write_reg(EN_RXADDR, 0x0F);
    nrf24_write_reg(SETUP_AW, 0x03);
    nrf24_write_reg(SETUP_RETR, 0x0F);
    nrf24_write_reg(RF_CH, 0x0A);
    nrf24_write_reg(RF_SETUP, 0x0E);
    nrf24_write_reg(STATUS, 0x00);
    nrf24_write_reg(RX_PW_P0, 0x20);
    nrf24_write_reg(RX_PW_P1, 0x20);
    nrf24_write_reg(RX_PW_P2, 0x20);
    nrf24_write_reg(RX_PW_P3, 0x00);
    nrf24_write_reg(RX_PW_P4, 0x00);
    nrf24_write_reg(RX_PW_P5, 0x00);
    nrf24_write_reg(DYNPD, 0x00);
    nrf24_write_reg(FEATURE, 0x00);

    // Clear any pending interrupt flags
    nrf24_clear_iqr();
    // Enable the chip after configuring the device
    CE_Enable();
}

void NRF24_TxMode_EnhancedShockBurst(uint8_t *Address, uint8_t channel)
{

    // disable the chip before configuring the device
    CE_Disable();

    nrf24_write_reg(RF_CH, channel); // select the channel

    nrf24_write_reg_multi(TX_ADDR, Address, 5);    // Write the TX address
    nrf24_write_reg_multi(RX_ADDR_P0, Address, 5); // Set RX_ADDR_P0 equal to TX_ADDR address to handle automatic acknowledge if this is a PTX device with Enhanced ShockBurst™ enabled

    // power up the device
    uint8_t config = nrf24_read_reg(CONFIG);

    config = config | (1 << 1) | (1 << EN_CRC) | (0 << CRCO);
    ; // write 1 in the PWR_UP bit
    nrf24_write_reg(CONFIG, config);
    uint8_t config2 = nrf24_read_reg(CONFIG);

    // Enable the chip after configuring the device
    CE_Enable();
}
void Nrf24_spi_init()
{
    esp_err_t ret;

    ESP_LOGI(TAG, "CONFIG_MISO_GPIO=%d", CONFIG_MISO_GPIO);
    ESP_LOGI(TAG, "CONFIG_MOSI_GPIO=%d", CONFIG_MOSI_GPIO);
    ESP_LOGI(TAG, "CONFIG_SCLK_GPIO=%d", CONFIG_SCLK_GPIO);
    ESP_LOGI(TAG, "CONFIG_CE_GPIO=%d", CONFIG_CE_GPIO);
    ESP_LOGI(TAG, "CONFIG_CSN_GPIO=%d", CONFIG_CSN_GPIO);
    // gpio_pad_select_gpio(CONFIG_CE_GPIO);
    gpio_reset_pin(CONFIG_CE_GPIO);
    gpio_set_direction(CONFIG_CE_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_CE_GPIO, 0);

    // gpio_pad_select_gpio(CONFIG_CSN_GPIO);
    gpio_reset_pin(CONFIG_CSN_GPIO);
    gpio_set_direction(CONFIG_CSN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_CSN_GPIO, 1);

    spi_bus_config_t spi_bus_config = {
        .sclk_io_num = CONFIG_SCLK_GPIO,
        .mosi_io_num = CONFIG_MOSI_GPIO,
        .miso_io_num = CONFIG_MISO_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1};

    ret = spi_bus_initialize(HOST_ID, &spi_bus_config, SPI_DMA_CH_AUTO);
    ESP_LOGI(TAG, "spi_bus_initialize=%d", ret);
    assert(ret == ESP_OK);

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(spi_device_interface_config_t));
    devcfg.clock_speed_hz = SPI_Frequency;
    devcfg.spics_io_num = -1;
    devcfg.queue_size = 7;
    devcfg.mode = 0;
    devcfg.flags = SPI_DEVICE_NO_DUMMY;

    ret = spi_bus_add_device(HOST_ID, &devcfg, &SPIHandle);
    ESP_LOGI(TAG, "spi_bus_add_device=%d", ret);
    assert(ret == ESP_OK);
}

uint8_t nrf24_transmit(uint8_t *data)
{

    uint8_t otx = nrf24_read_reg(OBSERVE_TX);
    ESP_LOGI(TAG, "otx %d ", otx);

    uint8_t *cmdtosend[1];
    cmdtosend[0] = W_TX_PAYLOAD;

    spi_transaction_t t = {
        .tx_buffer = cmdtosend,

        .length = 8,
    };

    spi_transaction_t t2 = {
        .tx_buffer = data,

        .length = 8 * 32,
    };

    esp_err_t err;
    esp_err_t err2;
    // select the device

    CS_Select();
    err = spi_device_transmit(SPIHandle, &t);
    err2 = spi_device_transmit(SPIHandle, &t2);
    ESP_LOGI(TAG, "err %d ", err);
    ESP_LOGI(TAG, "err2 %d ", err2);

    // Receive the payload

    // Unselect the device
    CS_UnSelect();
    uint8_t otx2 = nrf24_read_reg(OBSERVE_TX);
    ESP_LOGI(TAG, "otx2 %d ", otx);
    vTaskDelay(1);
    uint8_t fifostatus = nrf24_read_reg(FIFO_STATUS);
    ESP_LOGI(TAG, "fifostatus %d ", fifostatus);

    if ((fifostatus & (1 << 4)) && (!(fifostatus & (1 << 3))))
    {
        nrf24_send_cmd(FLUSH_TX);

        // reset FIFO_STATUS
        nrf24_reset(FIFO_STATUS);

        return 1;
    }
    else if ((otx2 & 0x0F) == 0x0F)
    {
        ESP_LOGI(TAG, "otx2 is  1111  ");

        nrf24_clear_iqr();
    }

    return 0;
}

void nrf24_set_crc_mode(nrf_crc_mode_t crc_mode)
{
    nrf24_write_reg(CONFIG, (nrf24_read_reg(CONFIG) &
                             ~((NRF_BIT_3 | NRF_BIT_2) | (uint8_t)(crc_mode << 2))));
}

uint8_t nrf24_read_reg(uint8_t Reg)
{

    esp_err_t err;
    uint8_t data[1];
    spi_transaction_t t;

    memset(&t, 0, sizeof(spi_transaction_t));

    t.tx_buffer = &Reg;
    t.rx_buffer = &data;
    t.length = 8;

    // Pull the CS Pin LOW to select the device
    CS_Select();

    spi_device_transmit(SPIHandle, &t);
    spi_device_transmit(SPIHandle, &t);

    // Pull the CS HIGH to release the device
    CS_UnSelect();

    return data[0];
}

void nrf24_clear_iqr(void)
{
    uint8_t reg = nrf24_read_reg(STATUS);
    reg |= STATUS_IRQ;
    nrf24_write_reg(STATUS, reg);
     
}

// send the command to the NRF
void nrf24_send_cmd(uint8_t cmd)
{

    spi_transaction_t t = {
        .tx_buffer = &cmd,

        .length = 8};
    // Pull the CS Pin LOW to select the device
    CS_Select();

    spi_device_transmit(SPIHandle, &t);

    // Pull the CS HIGH to release the device
    CS_UnSelect();
}

void nrf24_read_reg_multi(uint8_t Reg, uint8_t *data, int size)
{
    uint8_t reg_data[1];
    reg_data[0] = (R_REGISTER | (REGISTER_MASK & Reg));
    spi_transaction_t t = {
        .tx_buffer = &reg_data,

        .length = 8};
    spi_transaction_t t2 = {

        .rx_buffer = data,
        .length = 8 * size};
    // Pull the CS Pin LOW to select the device
    CS_Select();
    spi_device_transmit(SPIHandle, &t);
    spi_device_transmit(SPIHandle, &t2);

    // Pull the CS HIGH to release the device
    CS_UnSelect();
}

void nrf24_write_reg_multi(uint8_t Reg, uint8_t *data, int size)
{
    uint8_t buf[1];
    buf[0] = (W_REGISTER | (REGISTER_MASK & Reg));
    spi_transaction_t t = {
        .tx_buffer = buf,
        .length = 8};
    spi_transaction_t t2 = {
        .tx_buffer = data,
        .length = size * 8};

    // Pull the CS Pin LOW to select the device
    CS_Select();
    spi_device_transmit(SPIHandle, &t);
    spi_device_transmit(SPIHandle, &t2);

    // Pull the CS HIGH to release the device
    CS_UnSelect();
}

// write a single byte to the particular register
void nrf24_write_reg(uint8_t Reg, uint8_t Data)
{
    uint8_t reg[1];
    uint8_t data[1];
    reg[0] = W_REGISTER | (REGISTER_MASK & Reg);
    data[0] = Data;
    spi_transaction_t t = {
        .tx_buffer = reg,
        .length = 8};
    spi_transaction_t t2 = {
        .tx_buffer = data,
        .length = 8};

    // Pull the CS Pin LOW to select the device
    CS_Select();

    spi_device_transmit(SPIHandle, &t);
    spi_device_transmit(SPIHandle, &t2);

    // Pull the CS HIGH to release the device
    CS_UnSelect();
}

uint8_t send_nrf_msg(uint8_t ch, uint8_t address[5], uint8_t command[32])
{
    ESP_LOGI(pcTaskGetName(0), "command is %x:%x:%x:%x:%x:%x  ", command[5], command[4], command[3], command[2], command[1], command[0]);
    ESP_LOGI(pcTaskGetName(0), "device_id  %x:%x:%x:%x:%x  ", address[4], address[3], address[2], address[1], address[0]);

    if (mode == TX_MODE && ch == channel)
    {

        return nrf24_transmit(command);
    }
    else
    {

        channel = ch;
        mode = TX_MODE;
        nrf24_Init_e_shockburst();
        NRF24_TxMode_EnhancedShockBurst(address, channel);

        return nrf24_transmit(command);
    }
}
uint8_t nrf_read_rx(uint8_t ch, uint8_t address[5])
{
    uint8_t value[32];
    bzero(value,32);
    if (mode == RX_MODE && channel == ch)
    {
        if (is_data_available(1))
        {
            nrf24_receive(value);
            return true;
        }
    }
    else
    {

        nrf24_Init_e_shockburst();
        nrf24_rx_mode(address, channel);
        channel = ch;
        mode = RX_MODE;

        if (is_data_available(1))
        {
            nrf24_receive(value);
            return true;
        }
    }
    return false;
}

uint8_t nrf_send_command_to_devices(uint8_t command, uint8_t device_id[5], uint8_t channel, uint8_t user_parameter, uint8_t host_address[5])
{
    ESP_LOGI(pcTaskGetName(0), "nrf_send_command_to_devices command %d", command);
    uint8_t status = false;
    uint8_t nrf_command[32];

    nrf_command[0] = command;
    nrf_command[1] = host_address[0];
    nrf_command[2] = host_address[1];
    nrf_command[3] = host_address[2];
    nrf_command[4] = host_address[3];
    nrf_command[5] = host_address[4];
    nrf_command[6] = user_parameter;
    ESP_LOGI(pcTaskGetName(0), "nrf_command is %x:%x:%x:%x:%x:%x  ", nrf_command[5], nrf_command[4], nrf_command[3], nrf_command[2], nrf_command[1], nrf_command[0]);

    status = send_nrf_msg(channel, device_id, nrf_command);

    ESP_LOGI(pcTaskGetName(0), "nrf_send_command_to_devices send_nrf_msg status: %d", status);
    if (status && false)
    {
        ESP_LOGI(pcTaskGetName(0), "Listening...1");
        time_t endwait;
        time_t start = time(NULL);
        endwait = start + 5;
        while (start < endwait)
        {
            start = time(NULL);
            status = nrf_read_rx(channel, host_address);
            if (status)
            {
                ESP_LOGI(pcTaskGetName(0), "nrf_read_rx  status is ok");

                break;
            }
            vTaskDelay(1);
        }
        ESP_LOGI(pcTaskGetName(0), " nrf_read_rx status: %d", status);
    }
    return status;
}

 

esp_err_t Nrf24_setTADDR(uint8_t *adr)
{
    esp_err_t ret = ESP_OK;
    nrf24_write_reg(RX_ADDR_P0, *adr); // RX_ADDR_P0 must be set to the sending addr for auto ack to work.
    nrf24_write_reg(TX_ADDR, *adr);
    uint8_t buffer[5];
    nrf24_write_reg_multi(RX_ADDR_P0, buffer, sizeof(buffer));
    for (int i = 0; i < 5; i++)
    {
        ESP_LOGI(TAG, "Nrf24_setTADDR adr[%d]=0x%x buffer[%d]=0x%x", i, adr[i], i, buffer[i]);
        if (adr[i] != buffer[i])
        {
            ret = ESP_FAIL;
        }
    }
    return ret;
}
uint8_t nrf24_get_status()
{
    uint8_t rv = nrf24_read_reg(STATUS);
    return rv;
}
