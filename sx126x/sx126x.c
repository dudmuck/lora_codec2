#include "sx12xx.h"
#include <stdbool.h>
#include "stm32f4xx.h"

/* PA10 is radio NSS
 * PB1 is radio busy
 * PA5 is SCK
 * PA6 is MISO
 * PA7 is MOSI
 * PB3 is ant-sw-power
 * PA2 is dio1 interrupt in
 */
#define DIO1_PIN                        GPIO_Pin_2
#define DIO1_EXTI_PINSOURCE             EXTI_PinSource2
#define DIO1_EXTI_LINE                  EXTI_Line2
#define DIO1_EXTI_IRQn                  EXTI2_IRQn
#define DIO1_IRQHandler                 EXTI2_IRQHandler
#define DIO1_PORT                       GPIOA
#define DIO1_EXTI_PORTSOURCE            EXTI_PortSourceGPIOA
#define DIO1                            GPIO_ReadInputDataBit(DIO1_PORT, DIO1_PIN)

#define BUSY                            GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1)

#define ASSERT_NSS                      GPIO_ResetBits(GPIOA, GPIO_Pin_10)
#define UNASSERT_NSS                   GPIO_SetBits(GPIOA, GPIO_Pin_10)


#define SPIx                           SPI1
#define SPIx_CLK                       RCC_APB2Periph_SPI1
#define SPIx_CLK_INIT                  RCC_APB2PeriphClockCmd
#define SPIx_IRQn                      SPI2_IRQn
#define SPIx_IRQHANDLER                SPI2_IRQHandler

#define SPIx_SCK_PIN                   GPIO_Pin_5
#define SPIx_SCK_GPIO_PORT             GPIOA
#define SPIx_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOA
#define SPIx_SCK_SOURCE                GPIO_PinSource5
#define SPIx_SCK_AF                    GPIO_AF_SPI1

#define SPIx_MISO_PIN                  GPIO_Pin_6
#define SPIx_MISO_GPIO_PORT            GPIOA
#define SPIx_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define SPIx_MISO_SOURCE               GPIO_PinSource6
#define SPIx_MISO_AF                   GPIO_AF_SPI1

#define SPIx_MOSI_PIN                  GPIO_Pin_7
#define SPIx_MOSI_GPIO_PORT            GPIOA
#define SPIx_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define SPIx_MOSI_SOURCE               GPIO_PinSource7
#define SPIx_MOSI_AF                   GPIO_AF_SPI1

chipMote_e SX126x_chipMode;

void (*SX126x_dio1_topHalf)(void);
void (*SX126x_timeout)(bool tx);
void (*SX126x_txDone)(void);
void (*SX126x_rxDone)(uint8_t size, float rssi, float snr);
void (*SX126x_chipModeChange)(void);
void (*SX126x_cadDone)(bool detected);
void (*SX126x_preambleDetected)(void);

uint8_t SX126x_tx_buf[256];    // lora fifo size
uint8_t SX126x_rx_buf[256];    // lora fifo size

void vcp_printf( const char* format, ... ); // yyy remove

uint8_t spi_transfer(uint8_t data)
{
    // Wait for TX empty
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
	    ;
    SPI_SendData(SPIx, data);
    // Wait for RX not empty
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)
	    ;
    return SPI_ReceiveData(SPIx);
}


void SX126x_start_tx(uint8_t pktLen)
{
    uint8_t buf[8];
 
    {
        uint8_t i;
 
        while (BUSY)
            ;
 
        vcp_printf("txlen%u ", pktLen);
        ASSERT_NSS; // nss = 0;

        spi_transfer(OPCODE_WRITE_BUFFER);
        spi_transfer(0);   // offset
        i = 0;
        for (i = 0; i < pktLen; i++) {
            spi_transfer(SX126x_tx_buf[i]);
        }
        UNASSERT_NSS;
    }
 
    buf[0] = 0x40;
    buf[1] = 0x00;
    buf[2] = 0x00;
    SX126x_xfer(OPCODE_SET_TX, 3, 0, buf);
 
    SX126x_chipMode = CHIPMODE_TX;
    if (SX126x_chipModeChange)
        SX126x_chipModeChange();
}


void SX126x_xfer(uint8_t opcode, uint8_t wlen, uint8_t rlen, uint8_t* ptr)
{
    static bool sleeping = false;
    const uint8_t* stopPtr;
    const uint8_t* wstop;
    const uint8_t* rstop;
    uint8_t nop = 0;
 
    //vcp_printf("xfer%02x\r\n", opcode);
    if (sleeping) {
        ASSERT_NSS; // nss = 0;
        while (BUSY)
            ;
        sleeping = false;
    } else {
        while (BUSY)
            ;
        ASSERT_NSS; // nss = 0;
    }
 

    spi_transfer(opcode);
 
    wstop = ptr + wlen;
    rstop = ptr + rlen;
    if (rlen > wlen)
        stopPtr = rstop;
    else
        stopPtr = wstop;
 
    for (; ptr < stopPtr; ptr++) {
        if (ptr < wstop && ptr < rstop)
            *ptr = spi_transfer(*ptr);
        else if (ptr < wstop)
            spi_transfer(*ptr);
        else
            *ptr = spi_transfer(nop);    // n >= write length: send NOP
    }
 
    UNASSERT_NSS;
 
    if (opcode == OPCODE_SET_SLEEP)
        sleeping = true;
}

void SX126x_setPacketType(uint8_t type)
{
    SX126x_xfer(OPCODE_SET_PACKET_TYPE, 1, 0, &type);
}

uint8_t SX126x_getPacketType()
{
    uint8_t buf[2];
    SX126x_xfer(OPCODE_GET_PACKET_TYPE, 0, 2, buf);
    return buf[1];
}

uint32_t SX126x_readReg(uint16_t addr, uint8_t len)
{
    uint32_t ret = 0;
    unsigned i;
 
    uint8_t buf[7];
    buf[0] = addr >> 8;
    buf[1] = (uint8_t)addr;
    SX126x_xfer(OPCODE_READ_REGISTER, 2, 3+len, buf);
    for (i = 0; i < len; i++) {
        ret <<= 8;
        ret |= buf[i+3];
    }
    return ret;
}

void SX126x_writeReg(uint16_t addr, uint32_t data, uint8_t len)
{
    uint8_t buf[6];
    uint8_t n;
    buf[0] = addr >> 8;
    buf[1] = (uint8_t)addr;
    for (n = len; n > 0; n--) {
        buf[n+1] = (uint8_t)data;
        data >>= 8;
    }
    SX126x_xfer(OPCODE_WRITE_REGISTER, 2+len, 2+len, buf);
}

void SX126x_set_tx_dbm(bool is1262, int8_t dbm)
{
    uint8_t buf[4];
    // use OCP default
 
    buf[3] = 1;
    if (is1262) {
        buf[0] = 4;
        buf[1] = 7;
        buf[2] = 0;
 
        if (dbm > 22)
            dbm = 22;
        else if (dbm < -3)
            dbm = -3;
    } else {
        if (dbm == 15)
            buf[0] = 6;
        else
            buf[0] = 4;
        buf[1] = 0;
        buf[2] = 1;
 
        if (dbm > 14)
            dbm = 14;
        else if (dbm < -3)
            dbm = -3;
    }
    SX126x_xfer(OPCODE_SET_PA_CONFIG, 4, 0, buf);
 
    if (is1262 && dbm > 18) {
        /* OCP is set by chip whenever SetPaConfig() is called */
        SX126x_writeReg(REG_ADDR_OCP, 0x38, 1);
    }
 
    // SetTxParams
    buf[0] = dbm;
    //if (opt == 0) txco
    buf[1] = SET_RAMP_200U;
    SX126x_xfer(OPCODE_SET_TX_PARAMS, 2, 0, buf);
}

float SX126x_getMHz()
{
    uint32_t frf = SX126x_readReg(REG_ADDR_RFFREQ, 4);
    return frf / (float)MHZ_TO_FRF;
}

uint8_t SX126x_setMHz(float MHz)
{
    unsigned frf = MHz * MHZ_TO_FRF;
    uint8_t buf[4];
 
    buf[0] = frf >> 24;
    buf[1] = frf >> 16;
    buf[2] = frf >> 8;
    buf[3] = frf;
    SX126x_xfer(OPCODE_SET_RF_FREQUENCY, 4, 0, buf);
    return buf[3];
}

void SX126x_setStandby(stby_t stby)
{
    uint8_t octet = stby;
    SX126x_xfer(OPCODE_SET_STANDBY, 1, 0, &octet);
 
    SX126x_chipMode = CHIPMODE_NONE;
    if (SX126x_chipModeChange)
        SX126x_chipModeChange();
}

void SX126x_SetDIO2AsRfSwitchCtrl(uint8_t en)
{
    SX126x_xfer(OPCODE_SET_DIO2_AS_RFSWITCH, 1, 0, &en);
}

static void spi_begin(/*SPIFrequency frequency, uint32_t bitOrder, uint32_t mode*/)
{
    SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef   GPIO_InitStructure;
    /* Peripheral Clock Enable -------------------------------------------------*/
    /* Enable the SPI clock */
    RCC_APB2PeriphClockCmd(SPIx_CLK, ENABLE);

    /* Enable GPIO clocks */
    RCC_AHB1PeriphClockCmd(SPIx_SCK_GPIO_CLK | SPIx_MISO_GPIO_CLK | SPIx_MOSI_GPIO_CLK, ENABLE);

    /* SPI GPIO Configuration --------------------------------------------------*/
    /* GPIO Deinitialisation */
    //GPIO_DeInit(SPIx_SCK_GPIO_PORT);
    //GPIO_DeInit(SPIx_MISO_GPIO_PORT);
    //GPIO_DeInit(SPIx_MOSI_GPIO_PORT);

    /* Connect SPI pins to AF5 */  
    GPIO_PinAFConfig(SPIx_SCK_GPIO_PORT, SPIx_SCK_SOURCE, SPIx_SCK_AF);
    GPIO_PinAFConfig(SPIx_MISO_GPIO_PORT, SPIx_MISO_SOURCE, SPIx_MISO_AF);    
    GPIO_PinAFConfig(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_SOURCE, SPIx_MOSI_AF);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

    /* SPI SCK pin configuration */
    GPIO_InitStructure.GPIO_Pin = SPIx_SCK_PIN;
    GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStructure);

    /* SPI  MISO pin configuration */
    GPIO_InitStructure.GPIO_Pin =  SPIx_MISO_PIN;
    GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStructure);  

    /* SPI  MOSI pin configuration */
    GPIO_InitStructure.GPIO_Pin =  SPIx_MOSI_PIN;
    GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);

    /* SPI configuration -------------------------------------------------------*/
    SPI_I2S_DeInit(SPIx);
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    /*if (mode == SPI_MODE0)
    {*/
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    /*}
    else if (mode == SPI_MODE1)
    {
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    }
    else if (mode == SPI_MODE2)
    {
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    }
    else if (mode == SPI_MODE3)
    {
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    }*/

    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    // Prescaler is divided into PCLK2 (84MHz) to get SPI baud rate/clock speed
    // 256 => 328.125kHz
    // 128 => 656.25kHz
    // 64 => 1.3125MHz
    // 32 => 2.625MHz
    // 16 => 5.25MHz
    // 8  => 10.5MHz
    // 4  => 21.0MHz
    /*
    switch (frequency)
    {
    case SPI_21_0MHZ:
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    break;
    case SPI_10_5MHZ:
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    break;
    case SPI_5_25MHZ:
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    break;
    case SPI_2_625MHZ:
    default:
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    break;
    case SPI_1_3125MHZ:
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    break;
    case SPI_656_25KHZ:
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
    break;
    case SPI_328_125KHZ:
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    break;

    }*/
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;

    /*if (bitOrder == LSBFIRST)
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
    else*/
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

    /* Initializes the SPI communication */
    SPI_Init(SPIx, &SPI_InitStructure);
    /* Enable SPI1  */
    SPI_Cmd(SPIx, ENABLE);
#if 0
#endif /* if 0 */
}

void DIO1_IRQHandler(void)
{
    if (EXTI_GetITStatus(DIO1_EXTI_LINE) != RESET)
    {
        if (SX126x_dio1_topHalf)
            SX126x_dio1_topHalf();

        /* Clear the EXTI line 2 pending bit */
        EXTI_ClearITPendingBit(DIO1_EXTI_LINE);
    }
}


void init_sx126x()
{
    GPIO_InitTypeDef   GPIO_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;
    EXTI_InitTypeDef   EXTI_InitStructure;

    /* Enable GPIOA clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);


    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  // PA10 = nss
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;  // PB1 = busy
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    spi_begin();
    /************** PA2 for dio1 interrupt in **********************/

    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Configure PA2 pin as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = DIO1_PIN;
    GPIO_Init(DIO1_PORT, &GPIO_InitStructure);

    /* Connect EXTI Line2 to PA2 pin */
    SYSCFG_EXTILineConfig(DIO1_EXTI_PORTSOURCE, DIO1_EXTI_PINSOURCE);

    /* Configure EXTI Line2 */
    EXTI_InitStructure.EXTI_Line = DIO1_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI Line0 Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = DIO1_EXTI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


void SX126x_service()
{
    IrqFlags_t irqFlags, clearIrqFlags;
    uint8_t buf[4];
 
    if (BUSY) {
        return;
    }
 
    while (DIO1) {
        SX126x_xfer(OPCODE_GET_IRQ_STATUS, 0, 3, buf);
        irqFlags.word = buf[1] << 8;
        irqFlags.word |= buf[2];
        clearIrqFlags.word = 0;
        if (irqFlags.bits.TxDone) {
            SX126x_chipMode = CHIPMODE_NONE;
            if (SX126x_chipModeChange)
                SX126x_chipModeChange();  // might change to Rx
            if (SX126x_txDone)
                SX126x_txDone();
            clearIrqFlags.bits.TxDone = 1;
        }
        if (irqFlags.bits.RxDone) {
            if (SX126x_rxDone) {
                uint8_t len;
                float snr, rssi;
                int8_t s;
                SX126x_xfer(OPCODE_GET_RX_BUFFER_STATUS, 0, 3, buf);
                len = buf[1];
                SX126x_ReadBuffer(len, buf[2]);
                SX126x_xfer(OPCODE_GET_PACKET_STATUS, 0, 4, buf);
                rssi = -buf[1] / 2.0;   // TODO FSK
                s = buf[2];
                snr = s / 4.0;
                SX126x_rxDone(len, rssi, snr);
            }
            clearIrqFlags.bits.RxDone = 1;
        }
        if (irqFlags.bits.Timeout) {
            if (SX126x_chipMode != CHIPMODE_NONE) {
                if (SX126x_timeout)
                    SX126x_timeout(SX126x_chipMode == CHIPMODE_TX);
            }
            SX126x_chipMode = CHIPMODE_NONE;
            if (SX126x_chipModeChange)
                SX126x_chipModeChange();
            clearIrqFlags.bits.Timeout = 1;
        }
        if (irqFlags.bits.CadDone) {
            if (SX126x_cadDone)
                SX126x_cadDone(irqFlags.bits.CadDetected);
 
            clearIrqFlags.bits.CadDone = 1;
            clearIrqFlags.bits.CadDetected = irqFlags.bits.CadDetected;
        }
        if (irqFlags.bits.PreambleDetected) {
            clearIrqFlags.bits.PreambleDetected = 1;
            if (SX126x_preambleDetected)
                SX126x_preambleDetected();
        }
 
        if (clearIrqFlags.word != 0) {
            buf[0] = clearIrqFlags.word >> 8;
            buf[1] = (uint8_t)clearIrqFlags.word;
            SX126x_xfer(OPCODE_CLEAR_IRQ_STATUS, 2, 0, buf);
        }
 
    } // ...while (dio1)
 
} // ..service()

void SX126x_ReadBuffer(uint8_t size, uint8_t offset)
{
    unsigned i;
    while (BUSY)
        ;
 
    ASSERT_NSS; // nss = 0;
 
    spi_transfer(OPCODE_READ_BUFFER);
    spi_transfer(offset);
    spi_transfer(0);   // NOP
    i = 0;
    for (i = 0; i < size; i++) {
        SX126x_rx_buf[i] = spi_transfer(0);
    }
 
    UNASSERT_NSS;
}


void SX126x_start_rx(unsigned timeout)
{
    uint8_t buf[8];
 
    buf[0] = timeout >> 16;
    buf[1] = timeout >> 8;
    buf[2] = timeout;
    SX126x_xfer(OPCODE_SET_RX, 3, 0, buf);
 
    SX126x_chipMode = CHIPMODE_RX;
    if (SX126x_chipModeChange)
        SX126x_chipModeChange();
}

