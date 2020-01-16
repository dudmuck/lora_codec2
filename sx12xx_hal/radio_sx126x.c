#include "radio.h"
#include "stm32f4xx.h"

#define CLEAR_ANTSWPWR      GPIO_ResetBits(GPIOB, GPIO_Pin_3)
#define SET_ANTSWPWR      GPIO_SetBits(GPIOB, GPIO_Pin_3)

static PacketParams_t pp;
static bool paOff;
static uint8_t loraTimeoutSymbols;

const RadioEvents_t* RadioEvents;

void vcp_printf( const char* format, ... ); // yyy remove me

void Radio_printOpMode()
{
    status_t status;
    SX126x_xfer(OPCODE_GET_STATUS, 0, 1, &status.octet);
 
    switch (status.bits.chipMode) {
        case 2: vcp_printf("STBY_RC "); break; // STBY_RC
        case 3: vcp_printf("STBY_XOSC "); break; // STBY_XOSC
        case 4: vcp_printf("FS "); break; // FS
        case 5: vcp_printf("RX "); break; // RX
        case 6: vcp_printf("TX "); break; // TX
        default: vcp_printf("?%d? ", status.bits.chipMode); break;
    }
}

int Radio_Send(uint8_t size/*, timestamp_t maxListenTime, timestamp_t channelFreeTime, int rssiThresh*/)
{
    uint8_t buf[8];
    uint8_t pktType = SX126x_getPacketType();
 
    buf[0] = 0; // TX base address
    buf[1] = 0; // RX base address
    SX126x_xfer(OPCODE_SET_BUFFER_BASE_ADDR, 2, 0, buf);
 
    if (pktType == PACKET_TYPE_GFSK) {
        vcp_printf("Sendgfsklen%u ", size);
        pp.gfsk.PayloadLength = size;
        SX126x_xfer(OPCODE_SET_PACKET_PARAMS, 8, 0, pp.buf);
    } else if (pktType == PACKET_TYPE_LORA) {
        vcp_printf("SendLoRaLen%u ", size);
        pp.lora.PayloadLength = size;
        SX126x_xfer(OPCODE_SET_PACKET_PARAMS, 6, 0, pp.buf);
    }
 
    {
        IrqFlags_t irqEnable;
        irqEnable.word = 0;
        irqEnable.bits.TxDone = 1;
        irqEnable.bits.Timeout = 1;
 
        buf[0] = irqEnable.word >> 8;    // enable bits
        buf[1] = irqEnable.word; // enable bits
        buf[2] = irqEnable.word >> 8;     // dio1
        buf[3] = irqEnable.word;  // dio1
        buf[4] = 0; // dio2
        buf[5] = 0; // dio2
        buf[6] = 0; // dio3
        buf[7] = 0; // dio3
        SX126x_xfer(OPCODE_SET_DIO_IRQ_PARAMS, 8, 0, buf);
    }
 
    SET_ANTSWPWR; // antswPower = 1;
 
#if 0
    if (maxListenTime > 0) {
        int rssi;
        us_timestamp_t startAt, chFreeAt, now;
        uint8_t symbs = 0;
 
        SX126x_xfer(OPCODE_SET_LORA_SYMBOL_TIMEOUT, 1, 0, &symbs);
 
        radio.start_rx(RX_TIMEOUT_CONTINUOUS);
        startAt = lpt.read_us();
Lstart:
        do {
            now = lpt.read_us();
            if ((now - startAt) > maxListenTime) {
                return -1;
            }
            SX126x_xfer(OPCODE_GET_RSSIINST, 0, 2, buf);
            rssi = buf[1] / -2;
        } while (rssi > rssiThresh);
        chFreeAt = lpt.read_us();
        do {
            now = lpt.read_us();
            SX126x_xfer(OPCODE_GET_RSSIINST, 0, 2, buf);
            rssi = buf[1] / -2;
            if (rssi > rssiThresh) {
                goto Lstart;
            }
        } while ((now - chFreeAt) < channelFreeTime);
    }
#endif /* if 0 */
 
    if (paOff) {
        unsigned v = SX126x_readReg(REG_ADDR_ANACTRL16, 1);
        if ((v & 0x10) == 0) {
            v |= 0x10;
            SX126x_writeReg(REG_ADDR_ANACTRL16, v, 1);
        }
    }
    SX126x_start_tx(size);

    return 0;
} // ..Send()

void Radio_LoRaPacketConfig(unsigned preambleLen, bool fixLen, bool crcOn, bool invIQ)
{
    if (SX126x_getPacketType() != PACKET_TYPE_LORA)
        SX126x_setPacketType(PACKET_TYPE_LORA);
 
    pp.lora.PreambleLengthHi = preambleLen >> 8;
    pp.lora.PreambleLengthLo = preambleLen;
    pp.lora.HeaderType = fixLen;
    pp.lora.CRCType = crcOn;
    pp.lora.InvertIQ = invIQ;
 
    SX126x_xfer(OPCODE_SET_PACKET_PARAMS, 6, 0, pp.buf);
}

void Radio_set_tx_dbm(int8_t dbm)
{
    unsigned v = SX126x_readReg(REG_ADDR_ANACTRL16, 1);
 
    if (dbm == PA_OFF_DBM) {
        /* bench test: prevent overloading receiving station (very low tx power) */
        if ((v & 0x10) == 0) {
            v |= 0x10;
            SX126x_writeReg(REG_ADDR_ANACTRL16, v, 1);
        }
        paOff = true;
    } else {
        SX126x_set_tx_dbm(IS_SX1262, dbm);
        if (v & 0x10) {
            v &= ~0x10;
            SX126x_writeReg(REG_ADDR_ANACTRL16, v, 1);
        }
        paOff = false;
    }
}

void Radio_SetChannel(unsigned hz)
{
    SX126x_setMHz(hz / 1000000.0);
}

void Radio_LoRaModemConfig(unsigned bwKHz, uint8_t sf, uint8_t cr)
{
    ModulationParams_t mp;
    float khz, sp;
 
    if (SX126x_getPacketType() != PACKET_TYPE_LORA)
        SX126x_setPacketType(PACKET_TYPE_LORA);
 
    if (bwKHz > 250) {
        mp.lora.bandwidth = LORA_BW_500;
        khz = 500;
    } else if (bwKHz > 125) {
        mp.lora.bandwidth = LORA_BW_250;
        khz = 250;
    } else if (bwKHz > 63) {
        mp.lora.bandwidth = LORA_BW_125;
        khz = 125;
    } else if (bwKHz > 42) {
        mp.lora.bandwidth = LORA_BW_62;
        khz = 62.5;
    } else if (bwKHz > 32) {
        mp.lora.bandwidth = LORA_BW_41;
        khz = 41.67;
    } else if (bwKHz > 21) {
        mp.lora.bandwidth = LORA_BW_31;
        khz = 31.25;
    } else if (bwKHz > 16) {
        mp.lora.bandwidth = LORA_BW_20;
        khz = 20.83;
    } else if (bwKHz > 11) {
        mp.lora.bandwidth = LORA_BW_15;
        khz = 15.625;
    } else if (bwKHz > 11) {
        mp.lora.bandwidth = LORA_BW_10;
        khz = 10.42;
    } else {
        mp.lora.bandwidth = LORA_BW_7;
        khz = 7.81;
    }
 
    mp.lora.spreadingFactor = sf;
    mp.lora.codingRate = cr;
 
    sp = (1 << mp.lora.spreadingFactor) / khz;
    /* TCXO dependent */
    if (sp > 16)
        mp.lora.LowDatarateOptimize = 1; // param4
    else
        mp.lora.LowDatarateOptimize = 0; // param4
 
    SX126x_xfer(OPCODE_SET_MODULATION_PARAMS, 4, 0, mp.buf);
 
}

void Radio_Standby()
{
    SX126x_setStandby(STBY_RC);  // STBY_XOSC
 
    CLEAR_ANTSWPWR;
}


void Radio_txDoneBottom()
{
    if (RadioEvents->TxDone_botHalf)
        RadioEvents->TxDone_botHalf();

    CLEAR_ANTSWPWR;
}

void Radio_rx_done(uint8_t size, float rssi, float snr)
{
    RadioEvents->RxDone(size, rssi, snr);
}

void Radio_timeout_callback(bool tx)
{
    if (!tx) {
        if (RadioEvents->RxTimeout)
            RadioEvents->RxTimeout();
#ifdef RX_INDICATION
        RX_INDICATION = 0;
#endif
    } // else TODO tx timeout
}

void Radio_chipModeChange()
{
    /* indicate radio mode with LEDs, if desired */
}

void Radio_dio1_top_half()
{
    if (RadioEvents->DioPin_top_half)
        RadioEvents->DioPin_top_half();
 
    if (SX126x_chipMode == CHIPMODE_TX) {
        /* TxDone handling requires low latency */
        if (RadioEvents->TxDone_topHalf) {
            RadioEvents->TxDone_topHalf();
        } 
    } else {
#ifdef RX_INDICATION
        RX_INDICATION = 0;
#endif
    }
}

void Radio_Init(const RadioEvents_t* e)
{
    GPIO_InitTypeDef   GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;  // ant-sw-power
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);


    SX126x_txDone = Radio_txDoneBottom;
    SX126x_rxDone = Radio_rx_done;
    SX126x_timeout = Radio_timeout_callback;
    SX126x_chipModeChange = Radio_chipModeChange;
    SX126x_dio1_topHalf = Radio_dio1_top_half;
 
    RadioEvents = e;
    //lpt.start();

    init_sx126x();
 
    SX126x_SetDIO2AsRfSwitchCtrl(1);
}

void Radio_service()
{
    SX126x_service();
}
 

void Radio_Rx(unsigned timeout)
{
    SET_ANTSWPWR; // antswPower = 1;
 
    {
        uint8_t buf[8];
        IrqFlags_t irqEnable;
        irqEnable.word = 0;
        irqEnable.bits.RxDone = 1;
        irqEnable.bits.Timeout = 1;
 
        buf[0] = irqEnable.word >> 8;    // enable bits
        buf[1] = irqEnable.word; // enable bits
        buf[2] = irqEnable.word >> 8;     // dio1
        buf[3] = irqEnable.word;  // dio1
        buf[4] = 0; // dio2
        buf[5] = 0; // dio2
        buf[6] = 0; // dio3
        buf[7] = 0; // dio3
        SX126x_xfer(OPCODE_SET_DIO_IRQ_PARAMS, 8, 0, buf);
    }
 
#ifdef RX_INDICATION
    RX_INDICATION = 1;
#endif
    if (timeout == 0) {
        uint8_t symbs = 0;
        if (SX126x_getPacketType() == PACKET_TYPE_LORA) // shut off timeout
            SX126x_xfer(OPCODE_SET_LORA_SYMBOL_TIMEOUT, 1, 0, &symbs);
 
        SX126x_start_rx(RX_TIMEOUT_CONTINUOUS);
    } else {
        if (SX126x_getPacketType() == PACKET_TYPE_LORA)
            SX126x_xfer(OPCODE_SET_LORA_SYMBOL_TIMEOUT, 1, 0, &loraTimeoutSymbols);
 
        SX126x_start_rx(timeout * RC_TICKS_PER_US);
    }
}

void Radio_SetLoRaSymbolTimeout(uint16_t symbs)
{
    if (SX126x_getPacketType() != PACKET_TYPE_LORA)
        SX126x_setPacketType(PACKET_TYPE_LORA);
 
    loraTimeoutSymbols = symbs;
    SX126x_xfer(OPCODE_SET_LORA_SYMBOL_TIMEOUT, 1, 0, &loraTimeoutSymbols);
}

