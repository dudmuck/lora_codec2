#include <stdint.h>
#include <stdbool.h>

#define RC_TICKS_PER_MS         0.015625    /* 64KHz */
#define RC_TICKS_PER_US         15.625    /* 64KHz */
 
#define XTAL_FREQ_HZ            32000000
#define FREQ_DIV                33554432
#define FREQ_STEP               0.95367431640625 // ( ( double )( XTAL_FREQ / ( double )FREQ_DIV ) )
#define MHZ_TO_FRF              1048576 // = (1<<25) / Fxtal_MHz
#define KHZ_TO_FRF              1048.576
#define HZ_TO_FRF               1.048576 // = (1<<25) / Fxtal_Hz

 
/***************************************************************/
#define OPCODE_RESET_STATS              0x00
#define OPCODE_CLEAR_IRQ_STATUS         0x02
#define OPCODE_CLEAR_DEVICE_ERRORS      0x07
#define OPCODE_SET_DIO_IRQ_PARAMS       0x08
#define OPCODE_WRITE_REGISTER           0x0d
#define OPCODE_WRITE_BUFFER             0x0e
#define OPCODE_GET_STATS                0x10
#define OPCODE_GET_PACKET_TYPE          0x11
#define OPCODE_GET_IRQ_STATUS           0x12
#define OPCODE_GET_RX_BUFFER_STATUS     0x13
#define OPCODE_GET_PACKET_STATUS        0x14
#define OPCODE_GET_RSSIINST             0x15
#define OPCODE_GET_DEVICE_ERRORS        0x17
#define OPCODE_READ_REGISTER            0x1d
#define OPCODE_READ_BUFFER              0x1e
#define OPCODE_SET_STANDBY              0x80
#define OPCODE_SET_RX                   0x82
#define OPCODE_SET_TX                   0x83
#define OPCODE_SET_SLEEP                0x84
#define OPCODE_SET_RF_FREQUENCY         0x86
#define OPCODE_SET_CAD_PARAM            0x88
#define OPCODE_CALIBRATE                0x89
#define OPCODE_SET_PACKET_TYPE          0x8a
#define OPCODE_SET_MODULATION_PARAMS    0x8b
#define OPCODE_SET_PACKET_PARAMS        0x8c
#define OPCODE_SET_TX_PARAMS            0x8e
#define OPCODE_SET_BUFFER_BASE_ADDR     0x8f
#define OPCODE_SET_FALLBACK_MODE        0x93
#define OPCODE_SET_RX_DUTY_CYCLE        0x94
#define OPCODE_SET_PA_CONFIG            0x95
#define OPCODE_SET_REGULATOR_MODE       0x96
#define OPCODE_SET_DIO3_AS_TCXO_CTRL    0x97
#define OPCODE_CALIBRATE_IMAGE          0x98
#define OPCODE_SET_DIO2_AS_RFSWITCH     0x9d
#define OPCODE_STOP_TIMER_ON_PREAMBLE   0x9f
#define OPCODE_SET_LORA_SYMBOL_TIMEOUT  0xa0
#define OPCODE_GET_STATUS               0xc0
#define OPCODE_SET_FS                   0xc1
#define OPCODE_SET_CAD                  0xc5
#define OPCODE_SET_TX_CARRIER           0xd1
#define OPCODE_SET_TX_PREAMBLE          0xd2

/***************************************************************/
#define PACKET_TYPE_GFSK    0
#define PACKET_TYPE_LORA    1
 
#define HEADER_TYPE_VARIABLE_LENGTH     0
#define HEADER_TYPE_FIXED_LENGTH        1
 
#define LROA_CRC_OFF                    0
#define LORA_CRC_ON                     1
 
#define STANDARD_IQ                     0
#define INVERTED_IQ                     1
 
/* direct register access */
#define REG_ADDR_IRQ_STATUS            0x58a // 16bit
#define REG_ADDR_IRQ_MASK              0x58c // 16bit
#define REG_ADDR_MODCFG                0x680 // 8bit
#define REG_ADDR_BITRATE               0x6a1 // 24bit fsk
#define REG_ADDR_FREQDEV               0x6a4 // 18bit fsk
#define REG_ADDR_SHAPECFG              0x6a7 // 5bit
#define REG_ADDR_FSK_DEMOD_CFO         0x6b0 // 12bit  center frequency offset
#define REG_ADDR_FSK_PKTCTRL0          0x6b3 // 8bit
#define REG_ADDR_FSK_PKTCTRL1          0x6b4 // 3bit
#define REG_ADDR_FSK_PREAMBLE_TXLEN    0x6b5 // 16bit
#define REG_ADDR_FSK_SYNC_LEN          0x6b7 // 7bit
#define REG_ADDR_FSK_PKTCTRL1A         0x6b8 // 14bit   5bits+9bits
#define REG_ADDR_FSK_PKTCTRL2          0x6ba // 8bit
#define REG_ADDR_FSK_PAYLOAD_LEN       0x6bb // 8bit
#define REG_ADDR_FSK_CRCINIT           0x6bc // 16bit
#define REG_ADDR_FSK_CRCPOLY           0x6be // 16bit
#define REG_ADDR_SYNCADDR              0x6c0 // 64bit fsk
#define REG_ADDR_NODEADDR              0x6cd // 8bit fsk
#define REG_ADDR_BROADCAST             0x6ce // 8bit fsk
#define REG_ADDR_NODEADDRCOMP          0x6cf // 2bit fsk
 
#define REG_ADDR_LORA_TXPKTLEN         0x702 // 8bit
#define REG_ADDR_LORA_CONFIG0          0x703 // 8bit  bw/sf
#define REG_ADDR_LORA_CONFIG1          0x704 // 8bit  ppm_offset, fixlen, invertiq, cr
#define REG_ADDR_LORA_CONFIG2          0x705 // 8bit  crcType
#define REG_ADDR_LORA_IRQ_MASK         0x70a // 24bit
#define REG_ADDR_LORA_CONFIG9          0x724 // 8bit
#define REG_ADDR_LORA_PREAMBLE_SYMBNB  0x73a // 16bit
#define REG_ADDR_LORA_CAD_PN_RATIO     0x73e // 8bit
#define REG_ADDR_LORA_CAD_MINPEAK      0x73f // 8bit
#define REG_ADDR_LORA_SYNC             0x740 // config22, config23: frame sync peak position
#define REG_ADDR_LORA_STATUS           0x76b //
 
#define REG_ADDR_DIGFECTL              0x804 // 6bits
#define REG_ADDR_BWSEL                 0x807 // 5bits
#define REG_ADDR_RANDOM                0x819 //        ro
#define REG_ADDR_PA_CTRL0              0x880 // 8bits
#define REG_ADDR_PA_CTRL1              0x881 // 8bits
#define REG_ADDR_DIG_CTRL              0x882 // 8bits
#define REG_ADDR_PWR_CTRL              0x883 // 8bits
#define REG_ADDR_I_GAIN                0x884 // 8bits  integral gain in pi filter
#define REG_ADDR_P_GAIN                0x885 // 8bits  proportional gain in pi filter
#define REG_ADDR_RFFREQ                0x88b // 31bits
#define REG_ADDR_FREQ_OFFSET           0x88f // 19bits
#define REG_ADDR_ANACTRL6              0x8d7 // 6bits
#define REG_ADDR_ANACTRL7              0x8d8 // 6bits
#define REG_ADDR_ANACTRL15             0x8e1 // 7bits
#define REG_ADDR_ANACTRL16             0x8e2
#define REG_ADDR_PA_CTRL1B             0x8e6
#define REG_ADDR_OCP                   0x8e7 // 6bits   Imax        2.5mA steps
#define REG_ADDR_IMAX_OFFSET           0x8e8 // 5bits  OCP offset
#define REG_ADDR_XTA_TRIM              0x911 // crystal trim only in xosc
#define REG_ADDR_XTB_TRIM              0x912 // crystal trim only in xosc
#define REG_ADDR_                      0x
 
/**********************************************/

#define SET_RAMP_10U        0x00
#define SET_RAMP_20U        0x01
#define SET_RAMP_40U        0x02
#define SET_RAMP_80U        0x03
#define SET_RAMP_200U       0x04
#define SET_RAMP_800U       0x05
#define SET_RAMP_1700U      0x06
#define SET_RAMP_3400U      0x07
 
typedef union {
    struct {
        uint8_t PreambleLengthHi;   // param1
        uint8_t PreambleLengthLo;   // param2
        uint8_t HeaderType;         // param3
        uint8_t PayloadLength;      // param4
        uint8_t CRCType;            // param5
        uint8_t InvertIQ;           // param6
        uint8_t unused[2];
    } lora;
    struct {
        uint8_t PreambleLengthHi;       // param1
        uint8_t PreambleLengthLo;       // param2
        uint8_t PreambleDetectorLength; // param3
        uint8_t SyncWordLength;         // param4
        uint8_t AddrComp;               // param5
        uint8_t PacketType;             // param6
        uint8_t PayloadLength;          // param7
        uint8_t CRCType;                // param8
        uint8_t Whitening;              // param9
    } gfsk;
    uint8_t buf[9];
} PacketParams_t;

typedef union {
    struct {
        uint8_t spreadingFactor; // param1
        uint8_t bandwidth; // param2
        uint8_t codingRate; // param3
        uint8_t LowDatarateOptimize; // param4
    } lora;
    struct {
        uint8_t bitrateHi;  // param1
        uint8_t bitrateMid; // param2
        uint8_t bitrateLo;  // param3
        uint8_t PulseShape; // param4
        uint8_t bandwidth;  // param5
        uint8_t fdevHi;     // param6
        uint8_t fdevMid;    // param7
        uint8_t fdevLo;     // param8
    } gfsk;
    uint8_t buf[8];
} ModulationParams_t;

typedef union {
    struct {    // 
        uint16_t TxDone           : 1;    // 0
        uint16_t RxDone           : 1;    // 1
        uint16_t PreambleDetected : 1;    // 2
        uint16_t SyncWordValid    : 1;    // 3
        uint16_t HeaderValid      : 1;    // 4
        uint16_t HeaderErr        : 1;    // 5
        uint16_t CrCerr           : 1;    // 6
        uint16_t CadDone          : 1;    // 7
        uint16_t CadDetected      : 1;    // 8
        uint16_t Timeout          : 1;    // 9
        uint16_t res              : 6;    // 10,11,12,13,14,15
    } bits;
    uint16_t word;
} IrqFlags_t;

typedef union {
    struct {
        uint8_t modem_sf:  4; // 0,1,2,3
        uint8_t modem_bw:  4; // 4,5,6,7
    } bits;
    uint8_t octet;
} loraConfig0_t;  // at 0x703

#define LORA_BW_7           0x00 // 7.81 kHz real
#define LORA_BW_10          0x08 // 10.42 kHz real
#define LORA_BW_15          0x01 // 15.63 kHz real
#define LORA_BW_20          0x09 // 20.83 kHz real
#define LORA_BW_31          0x02 // 31.25 kHz real
#define LORA_BW_41          0x0A // 41.67 kHz real
#define LORA_BW_62          0x03 // 62.50 kHz real
#define LORA_BW_125         0x04 // 125 kHz real
#define LORA_BW_250         0x05 // 250 kHz real
#define LORA_BW_500         0x06 // 500 kHz real
 
#define LORA_CR_4_5         1
#define LORA_CR_4_6         2
#define LORA_CR_4_7         3
#define LORA_CR_4_8         4
 
#define GFSK_PREAMBLE_DETECTOR_OFF                  0x00
#define GFSK_PREAMBLE_DETECTOR_LENGTH_8BITS         0x04
#define GFSK_PREAMBLE_DETECTOR_LENGTH_16BITS        0x05
#define GFSK_PREAMBLE_DETECTOR_LENGTH_24BITS        0x06
#define GFSK_PREAMBLE_DETECTOR_LENGTH_32BITS        0x07
 
#define GFSK_WHITENING_OFF      0
#define GFSK_WHITENING_ON       1
 
#define GFSK_CRC_OFF            0x01
#define GFSK_CRC_1_BYTE         0x00
#define GFSK_CRC_2_BYTE         0x02
#define GFSK_CRC_1_BYTE_INV     0x04
#define GFSK_CRC_2_BYTE_INV     0x06
 
#define GFSK_RX_BW_4800             0x1F 
#define GFSK_RX_BW_5800             0x17 
#define GFSK_RX_BW_7300             0x0F 
#define GFSK_RX_BW_9700             0x1E 
#define GFSK_RX_BW_11700            0x16 
#define GFSK_RX_BW_14600            0x0E 
#define GFSK_RX_BW_19500            0x1D 
#define GFSK_RX_BW_23400            0x15 
#define GFSK_RX_BW_29300            0x0D 
#define GFSK_RX_BW_39000            0x1C 
#define GFSK_RX_BW_46900            0x14 
#define GFSK_RX_BW_58600            0x0C 
#define GFSK_RX_BW_78200            0x1B 
#define GFSK_RX_BW_93800            0x13 
#define GFSK_RX_BW_117300           0x0B 
#define GFSK_RX_BW_156200           0x1A 
#define GFSK_RX_BW_187200           0x12 
#define GFSK_RX_BW_234300           0x0A 
#define GFSK_RX_BW_312000           0x19 
#define GFSK_RX_BW_373600           0x11 
#define GFSK_RX_BW_467000           0x09 
 
#define GFSK_SHAPE_NONE             0x00
#define GFSK_SHAPE_BT0_3            0x08
#define GFSK_SHAPE_BT0_5            0x09
#define GFSK_SHAPE_BT0_7            0x0a
#define GFSK_SHAPE_BT1_0            0x0b
 
typedef enum {
    STBY_RC = 0,
    STBY_XOSC
} stby_t;
 
#define MOD_TYPE_IQ      0
#define MOD_TYPE_FSK     1
#define MOD_TYPE_MSK     2
#define MOD_TYPE_LORA    3

/**********************************************/

typedef enum {
    CHIPMODE_NONE = 0,
    CHIPMODE_RX,
    CHIPMODE_TX
} chipMote_e;

typedef union {
    struct {    // 
        uint8_t _reserved    : 1;    // 0
        uint8_t cmdStatus    : 3;    // 1,2,3
        uint8_t chipMode     : 3;    // 4,5,6
        uint8_t reserved_    : 1;    // 7
    } bits;
    uint8_t octet;
} status_t;

uint8_t SX126x_getPacketType(void);
void SX126x_setPacketType(uint8_t);
void SX126x_xfer(uint8_t opcode, uint8_t writeLen, uint8_t readLen, uint8_t* buf);
uint32_t SX126x_readReg(uint16_t addr, uint8_t len);
void SX126x_writeReg(uint16_t addr, uint32_t data, uint8_t len);
void SX126x_set_tx_dbm(bool is1262, int8_t dbm);
float SX126x_getMHz(void);
uint8_t SX126x_setMHz(float MHz);
void SX126x_setStandby(stby_t);
void SX126x_SetDIO2AsRfSwitchCtrl(uint8_t);
extern chipMote_e SX126x_chipMode;
void init_sx126x(void);


extern void (*SX126x_dio1_topHalf)(void);
extern void (*SX126x_timeout)(bool tx);
extern void (*SX126x_txDone)(void);
extern void (*SX126x_rxDone)(uint8_t size, float rssi, float snr);
extern void (*SX126x_chipModeChange)(void);
extern void (*SX126x_cadDone)(bool detected);
extern void (*SX126x_preambleDetected)(void);


extern uint8_t SX126x_tx_buf[];    // lora fifo size
extern uint8_t SX126x_rx_buf[];    // lora fifo size

/* start_tx and start_rx assumes DIO1 is connected, and only pin used to generate radio interrupt */
void SX126x_start_tx(uint8_t pktLen);  // tx_buf must be filled prior to calling
void SX126x_service(void);
void SX126x_ReadBuffer(uint8_t size, uint8_t offset);

#define RX_TIMEOUT_SINGLE         0x000000  /* stop RX after first packet */
#define RX_TIMEOUT_CONTINUOUS     0xffffff  /* keep RXing */
void SX126x_start_rx(unsigned);
