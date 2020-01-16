
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio_codec.h"
#include "stm32f4_discovery_lis302dl.h"
#include "stm32f4xx_it.h"
#include "codec2.h"

#if (CODEC2_MODE == CODEC2_MODE_3200)
    #define LORA_PAYLOAD_LENGTH        128
    #define SF_AT_500KHZ            10
#elif (CODEC2_MODE == CODEC2_MODE_2400)
    #define LORA_PAYLOAD_LENGTH        96
    #define SF_AT_500KHZ            10
#elif (CODEC2_MODE == CODEC2_MODE_1600)
    #define LORA_PAYLOAD_LENGTH        128
    #define SF_AT_500KHZ            11
#elif (CODEC2_MODE == CODEC2_MODE_1400)
    #define LORA_PAYLOAD_LENGTH        112
    #define SF_AT_500KHZ            11
#elif (CODEC2_MODE == CODEC2_MODE_1300)
    #define LORA_PAYLOAD_LENGTH         52  /* 8 c2 frames per packet */
    #define SF_AT_500KHZ            11
#elif (CODEC2_MODE == CODEC2_MODE_1200)
    #define LORA_PAYLOAD_LENGTH         96
    //#define LORA_PAYLOAD_LENGTH         192 /* 1281ms per packet */
    #define SF_AT_500KHZ            11
#elif (CODEC2_MODE == CODEC2_MODE_700C)
    #define LORA_PAYLOAD_LENGTH         56  /* 16 c2 frames per packet */
    #define SF_AT_500KHZ            12
    #if (LORA_BW_KHZ == 500)
        #error unreliable_at_500KHz
    #endif
#elif (CODEC2_MODE == CODEC2_MODE_450)  /* 360 samples per frame, 18 bits per frame */
    #define LORA_PAYLOAD_LENGTH         36  /* 16 c2 frames per packet */
    #define SF_AT_500KHZ            12
    #if (LORA_BW_KHZ == 500)
        #error unreliable_at_500KHz
    #endif
#endif

#if (LORA_BW_KHZ == 500)
    #define SPREADING_FACTOR    SF_AT_500KHZ
#elif (LORA_BW_KHZ == 250)
    #define SPREADING_FACTOR    (SF_AT_500KHZ-1)
#elif (LORA_BW_KHZ == 125)
    #define SPREADING_FACTOR    (SF_AT_500KHZ-2)
#elif (LORA_BW_KHZ == 62)
    #define SPREADING_FACTOR    (SF_AT_500KHZ-3)
#elif (LORA_BW_KHZ == 31)
    #define SPREADING_FACTOR    (SF_AT_500KHZ-4)
#endif

#define INTER_PKT_TIMEOUT               40 /* from observation of tick_at_decode_end  */
/* Exported types ------------------------------------------------------------*/
typedef enum {
    SPKR_NONE = 0,
    SPKR_LOWER,
    SPKR_UPPER
} spkr_e;
extern spkr_e fill_spkr;

typedef enum {
    MIC_RDY_NONE = 0,
    MIC_RDY_LOWER,
    MIC_RDY_UPPER
} mic_e;
extern mic_e mic_ready;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void ColorfulRingOfDeath(void);
void rxDoneCB(uint8_t size, float rssi, float snr);
void lora_rx_begin(void);

extern volatile unsigned _micSampCnt;
extern volatile unsigned micPutCnt;
extern volatile uint8_t micFrameCnt;
extern float micGain;
extern volatile uint8_t micOverrun;
extern volatile uint8_t _waveType;
extern volatile uint8_t vol;
extern volatile uint32_t _ticker;     // one millisecond systick
extern volatile uint32_t dbg_tick_at_radio_irq;
extern volatile uint32_t dbg_tick_at_send;
extern volatile uint32_t call_tx_encoded_at_tick;
extern volatile uint8_t to_rx_at_txdone;    // flag
extern volatile uint8_t _sched_tx_encoded;  // flag
extern volatile uint8_t tx_buf_idx;

extern unsigned nsamp;
extern unsigned nsamp_x2;
void vcp_printf( const char* format, ... );

void _speaker_init(unsigned);
void _microphone_init(void);
#define MAX_SPKR_BUFFER_SIZE    2560    // all codec2 modes except 450pwb
extern int16_t spkr_buffer[];

#define SINE_TABLE_LENGTH       1024
extern const uint16_t sine_table[];

extern short mic_buf[];

/* radio.c: */
void start_radio(void);
extern volatile uint8_t txing;

/* app_<radio>.c: */
void radio_print_status(void);
