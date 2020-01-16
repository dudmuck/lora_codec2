
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio_codec.h"
#include "stm32f4_discovery_lis302dl.h"
#include "stm32f4xx_it.h"

//#include "codec2/src/codec2_fifo.h"


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

extern volatile unsigned micSampCnt;
extern volatile unsigned micPutCnt;
extern float micGain;
extern volatile uint8_t micOverrun;
/*extern struct FIFO* mic_fifo;
extern struct FIFO* spkr_fifo;*/
extern volatile uint8_t _waveType;
extern volatile uint8_t vol;
void vcp_printf( const char* format, ... );

void _speaker_init(void);
void _microphone_init(void);
#define SPKR_BUFFER_SIZE    640
extern int16_t spkr_buffer[];
extern short mic_buf[];
