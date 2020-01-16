  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio_codec.h"
#include "stm32f4_discovery_lis302dl.h"
#include "stm32f4xx_it.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);

#endif /* __MAIN_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

void _microphone_init(void);
void _speaker_init(void);
void ColorfulRingOfDeath(void);
void vcp_printf( const char* format, ... );
extern volatile unsigned micSampCnt;
extern volatile unsigned micPutCnt;

//#define AUDIO_BUFFER_SIZE       2048
//#define AUDIO_BUFFER_SIZE       512
#define AUDIO_BUFFER_SIZE        256
//#define AUDIO_BUFFER_SIZE       64
//#define AUDIO_BUFFER_SIZE       4
extern uint16_t audio_buffer[];
extern unsigned audio_buffer_idx;

extern volatile uint8_t rshift;
extern volatile unsigned skip;
extern volatile uint8_t _waveType;
extern volatile uint8_t vol;
extern volatile unsigned mic_abi;
extern float micGain;

extern volatile uint8_t left_enable;
extern volatile uint8_t right_enable;

extern float rc;
