#include <math.h>
#include "main.h"
#include "pdm_fir.h"


#define MICROPHONE_USE_DMA

/* SPI Configuration defines */
#define SPI_SCK_PIN                       GPIO_Pin_10
#define SPI_SCK_GPIO_PORT                 GPIOB
#define SPI_SCK_GPIO_CLK                  RCC_AHB1Periph_GPIOB
#define SPI_SCK_SOURCE                    GPIO_PinSource10
#define SPI_SCK_AF                        GPIO_AF_SPI2

#define SPI_MOSI_PIN                      GPIO_Pin_3
#define SPI_MOSI_GPIO_PORT                GPIOC
#define SPI_MOSI_GPIO_CLK                 RCC_AHB1Periph_GPIOC
#define SPI_MOSI_SOURCE                   GPIO_PinSource3
#define SPI_MOSI_AF                       GPIO_AF_SPI2

#define MICROPHONE_I2S                      SPI2

#ifdef MICROPHONE_USE_DMA
    /* I2S DMA Stream definitions */
    #define MICROPHONE_I2S_DMA_CLOCK            RCC_AHB1Periph_DMA1
    #define MICROPHONE_I2S_DMA_DREG             (SPI2_BASE + 0x0c)    /* SPI2_DR  &SPI2->DR */

    #define MICROPHONE_I2S_DMA_RX_CHANNEL          DMA_Channel_0
    #define MICROPHONE_I2S_DMA_RX_REQ              SPI_I2S_DMAReq_Rx
    #define MICROPHONE_I2S_DMA_RX_STREAM           DMA1_Stream3
    #define MICROPHONE_I2S_DMA_RX_IRQ              DMA1_Stream3_IRQn
    #define MICROPHONE_I2S_DMA_RX_FLAG_TC          DMA_FLAG_TCIF3
    #define MICROPHONE_I2S_DMA_RX_FLAG_HT          DMA_FLAG_HTIF3
    #define MICROPHONE_I2S_DMA_RX_FLAG_FE          DMA_FLAG_FEIF3
    #define MICROPHONE_I2S_DMA_RX_FLAG_TE          DMA_FLAG_TEIF3
    #define MICROPHONE_I2S_DMA_RX_FLAG_DME         DMA_FLAG_DMEIF3
    #define Mic_DMA_I2S_RX_IRQHandler              DMA1_Stream3_IRQHandler
#else
    #define AUDIO_REC_SPI_IRQHANDLER            SPI2_IRQHandler
#endif


static uint32_t AudioRecInited = 0;
struct pdm_fir_filter filter;
__IO uint32_t Data_Status =0;

volatile unsigned mic_buf_idx;
short mic_buf[320];
mic_e mic_ready;
volatile uint8_t micOverrun;

/**
  * @brief  Initialize GPIO for wave recorder.
  * @param  None
  * @retval None
  */
static void WaveRecorder_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO clocks */
    RCC_AHB1PeriphClockCmd(SPI_SCK_GPIO_CLK | SPI_MOSI_GPIO_CLK, ENABLE);

    /* Enable GPIO clocks */
    RCC_AHB1PeriphClockCmd(SPI_SCK_GPIO_CLK | SPI_MOSI_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* SPI SCK pin configuration */
    GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN;
    GPIO_Init(SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

    /* Connect SPI pins to AF5 */  
    GPIO_PinAFConfig(SPI_SCK_GPIO_PORT, SPI_SCK_SOURCE, SPI_SCK_AF);

    /* SPI MOSI pin configuration */
    GPIO_InitStructure.GPIO_Pin =  SPI_MOSI_PIN;
    GPIO_Init(SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(SPI_MOSI_GPIO_PORT, SPI_MOSI_SOURCE, SPI_MOSI_AF);
}

#ifdef MICROPHONE_USE_DMA
    #define DMA_BUF_LEN         32
    uint16_t dma_buf[DMA_BUF_LEN];
#endif /* !MICROPHONE_USE_DMA */

/**
  * @brief  Initialize SPI peripheral.
  * @param  Freq :Audio frequency
  * @retval None
  */
static void WaveRecorder_SPI_Init(uint32_t Freq)
{
#ifdef MICROPHONE_USE_DMA
    DMA_InitTypeDef DMA_InitStructure; 
#endif /* MICROPHONE_USE_DMA */
    I2S_InitTypeDef I2S_InitStructure;

    /* Enable the SPI clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    /* SPI configuration */
    SPI_I2S_DeInit(MICROPHONE_I2S);
    I2S_InitStructure.I2S_AudioFreq = Freq;
    I2S_InitStructure.I2S_Standard = I2S_Standard_LSB;
    I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
    I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
    I2S_InitStructure.I2S_Mode = I2S_Mode_MasterRx;
    I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
    /* Initialize the I2S peripheral with the structure above */
    I2S_Init(MICROPHONE_I2S, &I2S_InitStructure);

#ifdef MICROPHONE_USE_DMA
    RCC_AHB1PeriphClockCmd(MICROPHONE_I2S_DMA_CLOCK, ENABLE); 

    /* Configure the DMA Stream */
    DMA_Cmd(AUDIO_I2S_DMA_STREAM, DISABLE);
    DMA_DeInit(AUDIO_I2S_DMA_STREAM);

    DMA_InitStructure.DMA_BufferSize = DMA_BUF_LEN;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&(MICROPHONE_I2S->DR));
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;

    DMA_InitStructure.DMA_Channel = MICROPHONE_I2S_DMA_RX_CHANNEL;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dma_buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_Init(MICROPHONE_I2S_DMA_RX_STREAM, &DMA_InitStructure);  

    DMA_ITConfig(MICROPHONE_I2S_DMA_RX_STREAM, DMA_IT_TC, ENABLE);
    DMA_ITConfig(MICROPHONE_I2S_DMA_RX_STREAM, DMA_IT_HT, ENABLE);

    SPI_I2S_DMACmd(MICROPHONE_I2S, MICROPHONE_I2S_DMA_RX_REQ, ENABLE);  

#else
    /* Enable the Rx buffer not empty interrupt */
    SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
#endif
}

/**
  * @brief  Initialize the NVIC.
  * @param  None
  * @retval None
  */
static void WaveRecorder_NVIC_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

#ifdef MICROPHONE_USE_DMA
    NVIC_InitStructure.NVIC_IRQChannel = MICROPHONE_I2S_DMA_RX_IRQ;
#else
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3); 
    /* Configure the SPI interrupt priority */
    NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
#endif

    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Initialize wave recording
  * @param  AudioFreq: Sampling frequency
  *         BitRes: Audio recording Samples format (from 8 to 16 bits)
  *         ChnlNbr: Number of input microphone channel
  * @retval None
  */
uint32_t WaveRecorderInit(uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr)
{ 
  /* Check if the interface is already initialized */
  if (AudioRecInited)
  {
    /* No need for initialization */
    return 0;
  }
  else
  {
    /* Enable CRC module */
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
    
	pdm_fir_flt_init(&filter);
    
    /* Configure the GPIOs */
    WaveRecorder_GPIO_Init();
    
    /* Configure the interrupts (for timer) */
    WaveRecorder_NVIC_Init();
    
    /* Configure the SPI */
    WaveRecorder_SPI_Init(AudioFreq);
    
    /* Set state of the audio recorder to initialized */
    AudioRecInited = 1;
    
    /* Return 0 if all operations are OK */
    return 0;
  }  
}



/**
  * @brief  Start audio recording
  * @param  pbuf: pointer to a buffer
  *         size: Buffer size
  * @retval None
  */
uint8_t WaveRecorderStart(/*uint16_t* pbuf*/)
{
/* Check if the interface has already been initialized */
  if (AudioRecInited)
  {
    /* Store the location and size of the audio buffer */
    //pAudioRecBuf = pbuf;
    
#ifndef MICROPHONE_USE_DMA
    /* Enable the Rx buffer not empty interrupt */
    SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
    /* The Data transfer is performed in the SPI interrupt routine */
    /* Enable the SPI peripheral */
#else
    //DMA_Cmd(MICROPHONE_I2S_DMA_TX_STREAM, ENABLE);
    DMA_Cmd(MICROPHONE_I2S_DMA_RX_STREAM, ENABLE);
#endif
    I2S_Cmd(SPI2, ENABLE); 
   
    /* Return 0 if all operations are OK */
    return 0;
  }
  else
  {
    /* Cannot perform operation */
    return 1;
  }
}

void _microphone_init()
{
    mic_ready = MIC_RDY_NONE;
    WaveRecorderInit(32000, 16, 1);

    WaveRecorderStart(/*audio_buffer*/);
}

#ifdef MICROPHONE_USE_DMA
void Mic_DMA_I2S_RX_IRQHandler()
{
    unsigned n, put = 0;

    /* Transfer complete interrupt */
    if (DMA_GetFlagStatus(MICROPHONE_I2S_DMA_RX_STREAM, MICROPHONE_I2S_DMA_RX_FLAG_TC) != RESET)
    {
        micSampCnt += 16;
        GPIO_ToggleBits(GPIOA, GPIO_Pin_2);
        for (n = 0; n < 16; n++)
            pdm_fir_flt_put(&filter, dma_buf[n]);

        put = 1;
        DMA_ClearFlag(MICROPHONE_I2S_DMA_RX_STREAM, MICROPHONE_I2S_DMA_RX_FLAG_TC);
    }

    /* Half Transfer complete interrupt */
    if (DMA_GetFlagStatus(MICROPHONE_I2S_DMA_RX_STREAM, MICROPHONE_I2S_DMA_RX_FLAG_HT) != RESET)
    {
        micSampCnt += 16;
        GPIO_ToggleBits(GPIOA, GPIO_Pin_3);
        for (n = 16; n < 32; n++)
            pdm_fir_flt_put(&filter, dma_buf[n]);

        put = 1;
        DMA_ClearFlag(MICROPHONE_I2S_DMA_RX_STREAM, MICROPHONE_I2S_DMA_RX_FLAG_HT);
    }

    if (put /*&& _waveType == 3*/) {
        short sample = pdm_fir_flt_get(&filter, 16);
        short out = sample * micGain;
        micPutCnt++;

        if (mic_ready != MIC_RDY_NONE) {
            micOverrun = 1;
        }

        mic_buf[mic_buf_idx++] = out;
        if (mic_buf_idx == 160)
            mic_ready = MIC_RDY_LOWER;
        else if (mic_buf_idx == 320) {
            mic_ready = MIC_RDY_UPPER;
            mic_buf_idx = 0;
        }
    }
}
#endif /* MICROPHONE_USE_DMA */

volatile unsigned micSampCnt;
volatile unsigned micPutCnt;
float micGain;

//#define CNT_TEST        16
/* PDM buffer input size */
#define INTERNAL_BUFF_SIZE      64

#define NNN     8

volatile unsigned mic_abi = 0;

#ifdef MICROPHONE_USE_DMA
#endif /* MICROPHONE_USE_DMA */

#ifndef MICROPHONE_USE_DMA
#endif /* !MICROPHONE_USE_DMA */

#ifndef MICROPHONE_USE_DMA
/**
  * @brief  This function handles AUDIO_REC_SPI global interrupt request.
  * @param  None
  * @retval None
*/
void AUDIO_REC_SPI_IRQHANDLER(void)
{
    u16 app;
    static uint8_t cnt = 0;

    //GPIO_SetBits(GPIOA, GPIO_Pin_3);
    /* Check if data are available in SPI Data register */
    if (SPI_GetITStatus(SPI2, SPI_I2S_IT_RXNE) != RESET) {
        app = SPI_I2S_ReceiveData(SPI2);
        micSampCnt++;

        pdm_fir_flt_put(&filter, app);
        if (cnt++ == 15) {
            short sample = pdm_fir_flt_get(&filter, 16);
            cnt = 0;
            /*if (_waveType == 3) { */
                short out = sample * micGain;
                audio_buffer[mic_abi++] = out;    // left
                audio_buffer[mic_abi++] = out;    // right
                micPutCnt += 2;

                if (mic_abi >= AUDIO_BUFFER_SIZE)
                    mic_abi = 0;
            /*}*/
            GPIO_ToggleBits(GPIOA, GPIO_Pin_2);
        }

    }
    //GPIO_ResetBits(GPIOA, GPIO_Pin_3);
    GPIO_ToggleBits(GPIOA, GPIO_Pin_3);
}
#endif /* !MICROPHONE_USE_DMA */


/*
 * https://www.theunterminatedstring.com/probing-pdm/
 */

