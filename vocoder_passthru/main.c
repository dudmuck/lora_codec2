#include <stdarg.h>
#include <float.h>
#include <limits.h>
#include "main.h"
#include "usbd_cdc_vcp.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usb_dcd_int.h"

#include "codec2/src/codec2.h"

volatile uint8_t usb_connected;
volatile uint8_t usb_suspended;

volatile uint32_t ticker, downTicker;

struct CODEC2 *c2;
unsigned nsamp; // 160 or 320, avoiding 450pwb for now
unsigned nsamp_x2;  // 320 or 640, avoiding 450pwb for now

/*
 * The USB data must be 4 byte aligned if DMA is enabled. This macro handles
 * the alignment, if necessary (it's actually magic, but don't tell anyone).
 */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;


char str[512];

void vcp_printf( const char* format, ... )
{
    va_list arglist;
    int n;

    va_start( arglist, format );
    n = vsnprintf(str, sizeof(str), format, arglist);
    va_end( arglist );

    VCP_send_buffer((uint8_t*)str, n);
}

void SysTick_Handler(void)
{
	ticker++;
	if (downTicker > 0)
	{
		downTicker--;
	}
}

/*
 * Call this to indicate a failure.  Blinks the STM32F4 discovery LEDs
 * in sequence.  At 168Mhz, the blinking will be very fast - about 5 Hz.
 * Keep that in mind when debugging, knowing the clock speed might help
 * with debugging.
 */
void ColorfulRingOfDeath(void)
{
	uint16_t ring = 1;
	while (1)
	{
		uint32_t count = 0;
		while (count++ < 500000);

		GPIOD->BSRRH = (ring << 12);
		ring = ring << 1;
		if (ring >= 1<<4)
		{
			ring = 1;
		}
		GPIOD->BSRRL = (ring << 12);
	}
}

void init()
{
    GPIO_InitTypeDef   GPIO_InitStructure;
	/* STM32F4 discovery LEDs */
	GPIO_InitTypeDef LED_Config;

	/* Always remember to turn on the peripheral clock...  If not, you may be up till 3am debugging... */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	LED_Config.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	LED_Config.GPIO_Mode = GPIO_Mode_OUT;
	LED_Config.GPIO_OType = GPIO_OType_PP;
	LED_Config.GPIO_Speed = GPIO_Speed_25MHz;
	LED_Config.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &LED_Config);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_5;  // audio out debug pins
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;  // codec timing pins
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_2;  // microphone debug pins
    GPIO_Init(GPIOA, &GPIO_InitStructure);


	/* Setup SysTick or CROD! */
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		ColorfulRingOfDeath();
	}

	/* Setup USB */
	USBD_Init(&USB_OTG_dev,
	            USB_OTG_FS_CORE_ID,
	            &USR_desc,
	            &USBD_CDC_cb,
	            &USR_cb);

	return;
}

volatile uint8_t _waveType;
volatile uint8_t vol;

/*void set_test_pattern()
{
    unsigned n, i;
    for (n = 0; n < SPKR_BUFFER_SIZE; n++) {
        spkr_buffer[n] = 0;
    }

    n = (SPKR_BUFFER_SIZE / 2) - 1;
    for (i = 0; i < 32; i++) {
        spkr_buffer[n--] = -8192;
    }

    n = SPKR_BUFFER_SIZE - 1;
    for (i = 0; i < 32; i++) {
        spkr_buffer[n--] = 8192;
    }
}*/

uint8_t encode_once = 0;
/**
  * @brief  Main program.
  * @param  None
  * @retval None
*/
int main(void)
{ 
    uint8_t encoded_[FRAME_LATENCY][8];
	short decoded[640];
    uint8_t encoded_rdy = 0;
    unsigned efidx = 0;
	//uint8_t decoded_ready = 0;
  	/* Set up the system clocks */
	SystemInit();

	/* Initialize USB, IO, SysTick, and all those other things you do in the morning */
	init();

    //set_test_pattern();

    _waveType = 3;  // 3: microphone audio

    micGain = 2.5;

	c2 = codec2_create(CODEC2_MODE);
    if (c2 == NULL) {
        ColorfulRingOfDeath();
    }
    nsamp = codec2_samples_per_frame(c2);
    nsamp_x2 = nsamp * 2;

    _microphone_init();
    _speaker_init(nsamp_x2 * 2);

    for (unsigned n = 0; n < FRAME_LATENCY; n++) {
        unsigned mi = nsamp_x2;

        while (mic_ready == MIC_RDY_NONE)
            asm("nop");

        if (mic_ready == MIC_RDY_LOWER)
            mi = 0;
        else if (mic_ready == MIC_RDY_UPPER)
            mi = nsamp;

        codec2_encode(c2, encoded_[n], &mic_buf[mi]);
        mic_ready = MIC_RDY_NONE;
    }
    efidx = 0;


	while (1)
	{
		/* Blink the orange LED at 1Hz */
		if (500 == ticker)
		{
			GPIOD->BSRRH = GPIO_Pin_13;
		}
		else if (1000 == ticker)
		{
			ticker = 0;
			GPIOD->BSRRL = GPIO_Pin_13;
		}


		/* If there's data on the virtual serial port:
		 *  - Echo it back
		 *  - Turn the green LED on for 10ms
		 */
		uint8_t theByte;
		if (VCP_get_char(&theByte))
		{
			VCP_put_char(theByte);


			GPIOD->BSRRL = GPIO_Pin_12;
			downTicker = 10;

            if (theByte == '.') {
                vcp_printf("CODEC2_MODE_");
#if (CODEC2_MODE == CODEC2_MODE_3200)
                vcp_printf("3200");
#elif (CODEC2_MODE == CODEC2_MODE_2400)
                vcp_printf("2400");
#elif (CODEC2_MODE == CODEC2_MODE_1600)
                vcp_printf("1600");
#elif (CODEC2_MODE == CODEC2_MODE_1400)
                vcp_printf("1400");
#elif (CODEC2_MODE == CODEC2_MODE_1300)
                vcp_printf("1300");
#elif (CODEC2_MODE == CODEC2_MODE_1200)
                vcp_printf("1200");
#elif (CODEC2_MODE == CODEC2_MODE_700C)
                vcp_printf("700C");
#elif (CODEC2_MODE == CODEC2_MODE_450)
                vcp_printf("450");
#else
                #error CODEC2_MODE
#endif
                vcp_printf(" micSampCnt:%u nsamp:%u\r\n", micSampCnt, nsamp);
                vcp_printf("_waveType:%u\r\n", _waveType);
            }
            else if (theByte == '?') {
                vcp_printf("'t':  switch audio modes\r\n");
                vcp_printf("'q' 'w':  right-shift down, up\r\n");
                vcp_printf("'a' 's':  sine tone down, up\r\n");
                vcp_printf("'o' 'p':  mic gain down, up\r\n");
                vcp_printf("'z' 'x':  speaker volume down, up\r\n");
            }
            else if (theByte == 'o' || theByte == 'p') {
                if (theByte == 'o' && micGain > 0.1) {
                    micGain -= 0.1;
                } else if (theByte == 'p' && micGain < 10.0) {
                    micGain += 0.1;
                }
                vcp_printf("micGain %f\r\n", (double)micGain);
            } else if (theByte == 'z' || theByte == 'x') {
                if (theByte == 'z' && vol > 0) {
                    vol--;
                }
                if (theByte == 'x' && vol < 255) {
                    vol++;
                }
                vcp_printf("vol %u\r\n", vol);
                EVAL_AUDIO_VolumeCtl(vol);
            } else if (theByte == 'e') {
                encode_once = 1;
                vcp_printf("encode_once\r\n");
            } else if (theByte == 23) {    // 23 = ctrl-W
                if (_waveType == 0) {
                    _waveType = 1;
                    vcp_printf("sine\r\n");
                } else if (_waveType == 1) {
                    _waveType = 2;
                    vcp_printf("ramp\r\n");
                } else if (_waveType == 2) {
                    _waveType = 3;
                    vcp_printf("mic\r\n");
                } else if (_waveType == 3) {
                    _waveType = 0;
                    vcp_printf("test\r\n");
                    //set_test_pattern();
                }
            } 
		}
		if (0 == downTicker)
		{
			GPIOD->BSRRH = GPIO_Pin_12;
		}

#if 0
		if (decoded_ready && fill_spkr != SPKR_NONE) {
			unsigned i, si;
            if (fill_spkr == SPKR_LOWER)
                si = 0;
            else if (fill_spkr == SPKR_UPPER)
                si = nsamp_x2;

			//vcp_printf("spkr%u\r\n", si);
            for (i = 0; i < nsamp; i++) {
                spkr_buffer[si++] = decoded[i];    // left
                spkr_buffer[si++] = decoded[i];    // right
            }

			decoded_ready = 0;
            fill_spkr = SPKR_NONE;
		}
#endif /* if 0 */

        if (/*decoded_ready == 0 &&*/ mic_ready != MIC_RDY_NONE) {
            int mi = nsamp_x2;
            if (mic_ready == MIC_RDY_LOWER) {
                //vcp_printf("MIC_RDY_LOWER\r\n");
                mi = 0;
            } else if (mic_ready == MIC_RDY_UPPER) {
                //vcp_printf("MIC_RDY_UPPER\r\n");
                mi = nsamp;
            }

            GPIO_SetBits(GPIOC, GPIO_Pin_5);
            codec2_encode(c2, encoded_[efidx], &mic_buf[mi]);
            GPIO_ResetBits(GPIOC, GPIO_Pin_5);
            if (++efidx == FRAME_LATENCY)
                efidx = 0;
            encoded_rdy = 1;
			//decoded_ready = 1;

            //vcp_printf("micRdy%u\r\n", mi);
            mic_ready = MIC_RDY_NONE;
        } // ..if (decoded_ready == 0  && mic_ready != MIC_RDY_NONE)

        if (encoded_rdy) {
            unsigned idx = efidx + 1;
			unsigned i, si = UINT_MAX;

            if (idx == FRAME_LATENCY)
                idx = 0;

            GPIO_SetBits(GPIOE, GPIO_Pin_7);
            codec2_decode(c2, decoded, encoded_[idx]);
            GPIO_ResetBits(GPIOE, GPIO_Pin_7);
            encoded_rdy = 0;

            while (fill_spkr == SPKR_NONE)
                asm("nop");
            if (fill_spkr == SPKR_LOWER)
                si = 0;
            else if (fill_spkr == SPKR_UPPER)
                si = nsamp_x2;

			//vcp_printf("spkr%u\r\n", si);
            for (i = 0; i < nsamp; i++) {
                spkr_buffer[si++] = decoded[i];    // left
                spkr_buffer[si++] = decoded[i];    // right
            }

            fill_spkr = SPKR_NONE;
        }

        if (micOverrun) {
            //vcp_printf("micOverrun\r\n");
            micOverrun = 0;
        }

	} // ..while (1)
}

uint32_t Codec_TIMEOUT_UserCallback(void)
{
    return 0;
}

void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{
#if 0
/* These are volatile to try and prevent the compiler/linker optimising them
away as the variables never actually get used.  If the debugger won't show the
values of the variables, make them global my moving their declaration outside
of this function. */
volatile uint32_t r0;
volatile uint32_t r1;
volatile uint32_t r2;
volatile uint32_t r3;
volatile uint32_t r12;
volatile uint32_t lr; /* Link register. */
volatile uint32_t pc; /* Program counter. */
volatile uint32_t psr;/* Program status register. */

    r0 = pulFaultStackAddress[ 0 ];
    r1 = pulFaultStackAddress[ 1 ];
    r2 = pulFaultStackAddress[ 2 ];
    r3 = pulFaultStackAddress[ 3 ];

    r12 = pulFaultStackAddress[ 4 ];
    lr = pulFaultStackAddress[ 5 ];
    pc = pulFaultStackAddress[ 6 ];
    psr = pulFaultStackAddress[ 7 ];
#endif /* if 0 */

    /* When the following line is hit, the variables contain the register values. */
    for( ;; );
}


void NMI_Handler(void)       {}
void HardFault_Handler(void) {
    //ColorfulRingOfDeath();
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
}
void MemManage_Handler(void) { ColorfulRingOfDeath(); }
void BusFault_Handler(void)  { ColorfulRingOfDeath(); }
void UsageFault_Handler(void){ ColorfulRingOfDeath(); }
void SVC_Handler(void)       {}
void DebugMon_Handler(void)  {}
void PendSV_Handler(void)    {}


void OTG_FS_IRQHandler(void)
{
    USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

void OTG_FS_WKUP_IRQHandler(void)
{
    if(USB_OTG_dev.cfg.low_power)
    {
        *(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ;
        SystemInit();
        USB_OTG_UngateClock(&USB_OTG_dev);
    }
    EXTI_ClearITPendingBit(EXTI_Line18);
}

