#include <stdarg.h>
#include <float.h>
#include "passthru.h"
#include "usbd_cdc_vcp.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usb_dcd_int.h"

unsigned audio_buffer_idx;
uint16_t audio_buffer[AUDIO_BUFFER_SIZE];

volatile uint32_t ticker, downTicker;

volatile uint8_t usb_connected;
volatile uint8_t usb_suspended;

volatile uint8_t left_enable;
volatile uint8_t right_enable;

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
	LED_Config.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	LED_Config.GPIO_Mode = GPIO_Mode_OUT;
	LED_Config.GPIO_OType = GPIO_OType_PP;
	LED_Config.GPIO_Speed = GPIO_Speed_25MHz;
	LED_Config.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &LED_Config);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;  // audio out debug pins
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

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

    left_enable = 1;
    right_enable = 1;

	return;
}

volatile uint8_t _waveType;
volatile uint8_t vol;

void set_test_pattern()
{
    unsigned n, i;
    for (n = 0; n < AUDIO_BUFFER_SIZE; n++) {
        audio_buffer[n] = 0;
    }

    n = (AUDIO_BUFFER_SIZE / 2) - 1;
    for (i = 0; i < 32; i++) {
        audio_buffer[n--] = -8192;
    }

    n = AUDIO_BUFFER_SIZE - 1;
    for (i = 0; i < 32; i++) {
        audio_buffer[n--] = 8192;
    }
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
*/
int main(void)
{ 
  	/* Set up the system clocks */
	SystemInit();

	/* Initialize USB, IO, SysTick, and all those other things you do in the morning */
	init();

    _microphone_init();
    _speaker_init();

    _waveType = 3;  // 3: microphone audio
    set_test_pattern();

    micGain = 5.5;

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
                vcp_printf("micSampCnt:%u spkr en:%u,%u\r\n", micSampCnt, left_enable, right_enable);
                vcp_printf("_waveType:%u\r\n", _waveType);
            }
            else if (theByte == '?') {
                vcp_printf("'R':  switch audio modes\r\n");
                vcp_printf("'q' 'w':  right-shift down, up\r\n");
                vcp_printf("'a' 's':  sine tone down, up\r\n");
                vcp_printf("'o' 'p':  mic gain down, up\r\n");
                vcp_printf("'z' 'x':  speaker volume down, up\r\n");
            } else if (theByte == 'q' || theByte == 'w') {
                if (theByte == 'q' && rshift > 0) {
                    rshift--;
                }
                if (theByte == 'w' && rshift < 15) {
                    rshift++;
                }
                vcp_printf("rshift %u\r\n", rshift);
            } else if (theByte == 'a' || theByte == 's') {
                if (theByte == 'a' && skip > 0) {
                    skip--;
                }
                if (theByte == 's' && skip < 1023) {
                    skip++;
                }
                vcp_printf("skip %u\r\n", skip);
            } else if (theByte == 'o' || theByte == 'p') {
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
            } else if (theByte == 'l') {
                left_enable ^= 1;
                vcp_printf("left_enable:%u\r\n", left_enable);
            } else if (theByte == 'r') {
                right_enable ^= 1;
                vcp_printf("right_enable:%u\r\n", right_enable);
            } else if (theByte == 'R') {
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
                    set_test_pattern();
                }
            } 
		}
		if (0 == downTicker)
		{
			GPIOD->BSRRH = GPIO_Pin_12;
		}


	} // ..while (1)
}

uint32_t Codec_TIMEOUT_UserCallback(void)
{
    return 0;
}


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

struct CODEC2 *c2;
void c2_check_mode(struct CODEC2 *c2, uint8_t x) { }
volatile uint8_t ___;
