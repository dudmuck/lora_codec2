#include <stdarg.h>
#include <float.h>
#include <limits.h>
#include <string.h>
#include "main.h"
#include "radio.h"
#include "usbd_cdc_vcp.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usb_dcd_int.h"

#include "codec2/src/codec2.h"

volatile uint32_t _ticker, downTicker;

volatile uint32_t dbg_tick_start;
volatile uint32_t dbg_tick_at_send;
volatile uint32_t dbg_tick_at_radio_irq;
volatile uint32_t call_tx_encoded_at_tick;
volatile uint32_t tick_at_decode_end;
volatile uint32_t terminate_spkr_at_tick;

volatile uint8_t tx_buf_idx;

volatile uint8_t user_button_pressed;   // flag
volatile uint8_t _sched_tx_encoded;  // flag
volatile uint8_t to_rx_at_txdone;   // flag
volatile uint8_t led_hi;   // flag
volatile uint8_t terminate_spkr_rx;   // flag

volatile uint8_t usb_connected;
volatile uint8_t usb_suspended;

volatile int rx_size;
volatile float rx_rssi;
volatile float rx_snr;
uint8_t currently_decoding;

struct CODEC2 *c2;
unsigned nsamp; // 160 or 320, avoiding 450pwb for now
unsigned nsamp_x2;  // 320 or 640, avoiding 450pwb for now
#if 0
unsigned c2_nsamp; // 160 or 320, avoiding 450pwb for now
unsigned c2_nsamp_x2;  // 320 or 640, avoiding 450pwb for now
unsigned audio_nsamp;
unsigned audio_nsamp_x2;
#endif /* if 0 */
uint8_t _bytes_per_frame;
uint8_t frame_length_bytes;

/*
 * The USB data must be 4 byte aligned if DMA is enabled. This macro handles
 * the alignment, if necessary (it's actually magic, but don't tell anyone).
 */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;


#define LFSR_INIT       0x1ff
unsigned int lfsr = LFSR_INIT;
uint8_t get_pn9_byte()
{
    uint8_t ret = 0;
    int xor_out;

    xor_out = ((lfsr >> 5) & 0xf) ^ (lfsr & 0xf);   // four bits at a time
    lfsr = (lfsr >> 4) | (xor_out << 5);    // four bits at a time

    ret |= (lfsr >> 5) & 0x0f;

    xor_out = ((lfsr >> 5) & 0xf) ^ (lfsr & 0xf);   // four bits at a time
    lfsr = (lfsr >> 4) | (xor_out << 5);    // four bits at a time

    ret |= ((lfsr >> 1) & 0xf0);

    return ret;
}


char str[512];

//extern uint32_t APP_Rx_ptr_in;
void vcp_printf( const char* format, ... )
{
#if ENABLE_VCP_PRINT
    va_list arglist;
    int n;

    va_start( arglist, format );
    n = vsnprintf(str, sizeof(str), format, arglist);
    va_end( arglist );

    VCP_send_buffer((uint8_t*)str, n);
#endif /* ENABLE_VCP_PRINT */
}

void tx_encoded()
{
    if (txing) {
        vcp_printf("\e[31mloraStillTxing\e[0m ");
    } else {
        vcp_printf("LoRaTx ");
    }
    dbg_tick_at_send = _ticker;
    Radio_Send(tx_buf_idx/*, 0, 0, 0*/);
    txing = 1;
    tx_buf_idx = 0;

    /* ? how many ticks between tx pkts ? */
    GPIO_WriteBit(GPIOD, GPIO_Pin_14, Bit_SET);  // red on
}

void SysTick_Handler(void)
{
    GPIO_ToggleBits(GPIOD, GPIO_Pin_11);
	_ticker++;
	if (downTicker > 0)
	{
		downTicker--;
	}

    if (_ticker == call_tx_encoded_at_tick) {
        tx_encoded();
    }

    if (_ticker == terminate_spkr_at_tick) {
        /* radio receiver end-of-transmission : for last pkt full length */
        terminate_spkr_rx = 1;
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

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_2;  // microphone debug pins
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;  // systick observation
    GPIO_Init(GPIOD, &GPIO_InitStructure);


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


#ifdef LOOPBACK
void set_test_pattern()
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
}
#endif /* LOOPBACK */

#define USER_BUTTON                           GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)
void app_gpio_init()
{
    GPIO_InitTypeDef   GPIO_InitStructure;

    /* Enable GPIOA clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* Configure PA0 pin as input for user button */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void rxDoneCB(uint8_t size, float rssi, float snr)
{
    //vcp_printf("rxDone %ubytes %ddBm(x10) %ddB(x10)\r\n", size, (int)(rssi*10), (int)(snr*10));
    vcp_printf("rxDone %ubytes %fdBm %fdB\r\n", size, (double)rssi, (double)snr);
    rx_size = size;
    rx_rssi = rssi;
    rx_snr = snr;
}

spkr_e tone_out(uint8_t mask)
{
    spkr_e ret;
    int ampl = 4096;
    int ampl_step = ampl / nsamp;
    unsigned si = INT_MAX, cnt = 0;
    while (fill_spkr == SPKR_NONE)
        asm("nop");

    if (fill_spkr == SPKR_LOWER)
        si = 0;
    else if (fill_spkr == SPKR_UPPER)
        si = nsamp_x2;

    ret = fill_spkr;
    vcp_printf("tone_out %u si:%u astep:%u\r\n", mask, si, ampl_step);
    for (unsigned i = 0; i < nsamp; i++) {
        short out;
        if (mask == 0xff) {
            out = get_pn9_byte();   // noise
            out <<= 8;
            out += get_pn9_byte();   // noise
            out &= 0x3fff;  // lower volume
        } else
            out = (cnt & mask) ? ampl: -ampl;

        spkr_buffer[si++] = out;    // left
        spkr_buffer[si++] = out;    // right
        cnt++;
        ampl -= ampl_step;
    }
    fill_spkr = SPKR_NONE;

    return ret;
}

void silence(spkr_e tone_at)
{
    unsigned si, start_a = INT_MAX, start_b = INT_MAX;
    spkr_e a = SPKR_NONE, b = SPKR_NONE;

    /* fill speaker buffer with silence */
    if (tone_at == SPKR_LOWER) {
        a = SPKR_UPPER;
        b = SPKR_LOWER;
        start_a = nsamp_x2;
        start_b = 0;
    } else if (tone_at == SPKR_UPPER) {
        b = SPKR_UPPER;
        a = SPKR_LOWER;
        start_b = nsamp_x2;
        start_a = 0;
    }

    while (fill_spkr != a)
        asm("nop");
    si = start_a;
    for (unsigned i = 0; i < nsamp; i++) {
        spkr_buffer[si++] = 0;//x8000;    // left
        spkr_buffer[si++] = 0;//x8000;    // right
    }
    while (fill_spkr != b)
        asm("nop");
    si = start_b;
    for (unsigned i = 0; i < nsamp; i++) {
        spkr_buffer[si++] = 0;//x8000;    // left
        spkr_buffer[si++] = 0;//x8000;    // right
    }
}

void end_rx_tone()
{
    spkr_e tone_at;
    currently_decoding = 0;
    tone_at = tone_out(0xff);   // 0xff noise
    silence(tone_at);
}

spkr_e sine_out(unsigned skipcnt)
{
    spkr_e ret;
    unsigned si = UINT_MAX, table_idx;
    vcp_printf("sine_out(%u)\r\n", skipcnt);
    while (fill_spkr == SPKR_NONE)
        asm("nop");

    if (fill_spkr == SPKR_LOWER)
        si = 0;
    else if (fill_spkr == SPKR_UPPER)
        si = nsamp_x2;

    ret = fill_spkr;
    table_idx = 0;
    for (unsigned i = 0; i < nsamp; i++) {
        int16_t out = (int16_t)(sine_table[table_idx] - 0x8000);
        out /= 4;
        spkr_buffer[si++] = out;    // left
        spkr_buffer[si++] = out;    // right
        table_idx += skipcnt;
        if (table_idx > SINE_TABLE_LENGTH)
            table_idx -= SINE_TABLE_LENGTH;
    }
    fill_spkr = SPKR_NONE;

    return ret;
}

void put_spkr(const short* decoded)
{
    unsigned i, si = INT_MAX;
    while (fill_spkr == SPKR_NONE)
        asm("nop");

    if (fill_spkr == SPKR_LOWER)
        si = 0;
    else if (fill_spkr == SPKR_UPPER)
        si = nsamp_x2;

    for (i = 0; i < nsamp; i++) {
        spkr_buffer[si++] = decoded[i];    // left
        spkr_buffer[si++] = decoded[i];    // right
    }
    fill_spkr = SPKR_NONE;
}

void parse_rx()
{
    short decoded_[640];
    unsigned n;
    if (currently_decoding == 0) {
        /* start tone */
        sine_out((rx_rssi+(rx_snr*4)) + 170);
        currently_decoding = 1;
    } else {
        uint32_t since_last_decode_end = _ticker - tick_at_decode_end;
        vcp_printf("since %u\r\n", since_last_decode_end);
        terminate_spkr_at_tick = 0; // cancel spkr-terminate, another pkt received
    }

#if (CODEC2_MODE == CODEC2_MODE_700C) || (CODEC2_MODE == CODEC2_MODE_1300) 
    for (n = 0; n < rx_size; n += _bytes_per_frame) {
        unsigned i, si = INT_MAX;
        uint8_t scratch[7];
        vcp_printf("%u,%u) ", n, frame_length_bytes);
        for (i = 0; i < _bytes_per_frame; i++)
            vcp_printf("%02x ", SX126x_rx_buf[n+i]);
        vcp_printf("\r\n");
        for (i = 0; i < frame_length_bytes; i++)
            scratch[i] = SX126x_rx_buf[n+i];
        scratch[i] = SX126x_rx_buf[n+i] & 0xf0;
        vcp_printf("A ");
        for (i = 0; i <= frame_length_bytes; i++)
            vcp_printf("%02x ", scratch[i]);

        GPIO_ToggleBits(GPIOD, GPIO_Pin_15);    // blue toggle during decode (blocking mainloop here)
        codec2_decode(c2, decoded_, scratch);
        while (fill_spkr == SPKR_NONE)
            asm("nop");

        if (fill_spkr == SPKR_LOWER)
            si = 0;
        else if (fill_spkr == SPKR_UPPER)
            si = nsamp_x2;

        for (i = 0; i < nsamp; i++) {
            spkr_buffer[si++] = decoded_[i];    // left
            spkr_buffer[si++] = decoded_[i];    // right
        }
        fill_spkr = SPKR_NONE;

        for (i = 0; i < frame_length_bytes; i++) {
            scratch[i] = SX126x_rx_buf[n+i+frame_length_bytes] & 0x0f;
            scratch[i] <<= 4;
            if (i < frame_length_bytes)
                scratch[i] |= (SX126x_rx_buf[n+i+frame_length_bytes+1] & 0xf0) >> 4;
        }
        vcp_printf("B ");
        for (i = 0; i <= frame_length_bytes; i++)
            vcp_printf("%02x ", scratch[i]);

        GPIO_ToggleBits(GPIOD, GPIO_Pin_15);    // blue toggle during decode (blocking mainloop here)
        codec2_decode(c2, decoded_, scratch);
        while (fill_spkr == SPKR_NONE)
            asm("nop");

        if (fill_spkr == SPKR_LOWER)
            si = 0;
        else if (fill_spkr == SPKR_UPPER)
            si = nsamp_x2;

        for (i = 0; i < nsamp; i++) {
            spkr_buffer[si++] = decoded_[i];    // left
            spkr_buffer[si++] = decoded_[i];    // right
        }
        fill_spkr = SPKR_NONE;
    }
#elif (CODEC2_MODE == CODEC2_MODE_450)
    for (n = 0; n < rx_size; /*n += _bytes_per_frame*/) {
        unsigned i, stop = n + _bytes_per_frame;
        if (stop > rx_size)
            stop = rx_size;
        for (i = n; i < stop; i++)
            vcp_printf("%02x ", SX126x_rx_buf[i]);
        vcp_printf("\r\n");
        uint8_t o, scratch[3];  // {} = 18bits to decoder
        scratch[0] = SX126x_rx_buf[n++];         // n=0 { 0, 1, 2, 3, 4, 5, 6, 7}
        scratch[1] = SX126x_rx_buf[n++];         // n=1 { 8, 9,10,11,12,13,14,15}
        scratch[2] = SX126x_rx_buf[n] & 0xc0;    // n=2 {16,17}
        if (n >= rx_size) {
            vcp_printf("cutoff\r\n");
            break;
        }
        vcp_printf("A) %02x %02x %02x\r\n", scratch[0], scratch[1], scratch[2]);
        codec2_decode(c2, decoded_, scratch);
        put_spkr(decoded_);
        scratch[0] = SX126x_rx_buf[n++] & 0x3f;  // n=2 {0,1,2,3,4,5}
        scratch[0] <<= 2;
        o = SX126x_rx_buf[n] & 0xc0;             // n=3 {6,7}
        o >>= 6;
        scratch[0] |= o;
        scratch[1] = SX126x_rx_buf[n++] & 0x3f;  // n=3 {8,9,10,11,12,13}
        scratch[1] <<= 2;
        o = SX126x_rx_buf[n] & 0xc0;             // n=4 {14,15}
        o >>= 6;
        scratch[1] |= o;
        o = SX126x_rx_buf[n] & 0x30;             // n=4 {16,17}
        o <<= 2;
        scratch[2] = o;
        if (n >= rx_size) {
            vcp_printf("cutoff\r\n");
            break;
        }
        vcp_printf("B) %02x %02x %02x\r\n", scratch[0], scratch[1], scratch[2]);
        codec2_decode(c2, decoded_, scratch);
        put_spkr(decoded_);
        o = SX126x_rx_buf[n++] & 0x0f;             // n=4 {0,1,2,3}
        o <<= 4;
        scratch[0] = o;
        o = SX126x_rx_buf[n] & 0xf0;             // n=5 {4,5,6,7}
        o >>= 4;
        scratch[0] |= o;
        o = SX126x_rx_buf[n++] & 0x0f;             // n=5 {8,9,10,11}
        o <<= 4;
        scratch[1] = o;
        o = SX126x_rx_buf[n] & 0xf0;             // n=6 {12,13,14,15}
        o >>= 4;
        scratch[1] |= o;
        o = SX126x_rx_buf[n] & 0x0c;            // n=6 {16,17}
        o <<= 4;
        scratch[2] = o;
        if (n >= rx_size) {
            vcp_printf("cutoff\r\n");
            break;
        }
        vcp_printf("C) %02x %02x %02x\r\n", scratch[0], scratch[1], scratch[2]);
        codec2_decode(c2, decoded_, scratch);
        put_spkr(decoded_);
        scratch[0] = SX126x_rx_buf[n++] & 0x03;            // n=6 {0,1}
        scratch[0] <<= 6;
        o = SX126x_rx_buf[n] & 0xfc;   // n=7 {2,3,4,5,6,7}
        o >>= 2;
        scratch[0] |= o;
        o = SX126x_rx_buf[n++] & 0x03;    // n=7 {8,9}
        o <<= 6;
        scratch[1] = o;
        o = SX126x_rx_buf[n] & 0xfc;    // n=8 {10,11,12,13,14,15}
        o >>= 2;
        scratch[1] |= o;
        scratch[2] = SX126x_rx_buf[n++] & 0x03;  // n=8 {16,17}
        scratch[2] <<= 6;
        if (n >= rx_size) {
            vcp_printf("cutoff\r\n");
            break;
        }
        vcp_printf("D) %02x %02x %02x\r\nrx_in:", scratch[0], scratch[1], scratch[2]);
        codec2_decode(c2, decoded_, scratch);
        put_spkr(decoded_);
    }
#else
    for (n = 0; n < rx_size; n += _bytes_per_frame) {
        unsigned i, si = INT_MAX;
        uint8_t *encoded = &SX126x_rx_buf[n];
        GPIO_ToggleBits(GPIOD, GPIO_Pin_15);    // blue toggle during decode (blocking mainloop here)
        codec2_decode(c2, decoded_, encoded);
        vcp_printf("decode%u\r\n", n);
        put_spkr(decoded_);
    }
#endif /* !((CODEC2_MODE == CODEC2_MODE_700C) || (CODEC2_MODE == CODEC2_MODE_1300)) */
    vcp_printf("decodeDone\r\n");

    if (rx_size < LORA_PAYLOAD_LENGTH) {
        /* end of transmission */
        vcp_printf("shortpkt\r\n");
        end_rx_tone();
    } else {
        vcp_printf("fullpkt\r\n");
        /* this packet is full length, unknown if last */
        tick_at_decode_end = _ticker;
        /* radio receiver end-of-transmission : for last pkt full length */
        terminate_spkr_at_tick = _ticker + INTER_PKT_TIMEOUT;
    }
}

/* start radio receive, and indicate as such */
void lora_rx_begin()
{
    Radio_Rx(0);
    GPIO_WriteBit(GPIOD, GPIO_Pin_14, Bit_RESET);  // red off
}

/*void print_buf(const uint8_t* buf, uint8_t len)
{
    unsigned i;
    for (i = 0; i < len; i++)
        vcp_printf("%02x ", buf[i]);
    vcp_printf("\r\n");
}*/

volatile uint8_t idx_start; // tmp dbg
uint8_t encode_once = 0;
uint8_t encoded[128];
/**
  * @brief  Main program.
  * @param  None
  * @retval None
*/
int main(void)
{ 
#if ((CODEC2_MODE == CODEC2_MODE_1300) || (CODEC2_MODE == CODEC2_MODE_700C) || (CODEC2_MODE == CODEC2_MODE_450))
    uint8_t scratch[8];
    uint8_t mid = 0;
    /*#if (CODEC2_MODE == CODEC2_MODE_450)
    uint64_t frame32H, frame32L;
    #endif*/
#endif
    unsigned prev_mic_samps = 0;
    unsigned prev_mic_frames = 0;
    uint32_t cpy_ticker = _ticker;
  	/* Set up the system clocks */
	SystemInit();

	/* Initialize USB, IO, SysTick, and all those other things you do in the morning */
	//init();

	c2 = codec2_create(CODEC2_MODE);
    if (c2 == NULL) {
        ColorfulRingOfDeath();
    }
    nsamp = codec2_samples_per_frame(c2);
    nsamp_x2 = nsamp * 2;
#if ((CODEC2_MODE == CODEC2_MODE_1300) || (CODEC2_MODE == CODEC2_MODE_700C))
    _bytes_per_frame = codec2_bits_per_frame(c2) / 4;       // dual frame for integer number of bytes
    frame_length_bytes = _bytes_per_frame >> 1;
#elif (CODEC2_MODE == CODEC2_MODE_450)
    _bytes_per_frame = codec2_bits_per_frame(c2) / 2;       // quad frame for integer number of bytes
    frame_length_bytes = _bytes_per_frame >> 2;
#else
    _bytes_per_frame = codec2_bits_per_frame(c2) / 8;
#endif

	/* Initialize USB, IO, SysTick, and all those other things you do in the morning */
    init();

#ifdef LOOPBACK
    set_test_pattern();
#endif /* LOOPBACK */

    _microphone_init();
    _speaker_init(nsamp_x2 * 2);

    _waveType = 3;  // 3: microphone audio

    micGain = 2.0;  // ? how sensitive ?

    start_radio();

    app_gpio_init();

    if (USER_BUTTON) {
        /* ? transmitting on startup ? */
        asm("nop");
    } else {
        lora_rx_begin();
        /*Radio_Rx(0);
        GPIO_WriteBit(GPIOD, GPIO_Pin_14, Bit_RESET);  // red off
        */
        currently_decoding = 0;
    }

    rx_size = -1;

	while (1)
	{
        unsigned rate;
        if (usb_suspended)
            rate = 1000;
        else
            rate = 500;
		/* Blink the orange LED at 1Hz */
		if (_ticker != cpy_ticker && 0 == (_ticker % rate))
		{
            //if (usb_connected)
                GPIOD->BSRRL = GPIO_Pin_15; // blue off
            /*else
                GPIOD->BSRRL = GPIO_Pin_13; // orange off
                */

            if (led_hi) {
                /*if (usb_connected)
			        GPIOD->BSRRL = GPIO_Pin_13; // orange
                else*/
			        GPIOD->BSRRL = GPIO_Pin_15; // blue
                led_hi = 0;
                //vcp_printf("lo %u\r\n", _ticker);
            } else {
                /*if (usb_connected)
			        GPIOD->BSRRH = GPIO_Pin_13; // orange
                else*/
			        GPIOD->BSRRH = GPIO_Pin_15; // blue

                led_hi = 1;
                //vcp_printf("hi %u\r\n", _ticker);
            }
            cpy_ticker = _ticker;
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
                vcp_printf("micPutCnt:%u micGain:%.1f\r\n", micPutCnt, (double)micGain);
                vcp_printf("_waveType:%u   [%u %u %u]\r\n", _waveType, _ticker, _ticker % 500, _ticker % 1000);
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
#endif
                vcp_printf(" %ubits-per-frame\tbytes_per_frame:%u payLen:%u\r\n", codec2_bits_per_frame(c2), _bytes_per_frame, LORA_PAYLOAD_LENGTH);

                radio_print_status();
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
                // float printf crashing
                vcp_printf("micGain %u (x10)\r\n", (unsigned)(micGain*10));
            } else if (theByte == 'z' || theByte == 'x') {
                if (theByte == 'z' && vol > 0) {
                    vol--;
                }
                if (theByte == 'x' && vol < 255) {
                    vol++;
                }
                vcp_printf("vol %u\r\n", vol);
                EVAL_AUDIO_VolumeCtl(vol);
            } else if (theByte >= '0' && theByte <= '9') {
                spkr_e ta;
                unsigned skip = theByte - '0';
                ta = sine_out(skip + 1);
                silence(ta);
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
		} // ..if (VCP_get_char(&theByte))
		if (0 == downTicker)
		{
			GPIOD->BSRRH = GPIO_Pin_12;
		}

#ifdef LOOPBACK
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

        if (decoded_ready == 0 && mic_ready != MIC_RDY_NONE) {
            int mi;
            if (mic_ready == MIC_RDY_LOWER) {
                //vcp_printf("MIC_RDY_LOWER\r\n");
                mi = 0; // 0 to 159
            } else if (mic_ready == MIC_RDY_UPPER) {
                //vcp_printf("MIC_RDY_UPPER\r\n");
                mi = nsamp; // 160 to 319
            }

            GPIO_SetBits(GPIOC, GPIO_Pin_5);
            codec2_encode(c2, encoded, &mic_buf[mi]);
            codec2_decode(c2, decoded, encoded);
            GPIO_ResetBits(GPIOC, GPIO_Pin_5);
			decoded_ready = 1;

            //vcp_printf("micRdy%u\r\n", mi);
            mic_ready = MIC_RDY_NONE;
        } // ..if (decoded_ready == 0  && mic_ready != MIC_RDY_NONE)

        if (micOverrun) {
            //vcp_printf("micOverrun\r\n");
            micOverrun = 0;
        }
#else   /* walkie-talkie mode: */
        if (USER_BUTTON) {
            unsigned i;
            if (_sched_tx_encoded == 0) {
                if (txing == 0 && user_button_pressed == 0) {
                    /* rx -> tx mode switch */
                    vcp_printf("keyup ");
                    Radio_Standby();
                    user_button_pressed = 1;
#if ((CODEC2_MODE == CODEC2_MODE_1300) || (CODEC2_MODE == CODEC2_MODE_700C))
                    mid = 0;
#endif
                }
                /* TX mode */
                if (mic_ready != MIC_RDY_NONE) {
                    //unsigned i;
                    int mi = nsamp_x2;
                    if (mic_ready == MIC_RDY_LOWER)
                        mi = 0;
                    else if (mic_ready == MIC_RDY_UPPER)
                        mi = nsamp;

#if ((CODEC2_MODE == CODEC2_MODE_1300) || (CODEC2_MODE == CODEC2_MODE_700C) || (CODEC2_MODE == CODEC2_MODE_450))
                    if (tx_buf_idx == 0 && mid == 0) {
#else
                    if (tx_buf_idx == 0) {
#endif
                        vcp_printf("(total:%u samps:%u, %uframes) ", _ticker - dbg_tick_start, micPutCnt - prev_mic_samps, micFrameCnt - prev_mic_frames);
                        dbg_tick_start = _ticker;
                        prev_mic_samps = micPutCnt;
                        prev_mic_frames = micFrameCnt;
                    }

                    vcp_printf("tx_buf_idx%u ", tx_buf_idx);
#if ((CODEC2_MODE == CODEC2_MODE_1300) || (CODEC2_MODE == CODEC2_MODE_700C))
                    if (mid) {
                        unsigned oidx, iidx;
                        codec2_encode(c2, scratch, &mic_buf[mi]);
                        vcp_printf("B ");
                        for (i = 0; i <= frame_length_bytes; i++) {
                            vcp_printf("%02x ", scratch[i]);
                        }
                        vcp_printf("\r\n");
                        oidx = frame_length_bytes;
                        SX126x_tx_buf[tx_buf_idx+oidx] |= scratch[0] >> 4;
                        oidx++;
                        iidx = 0;
                        for (i = 0; i <= frame_length_bytes; i++) {
                            SX126x_tx_buf[tx_buf_idx+oidx] = scratch[iidx++] << 4;
                            SX126x_tx_buf[tx_buf_idx+oidx] |= scratch[iidx] >> 4;
                            oidx++;
                        }

                        vcp_printf("out ");
                        for (i = 0; i < _bytes_per_frame; i++)
                            vcp_printf("%02x ", SX126x_tx_buf[tx_buf_idx+i]);
                        vcp_printf("\r\n");
                        tx_buf_idx += _bytes_per_frame;
                        mid = 0;
                    /* ...second half */ } else { /* first half: */
                        SX126x_tx_buf[tx_buf_idx+frame_length_bytes] = 0;
                        codec2_encode(c2, &SX126x_tx_buf[tx_buf_idx], &mic_buf[mi]);
                        vcp_printf("\r\nA ");
                        for (i = 0; i <= frame_length_bytes; i++)
                            vcp_printf("%02x ", SX126x_tx_buf[tx_buf_idx+i]);
                        vcp_printf("\r\n");
                        mid = 1;
                    }
#elif (CODEC2_MODE == CODEC2_MODE_450)
                    uint8_t o;
                    codec2_encode(c2, scratch, &mic_buf[mi]);
                    vcp_printf("%c) ", 'A' + mid);
                    for (i = 0; i <= frame_length_bytes; i++) {
                        vcp_printf("%02x ", scratch[i]);
                    }
                    vcp_printf("\r\n");
                    switch (mid) {  // {} = 18bit source
                        case 0:
                            idx_start = tx_buf_idx;
                            SX126x_tx_buf[tx_buf_idx++] = scratch[0];   // n=0 {0, 1, 2, 3, 4, 5, 6, 7}
                            SX126x_tx_buf[tx_buf_idx++] = scratch[1];   // n=1 {8, 9,10,11,12,13,14,15}
                            SX126x_tx_buf[tx_buf_idx] = scratch[2];     // n=2 {16,17}
                            mid = 1;
                            break;
                        case 1:
                            o = scratch[0] & 0xfc;                      // n=2 {0,1,2,3,4,5}
                            o >>= 2;
                            SX126x_tx_buf[tx_buf_idx++] |= o;
                            o = scratch[0] & 0x03;
                            o <<= 6;
                            SX126x_tx_buf[tx_buf_idx] = o;              // n=3 {6,7}
                            o = scratch[1] & 0xfc;
                            o >>= 2;
                            SX126x_tx_buf[tx_buf_idx++] |= o;            // n=3 {8,9,10,11,12,13}
                            o = scratch[1] & 0x03;                      // n=4 {14,15}
                            o <<= 6;
                            SX126x_tx_buf[tx_buf_idx] = o;
                            o = scratch[2];
                            o >>= 2;
                            SX126x_tx_buf[tx_buf_idx] |= o;             // n=4 {16,17}
                            mid = 2;
                            break;
                        case 2:
                            o = scratch[0] & 0xf0;                      // n=4 {0,1,2,3}
                            o >>= 4;
                            SX126x_tx_buf[tx_buf_idx++] |= o;
                            o = scratch[0] & 0x0f;                      // n=5 {4,5,6,7}
                            o <<= 4;
                            SX126x_tx_buf[tx_buf_idx] = o;
                            o = scratch[1] & 0xf0;                      // n=5 {8,9,10,11}
                            o >>= 4;
                            SX126x_tx_buf[tx_buf_idx++] |= o;
                            o = scratch[1] & 0x0f;                      // n=6 {12,13,14,15}
                            o <<= 4;
                            SX126x_tx_buf[tx_buf_idx] = o;
                            o = scratch[2];                             // n=6 {16,17}
                            o >>= 4;
                            SX126x_tx_buf[tx_buf_idx] |= o;
                            mid = 3;
                            break;
                        case 3:
                            o = scratch[0] & 0xc0;                      // n=6 {0,1}
                            o >>= 6;
                            SX126x_tx_buf[tx_buf_idx++] |= o;
                            o = scratch[0] & 0x3f;                      // n=7 {2,3,4,5,6,7}
                            o <<= 2;
                            SX126x_tx_buf[tx_buf_idx] = o;
                            o = scratch[1] & 0xc0;                      // n=7 {8,9}
                            o >>= 6;
                            SX126x_tx_buf[tx_buf_idx++] |= o;
                            o = scratch[1] & 0x3f;                      // n=8 {10,11,12 13,14,15}
                            o <<= 2;
                            SX126x_tx_buf[tx_buf_idx] = o;
                            o = scratch[2];                             // n=8 {16,17}
                            o >>= 6;
                            SX126x_tx_buf[tx_buf_idx++] |= o;
                            vcp_printf("out:");
                            for (unsigned n = idx_start; n < tx_buf_idx; n++) {
                                vcp_printf("%02x ", SX126x_tx_buf[n]);
                            }
                            vcp_printf("\r\n");
                            mid = 0;
                            break;
                    }
#else
                    codec2_encode(c2, &SX126x_tx_buf[tx_buf_idx], &mic_buf[mi]);
                    tx_buf_idx += _bytes_per_frame;
#endif
                    vcp_printf("->%u ", tx_buf_idx);
                    if (tx_buf_idx >= LORA_PAYLOAD_LENGTH) {
                        vcp_printf("tx");
                        // send radio packet here
                        tx_encoded();
#if ((CODEC2_MODE == CODEC2_MODE_1300) || (CODEC2_MODE == CODEC2_MODE_700C))
                        mid = 0;
#endif
                    }
                    vcp_printf("\r\n");

                    mic_ready = MIC_RDY_NONE;
                }
            } // ..if (_sched_tx_encoded == 0)
        } // ..if (USER_BUTTON)
        else {
            if (user_button_pressed == 1) {
                /* tx -> rx mode switch */
                vcp_printf("unkey%u ", tx_buf_idx);
                if (tx_buf_idx > 0) {
                    /* finish unsent encoded */
                    if (txing) {
                        _sched_tx_encoded = 1;
                        vcp_printf("sched ");
                    } else {
                        vcp_printf("txLastNow ");
                        tx_encoded();
                        to_rx_at_txdone = 1;
                    }
                } else {
                    lora_rx_begin();
                    /*Radio_Rx(0);
                    GPIO_WriteBit(GPIOD, GPIO_Pin_14, Bit_RESET);  // red off
                    */
                    currently_decoding = 0;
                }
                user_button_pressed = 0;
            }
        }
#endif /* !LOOPBACK */

        Radio_service();

        if (rx_size != -1) {
            parse_rx();
            rx_size = -1;
        }

        if (terminate_spkr_rx) {
            vcp_printf("terminate_spkr_rx ");
            end_rx_tone();
            terminate_spkr_rx = 0;
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
    GPIO_ToggleBits(GPIOD, GPIO_Pin_12);    // green
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

