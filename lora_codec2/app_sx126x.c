#include "main.h"
#include "radio.h"

void radio_print_status()
{
    float MHz;

    Radio_printOpMode();

    {
        loraConfig0_t conf0;
        conf0.octet = SX126x_readReg(REG_ADDR_LORA_CONFIG0, 1);
        vcp_printf("bw%u sf%u ", LORA_BW_KHZ, conf0.bits.modem_sf);
    }
    MHz = SX126x_getMHz();
    // float printf crashing
    vcp_printf("%uKHz\r\n", (unsigned)(MHz*1000));
}

