/* abstracted radio interface: works with any radio chip */
#include "main.h"
#include "radio.h"
#include "codec2.h"

#define TX_DBM              20
#define CF_HZ               917600000

volatile uint8_t txing;

void radio_irq_callback()
{
    dbg_tick_at_radio_irq = _ticker;
}

void txDoneCB()
{
    txing = 0;
    vcp_printf(" txDone_Dur:%u,%u,%u ", dbg_tick_at_radio_irq - dbg_tick_at_send, to_rx_at_txdone, _sched_tx_encoded);

    if (need_fhss_lfsr) {
        Radio_SetLoRaSymbolTimeout(24);
        Radio_Rx(999);
    } else if (to_rx_at_txdone) {
        vcp_printf("toRx ");
        lora_rx_begin();
        to_rx_at_txdone = 0;
    } else if (_sched_tx_encoded) {
        call_tx_encoded_at_tick = dbg_tick_at_radio_irq + 10;
        vcp_printf("schedIn:%d_forTxLen:%u ", call_tx_encoded_at_tick - _ticker, tx_buf_idx);
        to_rx_at_txdone = 1;
        _sched_tx_encoded = 0;
    }
#ifdef FHSS_BASE_FREQ
    else {
        fhss_set_next_channel("txing" );    // hopping on transmitter side during voice transmission
    }
#endif /* FHSS_BASE_FREQ */
}

void rxTimeoutCB()
{
    vcp_printf("rxTimeout\r\n");
#ifdef FHSS_BASE_FREQ
    /* only occurrs upon unanswered fhss request */
    if (USER_BUTTON) {
        fhss_set_next_channel("rxTimeout");    // LFSR request timeout
        Radio_Send(0);
    }
#endif /* FHSS_BASE_FREQ */
}
 
const RadioEvents_t rev = {
    /* DioPin_top_half */     radio_irq_callback,
    /* TxDone_topHalf */    NULL,
    /* TxDone_botHalf */    txDoneCB,
    /* TxTimeout  */        NULL,
    /* RxDone  */           rxDoneCB,
    /* RxTimeout  */        rxTimeoutCB,
    /* RxError  */          NULL,
    /* FhssChangeChannel  */NULL,
    /* CadDone  */          NULL
};

void start_radio()
{
    Radio_Init(&rev);

    Radio_Standby();
    Radio_LoRaModemConfig(LORA_BW_KHZ, SPREADING_FACTOR, 1);
    Radio_SetChannel(CF_HZ);

    Radio_set_tx_dbm(TX_DBM);

                      // preambleLen, fixLen, crcOn, invIQ
    Radio_LoRaPacketConfig(8, false, false, false);      // crcOff
}

