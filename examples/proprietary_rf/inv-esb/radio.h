#ifndef RADIO_H
#define RADIO_H

#include <stdbool.h>
#include <stdint.h>
#ifdef NRF52840_XXAA
#include "nrf52840_bitfields.h"
#endif
#ifdef NRF52811_XXAA
#include "nrf52811_bitfields.h"
#endif
#include "nrfx_timer.h"
#include "app_scheduler.h"

#include "nrf_esb.h"

#define RADIO_TIMER              	NRF_TIMER0
#define RADIO_TIMER_IRQn         	TIMER0_IRQn
#define RADIO_TIMER_IRQHandler		TIMER0_IRQHandler


#define RADIO_RTC									NRF_RTC0
#define RADIO_RTC_IRQn						RTC0_IRQn
#define RADIO_RTC_IRQHandler			RTC0_IRQHandler


/**@brief Enhanced ShockBurst radio transmission power modes. */
typedef enum {
#ifdef NRF52840_XXAA	
    RADIO_TX_POWER_8DBM       = NRF_ESB_TX_POWER_8DBM,    
#endif	
    RADIO_TX_POWER_4DBM				= NRF_ESB_TX_POWER_4DBM,
    RADIO_TX_POWER_3DBM 			= NRF_ESB_TX_POWER_3DBM,     
    RADIO_TX_POWER_0DBM				= NRF_ESB_TX_POWER_0DBM,
    RADIO_TX_POWER_NEG4DBM		= NRF_ESB_TX_POWER_NEG4DBM,
    RADIO_TX_POWER_NEG8DBM		= NRF_ESB_TX_POWER_NEG8DBM,  
    RADIO_TX_POWER_NEG12DBM		= NRF_ESB_TX_POWER_NEG12DBM,
    RADIO_TX_POWER_NEG16DBM		= NRF_ESB_TX_POWER_NEG16DBM,
    RADIO_TX_POWER_NEG20DBM		= NRF_ESB_TX_POWER_NEG20DBM,
    RADIO_TX_POWER_NEG30DBM		= NRF_ESB_TX_POWER_NEG30DBM,
    RADIO_TX_POWER_NEG40DBM		= NRF_ESB_TX_POWER_NEG40DBM
} radio_tx_power_t;

typedef enum
{
	RADIO_CENTRAL_DATA_RECEIVED,
	RADIO_PERIPH_DATA_SENT,
	
}
radio_events_t;	

typedef enum
{
  RADIO_PERIPH_SEARCH,
  RADIO_PERIPH_OPERATE,
	RADIO_PERIPH_WAIT_FOR_ACK_WR
}
radio_states_t;

typedef struct
{
	uint8_t length;                                 //!< Length of the packet (maximum value is @ref NRF_ESB_MAX_PAYLOAD_LENGTH).
    uint8_t periph_num;                             //!< Peripherial number [0..255]
	uint8_t data[NRF_ESB_MAX_PAYLOAD_LENGTH];       //!< The payload data.
}radio_data_t;


typedef void (*event_callback_t ) (uint8_t radio_event);

void radio_setup(bool is_central, radio_tx_power_t tx_power,  event_callback_t event_callback, uint8_t periph_num);

void radio_poll_timer_start(uint32_t cnt);

void radio_fetch_packet(radio_data_t * rcv_data);

void radio_scan_for_poll(void);

void radio_put_packet(radio_data_t * tx_data);

#endif
