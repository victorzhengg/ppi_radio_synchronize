
#include <string.h>
#include "sdk_common.h"
#include "boards.h"
#include "nrf_gpio.h"
#include "nrf.h"
#include "radio.h"
#include "radio_config.h"

#include "nrf_esb.h"
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define RSSI_SAMPLE_TIMES				6

#define MAX_RTC_TASKS_DELAY     47                                          /**< Maximum delay until an RTC task is executed. */


//aFH
static uint8_t 	central_loss_cnt[] = {0, 0, 0};
static bool 		update_ch_tab_flag = false;    		 // Flag for update channel table, =true allows update
//static uint8_t  switch_channel_counter[] = {0, 0}; //Central down counter for channel switch, zero means it is the current channel tab
static uint8_t  periph_sw_counter = 0;	

//log
static uint16_t m_log_total_cnt[] =  {0, 0};
static uint16_t m_log_success_cnt[]= {0, 0};

static bool m_rx_ack_wait = false;

static bool m_rx_received = false;

static uint8_t chan_cnt = 0;

static uint8_t radio_table[] = RADIO_CHAN_TAB;

static uint8_t unused_radio_table[RADIO_UNUSED_CHAN_TAB_SIZE];

static uint8_t m_radio_chan_tab[RADIO_CHAN_TAB_SIZE];
static uint8_t next_radio_chan_tab[RADIO_CHAN_TAB_SIZE];

static uint8_t rx_loss_cnt = 0;

static uint8_t periph_cnt = 0;

static nrf_esb_payload_t        dum_payload  = NRF_ESB_CREATE_PAYLOAD(DATA_PIPE, 0xC0, 0xC1, 0xC2, 0xC3);  // Poll packet contains 
/* 0xCO - switch channel counter
 * 0xC1 - Channel tab 0
 * 0xC2 - Channel tab 1
 * 0xC3 - Channel tab 2
 */

static nrf_esb_payload_t        data_payload;

// These function pointers are changed dynamically, depending on protocol configuration and state.
static void (*on_radio_rtc_interrupt)(void) = 0;

static event_callback_t m_event_callback;

static const uint8_t m_pair_address[]= PAIRING_ADDRESS ;

static const uint8_t m_system_address[]= SYSTEM_ADDRESS ;

static const uint8_t debug_pins[] = {DATA_SENDING_P0, DATA_SENDING_P1, DATA_SENDING_P2, DATA_SENDING_P3 ,DATA_SENDING_P4 };

static radio_states_t rx_state;

static bool is_rx_on = false;

/**@brief Function for init the RTC1 timer.
 */
static void radio_rtc_init(void)
{
    RADIO_RTC->INTENSET = RTC_INTENSET_COMPARE0_Msk;

    NVIC_ClearPendingIRQ(RADIO_RTC_IRQn);
    NVIC_EnableIRQ(RADIO_RTC_IRQn);
}	
 
 
/**@brief Function for starting the RTC1 timer.
 */ 
static void radio_rtc_start(void)
{
    
    RADIO_RTC->TASKS_START = 1;
    nrf_delay_us(MAX_RTC_TASKS_DELAY);

}


/**@brief Function for stopping the RTC1 timer.
 */
static void radio_rtc_stop(void)
{
    NVIC_DisableIRQ(RADIO_RTC_IRQn);

    RADIO_RTC->TASKS_STOP = 1;
    nrf_delay_us(MAX_RTC_TASKS_DELAY);

    RADIO_RTC->TASKS_CLEAR = 1;
    nrf_delay_us(MAX_RTC_TASKS_DELAY);

}

/**@brief Function for setting the RTC Capture Compare register 0, and enabling the corresponding
 *        event.
 *
 * @param[in] value   New value of Capture Compare register 0.
 */
static __INLINE void radio_rtc_compare0_set(uint32_t value)
{
    RADIO_RTC->CC[0] = value;
}


static __INLINE void radio_rtc_clear_count(void)
{
	 RADIO_RTC->TASKS_CLEAR =1;
}	


static void update_unused_radio_table(uint8_t ch)
{
		uint8_t tmp_array[RADIO_UNUSED_CHAN_TAB_SIZE-1];
	
		//Fill- in the temp array
		memcpy(tmp_array, &unused_radio_table[1], RADIO_UNUSED_CHAN_TAB_SIZE-1);
	
		// Fill back the unused radio table , last byte being the withdrawn channel 
		unused_radio_table[RADIO_UNUSED_CHAN_TAB_SIZE-1] = ch;
		memcpy(unused_radio_table, tmp_array, RADIO_UNUSED_CHAN_TAB_SIZE-1);  

}	


static void rtc_tx_event_handler(void)
{
	uint32_t volatile i= 0xFF; 
	
	uint8_t tmp_used_chan;
	
	nrf_esb_flush_tx();  //JS Modify: 4/23/2020  <--
	
	radio_rtc_clear_count();
	
	//server periphs 
	nrf_esb_set_addr_prefix(periph_cnt+1, 1);	
	
	//Upload data to dum payload
	//dum_payload.data[0] = switch_channel_counter[periph_cnt];
	memcpy(&dum_payload.data[1], m_radio_chan_tab, RADIO_CHAN_TAB_SIZE); 

	i= nrf_esb_write_payload(&dum_payload);
	if (i == NRF_SUCCESS)
	{
		nrf_gpio_pin_set(debug_pins[periph_cnt]); //Set debug pin for poll the periph
	}
	else
	{
		NRF_LOG_WARNING("Sending packet failed");	
		__nop();
	}
	
	if(update_ch_tab_flag)
	{	
		//aFH : increment loss cnt for every rtc timer expires, value resets to 0 if gets RX_RECEIVED event		
				central_loss_cnt[chan_cnt]++;
//				if (switch_channel_counter[periph_cnt]!=0) // only update switch cnt at the beginning of the frame		
//				{	
//						switch_channel_counter[periph_cnt]--;				
//				}
//				else // use new channel tab for hopping if switch_channel_counter goes to zero
//				{
//					memcpy(m_radio_chan_tab, next_radio_chan_tab, RADIO_CHAN_TAB_SIZE);
//				}

	
				//aFH: Change frequency table if >=  CENTRAL_LOSS_THRESHOLD, start 
				if (central_loss_cnt[chan_cnt] == CENTRAL_LOSS_THRESHOLD)
				{
					memset(central_loss_cnt, 0, RADIO_CHAN_TAB_SIZE); //reset central loss cnt as it only needs to enter once
					update_ch_tab_flag =false;
					//Update channel table here...
					//Replace the current frequency with new one
					tmp_used_chan = m_radio_chan_tab[chan_cnt];
					//next_radio_chan_tab[chan_cnt]   = unused_radio_table[0];
					m_radio_chan_tab[chan_cnt]   = unused_radio_table[0];
					//Put the taken-away channel to end of the unused radio table
					update_unused_radio_table(tmp_used_chan);
					//memcpy(m_radio_chan_tab, next_radio_chan_tab, RADIO_CHAN_TAB_SIZE);
//					switch_channel_counter[0] = CENTRAL_SWITCH_CHAN_CNT; //
//					switch_channel_counter[1] = CENTRAL_SWITCH_CHAN_CNT; //
				}
	}
}

static void rtc_rx_event_handler(void)
{
	
	uint8_t period_delta;
	
//	if(periph_sw_counter!=0) 
//	{	
//			periph_sw_counter--;
//	}
//	else if (update_ch_tab_flag)  //update channel table once if sw counter =0
//	{
//		memcpy(m_radio_chan_tab, next_radio_chan_tab, RADIO_CHAN_TAB_SIZE);
//		update_ch_tab_flag = false;
//	}

	//radio_rtc_clear_count();
	
	if(rx_state ==  RADIO_PERIPH_SEARCH)
	{
		 radio_rtc_clear_count();
		
			//For power saving, turn off scan for RX_SEARCH_PERIOD
			if(is_rx_on)
			{
				is_rx_on = false;
				nrf_esb_stop_rx();
				nrf_gpio_pin_clear(DATA_SENDING_P4);
				
			}
			else
			{
				is_rx_on = true;
				
				nrf_gpio_pin_set(DATA_SENDING_P4);
				
				chan_cnt = (chan_cnt +1) % RADIO_CHAN_TAB_SIZE;
				nrf_esb_set_rf_channel(m_radio_chan_tab[chan_cnt]);
				nrf_esb_start_rx();
				
			}
	}
	else if ( rx_state == RADIO_PERIPH_WAIT_FOR_ACK_WR)
	{
		nrf_esb_stop_rx();	
		nrf_gpio_pin_clear(DATA_SENDING_P4);
		rx_state = RADIO_PERIPH_OPERATE;
		radio_rtc_clear_count();
		radio_rtc_compare0_set(RX_OPERATE_PERIOD_W_WAIT);
		m_rx_received = true;
		m_rx_ack_wait = true;

	}

	
	else //rx_state == RADIO_PERIPH_OPERATE
	{
		
//		
//	if(periph_sw_counter!=0) 
//	{	
//			periph_sw_counter--;
//	}
//	else 
		if (update_ch_tab_flag)  //update channel table once if sw counter =0
	{
		memcpy(m_radio_chan_tab, next_radio_chan_tab, RADIO_CHAN_TAB_SIZE);
		update_ch_tab_flag = false;
	}


		if (rx_loss_cnt >= RX_LOSS_THRESHOLD	)
		{
			rx_state = RADIO_PERIPH_SEARCH;
			radio_rtc_clear_count();
			radio_rtc_compare0_set(RX_SEARCH_PERIOD);
			 m_rx_received = false;
		}
		else
		{		
			if(m_rx_received)
			{	
				 radio_rtc_clear_count();
				//JS Modify: 10/29/2020, switch to next rf chan for next round
				radio_rtc_compare0_set(RX_OPERATE_PERIOD);
				if(!m_rx_ack_wait)
				{
					nrf_esb_stop_rx();
				}
					nrf_esb_set_rf_channel(m_radio_chan_tab[chan_cnt]);	
					chan_cnt = (chan_cnt +1) % RADIO_CHAN_TAB_SIZE;	
				
					m_rx_ack_wait = false;
			}
			else
			{
				rx_loss_cnt++;
			}
			
			
			nrf_gpio_pin_set(DATA_SENDING_P4);
			//rx_loss_cnt++;
		}
		
		
		nrf_esb_start_rx();
		

	}
	
}



static void nrf_esb_ptx_event_handler(nrf_esb_evt_t const * p_event)
{	
//		uint16_t success_rate;
	
		switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
						NRF_LOG_DEBUG("TX SUCCESS EVENT");
				   
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            NRF_LOG_DEBUG("TX FAILED EVENT");
            (void) nrf_esb_flush_tx();
				
				
					
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
										 
            NRF_LOG_DEBUG("RX RECEIVED EVENT");
						
//aFH: Resets central loss cent & switch channel counter for particular peripheral if get rx received
						central_loss_cnt[chan_cnt] =0;
						//switch_channel_counter[periph_cnt] =0;
						update_ch_tab_flag = true;
				
						m_log_success_cnt[periph_cnt]++; //log
						
						nrf_gpio_pin_clear(debug_pins[periph_cnt]); //clear the debug pin if received data 
				
											
						m_event_callback(RADIO_CENTRAL_DATA_RECEIVED);		   
				
				
					break;
			
			
			default:
			
					break;
		}	
		
		if (p_event->evt_id != NRF_ESB_EVENT_TX_SUCCESS)   // TX_SUCCESS deplicates with DATA_RECEIVED
		{	
				periph_cnt = (periph_cnt +1) % NUM_OF_PERIPH; // periph_cnt = [0..(NUM_OF_PERIPH-1)]
			//JS Modify: 10/29/2020, switch to next rf chan for next round
			if(periph_cnt==0)
			{	
				chan_cnt = (chan_cnt +1) % RADIO_CHAN_TAB_SIZE;		
				nrf_esb_set_rf_channel(m_radio_chan_tab[chan_cnt]);
			}
			m_log_total_cnt[periph_cnt]++; //log
		}
		
		
		if  (m_log_total_cnt[periph_cnt]== LOG_CNT)		
		{		
			//success_rate = 	m_log_success_cnt[periph_cnt] *100 /m_log_total_cnt[periph_cnt] ;
			NRF_LOG_INFO("Success count for periph %d  = %d", periph_cnt+1, 	m_log_success_cnt[periph_cnt] );
			m_log_total_cnt[periph_cnt]=0;
			m_log_success_cnt[periph_cnt]=0;
		}
}


static void nrf_esb_prx_event_handler(nrf_esb_evt_t const * p_event)
{
	switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
							NRF_LOG_DEBUG("TX SUCCESS EVENT");
							
            break;
        case NRF_ESB_EVENT_TX_FAILED:
							NRF_LOG_DEBUG("TX FAILED EVENT");
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
							NRF_LOG_DEBUG("RX RECEIVED EVENT");
				
//							if (p_event->tx_attempts <= RETRAN_CNT)
//									m_rx_received = true;
				
							
							if (nrf_esb_read_rx_payload(&dum_payload) == NRF_SUCCESS  )          
							{								
								
									memcpy(m_radio_chan_tab, &dum_payload.data[1], RADIO_CHAN_TAB_SIZE);
									update_ch_tab_flag = true;
									//aFH: update channel switch counter and upload next channel tab
									//periph_sw_counter = dum_payload.data[0];									
									//if (periph_sw_counter >0)  update_ch_tab_flag = true;
//									if  ( (rx_state == RADIO_PERIPH_SEARCH)	&& dum_payload.data[0] ==0)
//									{
//											memcpy(m_radio_chan_tab, &dum_payload.data[1], RADIO_CHAN_TAB_SIZE);
//									}
//									else if (rx_state == RADIO_PERIPH_OPERATE)
//									{
//										periph_sw_counter = dum_payload.data[0];									
//									  if (periph_sw_counter >0)  
//										{	
//											update_ch_tab_flag = true;	
//											memcpy(next_radio_chan_tab, &dum_payload.data[1], RADIO_CHAN_TAB_SIZE);
//										}
//										else
//										{
//											memcpy(m_radio_chan_tab, &dum_payload.data[1], RADIO_CHAN_TAB_SIZE);
//										}
//									}
								
									nrf_esb_write_payload(&data_payload);
				
									m_event_callback(RADIO_PERIPH_DATA_SENT);
							}		
													
							rx_state = RADIO_PERIPH_WAIT_FOR_ACK_WR;
							radio_rtc_clear_count();
							radio_rtc_compare0_set(RX_WAIT_FOR_ACK_WR_PERIOD);
							rx_loss_cnt = 0;
				
					
							
				default: 
			
					break;
	}
}





static void radio_hfclk_start( void )
{
  //Start HFCLK
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}	



void radio_setup(bool is_central, radio_tx_power_t tx_power,  event_callback_t event_callback, uint8_t periph_num)	
{
	uint8_t addr_prefix[] = {0, 1};
	
	m_event_callback = event_callback;
	
	//aFH
	//Fill in the radio channel tables
	memcpy(m_radio_chan_tab, radio_table, RADIO_CHAN_TAB_SIZE); 
	memcpy(unused_radio_table, &radio_table[RADIO_CHAN_TAB_SIZE], RADIO_UNUSED_CHAN_TAB_SIZE);
	memcpy(next_radio_chan_tab, m_radio_chan_tab, RADIO_CHAN_TAB_SIZE); 
	
	nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
	nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
	nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_1MBPS_BLE;
	nrf_esb_config.event_handler            = (is_central)?nrf_esb_ptx_event_handler:nrf_esb_prx_event_handler;
	nrf_esb_config.mode                     = (is_central)?NRF_ESB_MODE_PTX:NRF_ESB_MODE_PRX;
	nrf_esb_config.selective_auto_ack       = false;
	nrf_esb_config.tx_output_power					= (nrf_esb_tx_power_t) tx_power;
	nrf_esb_config.retransmit_count					= RETRAN_CNT;
	nrf_esb_config.retransmit_delay					= 00;

	
	nrf_esb_init(&nrf_esb_config);
	
	nrf_esb_set_base_address_0(m_pair_address);
	nrf_esb_set_base_address_1(m_system_address);
	nrf_esb_set_rf_channel(m_radio_chan_tab[0]);
	
	if(!is_central)
	{
		addr_prefix[DATA_PIPE] = periph_num;
	}
	
	nrf_esb_set_prefixes(addr_prefix, sizeof(addr_prefix));
	
	
	nrf_esb_enable_pipes(  (1<< PAIRING_PIPE)  |  (1<< DATA_PIPE) );  //only enabled pipe 0 and 1
		
	
	on_radio_rtc_interrupt = (is_central)? rtc_tx_event_handler:rtc_rx_event_handler;
	

	radio_rtc_init();

	
	radio_hfclk_start();
		
	
}





void radio_poll_timer_start(uint32_t cnt)
{
	
	radio_rtc_compare0_set(cnt);
	
	radio_rtc_start();	
	
}



void radio_fetch_packet(radio_data_t * rcv_data)
{
	nrf_esb_payload_t	rx_payload;
	
	nrf_esb_read_rx_payload(&rx_payload);
	
	rcv_data->length = rx_payload.length;
	rcv_data->periph_num = nrf_esb_get_addr_prefix(rx_payload.pipe);
	memcpy(rcv_data->data, rx_payload.data, rx_payload.length);	
	
}	


void radio_scan_for_poll(void)
{
		uint32_t err_code;
	
		rx_state = RADIO_PERIPH_SEARCH;
	
		radio_rtc_compare0_set(RX_SEARCH_PERIOD);
	
		radio_rtc_start();
		
		err_code = nrf_esb_start_rx();
    APP_ERROR_CHECK(err_code);
		nrf_gpio_pin_set(DATA_SENDING_P4);
	
		is_rx_on = true;
	
}


void radio_put_packet(radio_data_t * tx_data)
{	
	data_payload.pipe = DATA_PIPE;
	data_payload.length = tx_data->length;
	memcpy(data_payload.data, tx_data->data, tx_data->length);
	
}
		
void RADIO_RTC_IRQHandler(void)
{
	if (RADIO_RTC->EVENTS_COMPARE[0] == 1)	
	{
     RADIO_RTC->EVENTS_COMPARE[0] =0;         //clear rtc compare event
	}
	
	   if(on_radio_rtc_interrupt)
       on_radio_rtc_interrupt();

	
}
