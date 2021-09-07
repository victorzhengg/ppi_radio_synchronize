/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb.h"
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_delay.h"
#include "app_util.h"
#include <nrfx_clock.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


static nrf_esb_payload_t        rx_payload;


/**************************victor add start
*  Harware Resource: 
*  1. RTC1   used by app timer lib to generate 10ms tx interval    
*  2. TIMER1 used for generate the accurate time counter that less than 1ms
*     (1) the delay between imu frame send end_event to sync frame task_enable.
*         when the event_end of sync was set, the gpio was set to low.
*     (2) the delay between sync frame send end_event to set gpio high
*  3. PPI channel 
*/

#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"

/*imu frame data packets define*/
/*data0: frame type: 0 = imu  1 = sync*/
/*data1: sync flag:  0 = no sync frame followed  1 = sync frame followed*/
/*data2 ~ data6: reserved for future*/
/*data7: led1 indicator: 0~255*/

/*sync frame data packets define*/
/*data0: frame type: 0 = imu  1 = sync*/
/*data1 ~ data6: reserved for future*/
/*data7: led1 indicator: 0~255*/

#define PACK_TYPE_OFFSET        0
#define HAVE_SYNC_FLAG_OFFSET   1
#define LED_OFFSET              7

#define LED_DUTY_CNT       64

#define PACK_TYPE_NONE      0
#define PACK_TYPE_IMU       1
#define PACK_TYPE_SYNC      2

#define SYNC_FRAME_INTERVAL             APP_TIMER_TICKS(10)             /*radio tx interval 10ms*/
#define SYNC_TO_GPIO_HIGH_DELAY_US      500     /**/

#define     ULTRASONIC_PPI_GPIO_HIGH               7
#define     ULTRASONIC_PPI_TIMER_START             8
#define     ULTRASONIC_PPI_GPIO_LOW                9  

#define     ULTRASONIC_PPI_GPIO_PIN                3
#define     ULTRASONIC_GPIOTE_CH0                  0
#define     ULTRASONIC_GPIOTE_CH1                  1

uint8_t comm_flag = PACK_TYPE_IMU;
uint8_t sync_send_flag = 0;

static nrf_esb_payload_t        imu_payload = {
																								.length = 8,
																								.pipe = 0,
																								.noack = 1,
																								.data = {PACK_TYPE_IMU,0x00,0x03,0x04,0x05,0x06,0x07,0x00}  
                                              };
static nrf_esb_payload_t        sync_payload = {
																								.length = 8,
																								.pipe = 0,
																								.noack = 1,
																								.data = {PACK_TYPE_SYNC,0x00,0x03,0x04,0x05,0x06,0x07,0x00}  
																							 };



#define NRF_ESB_PPI_SYNC_CONFIG     {.protocol               = NRF_ESB_PROTOCOL_ESB_DPL,         \
																		 .mode                   = NRF_ESB_MODE_PTX,                 \
																		 .event_handler          = 0,                                \
																		 .bitrate                = NRF_ESB_BITRATE_2MBPS,            \
																		 .crc                    = NRF_ESB_CRC_16BIT,                \
																		 .tx_output_power        = NRF_ESB_TX_POWER_0DBM,            \
																		 .retransmit_delay       = 200,                                \
																		 .retransmit_count       = 1,                                \
																		 .tx_mode                = NRF_ESB_TXMODE_MANUAL,            \
																		 .radio_irq_priority     = 1,                                \
																		 .event_irq_priority     = 2,                                \
																		 .payload_length         = 32,                               \
																		 .selective_auto_ack     = true                             \
}


APP_TIMER_DEF(m_ultrasonic_timer_id);
const nrf_drv_timer_t ULTRASONIC_TIMER = NRF_DRV_TIMER_INSTANCE(1);

static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

void radio_tx_send_imu(void)
{
		if (nrf_esb_write_payload(&imu_payload) == NRF_SUCCESS)
		{
				nrf_esb_start_tx();
			
				/*debug only to indicate the link of communication is good*/
				nrf_gpio_pin_write(LED_1, (imu_payload.data[LED_OFFSET] < LED_DUTY_CNT));
				imu_payload.data[LED_OFFSET]++;
		}
		else
		{				  
				NRF_LOG_WARNING("Sending imu without sync packet failed");
		}
}


void radio_tx_send_sync(void)
{
		if (nrf_esb_write_payload(&sync_payload) == NRF_SUCCESS)
		{
				nrf_esb_start_tx();
			
				/*debug only to indicate the link of communication is good*/
				nrf_gpio_pin_write(LED_2, (sync_payload.data[LED_OFFSET] < LED_DUTY_CNT));
				sync_payload.data[LED_OFFSET]++;
		}
		else
		{				  
				NRF_LOG_WARNING("Sending sync packet failed");
		}
}

nrfx_err_t ultrasonic_ppi_gpio_init(void)
{
		uint32_t err_code;
	  nrfx_gpiote_out_config_t  config;
	

	
    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        VERIFY_SUCCESS(err_code);
    }	  

		nrf_gpiote_task_configure(ULTRASONIC_GPIOTE_CH0,
															ULTRASONIC_PPI_GPIO_PIN,
															NRF_GPIOTE_POLARITY_LOTOHI,
															NRF_GPIOTE_INITIAL_VALUE_LOW);
		
		nrf_gpiote_task_enable(ULTRASONIC_GPIOTE_CH0);

	  nrf_gpio_cfg(ULTRASONIC_PPI_GPIO_PIN,
								 NRF_GPIO_PIN_DIR_OUTPUT,
								 NRF_GPIO_PIN_INPUT_DISCONNECT,
								 NRF_GPIO_PIN_NOPULL,
								 NRF_GPIO_PIN_H0H1,
								 NRF_GPIO_PIN_NOSENSE);		
		
		nrfx_gpiote_out_init(ULTRASONIC_PPI_GPIO_PIN,&config);
		
		NRF_PPI->CH[ULTRASONIC_PPI_TIMER_START].EEP = (uint32_t)&NRF_RADIO->EVENTS_END;
    NRF_PPI->CH[ULTRASONIC_PPI_TIMER_START].TEP = (uint32_t)&ULTRASONIC_TIMER.p_reg->TASKS_START;
	  
		NRF_PPI->CH[ULTRASONIC_PPI_GPIO_HIGH].EEP = (uint32_t)&NRF_RADIO->EVENTS_END;
    NRF_PPI->CH[ULTRASONIC_PPI_GPIO_HIGH].TEP = (uint32_t)&NRF_GPIOTE->TASKS_SET[ULTRASONIC_GPIOTE_CH0];

    NRF_PPI->CH[ULTRASONIC_PPI_GPIO_LOW].EEP  = (uint32_t)&ULTRASONIC_TIMER.p_reg->EVENTS_COMPARE[0];
    NRF_PPI->CH[ULTRASONIC_PPI_GPIO_LOW].TEP  = (uint32_t)&NRF_GPIOTE->TASKS_CLR[ULTRASONIC_GPIOTE_CH0];
		
		return NRFX_SUCCESS;
}

void ultrasonic_ppi_ppi_enable(void)
{
    NRF_RADIO->EVENTS_END = 0;
    ULTRASONIC_TIMER.p_reg->EVENTS_COMPARE[0] = 0;

    NRF_PPI->CHENSET = (1 << ULTRASONIC_PPI_TIMER_START) |
                       (1 << ULTRASONIC_PPI_GPIO_HIGH) |
                       (1 << ULTRASONIC_PPI_GPIO_LOW);
    
    
}
void ultrasonic_ppi_ppi_disable(void)
{
		NRF_PPI->CHENCLR = (1 << ULTRASONIC_PPI_TIMER_START) |
											 (1 << ULTRASONIC_PPI_GPIO_HIGH) |
											 (1 << ULTRASONIC_PPI_GPIO_LOW);
}
uint32_t test_flag_cnt = 0;
void radio_tx_high_freq_timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
		switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:	
						ultrasonic_ppi_ppi_disable();
						test_flag_cnt++;
            break;
				
        default:
            //Do nothing.
            break;
    }
}
static void radio_tx_timer_init(void)
{
		ret_code_t err_code;
	  uint32_t cc0_ticks;
	
	  //Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&ULTRASONIC_TIMER, &timer_cfg, 
	                                 radio_tx_high_freq_timer_event_handler);
    APP_ERROR_CHECK(err_code);

	
    cc0_ticks = nrf_drv_timer_us_to_ticks(&ULTRASONIC_TIMER, SYNC_TO_GPIO_HIGH_DELAY_US);
	
    /*set CC0*/
    nrf_drv_timer_extended_compare(&ULTRASONIC_TIMER, NRF_TIMER_CC_CHANNEL0,
                                  	cc0_ticks,
                                   	NRF_TIMER_SHORT_COMPARE0_STOP_MASK | NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, 
	                                  true);    
}
/**@brief Function for handling the radio tx timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void radio_tx_low_freq_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
	
	  ultrasonic_ppi_ppi_disable();
	
	  comm_flag = PACK_TYPE_IMU;
		radio_tx_send_imu();
    
    if(sync_send_flag == 0)
		{
				sync_send_flag = 1;
		}
		else
		{
			  sync_send_flag = 0;
		}
}

/**@brief Function for the app timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void app_timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_ultrasonic_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                radio_tx_low_freq_timer_handler);
    APP_ERROR_CHECK(err_code);
}


/**********************************victor add end*/

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            NRF_LOG_DEBUG("TX SUCCESS EVENT");
				    if(comm_flag == PACK_TYPE_IMU)
						{
								if(sync_send_flag == 1)
								{
										ultrasonic_ppi_ppi_enable();
										comm_flag = PACK_TYPE_SYNC;
										radio_tx_send_sync();
								}
						}
						else
						{
						}
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            NRF_LOG_DEBUG("TX FAILED EVENT");
            (void) nrf_esb_flush_tx();
            (void) nrf_esb_start_tx();
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            NRF_LOG_DEBUG("RX RECEIVED EVENT");
            while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {
                if (rx_payload.length > 0)
                {
                    NRF_LOG_DEBUG("RX RECEIVED PAYLOAD");
                }
            }
            break;
    }
}


void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}


void gpio_init( void )
{
    nrf_gpio_range_cfg_output(8, 15);
    bsp_board_init(BSP_INIT_LEDS);
}


uint32_t esb_init( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

    nrf_esb_config_t nrf_esb_config         = NRF_ESB_PPI_SYNC_CONFIG;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;


    err_code = nrf_esb_init(&nrf_esb_config);

    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, NRF_ESB_PIPE_COUNT);
    VERIFY_SUCCESS(err_code);

    return err_code;
}


int main(void)
{
    ret_code_t err_code;

    gpio_init();
	
		ultrasonic_ppi_gpio_init();

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    clocks_start();


    err_code = esb_init();
    APP_ERROR_CHECK(err_code);
	
	  /*victor add new init*/
		lfclk_config();
		app_timers_init();
	
		radio_tx_timer_init();
	
    NRF_LOG_INFO("Enhanced ShockBurst Transmitter Example started.");
	
	  err_code = app_timer_start(m_ultrasonic_timer_id, SYNC_FRAME_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    while (true)
    {			
			NRF_LOG_PROCESS();
    }
}
