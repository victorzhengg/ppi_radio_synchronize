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
#include "nrf_esb.h"

#include <stdbool.h>
#include <stdint.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_error.h"
#include "boards.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

uint8_t led_nr;

nrf_esb_payload_t rx_payload;


/***********victor add start*/
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"


#define PACK_TYPE_OFFSET        0
#define HAVE_SYNC_FLAG_OFFSET   1
#define LED_OFFSET              7

#define LED_DUTY_CNT       64

#define PACK_TYPE_IMU       1
#define PACK_TYPE_SYNC      2

#define     ULTRASONIC_PPI_GPIO_PIN                3

#define     ULTRASONIC_GPIOTE_CH0                  0

 
#define     ULTRASONIC_PPI_CH_TIMER_START          7
#define     ULTRASONIC_PPI_CH_GPIO_LOW             8 
#define     ULTRASONIC_PPI_CH_GPIO_HIGH            9 

#define     EVENT_END_GPIO_LOW_DELAY_US            50
#define     GPIO_LOW_TO_HIGH_DELAY_US              EVENT_END_GPIO_LOW_DELAY_US + 50

const nrf_drv_timer_t ULTRASONIC_TIMER = NRF_DRV_TIMER_INSTANCE(1);

const nrf_drv_timer_t *p_ultrasonic_timer = &ULTRASONIC_TIMER;

nrfx_err_t ultrasonic_ppi_gpio_init(void)
{
		uint32_t err_code;

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        VERIFY_SUCCESS(err_code);
    }	  

		nrf_gpiote_task_configure(ULTRASONIC_GPIOTE_CH0,
															ULTRASONIC_PPI_GPIO_PIN,
															NRF_GPIOTE_POLARITY_HITOLO,
															NRF_GPIOTE_INITIAL_VALUE_HIGH);
		
		nrf_gpiote_task_enable(ULTRASONIC_GPIOTE_CH0);

	  nrf_gpio_cfg(ULTRASONIC_PPI_GPIO_PIN,
								 NRF_GPIO_PIN_DIR_OUTPUT,
								 NRF_GPIO_PIN_INPUT_DISCONNECT,
								 NRF_GPIO_PIN_NOPULL,
								 NRF_GPIO_PIN_H0H1,
								 NRF_GPIO_PIN_NOSENSE);		
		
		
		NRF_PPI->CH[ULTRASONIC_PPI_CH_TIMER_START].EEP = (uint32_t)&NRF_RADIO->EVENTS_END;
    NRF_PPI->CH[ULTRASONIC_PPI_CH_TIMER_START].TEP = (uint32_t)&ULTRASONIC_TIMER.p_reg->TASKS_START;
		
    NRF_PPI->CH[ULTRASONIC_PPI_CH_GPIO_LOW].EEP  = (uint32_t)&ULTRASONIC_TIMER.p_reg->EVENTS_COMPARE[0];
    NRF_PPI->CH[ULTRASONIC_PPI_CH_GPIO_LOW].TEP  = (uint32_t)&NRF_GPIOTE->TASKS_CLR[ULTRASONIC_GPIOTE_CH0];

    NRF_PPI->CH[ULTRASONIC_PPI_CH_GPIO_HIGH].EEP  = (uint32_t)&ULTRASONIC_TIMER.p_reg->EVENTS_COMPARE[1];
    NRF_PPI->CH[ULTRASONIC_PPI_CH_GPIO_HIGH].TEP  = (uint32_t)&NRF_GPIOTE->TASKS_SET[ULTRASONIC_GPIOTE_CH0];
		
		NRF_RADIO->EVENTS_END = 0;
    ULTRASONIC_TIMER.p_reg->EVENTS_COMPARE[0] = 0;
		ULTRASONIC_TIMER.p_reg->EVENTS_COMPARE[1] = 0;

    NRF_PPI->CHENSET = 1 << ULTRASONIC_PPI_CH_TIMER_START | 
		                   1 << ULTRASONIC_PPI_CH_GPIO_LOW |
											 1 << ULTRASONIC_PPI_CH_GPIO_HIGH;
		
		return NRFX_SUCCESS;
}
uint32_t test_isr_cnt = 0;
void radio_tx_high_freq_timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
		switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE1:	
						test_isr_cnt++;
            break;
				
        default:
            //Do nothing.
            break;
    }
}
static void radio_tx_high_freqtimer_init(void)
{
		ret_code_t err_code;
	  uint32_t cc0_ticks, cc1_ticks;
	
	  //Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&ULTRASONIC_TIMER, &timer_cfg, 
	                                 radio_tx_high_freq_timer_event_handler);
    APP_ERROR_CHECK(err_code);

	
    cc0_ticks = nrf_drv_timer_us_to_ticks(&ULTRASONIC_TIMER, EVENT_END_GPIO_LOW_DELAY_US);
		cc1_ticks = nrf_drv_timer_us_to_ticks(&ULTRASONIC_TIMER, GPIO_LOW_TO_HIGH_DELAY_US);
	
    /*set CC0*/
    nrf_drv_timer_extended_compare(&ULTRASONIC_TIMER, NRF_TIMER_CC_CHANNEL0,
                                  	cc0_ticks,
                                   	0,
	                                  false);
    nrf_drv_timer_extended_compare(&ULTRASONIC_TIMER, NRF_TIMER_CC_CHANNEL1,
                                  	cc1_ticks,
                                   	NRF_TIMER_SHORT_COMPARE1_STOP_MASK | NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK, 
	                                  true); 	
}
/***********victor add end*/


/*lint -save -esym(40, BUTTON_1) -esym(40, BUTTON_2) -esym(40, BUTTON_3) -esym(40, BUTTON_4) -esym(40, LED_1) -esym(40, LED_2) -esym(40, LED_3) -esym(40, LED_4) */

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
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
            if (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {
                /*victor modify start*/
							  if(rx_payload.data[PACK_TYPE_OFFSET] == PACK_TYPE_IMU)
								{
										nrf_gpio_pin_write(LED_1, (rx_payload.data[LED_OFFSET] < LED_DUTY_CNT));
								}
								else /*PACK_TYPE_SYNC*/
								{
										nrf_gpio_pin_write(LED_2, (rx_payload.data[LED_OFFSET] < LED_DUTY_CNT));
								}
                

                NRF_LOG_DEBUG("Receiving packet: %02x", rx_payload.data[1]);
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
    bsp_board_init(BSP_INIT_LEDS);
}


uint32_t esb_init( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };
    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.payload_length           = 8;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PRX;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.selective_auto_ack       = true;

    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    VERIFY_SUCCESS(err_code);

    return err_code;
}


int main(void)
{
    uint32_t err_code;

    gpio_init();

		ultrasonic_ppi_gpio_init();  /*victor add */
	
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    clocks_start();
	
	  radio_tx_high_freqtimer_init();  /*victor add */

    err_code = esb_init();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEBUG("Enhanced ShockBurst Receiver Example started.");

    err_code = nrf_esb_start_rx();
    APP_ERROR_CHECK(err_code);

    while (true)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            __WFE();
        }
    }
}
/*lint -restore */
