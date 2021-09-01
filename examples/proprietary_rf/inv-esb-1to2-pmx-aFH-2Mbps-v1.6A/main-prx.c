/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
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

#include "nordic_common.h"
#include "bsp.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "app_util_platform.h"
#include "nrf_drv_clock.h"


#include "radio.h"
#include "radio_config.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define PERIPH_NUM   1//1..2

#define	PAYLOAD_SIZE	64  //bytes

static radio_data_t data_send;

uint8_t data_packet[] = {   1,  0,  3,  4,  5,  6,  7,  8,
														9, 10, 11, 12, 13, 14, 15, 16,
													 17, 18, 19, 20, 21, 22, 23, 24,
													 25, 26, 27, 28, 29, 30, 31, 32,
													 33, 34, 35, 36, 37, 38, 39, 40,
													 41, 42, 43, 44, 45, 46, 47, 48,
													 49, 50, 51, 52, 53, 54, 55, 56,
													 57, 58, 59, 60, 61, 62, 63, 64 };

static void clocks_start( void )
{
  uint32_t err_code = nrf_drv_clock_init();
  APP_ERROR_CHECK(err_code);
//nrf_drv_clock_hfclk_request(NULL);
  nrf_drv_clock_lfclk_request(NULL);
}



void gpio_init( void )
{
		nrf_gpio_range_cfg_output(8, 15);
    bsp_board_init(BSP_INIT_LEDS);
	
	  nrf_gpio_cfg_output(DATA_SENDING_P1);
	  nrf_gpio_cfg_output(DATA_SENDING_P2);
		nrf_gpio_cfg_output(DATA_SENDING_P3);
	  nrf_gpio_cfg_output(DATA_SENDING_P4);
	  nrf_gpio_pin_clear(DATA_SENDING_P1);
	  nrf_gpio_pin_clear(DATA_SENDING_P2);
		nrf_gpio_pin_clear(DATA_SENDING_P3);
		nrf_gpio_pin_clear(DATA_SENDING_P4);
	
}


void power_manage(void)
{
    __WFE();
    __SEV();
    __WFE();
}

void radio_evt_cb(uint8_t radio_event)
{
	if(radio_event == RADIO_PERIPH_DATA_SENT)
	{
		
		  // Set LEDs identical to the ones on the PTX.
			nrf_gpio_pin_write(LED_1, !(data_send.data[1]%8>0 && data_send.data[1]%8<=4));
			nrf_gpio_pin_write(LED_2, !(data_send.data[1]%8>1 && data_send.data[1]%8<=5));
			nrf_gpio_pin_write(LED_3, !(data_send.data[1]%8>2 && data_send.data[1]%8<=6));
			nrf_gpio_pin_write(LED_4, !(data_send.data[1]%8>3));
			
			data_send.data[1]++;
		
			radio_put_packet(&data_send);
		
	}

}


void data_init(void)
{
	data_send.length = PAYLOAD_SIZE;
	data_send.periph_num = PERIPH_NUM;
	memcpy(data_send.data, data_packet, PAYLOAD_SIZE);	
}

/**
 * @brief Function for application main entry.
 * @return 0. int return type required by ANSI/ISO standard.
 */

int main(void)
{
    ret_code_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

		gpio_init();
	
    clocks_start();
	
		data_init();
	
		radio_setup(false, RADIO_TX_POWER_4DBM, radio_evt_cb, PERIPH_NUM);

		radio_put_packet(&data_send);
	
		radio_scan_for_poll();
	

    while (true)
    {
        power_manage();
			
				UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    }
}
