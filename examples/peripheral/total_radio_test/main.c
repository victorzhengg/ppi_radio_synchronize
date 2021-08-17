/**
 * Copyright (c) 2012 - 2019, Nordic Semiconductor ASA
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
/**
  @defgroup dtm_standalone main.c
  @{
  @ingroup ble_sdk_app_dtm_serial
  @brief Stand-alone DTM application for UART interface.

 */

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "bsp.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_clock.h"

#include "app_uart.h"
#include "nrf_cli.h"
#include "nrf_cli_uart.h"

#include "rt_core.h"

#define CLI_EXAMPLE_LOG_QUEUE_SIZE  20
#define WELCOME_MESSAGE             "Start total radio test"

NRF_CLI_UART_DEF(m_cli_uart_transport, 0, 128, 128);
NRF_CLI_DEF(m_cli_uart,
            "cli:~$ ",
            &m_cli_uart_transport.transport,
            '\r',
            CLI_EXAMPLE_LOG_QUEUE_SIZE);

static void cli_start(void)
{
    ret_code_t ret;

    ret = nrf_cli_start(&m_cli_uart);
    APP_ERROR_CHECK(ret);
}

static void cli_init(void)
{
    ret_code_t ret;


    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;

    uart_config.pseltxd = TX_PIN_NUMBER;
    uart_config.pselrxd = RX_PIN_NUMBER;
    uart_config.hwfc    = NRF_UART_HWFC_DISABLED;
    ret                 = nrf_cli_init(&m_cli_uart, &uart_config, false, false, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK(ret);
}

int main(void)
{
    ret_code_t ret_code;

    // Use RTT output
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    // NOTE: app_timer is necessary by CLI_UART
    ret_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret_code);
    nrf_drv_clock_lfclk_request(NULL);
    ret_code = app_timer_init();
    APP_ERROR_CHECK(ret_code);

    cli_init();
    cli_start();

    nrf_cli_print(&m_cli_uart, WELCOME_MESSAGE);
    NRF_LOG_INFO(WELCOME_MESSAGE);

    rt_test_init();

    rt_radio_channel_set(11);
    rt_radio_mode_set(RADIO_MODE_MODE_Ble_1Mbit);
    rt_radio_tx_power_set(RADIO_TXPOWER_TXPOWER_0dBm);
    rt_pkt_type_set(DTM_PKT_PRBS9); 
    rt_pkt_len_set(10);
    rt_pkt_tx_count_max_set(100);
    rt_pkt_tx_interval_set(100);


    while (1)
    {
        NRF_LOG_PROCESS();
        nrf_cli_process(&m_cli_uart);   
    }
}

/// @}
