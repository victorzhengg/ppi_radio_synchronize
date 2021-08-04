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
#include "sdk_common.h"
#include <stdbool.h>
#include <string.h>
#include "nrf.h"
#include "nrf_cli.h"
#include "rt_core.h"

static bool m_tx_done;

static void cmd_dummy(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    nrf_cli_print(p_cli, "Dummy...");
}

static void cmd_set_radio_channel(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if ((argc == 1) || nrf_cli_help_requested(p_cli))
    {
        nrf_cli_help_print(p_cli, NULL, 0);
        return;
    }

    if (argc > 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameters count.\n", argv[0]);
        return;
    }

    uint32_t channel;

    channel = atoi(argv[1]);

    if (channel > 39)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Warning: channel should be:[0..39].\n");
        return;
    }

    rt_radio_channel_set(channel);
    rt_config_validate();
    channel = rt_radio_channel_get();

    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Radio channel set to: %d.\n", channel);
}

static void cmd_nrf_1mbit(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_mode_set(RADIO_MODE_MODE_Nrf_1Mbit);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, 
                    "Radio Mode: %s\n", 
                    STRINGIFY_(RADIO_MODE_MODE_Nrf_1Mbit));
}

static void cmd_nrf_2mbit(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_mode_set(RADIO_MODE_MODE_Nrf_2Mbit);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, 
                    "Radio Mode: %s\n", 
                    STRINGIFY_(RADIO_MODE_MODE_Nrf_2Mbit));
}

static void cmd_ble_1mbit(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_mode_set(RADIO_MODE_MODE_Ble_1Mbit);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, 
                    "Radio Mode: %s\n", 
                    STRINGIFY_(RADIO_MODE_MODE_Ble_1Mbit));
}

static void cmd_ble_2mbit(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_mode_set(RADIO_MODE_MODE_Ble_2Mbit);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, 
                    "Radio Mode: %s\n", 
                    STRINGIFY_(RADIO_MODE_MODE_Ble_2Mbit));
}

static void cmd_ble_lr125kbit(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_mode_set(RADIO_MODE_MODE_Ble_LR125Kbit);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, 
                    "Radio Mode: %s\n",
                    STRINGIFY_(RADIO_MODE_MODE_Ble_LR125Kbit));
}

static void cmd_ble_lr500kbit(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_mode_set(RADIO_MODE_MODE_Ble_LR500Kbit);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, 
                    "Radio Mode: %s\n",
                    STRINGIFY_(RADIO_MODE_MODE_Ble_LR500Kbit));
}

static void cmd_ble_ieee(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_mode_set(RADIO_MODE_MODE_Ieee802154_250Kbit);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, 
                    "Radio Mode: %s\n",
                    STRINGIFY_(RADIO_MODE_MODE_Ieee802154_250Kbit));
}

static void cmd_set_radio_mode(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if ((argc == 1) || nrf_cli_help_requested(p_cli))
    {
        nrf_cli_help_print(p_cli, NULL, 0);
        return;
    }

    if (argc > 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameters count.\n", argv[0]);
        return;
    }

    if (argc == 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Uknown argument: %s.\n", argv[1]);
        return;
    }
}

static void cmd_tx_power_0dbm(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_tx_power_set(RADIO_TXPOWER_TXPOWER_0dBm);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "TX Power Level: %s\n", STRINGIFY_(RADIO_TXPOWER_TXPOWER_0dBm));
}

static void cmd_tx_power_pos2dbm(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_tx_power_set(RADIO_TXPOWER_TXPOWER_Pos2dBm);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "TX Power Level: %s\n", STRINGIFY_(RADIO_TXPOWER_TXPOWER_Pos2dBm));
}

static void cmd_tx_power_pos3dbm(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_tx_power_set(RADIO_TXPOWER_TXPOWER_Pos3dBm);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "TX Power Level: %s\n", STRINGIFY_(RADIO_TXPOWER_TXPOWER_Pos3dBm));
}

static void cmd_tx_power_pos4dbm(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_tx_power_set(RADIO_TXPOWER_TXPOWER_Pos4dBm);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "TX Power Level: %s\n", STRINGIFY_(RADIO_TXPOWER_TXPOWER_Pos4dBm));
}

static void cmd_tx_power_pos5dbm(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_tx_power_set(RADIO_TXPOWER_TXPOWER_Pos5dBm);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "TX Power Level: %s\n", STRINGIFY_(RADIO_TXPOWER_TXPOWER_Pos5dBm));
}

static void cmd_tx_power_pos6dbm(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_tx_power_set(RADIO_TXPOWER_TXPOWER_Pos6dBm);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "TX Power Level: %s\n", STRINGIFY_(RADIO_TXPOWER_TXPOWER_Pos6dBm));
}

static void cmd_tx_power_pos7dbm(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_tx_power_set(RADIO_TXPOWER_TXPOWER_Pos7dBm);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "TX Power Level: %s\n", STRINGIFY_(RADIO_TXPOWER_TXPOWER_Pos7dBm));
}

static void cmd_tx_power_pos8dbm(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_tx_power_set(RADIO_TXPOWER_TXPOWER_Pos8dBm);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "TX Power Level: %s\n", STRINGIFY_(RADIO_TXPOWER_TXPOWER_Pos8dBm));
}

static void cmd_tx_power_neg4dbm(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_tx_power_set(RADIO_TXPOWER_TXPOWER_Neg4dBm);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "TX Power Level: %s\n", STRINGIFY_(RADIO_TXPOWER_TXPOWER_Neg4dBm));
}

static void cmd_tx_power_neg8dbm(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_tx_power_set(RADIO_TXPOWER_TXPOWER_Neg8dBm);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "TX Power Level: %s\n", STRINGIFY_(RADIO_TXPOWER_TXPOWER_Neg8dBm));
}

static void cmd_tx_power_neg12dbm(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_tx_power_set(RADIO_TXPOWER_TXPOWER_Neg12dBm);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "TX Power Level: %s\n", STRINGIFY_(RADIO_TXPOWER_TXPOWER_Neg12dBm));
}

static void cmd_tx_power_neg16dbm(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_tx_power_set(RADIO_TXPOWER_TXPOWER_Neg16dBm);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "TX Power Level: %s\n", STRINGIFY_(RADIO_TXPOWER_TXPOWER_Neg16dBm));
}

static void cmd_tx_power_neg20dbm(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_tx_power_set(RADIO_TXPOWER_TXPOWER_Neg20dBm);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "TX Power Level: %s\n", STRINGIFY_(RADIO_TXPOWER_TXPOWER_Neg20dBm));
}

static void cmd_tx_power_neg30dbm(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_tx_power_set(RADIO_TXPOWER_TXPOWER_Neg30dBm);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "TX Power Level: %s\n", STRINGIFY_(RADIO_TXPOWER_TXPOWER_Neg30dBm));
}

static void cmd_tx_power_neg40dbm(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_radio_tx_power_set(RADIO_TXPOWER_TXPOWER_Neg40dBm);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "TX Power Level: %s\n", STRINGIFY_(RADIO_TXPOWER_TXPOWER_Neg40dBm));
}

static void cmd_set_radio_tx_power(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if ((argc == 1) || nrf_cli_help_requested(p_cli))
    {
        nrf_cli_help_print(p_cli, NULL, 0);
        return;
    }

    if (argc > 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameters count.\n", argv[0]);
        return;
    }

    if (argc == 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Uknown argument: %s.\n", argv[1]);
        return;
    }
}

static void cmd_pkt_type_prbs9(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_pkt_type_set(DTM_PKT_PRBS9);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Packet Type: %s\n", STRINGIFY_(DTM_PKT_PRBS9));
}

static void cmd_pkt_type_0x0f(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_pkt_type_set(DTM_PKT_0X0F);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Packet Type: %s\n", STRINGIFY_(DTM_PKT_0X0F));
}

static void cmd_pkt_type_0x55(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_pkt_type_set(DTM_PKT_0X55);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Packet Type: %s\n", STRINGIFY_(DTM_PKT_0X55));
}

static void cmd_pkt_type_0xff(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    rt_pkt_type_set(DTM_PKT_0XFF);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Packet Type: %s\n", STRINGIFY_(DTM_PKT_0XFF));
}

static void cmd_set_packet_type(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if ((argc == 1) || nrf_cli_help_requested(p_cli))
    {
        nrf_cli_help_print(p_cli, NULL, 0);
        return;
    }

    if (argc > 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameters count.\n", argv[0]);
        return;
    }

    if (argc == 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Uknown argument: %s.\n", argv[1]);
        return;
    }
}

static void cmd_set_packet_length(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if ((argc == 1) || nrf_cli_help_requested(p_cli))
    {
        nrf_cli_help_print(p_cli, NULL, 0);
        return;
    }

    if (argc > 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameters count.\n", argv[0]);
        return;
    }

    uint32_t pkt_length = atoi(argv[1]);

    if (pkt_length > PACKET_LENGTH_MAX || pkt_length == 0)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Warning: packet length should be: [1..%d].\n", PACKET_LENGTH_MAX);
        return;
    }

    rt_pkt_len_set(pkt_length);
    rt_config_validate();
    pkt_length = rt_pkt_len_get();

    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Packet length is: %d.\n", pkt_length);
}

static void cmd_set_packet_max_count(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if ((argc == 1) || nrf_cli_help_requested(p_cli))
    {
        nrf_cli_help_print(p_cli, NULL, 0);
        return;
    }

    if (argc > 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameters count.\n", argv[0]);
        return;
    }

    uint32_t pkt_max_cnt = atoi(argv[1]);

    if (pkt_max_cnt > PACKET_TX_COUNT_MAX || pkt_max_cnt == 0)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Warning: packet max count should be [1..%d].\n", PACKET_TX_COUNT_MAX);
        return;
    }

    rt_pkt_tx_count_max_set(pkt_max_cnt);
    rt_config_validate();
    pkt_max_cnt = rt_pkt_tx_count_max_get();

    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Packet max count is: %d.\n", pkt_max_cnt);
}

static void cmd_set_packet_interval(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if ((argc == 1) || nrf_cli_help_requested(p_cli))
    {
        nrf_cli_help_print(p_cli, NULL, 0);
        return;
    }

    if (argc > 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameters count.\n", argv[0]);
        return;
    }

    uint32_t pkt_interval = atoi(argv[1]);

    if (pkt_interval > PACKET_TX_INTERVAL_MAX || pkt_interval == 0)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Warning: packet interval should be [1..%d].\n", PACKET_TX_INTERVAL_MAX);
        return;
    }

    rt_pkt_tx_interval_set(pkt_interval);
    rt_config_validate();
    pkt_interval = rt_pkt_tx_interval_get();

    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Packet interval is: %d(ms).\n", pkt_interval);
}


static void cmd_print_config(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    rt_config_validate();

    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "\n--------\n");
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, 
                    "Radio Channel: %d\n", 
                    rt_radio_channel_get());
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, 
                    "Radio Mode [0: NRF1M, 1: NRF2M, 3: BLE1M, 4: BLE2M, 5: LR125, 6: LR500, 15: 15.4]: %d\n", 
                    rt_radio_mode_get());

    int32_t tx_power = rt_radio_tx_power_get();
    if (tx_power > 8) 
    {
        tx_power = tx_power - 0x100;
    }
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Radio Tx Power : %d dBm\n", tx_power);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, 
                    "Packet Type [0: PRBS9, 1: 0x0F, 2: 0x55, 3: 0xFF]: %d\n", 
                    rt_pkt_type_get());
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, 
                    "Packet Length: %d\n",
                    rt_pkt_len_get());
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, 
                    "Packet Max Count: %d\n", 
                    rt_pkt_tx_count_max_get());
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, 
                    "Packet Interval(ms): %d\n", 
                    rt_pkt_tx_interval_get());
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "--------\n");
}


void tx_done_handler(void)
{
    m_tx_done = true;
}
static void cmd_start_tx(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    m_tx_done = false;
    rt_tx_done_handler_set(tx_done_handler);

    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "TX is running, please wait...\n");
    nrf_cli_process(p_cli); 

    rt_test_transmit();

    while (1)
    {
        if (m_tx_done)
        {
            m_tx_done = false;

            nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "TX sending is completed\n");
            nrf_cli_process(p_cli); 
            break;
        }
    }
}

static void cmd_start_rx(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    rt_test_receive();
}

static void cmd_stop_rx(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    rt_test_stop_receiving();
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Received packet number: %d.\n", rt_test_rx_count_get());
}

static void cmd_get_rx_count(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Received packet number: %d.\n", rt_test_rx_count_get());
}


NRF_CLI_CMD_REGISTER(dummy, NULL, "Dummy command", cmd_dummy);
NRF_CLI_CMD_REGISTER(radio_channel, NULL, "Set radio channel(0..39)", cmd_set_radio_channel);
NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_sub_radio_mode)
{
    NRF_CLI_CMD(nrf_1Mbit, NULL, 
                "1 Mbit/s Nordic proprietary radio mode[id=0]", 
                cmd_nrf_1mbit),
    NRF_CLI_CMD(nrf_2Mbit, NULL, 
                "2 Mbit/s Nordic proprietary radio mode[id=1]", 
                cmd_nrf_2mbit),
    NRF_CLI_CMD(ble_1Mbit, NULL, 
                "1 Mbit/s Bluetooth Low Energy[id=3]", 
                cmd_ble_1mbit),
    NRF_CLI_CMD(ble_2Mbit, NULL, 
                "2 Mbit/s Bluetooth Low Energy[id=4]", 
                cmd_ble_2mbit),
    NRF_CLI_CMD(ble_lr125Kbit, NULL,
                "Long range 125 kbit/s TX, 125 kbit/s and 500 kbit/s RX[id=5]",
                cmd_ble_lr125kbit),
    NRF_CLI_CMD(ble_lr500Kbit, NULL,
                "Long range 500 kbit/s TX, 125 kbit/s and 500 kbit/s RX[id=6]",
                cmd_ble_lr500kbit),
    NRF_CLI_CMD(ieee802154_250Kbit, NULL, 
                "IEEE 802.15.4-2006 250 kbit/s[id=15]", 
                cmd_ble_ieee),

    NRF_CLI_SUBCMD_SET_END
};
NRF_CLI_CMD_REGISTER(radio_mode, &m_sub_radio_mode, "Set radio mode <sub_cmd>", cmd_set_radio_mode);
NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_sub_radio_tx_power)
{
    NRF_CLI_CMD(power_0dbm, NULL, "TX Power is 0dBm", cmd_tx_power_0dbm),
    NRF_CLI_CMD(power_pos2dbm, NULL, "TX Power is 2dBm", cmd_tx_power_pos2dbm),
    NRF_CLI_CMD(power_pos3dbm, NULL, "TX Power is 3dBm", cmd_tx_power_pos3dbm),
    NRF_CLI_CMD(power_pos4dbm, NULL, "TX Power is 4dBm", cmd_tx_power_pos4dbm),
    NRF_CLI_CMD(power_pos5dbm, NULL, "TX Power is 5dBm", cmd_tx_power_pos5dbm),
    NRF_CLI_CMD(power_pos6dbm, NULL, "TX Power is 6dBm", cmd_tx_power_pos6dbm),
    NRF_CLI_CMD(power_pos7dbm, NULL, "TX Power is 7dBm", cmd_tx_power_pos7dbm),
    NRF_CLI_CMD(power_pos8dbm, NULL, "TX Power is 8dBm", cmd_tx_power_pos8dbm),
    NRF_CLI_CMD(power_neg4dbm, NULL, "TX Power is -4dBm", cmd_tx_power_neg4dbm),
    NRF_CLI_CMD(power_neg8dbm, NULL, "TX Power is -8dBm", cmd_tx_power_neg8dbm),
    NRF_CLI_CMD(power_neg12dbm, NULL, "TX Power is -12dBm", cmd_tx_power_neg12dbm),
    NRF_CLI_CMD(power_neg16dbm, NULL, "TX Power is -16dBm", cmd_tx_power_neg16dbm),
    NRF_CLI_CMD(power_neg20dbm, NULL, "TX Power is -20dBm", cmd_tx_power_neg20dbm),
    NRF_CLI_CMD(power_neg30dbm, NULL, "TX Power is -30dBm", cmd_tx_power_neg30dbm),
    NRF_CLI_CMD(power_neg40dbm, NULL, "TX Power is -40dBm", cmd_tx_power_neg40dbm),

    NRF_CLI_SUBCMD_SET_END
};
NRF_CLI_CMD_REGISTER(radio_tx_power, &m_sub_radio_tx_power, "Set radio tx power <sub_cmd>", cmd_set_radio_tx_power);

NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_sub_packet_type)
{
    NRF_CLI_CMD(PKT_PRBS9, NULL, "Packet type is PRBS9[id=0]", cmd_pkt_type_prbs9),
    NRF_CLI_CMD(PKT_0X0F, NULL,  "Packet type is 0x0F series[id=1]", cmd_pkt_type_0x0f),
    NRF_CLI_CMD(PKT_0X55, NULL,  "Packet type is 0x55 series[id=2]", cmd_pkt_type_0x55),
    NRF_CLI_CMD(PKT_0XFF, NULL,  "Packet type is 0xFF series[id=3]", cmd_pkt_type_0xff),

    NRF_CLI_SUBCMD_SET_END
};
NRF_CLI_CMD_REGISTER(packet_type, &m_sub_packet_type, "Set packet type <sub_cmd>", cmd_set_packet_type);

NRF_CLI_CMD_REGISTER(packet_length, NULL, "Set packet length(1..255)", cmd_set_packet_length);

NRF_CLI_CMD_REGISTER(packet_max_count, NULL, "Set packet max count(1..1,000,000)", cmd_set_packet_max_count);
NRF_CLI_CMD_REGISTER(packet_interval, NULL, "Set packet interval(1..1000 ms)", cmd_set_packet_interval);
NRF_CLI_CMD_REGISTER(print_config, NULL, "Print current config", cmd_print_config);
NRF_CLI_CMD_REGISTER(tx, NULL, "Start TX test", cmd_start_tx);
NRF_CLI_CMD_REGISTER(rx, NULL, "Start RX test", cmd_start_rx);
NRF_CLI_CMD_REGISTER(stop, NULL, "Stop RX test", cmd_stop_rx);
NRF_CLI_CMD_REGISTER(rx_count, NULL, "Report RX count", cmd_get_rx_count);

