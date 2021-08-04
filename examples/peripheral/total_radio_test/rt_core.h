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
/** @file
 *
 * @defgroup ble_dtm DTM - Direct Test Mode
 * @{
 * @ingroup ble_sdk_lib
 * @brief Module for testing RF/PHY using DTM commands.
 */

#ifndef RT_CORE_H__
#define RT_CORE_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"


#define PACKET_TX_INTERVAL_MAX          1000UL
#define PACKET_TX_COUNT_MAX             1000000UL
#define PACKET_LENGTH_MAX               255
#define INTER_PACKETS_INTERVAL          249  /**< Ref. BT core spec > LE Test Packet Interval */

#define DTM_PKT_PRBS9                   0x00                                /**< Bit pattern PRBS9. */
#define DTM_PKT_0X0F                    0x01                                /**< Bit pattern 11110000 (LSB is the leftmost bit). */
#define DTM_PKT_0X55                    0x02                                /**< Bit pattern 10101010 (LSB is the leftmost bit). */
#define DTM_PKT_0XFF                    0x03 

#define IEEE_DEFAULT_FREQ               (5)   /**< IEEE 802.15.4 default frequency. */
#define IEEE_MIN_CHANNEL                11    /**< IEEE 802.15.4 minimum channel. */
#define IEEE_MAX_CHANNEL                26    /**< IEEE 802.15.4 maximum channel. */
#define IEEE_MAX_PAYLOAD_LEN            127   /**< IEEE 802.15.4 maximum payload length. */

#define IEEE_FREQ_CALC(_channel) (IEEE_DEFAULT_FREQ + \
                                  (IEEE_DEFAULT_FREQ * ((_channel) - IEEE_MIN_CHANNEL))) /**< Frequency calculation for a given channel in the IEEE 802.15.4 radio mode. */
#define BLE_FREQ_CALC(_channel) ((_channel << 1) + 2)

#define PRBS9_CONTENT  {0xFF, 0xC1, 0xFB, 0xE8, 0x4C, 0x90, 0x72, 0x8B,   \
                        0xE7, 0xB3, 0x51, 0x89, 0x63, 0xAB, 0x23, 0x23,   \
                        0x02, 0x84, 0x18, 0x72, 0xAA, 0x61, 0x2F, 0x3B,   \
                        0x51, 0xA8, 0xE5, 0x37, 0x49, 0xFB, 0xC9, 0xCA,   \
                        0x0C, 0x18, 0x53, 0x2C, 0xFD, 0x45, 0xE3, 0x9A,   \
                        0xE6, 0xF1, 0x5D, 0xB0, 0xB6, 0x1B, 0xB4, 0xBE,   \
                        0x2A, 0x50, 0xEA, 0xE9, 0x0E, 0x9C, 0x4B, 0x5E,   \
                        0x57, 0x24, 0xCC, 0xA1, 0xB7, 0x59, 0xB8, 0x87,   \
                        0xFF, 0xE0, 0x7D, 0x74, 0x26, 0x48, 0xB9, 0xC5,   \
                        0xF3, 0xD9, 0xA8, 0xC4, 0xB1, 0xD5, 0x91, 0x11,   \
                        0x01, 0x42, 0x0C, 0x39, 0xD5, 0xB0, 0x97, 0x9D,   \
                        0x28, 0xD4, 0xF2, 0x9B, 0xA4, 0xFD, 0x64, 0x65,   \
                        0x06, 0x8C, 0x29, 0x96, 0xFE, 0xA2, 0x71, 0x4D,   \
                        0xF3, 0xF8, 0x2E, 0x58, 0xDB, 0x0D, 0x5A, 0x5F,   \
                        0x15, 0x28, 0xF5, 0x74, 0x07, 0xCE, 0x25, 0xAF,   \
                        0x2B, 0x12, 0xE6, 0xD0, 0xDB, 0x2C, 0xDC, 0xC3,   \
                        0x7F, 0xF0, 0x3E, 0x3A, 0x13, 0xA4, 0xDC, 0xE2,   \
                        0xF9, 0x6C, 0x54, 0xE2, 0xD8, 0xEA, 0xC8, 0x88,   \
                        0x00, 0x21, 0x86, 0x9C, 0x6A, 0xD8, 0xCB, 0x4E,   \
                        0x14, 0x6A, 0xF9, 0x4D, 0xD2, 0x7E, 0xB2, 0x32,   \
                        0x03, 0xC6, 0x14, 0x4B, 0x7F, 0xD1, 0xB8, 0xA6,   \
                        0x79, 0x7C, 0x17, 0xAC, 0xED, 0x06, 0xAD, 0xAF,   \
                        0x0A, 0x94, 0x7A, 0xBA, 0x03, 0xE7, 0x92, 0xD7,   \
                        0x15, 0x09, 0x73, 0xE8, 0x6D, 0x16, 0xEE, 0xE1,   \
                        0x3F, 0x78, 0x1F, 0x9D, 0x09, 0x52, 0x6E, 0xF1,   \
                        0x7C, 0x36, 0x2A, 0x71, 0x6C, 0x75, 0x64, 0x44,   \
                        0x80, 0x10, 0x43, 0x4E, 0x35, 0xEC, 0x65, 0x27,   \
                        0x0A, 0xB5, 0xFC, 0x26, 0x69, 0x3F, 0x59, 0x99,   \
                        0x01, 0x63, 0x8A, 0xA5, 0xBF, 0x68, 0x5C, 0xD3,   \
                        0x3C, 0xBE, 0x0B, 0xD6, 0x76, 0x83, 0xD6, 0x57,   \
                        0x05, 0x4A, 0x3D, 0xDD, 0x81, 0x73, 0xC9, 0xEB,   \
                        0x8A, 0x84, 0x39, 0xF4, 0x36, 0x0B, 0xF7}           /**< The PRBS9 sequence used as packet payload.
                                                                                 The bytes in the sequence is in the right order, but the bits of each byte in the array is reverse.
                                                                                 of that found by running the PRBS9 algorithm. This is because of the endianess of the nRF5 radio. */

#define DTM_PDU_TYPE_PRBS9              0x00
#define DTM_PDU_TYPE_0X0F               0x01
#define DTM_PDU_TYPE_0X55               0x02
#define DTM_PDU_TYPE_0XFF               0x04

void     rt_radio_channel_set(uint32_t channel);
uint32_t rt_radio_channel_get(void);

void     rt_radio_mode_set(uint32_t mode);
uint32_t rt_radio_mode_get(void);

void     rt_radio_tx_power_set(uint32_t tx_power);
uint32_t rt_radio_tx_power_get(void);

void     rt_pkt_type_set(uint32_t type);
uint32_t rt_pkt_type_get(void);

void     rt_pkt_len_set(uint32_t len);
uint32_t rt_pkt_len_get(void);

void     rt_pkt_tx_count_max_set(uint32_t tx_count_max);
uint32_t rt_pkt_tx_count_max_get(void);
uint32_t rt_test_rx_count_get(void);

void     rt_pkt_tx_interval_set(uint32_t interval);
uint32_t rt_pkt_tx_interval_get(void);

void     rt_test_init(void);
void     rt_test_transmit(void);
void     rt_test_receive(void);
void     rt_test_stop_receiving(void);
uint32_t rt_test_rx_count_get(void);
void     rt_tx_done_handler_set(void (*handler)(void));
void     rt_config_validate(void);

#endif // RT_CORE_H__