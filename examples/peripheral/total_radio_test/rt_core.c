#include <stdbool.h>
#include <string.h>

#include "nrf.h"
#include "sdk_common.h"
#include "ble_dtm_hw.h"

#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "rt_core.h"

#define E172_TIMER                          NRF_TIMER1                       /**< Timer used for the workaround for errata 172 on affected nRF5 devices. */
#define E172_TIMER_IRQn                     TIMER1_IRQn                      /**< IRQ used for timer. NOTE: MUST correspond to ERRATA_172_TIMER. */
#define E172_TIMER_IRQHandler               TIMER1_IRQHandler                /**< IRQHandler used for timer. NOTE: MUST correspond to ERRATA_172_TIMER. */
#define BLOCKER_FIX_RSSI_THRESHOLD          95                               /**< The RSSI threshold at which to toggle strict mode. */
#define BLOCKER_FIX_WAIT_DEFAULT            1250                             /**< 1250 * 8 = 10000 us = 10 ms. */
#define BLOCKER_FIX_WAIT_END                63                               /**< 63 * 8 = ~500us. */
#define BLOCKER_FIX_CNTDETECTTHR            15                               /**< Threshold used to determine necessary strict mode status changes. */
#define BLOCKER_FIX_CNTADDRTHR              2                                /**< 1250 * 8 = 10000 us = 10 ms. */

#define DTM_HEADER_SIZE                     2                                         /**< Size of PDU header. */
#define DTM_PAYLOAD_MAX_SIZE                255                                       /**< Maximum payload size allowed during dtm execution. */
#define DTM_PDU_MAX_MEMORY_SIZE             (DTM_HEADER_SIZE + DTM_PAYLOAD_MAX_SIZE)  /**< Maximum PDU size allowed during dtm execution. */
#define IEEE_PDU_MAX_SIZE                   (IEEE_MAX_PAYLOAD_LEN - 2)                /**< Ref. PS. 6.20.12.1 */


typedef enum
{
    STATE_UNINITIALIZED,                                                     /**< The DTM is uninitialized. */
    STATE_IDLE,                                                              /**< State when system has just initialized, or current test has completed. */
    STATE_TRANSMITTER_TEST,                                                  /**< State used when a DTM Transmission test is running. */
    STATE_CARRIER_TEST,                                                      /**< State used when a DTM Carrier test is running (Vendor specific test). */
    STATE_RECEIVER_TEST                                                      /**< State used when a DTM Receive test is running. */
} state_t;

typedef struct
{
    uint8_t content[DTM_PDU_MAX_MEMORY_SIZE];                 /**< PDU packet content. */
} pdu_type_t;

static uint8_t const     m_prbs_content[] = PRBS9_CONTENT;

static bool        m_strict_mode;
static pdu_type_t  m_pdu;

static uint32_t    m_radio_mode;
static uint32_t    m_radio_tx_power;
static uint32_t    m_radio_channel;

static uint32_t    m_pkt_type;
static uint32_t    m_pkt_len;
static uint32_t    m_pkt_tx_cnt_max;
static uint32_t    m_pkt_tx_interval;

static uint32_t    m_pkt_rx_cnt;
static uint32_t    m_pkt_tx_cnt;

static state_t     m_test_state;
static bool        m_radio_tx_ready;

static bool        anomaly_172_wa_enabled;

static void        (*m_tx_done_cb)(void);

static bool check_pdu(void)
{
    uint8_t        k;                // Byte pointer for running through PDU payload
    uint8_t        pattern;          // Repeating octet value in payload
    uint8_t        pdu_packet_type;  // Note: PDU packet type is a 4-bit field in HCI, but 2 bits in BLE DTM
    uint32_t       length = 0;

    if (m_radio_mode == RADIO_MODE_MODE_Ieee802154_250Kbit)
    {
        length = m_pdu.content[0] - 2;
        return (memcmp(m_pdu.content + 1, m_prbs_content, length) == 0);
    }

    pdu_packet_type = (m_pdu.content[0] & 0x0F);
    length          = m_pdu.content[1];

    // Check that the length is valid.
    if (length > DTM_PAYLOAD_MAX_SIZE)
    {
        return false;
    }

    // If the 1Mbit or 2Mbit radio mode is active, check that one of the three valid uncoded DTM packet types are selected.
    if ((m_radio_mode == RADIO_MODE_MODE_Ble_1Mbit || m_radio_mode == RADIO_MODE_MODE_Ble_2Mbit) && (pdu_packet_type > DTM_PKT_0X55))
    {
        return false;
    }

#if defined(NRF52840_XXAA) || defined(NRF52833_XXAA) || defined(NRF52811_XXAA)
    // If a long range radio mode is active, check that one of the four valid coded DTM packet types are selected.
    if ((m_radio_mode == RADIO_MODE_MODE_Ble_LR500Kbit || m_radio_mode == RADIO_MODE_MODE_Ble_LR125Kbit) && (pdu_packet_type > DTM_PKT_0XFF))
    {
        return false;
    }
#endif //defined(NRF52840_XXAA) || defined(NRF52833_XXAA) || defined(NRF52811_XXAA)

    if (pdu_packet_type == DTM_PKT_PRBS9)
    {
        // Payload does not consist of one repeated octet; must compare ir with entire block into
        return (memcmp(m_pdu.content + DTM_HEADER_SIZE, m_prbs_content, length) == 0);
    }

    if (pdu_packet_type == DTM_PKT_0X0F)
    {
        pattern = 0x0F;
    }
    else if (pdu_packet_type == DTM_PKT_0X55)
    {
        pattern = 0x55;
    }
    else if (pdu_packet_type == DTM_PKT_0XFF)
    {
        pattern = 0xFF;
    }
    else
    {
        return false;
    }

    for (k = 0; k < length; k++)
    {
        if (m_pdu.content[k + 2] != pattern)
        {
            return false;
        }
    }
    return true;
}

// Strict mode setting will be used only by devices affected by nRF52840 anomaly 172
void set_strict_mode (bool enable)
{
   uint8_t dbcCorrTh;
   uint8_t dsssMinPeakCount;
   if (enable == true)
   {
      dbcCorrTh = 0x7d;
      dsssMinPeakCount = 6;
      *(volatile uint32_t *) 0x4000173c = ((*((volatile uint32_t *) 0x4000173c)) & 0x7FFFFF00) | 0x80000000 | (((uint32_t)(dbcCorrTh)) << 0);
      *(volatile uint32_t *) 0x4000177c = ((*((volatile uint32_t *) 0x4000177c)) & 0x7FFFFF8F) | 0x80000000 | ((((uint32_t)dsssMinPeakCount) & 0x00000007) << 4);
   }
   else
   {
      *(volatile uint32_t *) 0x4000173c = 0x40003034;
      *(volatile uint32_t *) 0x4000177c = ((*((volatile uint32_t *) 0x4000177c)) & 0x7FFFFFFF); // Unset override of dsssMinPeakCount
   }

   m_strict_mode = true;
}

// Radio configuration used as a workaround for nRF52840 anomaly 172
void anomaly_172_radio_operation(void)
{
    *(volatile uint32_t *) 0x40001040 = 1;
    *(volatile uint32_t *) 0x40001038 = 1;
}

uint8_t compensate_rssi_for_temp(uint8_t rssi_sample)
{
    int32_t temp;
    NRF_TEMP->TASKS_START = 1;
    while (NRF_TEMP->EVENTS_DATARDY == 0) {}

    NRF_TEMP->EVENTS_DATARDY = 0;
    temp = NRF_TEMP->TEMP;
    NRF_TEMP->TASKS_STOP = 1;

    // temp is in 0.25 increments (times 4)
    if (temp <= -120) // <= -30 degree
    {
        rssi_sample += 3;
    }
    else if (temp <= -40) // <=-10 degree
    {
        rssi_sample += 2;
    }
    else if (temp <= 40) // <= 10 degree
    {
        rssi_sample += 1;
    }
    else if (temp <= 120) // <= 30 degree
    {
        rssi_sample += 0;
    }
    else if (temp <= 200) // <= 50 degree
    {
        rssi_sample -= 1;
    }
    else if (temp <= 280) // <= 70 degree
    {
        rssi_sample -= 2;
    }
    else // > 70 degree
    {
        rssi_sample -= 3;
    }
    return rssi_sample;
}


// Function to gather RSSI data and set strict mode accordingly. Used as part of the workaround for nRF52840 anomaly 172
uint8_t anomaly_172_rssi_check(void)
{
    NRF_RADIO->EVENTS_RSSIEND = 0;
    NRF_RADIO->TASKS_RSSISTART = 1;
    while (NRF_RADIO->EVENTS_RSSIEND == 0);
    uint8_t rssi = NRF_RADIO->RSSISAMPLE;

    // Adjust RSSI to compensate for anomaly 153.
    rssi = compensate_rssi_for_temp(rssi);

    return rssi;
}


static void enable_anomaly_191(void)
{
#ifdef NRF52840_XXAA
    *(volatile uint32_t *) 0x40001740 = ((*((volatile uint32_t *) 0x40001740)) & 0x7FFF00FF) | 0x80000000 | (((uint32_t)(196)) << 8);
#endif
}

static void disable_anomaly_191(void)
{
#ifdef NRF52840_XXAA
    *(volatile uint32_t *) 0x40001740 = ((*((volatile uint32_t *) 0x40001740)) & 0x7FFFFFFF);
#endif
}

static bool anomaly_172_check(void)
{
#ifdef NRF52840_XXAA
    return ((*(volatile uint32_t *)0x40001788) == 0);
#else
    return false;
#endif
}


void rt_radio_channel_set(uint32_t channel)
{
    m_radio_channel = channel;
}

uint32_t rt_radio_channel_get(void)
{
    return m_radio_channel;
}

void rt_radio_mode_set(uint32_t mode)
{
    m_radio_mode = mode;
}

uint32_t rt_radio_mode_get(void)
{
    return m_radio_mode;
}

void rt_radio_tx_power_set(uint32_t tx_power)
{
    m_radio_tx_power = tx_power;
}

uint32_t rt_radio_tx_power_get(void)
{
    return m_radio_tx_power;
}

void rt_pkt_type_set(uint32_t type)
{
    uint32_t val = type;
    if ((type != DTM_PKT_PRBS9) &&
        (type != DTM_PKT_0X0F)  &&
        (type != DTM_PKT_0X55)  &&
        (type != DTM_PKT_0XFF))
    {
        val = DTM_PKT_PRBS9;
    }
    m_pkt_type = val;
}

uint32_t rt_pkt_type_get(void)
{
    return m_pkt_type;
}

void rt_pkt_len_set(uint32_t len)
{
    m_pkt_len = len;
}

uint32_t rt_pkt_len_get(void)
{
    return m_pkt_len;
}

void rt_pkt_tx_count_max_set(uint32_t tx_count_max)
{
    m_pkt_tx_cnt_max = tx_count_max;
}

uint32_t rt_pkt_tx_count_max_get(void)
{
    return m_pkt_tx_cnt_max;
}

uint32_t rt_test_rx_count_get(void)
{
    NRF_LOG_INFO("RX count is: %ld", m_pkt_rx_cnt);

    return m_pkt_rx_cnt;
}

void rt_pkt_tx_interval_set(uint32_t interval)
{
    m_pkt_tx_interval = interval;
}

uint32_t rt_pkt_tx_interval_get(void)
{
    return m_pkt_tx_interval;
}

void rt_tx_done_handler_set(void (*handler)(void))
{
    m_tx_done_cb = handler;
}

static void rt_timer_init(void)
{
    NRF_TIMER0->TASKS_STOP          = 1;
    NRF_TIMER0->TASKS_CLEAR         = 1;
    NRF_TIMER0->MODE                = TIMER_MODE_MODE_Timer;
    NRF_TIMER0->EVENTS_COMPARE[0]   = 0;
    NRF_TIMER0->INTENSET            = TIMER_INTENSET_COMPARE0_Msk;
    NRF_TIMER0->SHORTS              = (1 << TIMER_SHORTS_COMPARE0_CLEAR_Pos);

    // f_timer = 16 MHz / (2 ^ prescaler) ==> interval = 1us
    NRF_TIMER0->PRESCALER           = 4;
    // Enable 32bit counter to avoid overflow
    NRF_TIMER0->BITMODE             = 3;
    NRF_TIMER0->CC[0]               = 0;
    NRF_TIMER0->CC[1]               = 0;
}

static void rt_errata_timer_init(void)
{
    E172_TIMER->TASKS_STOP          = 1;
    E172_TIMER->TASKS_CLEAR         = 1;
    E172_TIMER->MODE                = TIMER_MODE_MODE_Timer;
    E172_TIMER->EVENTS_COMPARE[0]   = 0;
    E172_TIMER->EVENTS_COMPARE[1]   = 0;

    E172_TIMER->CC[0] = BLOCKER_FIX_WAIT_DEFAULT;
    E172_TIMER->CC[1] = 0;

    NVIC_ClearPendingIRQ(E172_TIMER_IRQn);

    E172_TIMER->PRESCALER   = 7;
}

void rt_test_init(void)
{
    // Enable HFCLK
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {}

    rt_timer_init();
    rt_errata_timer_init();

    NVIC_EnableIRQ(TIMER0_IRQn);
    NVIC_EnableIRQ(TIMER1_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);

//    NVIC_SetPriority(RADIO_IRQn, 2);

    __enable_irq();

    // Enable wake-up on event
    SCB->SCR |= SCB_SCR_SEVONPEND_Msk;

#if defined(NRF52832_XXAA) || defined(NRF52840_XXAA) || defined(NRF52833_XXAA)
    // Enable cache
    NRF_NVMC->ICACHECNF = (NVMC_ICACHECNF_CACHEEN_Enabled << NVMC_ICACHECNF_CACHEEN_Pos) & NVMC_ICACHECNF_CACHEEN_Msk;
#endif

    disable_anomaly_191();

    rt_radio_mode_set(RADIO_MODE_MODE_Ble_1Mbit);
    rt_radio_channel_set(11);
    rt_radio_tx_power_set(RADIO_TXPOWER_TXPOWER_0dBm);

    rt_pkt_type_set(DTM_PKT_PRBS9);
    rt_pkt_len_set(10);
    rt_pkt_tx_count_max_set(100);
    rt_pkt_tx_interval_set(5);

    m_test_state            = STATE_UNINITIALIZED;
    m_pkt_rx_cnt            = 0;
    m_pkt_tx_cnt            = 0;
}

static void rt_radio_setup(void)
{
    uint32_t m_address     = 0x71764129;

    NRF_RADIO->TXPOWER     = m_radio_tx_power;
    NRF_RADIO->MODE        = m_radio_mode;

    NRF_RADIO->MODECNF0    = (RADIO_MODECNF0_RU_Default << RADIO_MODECNF0_RU_Pos);

    // Set the access address, address0/prefix0 used for both Rx and Tx address
    NRF_RADIO->PREFIX0    &= ~RADIO_PREFIX0_AP0_Msk;
    NRF_RADIO->PREFIX0    |= (m_address >> 24) & RADIO_PREFIX0_AP0_Msk;
    NRF_RADIO->BASE0       = m_address << 8;
    NRF_RADIO->RXADDRESSES = RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos;
    NRF_RADIO->TXADDRESS   = (0x00 << RADIO_TXADDRESS_TXADDRESS_Pos) & RADIO_TXADDRESS_TXADDRESS_Msk;

    NRF_RADIO->EVENTS_READY = 0;

    NRF_RADIO->CRCPOLY      = 0x0000065B;
    NRF_RADIO->CRCINIT      = 0x00555555;
    NRF_RADIO->PACKETPTR    = (uint32_t)&m_pdu;                      // Setting packet pointer will start the radio

    // Configure CRC calculation
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIP_ADDR_Pos) |
                        (RADIO_CRCCNF_LEN_Three     << RADIO_CRCCNF_LEN_Pos);

    NRF_RADIO->PCNF1  = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos) |
                        (RADIO_PCNF1_ENDIAN_Little    << RADIO_PCNF1_ENDIAN_Pos)  |
                        (3                            << RADIO_PCNF1_BALEN_Pos)   |
                        (0                            << RADIO_PCNF1_STATLEN_Pos) |
                        (DTM_PAYLOAD_MAX_SIZE         << RADIO_PCNF1_MAXLEN_Pos);

    if (m_radio_mode == RADIO_MODE_MODE_Ble_1Mbit)
    {
        NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S1LEN_Pos) |
                           (1 << RADIO_PCNF0_S0LEN_Pos) |
                           (8 << RADIO_PCNF0_LFLEN_Pos) |
                           (RADIO_PCNF0_PLEN_8bit  << RADIO_PCNF0_PLEN_Pos);
    }
    else if (m_radio_mode == RADIO_MODE_MODE_Ble_2Mbit)
    {
        NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S1LEN_Pos) |
                           (1 << RADIO_PCNF0_S0LEN_Pos) |
                           (8 << RADIO_PCNF0_LFLEN_Pos) |
                           (RADIO_PCNF0_PLEN_16bit << RADIO_PCNF0_PLEN_Pos);
    }
    else if (m_radio_mode == RADIO_MODE_MODE_Ble_LR500Kbit ||
             m_radio_mode == RADIO_MODE_MODE_Ble_LR125Kbit)
    {
        NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S1LEN_Pos)   |
                           (1 << RADIO_PCNF0_S0LEN_Pos)   |
                           (8 << RADIO_PCNF0_LFLEN_Pos)   |
                           (3 << RADIO_PCNF0_TERMLEN_Pos) |
                           (2 << RADIO_PCNF0_CILEN_Pos)   |
                           (RADIO_PCNF0_PLEN_LongRange << RADIO_PCNF0_PLEN_Pos);

        NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_RU_Default << RADIO_MODECNF0_RU_Pos) |
                              (RADIO_MODECNF0_DTX_Center << RADIO_MODECNF0_DTX_Pos);
    }
    else if (m_radio_mode == RADIO_MODE_MODE_Ieee802154_250Kbit)
    {
        // REF: https://devzone.nordicsemi.com/f/nordic-q-a/34460/802-15-4-test-dev-migration-to-52840/134750#134750

        NRF_RADIO->PCNF0 = (8                          << RADIO_PCNF0_LFLEN_Pos) |
                           (RADIO_PCNF0_PLEN_32bitZero << RADIO_PCNF0_PLEN_Pos)  |
                           (RADIO_PCNF0_CRCINC_Exclude << RADIO_PCNF0_CRCINC_Pos);

        NRF_RADIO->PCNF1 = (IEEE_MAX_PAYLOAD_LEN << RADIO_PCNF1_MAXLEN_Pos);

        NRF_RADIO->CRCCNF = (RADIO_CRCCNF_SKIPADDR_Ieee802154 << RADIO_CRCCNF_SKIP_ADDR_Pos) |
                            (RADIO_CRCCNF_LEN_Two             << RADIO_CRCCNF_LEN_Pos);

        NRF_RADIO->CRCPOLY = 0x11021UL;
        NRF_RADIO->CRCINIT = 0;

        NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_RU_Fast    << RADIO_MODECNF0_RU_Pos) |
                              (RADIO_MODECNF0_DTX_Center << RADIO_MODECNF0_DTX_Pos);
    }
    else if ((m_radio_mode == RADIO_MODE_MODE_Nrf_1Mbit) ||
             (m_radio_mode == RADIO_MODE_MODE_Nrf_2Mbit))
    {
        NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S1LEN_Pos) |
                           (0 << RADIO_PCNF0_S0LEN_Pos) |
                           (8 << RADIO_PCNF0_LFLEN_Pos) |
                           (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos);
    }
    else
    {
        NRF_LOG_ERROR("Unknown radio mode");
    }

    // Handle errata
    if (m_radio_mode == RADIO_MODE_MODE_Ble_LR500Kbit ||
        m_radio_mode == RADIO_MODE_MODE_Ble_LR125Kbit)
    {
        enable_anomaly_191();

        // Enable the workaround for nRF52840 anomaly 172 on affected devices.
        if (anomaly_172_check())
        {
            anomaly_172_wa_enabled = true;

            NRF_LOG_INFO("172 workaround is enabled");
        }
    }
    else
    {
        disable_anomaly_191();

        // Disable the workaround for nRF52840 anomaly 172.
        set_strict_mode(0);
        E172_TIMER->TASKS_SHUTDOWN = 1;
        anomaly_172_wa_enabled = false;
    }

    if (m_radio_mode == RADIO_MODE_MODE_Ieee802154_250Kbit)
    {
        NRF_RADIO->FREQUENCY  = IEEE_FREQ_CALC(m_radio_channel);
    }
    else
    {
        NRF_RADIO->FREQUENCY  = BLE_FREQ_CALC(m_radio_channel);
    }

    if (m_radio_mode == RADIO_MODE_MODE_Ieee802154_250Kbit ||
        m_radio_mode == RADIO_MODE_MODE_Ble_LR125Kbit      ||
        m_radio_mode == RADIO_MODE_MODE_Ble_LR500Kbit)
    {
        NRF_RADIO->SHORTS = (1 << RADIO_SHORTS_READY_START_Pos) |
                            (1 << RADIO_SHORTS_PHYEND_DISABLE_Pos);
    }
    else
    {
        NRF_RADIO->SHORTS = (1 << RADIO_SHORTS_READY_START_Pos) |
                            (1 << RADIO_SHORTS_END_DISABLE_Pos);
    }
}


static void rt_radio_reset(void)
{
    NRF_LOG_INFO("radio reset");

    m_test_state = STATE_IDLE;

    NVIC_ClearPendingIRQ(RADIO_IRQn);

    NRF_RADIO->SHORTS          = 0;
    NRF_RADIO->TASKS_RXEN      = 0;
    NRF_RADIO->TASKS_TXEN      = 0;

    NRF_RADIO->INTENSET        = 0;
    NRF_RADIO->INTENCLR        = (RADIO_INTENCLR_ADDRESS_Msk) |
                                 (RADIO_INTENCLR_END_Msk)     |
                                 (RADIO_INTENCLR_READY_Msk)   |
                                 (RADIO_INTENCLR_DISABLED_Msk);
    NRF_RADIO->TASKS_DISABLE   = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0) {;}

    NRF_RADIO->EVENTS_DISABLED = 0;
}

static void rt_transmit_complete(void)
{
    NRF_LOG_INFO("TX complete");

    NRF_TIMER0->EVENTS_COMPARE[0] = 0;
    NVIC_ClearPendingIRQ(TIMER0_IRQn);

    NRF_TIMER0->TASKS_STOP        = 1;                      // Stop timer, if it was running
    NRF_TIMER0->TASKS_CLEAR       = 1;
    E172_TIMER->TASKS_SHUTDOWN    = 1;

    if (m_tx_done_cb != NULL)
    {
        m_tx_done_cb();
    }

    m_test_state     = STATE_IDLE;
    m_pkt_tx_cnt     = 0;
}

static uint32_t rt_test_interval_cal(uint32_t radio_mode, uint32_t pkt_len)
{
    // Refer to DTM project > dtm_packet_interval_calculate()

    uint32_t bit_in_air = 0;
    uint32_t packet_interval    = 0; // us
    uint32_t overhead_bits      = 0; // bits

   if (radio_mode == RADIO_MODE_MODE_Ble_2Mbit || radio_mode == RADIO_MODE_MODE_Nrf_2Mbit)
    {
        overhead_bits = 88; // 11 bytes
    }
    else if (radio_mode == RADIO_MODE_MODE_Ble_1Mbit || radio_mode == RADIO_MODE_MODE_Nrf_1Mbit)
    {
        overhead_bits = 80; // 10 bytes
    }
    else if (radio_mode == RADIO_MODE_MODE_Ble_LR125Kbit)
    {
        overhead_bits = 720; // 90 bytes
    }
    else if (radio_mode == RADIO_MODE_MODE_Ble_LR500Kbit)
    {
        overhead_bits = 462; // 57.75 bytes
    }
    else if (radio_mode == RADIO_MODE_MODE_Ieee802154_250Kbit)
    {
        overhead_bits = 40; // 5 bytes, ref. product spec
    }

    bit_in_air = (pkt_len * 8); // in bits

    if (radio_mode == RADIO_MODE_MODE_Ble_LR125Kbit)
    {
        bit_in_air *= 8; // 1 to 8 encoding
    }
    else if (radio_mode == RADIO_MODE_MODE_Ble_LR500Kbit)
    {
        bit_in_air *= 2; //  1 to 2 encoding
    }


    bit_in_air += overhead_bits;

    if (radio_mode == RADIO_MODE_MODE_Ble_2Mbit || radio_mode == RADIO_MODE_MODE_Nrf_2Mbit)
    {
        bit_in_air /= 2; // double speed
    }
    else if (radio_mode == RADIO_MODE_MODE_Ieee802154_250Kbit)
    {
        bit_in_air *= 4; // 250kbps
    }

    bit_in_air += INTER_PACKETS_INTERVAL;

    return (bit_in_air + 999) / 1000; // Interval unit is 1ms
}

void rt_config_validate(void)
{
    // Validate radio mode
    if ((m_radio_mode != RADIO_MODE_MODE_Nrf_1Mbit)     &&
        (m_radio_mode != RADIO_MODE_MODE_Nrf_2Mbit)     &&
        (m_radio_mode != RADIO_MODE_MODE_Ble_1Mbit)     &&
        (m_radio_mode != RADIO_MODE_MODE_Ble_2Mbit)     &&
        (m_radio_mode != RADIO_MODE_MODE_Ble_LR125Kbit) &&
        (m_radio_mode != RADIO_MODE_MODE_Ble_LR500Kbit) &&
        (m_radio_mode != RADIO_MODE_MODE_Ieee802154_250Kbit))
    {
        m_radio_mode = RADIO_MODE_MODE_Nrf_1Mbit;
    }

    // Validate packet interval
    uint32_t tx_interval = rt_test_interval_cal(m_radio_mode, m_pkt_len);
    if (m_pkt_tx_interval < tx_interval)
    {
        m_pkt_tx_interval = tx_interval;
    }
    else if (m_pkt_tx_interval > PACKET_TX_INTERVAL_MAX)
    {
        m_pkt_tx_interval = PACKET_TX_INTERVAL_MAX;
    }

    // Validate packet length
    if (m_radio_mode == RADIO_MODE_MODE_Ieee802154_250Kbit)
    {
        if (m_pkt_len > IEEE_PDU_MAX_SIZE)
        {
            m_pkt_len = IEEE_PDU_MAX_SIZE;
        }
    }
    else
    {
        if (m_pkt_len > PACKET_LENGTH_MAX)
        {
            m_pkt_len = PACKET_LENGTH_MAX;
        }
    }

    // Validate packet max count
    if (m_pkt_tx_cnt_max > PACKET_TX_COUNT_MAX)
    {
        m_pkt_tx_cnt_max = PACKET_TX_COUNT_MAX;
    }

    // Valid packet type
    if ((m_pkt_type != DTM_PKT_PRBS9) &&
        (m_pkt_type != DTM_PKT_0X0F)  &&
        (m_pkt_type != DTM_PKT_0X55)  &&
        (m_pkt_type != DTM_PKT_0XFF))
    {
        m_pkt_type = DTM_PKT_PRBS9;
    }

    // Validate radio channel
    if (m_radio_mode == RADIO_MODE_MODE_Ieee802154_250Kbit)
    {
        if (m_radio_channel < 11)
        {
            m_radio_channel = 11;
        }
        else if (m_radio_channel > 26)
        {
            m_radio_channel = 26;
        }
    }
    else
    {
        if (m_radio_channel > 39)
        {
            m_radio_channel = 39;
        }
    }
}

void rt_test_receive(void)
{
    NRF_LOG_INFO("Start RX...");

    memset(&m_pdu, 0, DTM_PDU_MAX_MEMORY_SIZE);

    rt_config_validate();

    rt_radio_reset();
    rt_radio_setup();

    m_test_state          = STATE_RECEIVER_TEST;
    m_pkt_rx_cnt          = 0;

    NRF_RADIO->INTENCLR   = 0;

    if (anomaly_172_wa_enabled)
    {
        set_strict_mode(1);

        NRF_RADIO->INTENSET    = (RADIO_INTENSET_ADDRESS_Msk) |
                                 (RADIO_INTENSET_END_Msk)     |
                                 (RADIO_INTENSET_READY_Msk);
    }
    else
    {
        NRF_RADIO->INTENSET    = (RADIO_INTENSET_END_Msk);
    }

    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_RXEN = 1;

}

void rt_test_stop_receiving(void)
{
    if (m_test_state == STATE_RECEIVER_TEST)
    {
        NRF_LOG_INFO("Stop RX");

        m_test_state = STATE_IDLE;

        NRF_RADIO->TASKS_DISABLE   = 1;
        while (NRF_RADIO->EVENTS_DISABLED == 0) {;}
        NRF_RADIO->EVENTS_DISABLED = 0;
    }
}

static void rt_pkt_fill_pdu(void)
{
    // 802.15.4 uses PPDU structure, see PS > 6.20.12:
    // Length(1) | Payload(N)
    if (m_radio_mode == RADIO_MODE_MODE_Ieee802154_250Kbit)
    {
        m_pdu.content[0] = m_pkt_len + 2; // Data + CRC(2 bytes)
        memcpy(m_pdu.content + 1, m_prbs_content, m_pkt_len);
    }
    // Others uses basic radio structure:
    // Type(1) | Length(1) | Payload(N)
    else
    {
        m_pdu.content[1] = m_pkt_len;
        switch (m_pkt_type)
        {
            case DTM_PKT_PRBS9:
                m_pdu.content[0] = DTM_PDU_TYPE_PRBS9;
                memcpy(m_pdu.content + DTM_HEADER_SIZE, m_prbs_content, m_pkt_len);
                break;

            case DTM_PKT_0X0F:
                m_pdu.content[0] = DTM_PDU_TYPE_0X0F;
                memset(m_pdu.content + DTM_HEADER_SIZE, 0x0F, m_pkt_len);
                break;

            case DTM_PKT_0X55:
                m_pdu.content[0] = DTM_PDU_TYPE_0X55;
                memset(m_pdu.content + DTM_HEADER_SIZE, 0x55, m_pkt_len);
                break;

            case DTM_PKT_0XFF:
                m_pdu.content[0] = DTM_PDU_TYPE_0XFF;
                // Bit pattern 11111111 repeated. Only available in coded PHY (Long range).
                memset(m_pdu.content + DTM_HEADER_SIZE, 0xFF, m_pkt_len);
                break;

            default:
                NRF_LOG_ERROR("Wrong packet type");
                break;
        }
    }

}

void rt_test_transmit(void)
{
    NRF_LOG_INFO("Start TX...");

    uint32_t pkt_len_offset;
    uint32_t pkt_type_offset;

    rt_config_validate();

    rt_radio_reset();
    rt_radio_setup();

    rt_pkt_fill_pdu();

    NRF_RADIO->INTENCLR = 0;
    NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;

    // Stop the timer used by nRF52840 anomaly 172 if running on an affected device.
    if (anomaly_172_wa_enabled)
    {
        E172_TIMER->TASKS_CLEAR          = 1;
        E172_TIMER->TASKS_STOP           = 1;
        E172_TIMER->EVENTS_COMPARE[0]    = 0;
        E172_TIMER->EVENTS_COMPARE[1]    = 0;
    }

    m_test_state            = STATE_TRANSMITTER_TEST;
    m_pkt_tx_cnt            = 0;
    m_radio_tx_ready        = true;

    uint64_t cc_value       = (m_pkt_tx_interval << 17) / 125; // x * 1024 * 1024 / 1000
    NRF_TIMER0->CC[0]       = (uint32_t)(cc_value & 0xFFFFFFFF);
    NRF_TIMER0->TASKS_START = 1;
}

void rt_test_carrier_start(void)
{
    NRF_LOG_INFO("Start carrier test");

    m_pkt_len        = 1;
    m_pdu.content[0] = 0;       // Copied from DTM project
    m_pdu.content[1] = m_pkt_len;

    m_radio_mode     = RADIO_MODE_MODE_Ble_1Mbit;

    rt_radio_reset();
    rt_radio_setup();
    dtm_constant_carrier();

    if (anomaly_172_wa_enabled)
    {
        E172_TIMER->TASKS_CLEAR          = 1;
        E172_TIMER->TASKS_STOP           = 1;
        E172_TIMER->EVENTS_COMPARE[0]    = 0;
        E172_TIMER->EVENTS_COMPARE[1]    = 0;
    }

    m_test_state = STATE_CARRIER_TEST;
    NRF_RADIO->TASKS_TXEN = 1;
}

void rt_test_carrier_stop(void)
{
    NRF_LOG_INFO("Stop carrier test");

    NRF_RADIO->TASKS_DISABLE   = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0) {;}

    NRF_RADIO->EVENTS_DISABLED = 0;
}


void TIMER0_IRQHandler(void)
{
    if (NRF_TIMER0->EVENTS_COMPARE[0])
    {
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
        NVIC_ClearPendingIRQ(TIMER0_IRQn);

        if (m_radio_tx_ready)
        {
            m_radio_tx_ready      = false;
            NRF_RADIO->TASKS_TXEN = 1;
        }
    }
}


void TIMER1_IRQHandler(void)
{
    if (E172_TIMER->EVENTS_COMPARE[0] == 1)
    {
        uint8_t rssi = anomaly_172_rssi_check();
        if (m_strict_mode)
        {
            if (rssi > BLOCKER_FIX_RSSI_THRESHOLD)
            {
                set_strict_mode(0);
            }
        }
        else
        {
            bool too_many_detects = false;
            uint32_t packetcnt2 = *(volatile uint32_t *) 0x40001574;
            uint32_t detect_cnt = packetcnt2 & 0xffff;
            uint32_t addr_cnt   = (packetcnt2 >> 16) & 0xffff;

            if ((detect_cnt > BLOCKER_FIX_CNTDETECTTHR) && (addr_cnt < BLOCKER_FIX_CNTADDRTHR))
            {
                too_many_detects = true;
            }

            if ((rssi < BLOCKER_FIX_RSSI_THRESHOLD) || too_many_detects)
            {
                set_strict_mode(1);
            }
        }

        anomaly_172_radio_operation();

        E172_TIMER->CC[0]                = BLOCKER_FIX_WAIT_DEFAULT;
        E172_TIMER->TASKS_STOP           = 1;
        E172_TIMER->TASKS_CLEAR          = 1;
        E172_TIMER->EVENTS_COMPARE[0]    = 0;
        E172_TIMER->TASKS_START          = 1;

        NVIC_ClearPendingIRQ(E172_TIMER_IRQn);
    }

    if (E172_TIMER->EVENTS_COMPARE[1] != 0)
    {
        uint8_t rssi = anomaly_172_rssi_check();
        if (rssi >= BLOCKER_FIX_RSSI_THRESHOLD)
        {
            set_strict_mode(0);
        }
        else
        {
            set_strict_mode(1);
        }

        anomaly_172_radio_operation();

        // Disable this event.
        E172_TIMER->CC[1] = 0;
        E172_TIMER->EVENTS_COMPARE[1] = 0;
        NVIC_ClearPendingIRQ(E172_TIMER_IRQn);
    }
}

void RADIO_IRQHandler(void)
{
    // For errata
    if (NRF_RADIO->EVENTS_READY)
    {
        NRF_RADIO->EVENTS_READY = 0;

        if (m_test_state == STATE_RECEIVER_TEST)
        {
            if (anomaly_172_wa_enabled)
            {
                E172_TIMER->TASKS_CLEAR = 1;
                E172_TIMER->TASKS_START = 1;
            }
        }
    }

    // For errata
    if (NRF_RADIO->EVENTS_ADDRESS)
    {
        NRF_RADIO->EVENTS_ADDRESS = 0;

        if (anomaly_172_wa_enabled)
        {
            E172_TIMER->TASKS_SHUTDOWN = 1;
        }
    }

    // For RX
    if (NRF_RADIO->EVENTS_END)
    {
        NRF_RADIO->EVENTS_END = 0;
        NVIC_ClearPendingIRQ(RADIO_IRQn);

        if (m_test_state == STATE_RECEIVER_TEST)
        {
            if (anomaly_172_wa_enabled)
            {
                E172_TIMER->CC[0]                = BLOCKER_FIX_WAIT_DEFAULT;
                E172_TIMER->CC[1]                = BLOCKER_FIX_WAIT_END;
                E172_TIMER->TASKS_CLEAR          = 1;
                E172_TIMER->EVENTS_COMPARE[0]    = 0;
                E172_TIMER->EVENTS_COMPARE[1]    = 0;
                E172_TIMER->TASKS_START          = 1;
            }

            if ((NRF_RADIO->CRCSTATUS == 1) && check_pdu())
            {
                m_pkt_rx_cnt++;
            }

            memset(&m_pdu, 0, DTM_PDU_MAX_MEMORY_SIZE);
        }
    }
    if (NRF_RADIO->EVENTS_DISABLED)
    {
        NRF_RADIO->EVENTS_DISABLED = 0;
        NVIC_ClearPendingIRQ(RADIO_IRQn);

        // For TX
        if (m_test_state == STATE_TRANSMITTER_TEST)
        {
            m_pkt_tx_cnt++;

            if (m_pkt_tx_cnt >= m_pkt_tx_cnt_max)
            {
                NRF_LOG_INFO("Sent %d packets", m_pkt_tx_cnt_max);

                rt_transmit_complete();
            }
            else
            {
                m_radio_tx_ready = true;
            }
        }
        // Restart RX
        else if (m_test_state == STATE_RECEIVER_TEST)
        {
            NRF_RADIO->TASKS_RXEN = 1;
        }
    }
}