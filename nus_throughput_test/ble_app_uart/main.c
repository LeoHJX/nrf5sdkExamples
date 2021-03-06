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
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#include "throughput_measure.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "app_scheduler.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define START_DATA_BUTTON BSP_BUTTON_0   /**< Button that will trigger the notification event with the LED Button Service */
#define STOP_DATA_BUTTON  BSP_BUTTON_1


#define ADVERTISING_LED      BSP_BOARD_LED_0   /**< Is on when device is advertising. */
#define CONNECTED_LED        BSP_BOARD_LED_1   /**< Is on when device has connected. */
#define DATA_SENDING_LED     BSP_BOARD_LED_2
#define DATA_RECEIVE_LED     BSP_BOARD_LED_3


#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "HIGHUART"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                0                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(5000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50) /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define TX_POWER_LEVEL (0)              /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE 40 /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE 20 /**< Maximum number of events in the scheduler queue. */
#endif

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

typedef enum {
        APP_STATE_IDLE = 0,
        APP_STATE_ADVERTISING,
        APP_STATE_CONNECTED,
        APP_STATE_DISCONNECTED
} app_state_t;

typedef enum {
        APP_PHY_1M = 0,
        APP_PHY_2M,
        APP_PHY_MULTI,
        APP_PHY_CODED,
        APP_PHY_LIST_END
} app_phy_t;

typedef struct {
        app_state_t app_state;
        app_phy_t phy;
} app_display_content_t;

app_display_content_t m_application_state = {0};

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
        .adv_data =
        {
                .p_data = m_enc_advdata,
                .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
        },
        .scan_rsp_data =
        {
                .p_data = m_enc_scan_response_data,
                .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

        }
};

static uint16_t m_conn_handle          = BLE_CONN_HANDLE_INVALID;                   /**< Handle of the current connection. */
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;              /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
        {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

static bool m_advertising_is_start = false;
static bool m_data_is_sending      = false;

static uint32_t m_data_sending_counter = 0;

static uint8_t m_test_buffer [] =
{
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
        10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
        20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
        30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
        40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
        50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
        60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
        70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
        80, 81, 82, 83, 84, 85, 86, 87, 88, 89,
        90, 91, 92, 93, 94, 95, 96, 97, 98, 99,
        100, 101, 102, 103, 104, 105, 106, 107, 108, 109,
        110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
        120, 121, 122, 123, 124, 125, 126, 127, 128, 129,
        130, 131, 132, 133, 134, 135, 136, 137, 138, 139,
        140, 141, 142, 143, 144, 145, 146, 147, 148, 149,
        150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
        160, 161, 162, 163, 164, 165, 166, 167, 168, 169,
        170, 171, 172, 173, 174, 175, 176, 177, 178, 179,
        180, 181, 182, 183, 184, 185, 186, 187, 188, 189,
        190, 191, 192, 193, 194, 195, 196, 197, 198, 199,
        200, 201, 202, 203, 204, 205, 206, 207, 208, 209,
        210, 211, 212, 213, 214, 215, 216, 217, 218, 219,
        220, 221, 222, 223, 224, 225, 226, 227, 228, 229,
        230, 231, 232, 233, 234, 235, 236, 237, 238, 239,
        240, 241, 242, 243, 244
};

static throughput_measure_init_t m_throughput_measure_init;
static throughput_measure_config_t m_throughput_measure_config;
static throughput_measure_data_receive_t m_throughput_measure_receive_data;
static throughput_measure_data_send_t m_throughput_measure_send_data;

static void throughput_measure_evt_handler(throughput_measure_evt_t * p_throughput_measure_evt)
{
        switch(p_throughput_measure_evt->evt_type)
        {
        case eTHROUGHPUT_INIT_DONE:
                break;
        case eTHROUGHPUT_LIVE:
                break;
        case eTHROUGHPUT_RESET:
                break;
        case eTHROUGHPUT_START:
                NRF_LOG_INFO("eTHROUGHPUT_START");
                break;
        case eTHROUGHPUT_STOP:
                break;
        case eTHROUGHPUT_THRESHOLD:
                break;
        case eTHROUGPPUT_SUMMARY:
                break;
        default:
                break;
        }
}

static uint32_t throughput_measure_setup(void)
{
        m_throughput_measure_init.evt_handler  = throughput_measure_evt_handler;
        m_throughput_measure_config.payload_length  = m_ble_nus_max_data_len;
        m_throughput_measure_config.live_timer_interval = 1000; // one seconds
        m_throughput_measure_config.receive_data_threshold = 0x100000; // 1024KB
        m_throughput_measure_config.send_data_threshold = 0x100000; // 64KB

        memcpy(&m_throughput_measure_init.config, &m_throughput_measure_config, sizeof(m_throughput_measure_config));
        throughput_measure_init(&m_throughput_measure_init);
}
static uint32_t data_transmit_handler(uint8_t *send_data, uint16_t len)
{
        //if (!m_data_is_sending)
        uint32_t err_code = NRF_SUCCESS;
        if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
        {
                uint16_t length = len;
                do
                {
                        // Send data back to the peripheral.
                        uint32_t err_code = ble_nus_data_send(&m_nus, send_data, &length, m_conn_handle);
                        if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY) && (err_code != NRF_ERROR_RESOURCES))
                        {
                                NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", err_code);
                                APP_ERROR_CHECK(err_code);
                        }
                        if (err_code == NRF_ERROR_RESOURCES)
                                break;

                        throughput_measure_send_update(length);
                } while (err_code != NRF_ERROR_RESOURCES);
                m_data_is_sending = true;
                //NRF_LOG_INFO("counter_loop = %x", counter_loop);
        }
        return err_code;
}

static uint32_t data_received_handler(uint8_t *receive_data, uint16_t len)
{

}

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
        app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/****************************************************************/
static void Get_Connect_MAC_Address(ble_gap_addr_t *gap_address)
{
        NRF_LOG_INFO("Connected to Central address to: %02X:%02X:%02X:%02X:%02X:%02X",
                     gap_address->addr[5],
                     gap_address->addr[4],
                     gap_address->addr[3],
                     gap_address->addr[2],
                     gap_address->addr[1],
                     gap_address->addr[0]);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
        ret_code_t err_code = app_timer_init();
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
        uint32_t err_code;
        ble_gap_conn_params_t gap_conn_params;
        ble_gap_conn_sec_mode_t sec_mode;

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

        err_code = sd_ble_gap_device_name_set(&sec_mode,
                                              (const uint8_t *) DEVICE_NAME,
                                              strlen(DEVICE_NAME));
        APP_ERROR_CHECK(err_code);

        memset(&gap_conn_params, 0, sizeof(gap_conn_params));

        gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
        gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
        gap_conn_params.slave_latency     = SLAVE_LATENCY;
        gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

        err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for changing the tx power.
 */
static void tx_power_advertising_set(void)
{
        ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, TX_POWER_LEVEL);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for changing the tx power.
 */
static void tx_power_connected_set(void)
{
        ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, TX_POWER_LEVEL);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
        APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static uint32_t receive_packet_count = 0;
static uint32_t receive_packet_bytes = 0;

static void nus_data_handler(ble_nus_evt_t * p_evt)
{

        if (p_evt->type == BLE_NUS_EVT_RX_DATA)
        {
                uint32_t err_code;
                receive_packet_bytes += p_evt->params.rx_data.length;
                receive_packet_count++;

                //NRF_LOG_INFO("Received data %08x, %08x", receive_packet_bytes(), receive_packet_count++);
                NRF_LOG_INFO("Received data %08x, %08x", app_timer_cnt_get(), receive_packet_bytes);
                //NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
//                for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
//                {
//                        do
//                        {
//                                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
//                                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
//                                {
//                                        NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
//                                        APP_ERROR_CHECK(err_code);
//                                }
//                        } while (err_code == NRF_ERROR_BUSY);
//                }
//                if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
//                {
//                        while (app_uart_put('\n') == NRF_ERROR_BUSY);
//                }
        }
        else if (p_evt->type == BLE_NUS_EVT_TX_RDY) //,       /**< Service is ready to accept new data to be transmitted. */
        {
        }
        else if (p_evt->type == BLE_NUS_EVT_COMM_STARTED) //,       /**< Notification has been enabled. */
        {
                // After BLE Subscription enable (CCCD enable)
        }
        else if (p_evt->type == BLE_NUS_EVT_COMM_STOPPED) //,       /**< /**< Notification has been disabled. */. */
        {

        }
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
        uint32_t err_code;
        ble_nus_init_t nus_init;
        nrf_ble_qwr_init_t qwr_init = {0};

        // Initialize Queued Write Module.
        qwr_init.error_handler = nrf_qwr_error_handler;

        err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
        APP_ERROR_CHECK(err_code);

        // Initialize NUS.
        memset(&nus_init, 0, sizeof(nus_init));

        nus_init.data_handler = nus_data_handler;

        err_code = ble_nus_init(&m_nus, &nus_init);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
        uint32_t err_code;

        if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
        {
                err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
                APP_ERROR_CHECK(err_code);
        }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
        APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
        uint32_t err_code;
        ble_conn_params_init_t cp_init;

        memset(&cp_init, 0, sizeof(cp_init));

        cp_init.p_conn_params                  = NULL;
        cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
        cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
        cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
        cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
        cp_init.disconnect_on_fail             = false;
        cp_init.evt_handler                    = on_conn_params_evt;
        cp_init.error_handler                  = conn_params_error_handler;

        err_code = ble_conn_params_init(&cp_init);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
        if (!m_advertising_is_start)
        {
                uint32_t err_code;
                err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
                APP_ERROR_CHECK(err_code);
                m_advertising_is_start = true;
                bsp_board_led_on(ADVERTISING_LED);
                m_application_state.app_state = APP_STATE_ADVERTISING;
        }
}

static void advertising_stop(void)
{
        if (m_advertising_is_start)
        {
                uint32_t err_code = sd_ble_gap_adv_stop(m_adv_handle);
                APP_ERROR_CHECK(err_code);

                m_advertising_is_start = false;
                bsp_board_led_off(ADVERTISING_LED);
        }
}

static void request_phy(uint16_t c_handle, uint8_t phy)
{
        ble_gap_phys_t phy_req;
        phy_req.tx_phys = phy;
        phy_req.rx_phys = phy;
        sd_ble_gap_phy_update(c_handle, &phy_req);
        NRF_LOG_INFO("Request to do the PHY update");

        switch (phy)
        {
        case BLE_GAP_PHY_1MBPS:
                break;
        case BLE_GAP_PHY_2MBPS:

                break;
        default:
                break;
        }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
        uint32_t err_code;
        ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

        //NRF_LOG_INFO("%s, %02x", __func__, p_ble_evt->header.evt_id);
        switch (p_ble_evt->header.evt_id)
        {
        case BLE_GAP_EVT_CONNECTED:
                NRF_LOG_INFO("Connected");

                bsp_board_led_on(CONNECTED_LED);
                bsp_board_led_off(ADVERTISING_LED);

                m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
                APP_ERROR_CHECK(err_code);

                // NRF_LOG_HEXDUMP_INFO(p_gap_evt->params.connected.peer_addr.addr, 6);
                Get_Connect_MAC_Address((ble_gap_addr_t *)&p_gap_evt->params.connected.peer_addr);

                m_advertising_is_start = false;

                if (m_application_state.phy == APP_PHY_2M)
                {
                        request_phy(0, BLE_GAP_PHY_2MBPS);
                }
                m_application_state.app_state = APP_STATE_CONNECTED;

                break;

        case BLE_GAP_EVT_DISCONNECTED:
                NRF_LOG_INFO("Disconnected");
                // LED indication will be changed when advertising starts.
                m_conn_handle = BLE_CONN_HANDLE_INVALID;
                advertising_start();
                bsp_board_led_off(CONNECTED_LED);
                bsp_board_led_on(ADVERTISING_LED);

                m_application_state.app_state = APP_STATE_DISCONNECTED;

                break;

        case BLE_GAP_EVT_PHY_UPDATE:
                NRF_LOG_INFO("PHY update Complete");
                break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
                NRF_LOG_INFO("PHY update request.");
                ble_gap_phys_t const phys =
                {
                        .rx_phys = BLE_GAP_PHY_2MBPS,
                        .tx_phys = BLE_GAP_PHY_2MBPS,
                };
                err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
                APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
                // Pairing not supported
                err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
                // No system attributes have been stored.
                err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GATTC_EVT_TIMEOUT:
                // Disconnect on GATT Client timeout event.
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GATTS_EVT_TIMEOUT:
                // Disconnect on GATT Server timeout event.
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
                if (m_data_is_sending)
                {
                        if (throughput_measure_get_send_total() < m_throughput_measure_config.send_data_threshold)
                                data_transmit_handler(m_test_buffer, sizeof(m_test_buffer));
                }
                break;
        case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
                // NRF_LOG_INFO("BLE_GATTC_EVT_EXCHANGE_MTU_RSP");
                break;

        case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
//                NRF_LOG_INFO("BLE_GAP_EVT_DATA_LENGTH_UPDATE");

                break;

        default:
                // No implementation needed.
                break;
        }
}

/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
        APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


void conn_evt_len_ext_set(bool status)
{
        ret_code_t err_code;
        ble_opt_t opt;

        memset(&opt, 0x00, sizeof(opt));
        opt.common_opt.conn_evt_ext.enable = status ? 1 : 0;

        err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
        APP_ERROR_CHECK(err_code);

}

/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
        ret_code_t err_code;

        err_code = nrf_sdh_enable_request();
        APP_ERROR_CHECK(err_code);

        // Configure the BLE stack using the default settings.
        // Fetch the start address of the application RAM.
        uint32_t ram_start = 0;
        err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
        APP_ERROR_CHECK(err_code);

        // Enable BLE stack.
        err_code = nrf_sdh_ble_enable(&ram_start);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
        APP_ERROR_CHECK(err_code);

        // Register a handler for BLE events.
        NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
        if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
        {
                m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
        }
        NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                      p_gatt->att_mtu_desired_central,
                      p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
        ret_code_t err_code;

        err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, NRF_SDH_BLE_GAP_DATA_LENGTH);
        APP_ERROR_CHECK(err_code);
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
        static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
        static uint8_t index = 0;
        uint32_t err_code;

        switch (p_event->evt_type)
        {
        case APP_UART_DATA_READY:
                UNUSED_VARIABLE(app_uart_get(&data_array[index]));
                index++;

                if ((data_array[index - 1] == '\n') ||
                    (data_array[index - 1] == '\r') ||
                    (index >= m_ble_nus_max_data_len))
                {
                        if (index > 1)
                        {
                                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                                NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                                do
                                {
                                        uint16_t length = (uint16_t)index;
                                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                                            (err_code != NRF_ERROR_RESOURCES) &&
                                            (err_code != NRF_ERROR_NOT_FOUND))
                                        {
                                                APP_ERROR_CHECK(err_code);
                                        }
                                } while (err_code == NRF_ERROR_RESOURCES);
                        }

                        index = 0;
                }
                break;

        case APP_UART_COMMUNICATION_ERROR:
                APP_ERROR_HANDLER(p_event->data.error_communication);
                break;

        case APP_UART_FIFO_ERROR:
                APP_ERROR_HANDLER(p_event->data.error_code);
                break;

        default:
                break;
        }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
        uint32_t err_code;
        app_uart_comm_params_t const comm_params =
        {
                .rx_pin_no    = RX_PIN_NUMBER,
                .tx_pin_no    = TX_PIN_NUMBER,
                .rts_pin_no   = RTS_PIN_NUMBER,
                .cts_pin_no   = CTS_PIN_NUMBER,
                .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
                .use_parity   = false,
#if defined (UART_PRESENT)
                .baud_rate    = NRF_UART_BAUDRATE_115200
#else
                .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
        };

        APP_UART_FIFO_INIT(&comm_params,
                           UART_RX_BUF_SIZE,
                           UART_TX_BUF_SIZE,
                           uart_event_handle,
                           APP_IRQ_PRIORITY_LOWEST,
                           err_code);
        APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
        ret_code_t err_code;
        ble_advdata_t advdata;
        ble_advdata_t srdata;

        //ble_uuid_t adv_uuids[] = {{LBS_UUID_SERVICE, m_lbs.uuid_type}};

        // Build and set advertising data.
        memset(&advdata, 0, sizeof(advdata));
        advdata.name_type          = BLE_ADVDATA_FULL_NAME;
        advdata.include_appearance = true;
        advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


        memset(&srdata, 0, sizeof(srdata));
        srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
        srdata.uuids_complete.p_uuids  = m_adv_uuids;

        err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
        APP_ERROR_CHECK(err_code);

        err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
        APP_ERROR_CHECK(err_code);

        ble_gap_adv_params_t adv_params;

        // Set advertising parameters.
        memset(&adv_params, 0, sizeof(adv_params));

        NRF_LOG_INFO("%s: m_application_state = %d", __func__, m_application_state.phy);

        // Set advertising parameters.
        memset(&adv_params, 0, sizeof(adv_params));
        {
                adv_params.primary_phy = BLE_GAP_PHY_1MBPS;
                adv_params.secondary_phy = BLE_GAP_PHY_1MBPS;
                adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
        }
        adv_params.duration = APP_ADV_DURATION;
        adv_params.p_peer_addr     = NULL;
        adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
        adv_params.interval        = APP_ADV_INTERVAL;

        err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
        APP_ERROR_CHECK(err_code);

        tx_power_advertising_set();

}

/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
        ret_code_t err_code;

        switch (pin_no)
        {
        case START_DATA_BUTTON:
                if (button_action == APP_BUTTON_PUSH)
                {
                        if (!m_data_is_sending)
                        {
                                m_data_sending_counter = 0;
                                uint16_t length =sizeof(m_test_buffer);
                                throughput_measure_reset_send_count();
                                throughput_measure_live_timer_start();
                                data_transmit_handler(m_test_buffer,length);
                                m_data_is_sending = true;
                        }
                }
                break;
        case STOP_DATA_BUTTON:
                if (button_action == APP_BUTTON_PUSH)
                {
                        m_data_is_sending = false;
                        throughput_measure_live_timer_stop();
                        throughput_measure_reset_receive_count();
                }
                break;

        default:
                APP_ERROR_HANDLER(pin_no);
                break;
        }
}



/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
        ret_code_t err_code;

        //The array must be static because a pointer to it will be saved in the button handler module.
        static app_button_cfg_t buttons[] =
        {
                {START_DATA_BUTTON, false, BUTTON_PULL, button_event_handler},
                {STOP_DATA_BUTTON, false, BUTTON_PULL, button_event_handler},
        };

        err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                                   BUTTON_DETECTION_DELAY);
        APP_ERROR_CHECK(err_code);

        err_code = app_button_enable();
        APP_ERROR_CHECK(err_code);


}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
        ret_code_t err_code = NRF_LOG_INIT(NULL);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
        ret_code_t err_code;
        err_code = nrf_pwr_mgmt_init();
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for placing the application in low power state while waiting for events.
 */
#define FPU_EXCEPTION_MASK 0x0000009F


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
  #if defined (NRF52832_XXAA) || defined (NRF52840_XXAA)
        __set_FPSCR(__get_FPSCR()  & ~(FPU_EXCEPTION_MASK));
        (void) __get_FPSCR();
        NVIC_ClearPendingIRQ(FPU_IRQn);
  #endif
        app_sched_execute();
        if (NRF_LOG_PROCESS() == false)
        {
                nrf_pwr_mgmt_run();
        }
}




/**@brief Application main function.
 */
int main(void)
{
        bool erase_bonds;

        // Initialize.
        //uart_init();
        log_init();

        NRF_LOG_INFO("\n\nReset reason = %x\n\n", NRF_POWER->RESETREAS);

        timers_init();

        buttons_init();
        bsp_board_init(BSP_INIT_LEDS);

        power_management_init();
        ble_stack_init();
        scheduler_init();
        gap_params_init();
        gatt_init();
        services_init();
        advertising_init();
        conn_params_init();

        // Start execution.
        NRF_LOG_INFO("BLE Throughput Demo on BLE 2Mbps (Peripheral)");
        advertising_start();

        m_application_state.phy = APP_PHY_2M;
        m_data_is_sending = false;

        conn_evt_len_ext_set(true);

        throughput_measure_setup();

        // Enter main loop.
        for (;;)
        {
                idle_state_handle();
        }
}


/**
 * @}
 */
