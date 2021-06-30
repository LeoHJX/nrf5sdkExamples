/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_nus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"
#include "ble_conn_state.h"

#include "throughput_measure.h"

#include "nrf_libuarte_async.h"
#include "nrf_atomic.h"

#include "ble_radio_notification.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define START_DATA_BUTTON BSP_BUTTON_0   /**< Button that will trigger the notification event with the LED Button Service */
#define STOP_DATA_BUTTON  BSP_BUTTON_1

#define SCANNING_LED     BSP_BOARD_LED_0   /**< Is on when device is advertising. */
#define CONNECTED_LED    BSP_BOARD_LED_1   /**< Is on when device has connected. */
#define DATA_SENDING_LED     BSP_BOARD_LED_2
#define DATA_RECEIVE_LED     BSP_BOARD_LED_3

#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50) /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define TX_POWER_LEVEL (4)              /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE 40 /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE 20 /**< Maximum number of events in the scheduler queue. */
#endif

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */

BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static uint32_t m_data_sending_counter = 0;
static uint32_t m_data_uart_busy_counter = 0;

static uint32_t m_data_ble_receive_count = 0;
static uint32_t m_data_libuarte_tx_done_count = 0;


#define DEBUG_BLE_RX_PIN                27
#define DEBUG_LIBUART_TX_COMPLETE_PIN   26
#define DEBUG_LIBUART_TX_BUSY           02
#define DEBUG_RSSI_CHANGE_PIN           25

static void debug_init(void)
{
        nrf_gpio_cfg_output(DEBUG_BLE_RX_PIN);
        nrf_gpio_cfg_output(DEBUG_LIBUART_TX_COMPLETE_PIN);
        nrf_gpio_cfg_output(DEBUG_LIBUART_TX_BUSY);
        nrf_gpio_cfg_output(DEBUG_RSSI_CHANGE_PIN);

        nrf_gpio_pin_clear(DEBUG_BLE_RX_PIN);
        nrf_gpio_pin_clear(DEBUG_BLE_RX_PIN);
        nrf_gpio_pin_clear(DEBUG_LIBUART_TX_BUSY);
        nrf_gpio_pin_clear(DEBUG_RSSI_CHANGE_PIN);
}

static void debug_ble_rx(void)
{
        nrf_gpio_pin_toggle(DEBUG_BLE_RX_PIN);
}

static void debug_libuart_tx_request(void)
{
        nrf_gpio_pin_set(DEBUG_LIBUART_TX_COMPLETE_PIN);
}

static void debug_libuart_tx_complete(void)
{
        nrf_gpio_pin_clear(DEBUG_LIBUART_TX_COMPLETE_PIN);
}

static void debug_libuart_tx_busy(void)
{
        nrf_gpio_pin_toggle(DEBUG_LIBUART_TX_BUSY);
}

static void debug_rssi_change(void)
{
        nrf_gpio_pin_toggle(DEBUG_RSSI_CHANGE_PIN);
}

static bool m_scanning_is_start    = false;
static bool m_data_is_sending      = false;

//NRF_LIBUARTE_ASYNC_DEFINE(libuarte, 0, 2, 0, NRF_LIBUARTE_PERIPHERAL_NOT_USED, HS_LIBUARTE_DATA_BUFFER_LEN, 3);
NRF_LIBUARTE_ASYNC_DEFINE(libuarte, 0, 2, 0, NRF_LIBUARTE_PERIPHERAL_NOT_USED, 244, 5);


#define HS_LIBUARTE_DATA_BUFFER_LEN   244

typedef struct {
        uint8_t data_array[HS_LIBUARTE_DATA_BUFFER_LEN];
        uint8_t len;
} buffer_t;

#if defined (NRF52832_XXAA) || defined (NRF52840_XXAA)
NRF_QUEUE_DEF(buffer_t, m_buf_queue, 10, NRF_QUEUE_MODE_NO_OVERFLOW);
#else
NRF_QUEUE_DEF(buffer_t, m_buf_queue, 5, NRF_QUEUE_MODE_NO_OVERFLOW);
#endif

#define MAXIMUM_NUMBER_PACKET_PER_INTERVAL 5

static uint8_t m_number_of_packets_per_interval = MAXIMUM_NUMBER_PACKET_PER_INTERVAL;

static uint32_t m_received_count  = 0;

static void libuart_event_handler(void * context, nrf_libuarte_async_evt_t * p_evt)
{
        nrf_libuarte_async_t * p_libuarte = (nrf_libuarte_async_t *)context;
        ret_code_t ret, err_code;

        switch (p_evt->type)
        {
        case NRF_LIBUARTE_ASYNC_EVT_ERROR:
                break;

        case NRF_LIBUARTE_ASYNC_EVT_RX_DATA:
                m_received_count += p_evt->data.rxtx.length;
                NRF_LOG_INFO("RX DATA %08d %08d", p_evt->data.rxtx.length, m_received_count);
                nrf_libuarte_async_rx_free(p_libuarte, p_evt->data.rxtx.p_data, p_evt->data.rxtx.length);
                break;

        case NRF_LIBUARTE_ASYNC_EVT_TX_DONE:
                NRF_LOG_INFO("receive=%08d, uart tx=%08d, busy=%08d", m_data_ble_receive_count, m_data_libuarte_tx_done_count, m_data_uart_busy_counter);
                debug_libuart_tx_complete();
                m_data_libuarte_tx_done_count++;

                // add tx_done app logic
                if (!nrf_queue_is_empty(&m_buf_queue)) {
                        buffer_t buf;
                        err_code = nrf_queue_pop(&m_buf_queue, &buf);
                        APP_ERROR_CHECK(err_code);
                        err_code = nrf_libuarte_async_tx(p_libuarte, buf.data_array, buf.len);
                        APP_ERROR_CHECK(err_code);
                }
                break;

        default:
                break;
        }
}

/**
 * @brief Function for main application entry.
 */
void libuart_init(void)
{
        ret_code_t err_code;
        nrf_libuarte_async_config_t nrf_libuarte_async_config = {
                .tx_pin     = TX_PIN_NUMBER,
                .rx_pin     = RX_PIN_NUMBER,
                .baudrate   = NRF_UARTE_BAUDRATE_1000000,
                .parity     = NRF_UARTE_PARITY_EXCLUDED,
                .hwfc       = NRF_UARTE_HWFC_DISABLED,
                .timeout_us = 10,
                .int_prio   = APP_IRQ_PRIORITY_LOW_MID
        };
        // m_uart_busy = false;
        err_code = nrf_libuarte_async_init(&libuarte, &nrf_libuarte_async_config, libuart_event_handler, (void *)&libuarte);

        APP_ERROR_CHECK(err_code);

        nrf_libuarte_async_enable(&libuarte);

}


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

static uint16_t counter=0;

static uint32_t data_transmit_handler(uint8_t *send_data, uint16_t len)
{
        uint32_t err_code = NRF_SUCCESS;
        if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
        {
                uint16_t length = len;
                counter=0;
                do
                {
                        // Send data back to the peripheral.
                        //err_code = ble_nus_c_string_send(&m_ble_nus_c, m_test_buffer, length);
                        //NRF_LOG_INFO("ble_nus_c_data_send");
                        err_code = ble_nus_c_data_send(&m_ble_nus_c, m_test_buffer, length);
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
        }

        return err_code;
}

static uint32_t data_received_handler(uint8_t *receive_data, uint16_t len)
{

}

/**@brief NUS UUID. */
static ble_uuid_t const m_nus_uuid =
{
        .uuid = BLE_UUID_NUS_SERVICE,
        .type = NUS_SERVICE_UUID_TYPE
};

/**< Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t const m_scan_param =
{
        .active        = 0x01,
        .interval      = NRF_BLE_SCAN_SCAN_INTERVAL,
        .window        = NRF_BLE_SCAN_SCAN_WINDOW,
        .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
        .timeout       = 0,
        .scan_phys     = BLE_GAP_PHY_1MBPS,
};

static ble_gap_addr_t const m_target_periph_addr =
{
        /* Possible values for addr_type:
           BLE_GAP_ADDR_TYPE_PUBLIC,
           BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
           BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE,
           BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE. */
        .addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
        .addr      = {0x8D, 0xFE, 0x23, 0x86, 0x77, 0xD9}
};

/**@brief Names which the central applications will scan for, and which will be advertised by the peripherals.
 *  if these are set to empty strings, the UUIDs defined below will be used
 */
static char const m_target_periph_name[] = "HIGHUART";      /**< If you want to connect to a peripheral using a given advertising name, type its name here. */
static bool is_connect_per_addr = false;                    /**< If you want to connect to a peripheral with a given address, set this to true and put the correct address in the variable below. */


/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
        app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for changing the tx power.
 */
static void tx_power_connected_set(void)
{
        ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, TX_POWER_LEVEL);
        APP_ERROR_CHECK(err_code);
}

static void tx_power_scanning_set(void)
{
        ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_SCAN_INIT, m_conn_handle, TX_POWER_LEVEL);
        APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling the Nordic UART Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nus_error_handler(uint32_t nrf_error)
{
        APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function to start scanning. */
static void scan_start(void)
{
        ret_code_t ret;

        if (!m_scanning_is_start)
        {
                NRF_LOG_INFO("Scan Start");

                ret = nrf_ble_scan_start(&m_scan);
                APP_ERROR_CHECK(ret);

                m_scanning_is_start = true;
                bsp_board_led_on(SCANNING_LED);
        }
}

/**@brief Function for starting scanning. */
static void scan_stop(void)
{
        if (m_scanning_is_start)
        {
                NRF_LOG_INFO("Scan Stop");
                nrf_ble_scan_stop();

                bsp_board_led_off(SCANNING_LED);
                m_scanning_is_start = false;
        }
}



/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
        ret_code_t err_code;

        switch(p_scan_evt->scan_evt_id)
        {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
                err_code = p_scan_evt->params.connecting_err.err_code;
                APP_ERROR_CHECK(err_code);
        } break;

        case NRF_BLE_SCAN_EVT_CONNECTED:
        {
                ble_gap_evt_connected_t const * p_connected =
                        p_scan_evt->params.connected.p_connected;
                m_scanning_is_start = false;
                // Scan is automatically stopped by the connection.
                NRF_LOG_INFO("Connecting to target 0x%02x:%02x:%02x:%02x:%02x:%02x",
                             p_connected->peer_addr.addr[5],
                             p_connected->peer_addr.addr[4],
                             p_connected->peer_addr.addr[3],
                             p_connected->peer_addr.addr[2],
                             p_connected->peer_addr.addr[1],
                             p_connected->peer_addr.addr[0]
                             );
        } break;

        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
                NRF_LOG_INFO("Scan timed out.");
                scan_start();
        } break;

        default:
                break;
        }
}


/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
        ret_code_t err_code;
        nrf_ble_scan_init_t init_scan;

        memset(&init_scan, 0, sizeof(init_scan));

        init_scan.p_scan_param     = &m_scan_param;
        init_scan.connect_if_match = true;
        init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

        err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
        APP_ERROR_CHECK(err_code);

        if (strlen(m_target_periph_name) != 0)
        {
                err_code = nrf_ble_scan_filter_set(&m_scan,
                                                   SCAN_NAME_FILTER,
                                                   m_target_periph_name);
                APP_ERROR_CHECK(err_code);

                err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
                APP_ERROR_CHECK(err_code);
        }

}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
        ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
}


  static uint32_t receive_packet_count = 0;
static uint32_t receive_packet_bytes = 0;

/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
        ret_code_t err_code;
        static uint8_t ble_data[HS_LIBUARTE_DATA_BUFFER_LEN];
        {
                debug_ble_rx();
                debug_libuart_tx_request();
                memcpy(ble_data, p_data, data_len);

                receive_packet_bytes += data_len;
                receive_packet_count++;
                NRF_LOG_INFO("Received data %08x, %08x", app_timer_cnt_get(), receive_packet_bytes);
//                err_code = nrf_libuarte_async_tx(&libuarte, ble_data, data_len);
//                if (err_code == NRF_ERROR_BUSY)
//                {
//                        debug_libuart_tx_busy();
//                        buffer_t buf;
//                        buf.len = data_len;
//                        memcpy(buf.data_array, p_data, buf.len);
//                        memcpy(m_test_buffer,  p_data, buf.len);
//                        err_code = nrf_queue_push(&m_buf_queue, &buf);
//                        APP_ERROR_CHECK(err_code);
//                } else {
//                        APP_ERROR_CHECK(err_code);
//                }

        }

//        for (uint32_t i = 0; i < data_len; i++)
//        {
//                do
//                {
//                        ret_val = app_uart_put(p_data[i]);
//                        if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
//                        {
//                                NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
//                                APP_ERROR_CHECK(ret_val);
//                        }
//                } while (ret_val == NRF_ERROR_BUSY);
//        }
//        if (p_data[data_len-1] == '\r')
//        {
//                while (app_uart_put('\n') == NRF_ERROR_BUSY);
//        }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
        static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
        static uint16_t index = 0;
        uint32_t ret_val;

        switch (p_event->evt_type)
        {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
                UNUSED_VARIABLE(app_uart_get(&data_array[index]));
                index++;

                if ((data_array[index - 1] == '\n') ||
                    (data_array[index - 1] == '\r') ||
                    (index >= (m_ble_nus_max_data_len)))
                {
                        NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                        NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                        do
                        {
                                ret_val = ble_nus_c_string_send(&m_ble_nus_c, data_array, index);
                                if ( (ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_RESOURCES) )
                                {
                                        APP_ERROR_CHECK(ret_val);
                                }
                        } while (ret_val == NRF_ERROR_RESOURCES);

                        index = 0;
                }
                break;

        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
                NRF_LOG_ERROR("Communication error occurred while handling UART.");
                APP_ERROR_HANDLER(p_event->data.error_communication);
                break;

        case APP_UART_FIFO_ERROR:
                NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
                APP_ERROR_HANDLER(p_event->data.error_code);
                break;

        default:
                break;
        }
}


/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
        ret_code_t err_code;

        switch (p_ble_nus_evt->evt_type)
        {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
                NRF_LOG_INFO("Discovery complete.");
                err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
                APP_ERROR_CHECK(err_code);

                err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
                APP_ERROR_CHECK(err_code);
                NRF_LOG_INFO("Connected to device with Nordic UART Service.");
                break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
                ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
                break;

        case BLE_NUS_C_EVT_DISCONNECTED:
                NRF_LOG_INFO("Disconnected.");
                scan_start();
                break;
        }
}
/**@snippet [Handling events from the ble_nus_c module] */

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
        ret_code_t err_code;
        ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

        switch (p_ble_evt->header.evt_id)
        {
        case BLE_GAP_EVT_CONNECTED:

                m_conn_handle = p_gap_evt->conn_handle;
                err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
                APP_ERROR_CHECK(err_code);

                // start discovery of services. The NUS Client waits for a discovery result
                err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
                APP_ERROR_CHECK(err_code);

                bsp_board_led_on(CONNECTED_LED);
                bsp_board_led_off(SCANNING_LED);

                break;

        case BLE_GAP_EVT_DISCONNECTED:

                NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                             p_gap_evt->conn_handle,
                             p_gap_evt->params.disconnected.reason);

                m_conn_handle = BLE_CONN_HANDLE_INVALID;

                scan_start();

                bsp_board_led_off(CONNECTED_LED);
                bsp_board_led_on(SCANNING_LED);
                break;

        case BLE_GAP_EVT_TIMEOUT:
                if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
                {
                        NRF_LOG_INFO("Connection Request timed out.");
                }
                break;

        case BLE_GATTC_EVT_TIMEOUT:
                // Disconnect on GATT Client timeout event.
                NRF_LOG_DEBUG("GATT Client Timeout.");
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GATTS_EVT_TIMEOUT:
                // Disconnect on GATT Server timeout event.
                NRF_LOG_DEBUG("GATT Server Timeout.");
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
                // Pairing not supported.
                err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
                // Accepting parameters requested by peer.
                err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                        &p_gap_evt->params.conn_param_update_request.conn_params);
                APP_ERROR_CHECK(err_code);
        }
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

        case BLE_GAP_EVT_AUTH_STATUS:
        {
                NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                             p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                             p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                             p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                             *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                             *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
        }
        break;

        case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:

//                NRF_LOG_INFO("BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE %d", m_data_is_sending);
                if (m_data_is_sending)
                {

                        if (throughput_measure_get_send_total() < m_throughput_measure_config.send_data_threshold)
                                data_transmit_handler(m_test_buffer, sizeof(m_test_buffer));
                }
                break;

        default:
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


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
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

//        ble_cfg_t ble_cfg;
//        memset(&ble_cfg, 0x00, sizeof(ble_cfg));
//        ble_cfg.conn_cfg.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
//        ble_cfg.conn_cfg.params.gattc_conn_cfg.write_cmd_tx_queue_size = 2;
//        err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATTC, &ble_cfg, ram_start);

        ble_cfg_t ble_cfg;
        memset(&ble_cfg, 0, sizeof(ble_cfg));
        ble_cfg.conn_cfg.conn_cfg_tag                     = APP_BLE_CONN_CFG_TAG;
        ble_cfg.conn_cfg.params.gap_conn_cfg.event_length = NRF_SDH_BLE_GAP_EVENT_LENGTH;

        err_code = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &ble_cfg, ram_start);

        // Enable BLE stack.
        err_code = nrf_sdh_ble_enable(&ram_start);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
        APP_ERROR_CHECK(err_code);

        // Register a handler for BLE events.
        NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
        //NRF_SDH_SOC_OBSERVER(m_soc_observer, APP_SOC_OBSERVER_PRIO, soc_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
        if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
        {
                NRF_LOG_INFO("ATT MTU exchange completed.");

                m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
        }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
        ret_code_t err_code;

        err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, NRF_SDH_BLE_GAP_DATA_LENGTH);
        APP_ERROR_CHECK(err_code);
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
                        if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
                        {
                                if (!m_data_is_sending)
                                {
                                        m_data_sending_counter = 0;
                                        uint16_t length =sizeof(m_test_buffer);
                                        throughput_measure_reset_send_count();
                                        throughput_measure_live_timer_start();
                                        m_data_is_sending = true;
                                        data_transmit_handler(m_test_buffer,length);

                                }
                        }
                }
                break;
        case STOP_DATA_BUTTON:
                if (button_action == APP_BUTTON_PUSH)
                {
                        m_data_is_sending = false;
                        throughput_measure_live_timer_stop();
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
                {STOP_DATA_BUTTON,  false, BUTTON_PULL, button_event_handler},
        };

        err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                                   BUTTON_DETECTION_DELAY);
        APP_ERROR_CHECK(err_code);

        err_code = app_button_enable();
        APP_ERROR_CHECK(err_code);


}


/**@brief Function for initializing the UART. */
static void uart_init(void)
{
        ret_code_t err_code;

        app_uart_comm_params_t const comm_params =
        {
                .rx_pin_no    = RX_PIN_NUMBER,
                .tx_pin_no    = TX_PIN_NUMBER,
                .rts_pin_no   = RTS_PIN_NUMBER,
                .cts_pin_no   = CTS_PIN_NUMBER,
                .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
                .use_parity   = false,
                .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
        };

        APP_UART_FIFO_INIT(&comm_params,
                           UART_RX_BUF_SIZE,
                           UART_TX_BUF_SIZE,
                           uart_event_handle,
                           APP_IRQ_PRIORITY_LOWEST,
                           err_code);

        APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void)
{
        ret_code_t err_code;
        ble_nus_c_init_t init;

        init.evt_handler   = ble_nus_c_evt_handler;
        init.error_handler = nus_error_handler;
        init.p_gatt_queue  = &m_ble_gatt_queue;

        err_code = ble_nus_c_init(&m_ble_nus_c, &init);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
        ret_code_t err_code = app_timer_init();
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module. */
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


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
        ble_db_discovery_init_t db_init;

        memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

        db_init.evt_handler  = db_disc_handler;
        db_init.p_gatt_queue = &m_ble_gatt_queue;

        ret_code_t err_code = ble_db_discovery_init(&db_init);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for placing the application in low power state while waiting for events.
 */
#define FPU_EXCEPTION_MASK 0x0000009F

/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
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

static ble_radio_notification_evt_handler_t radio_notification_handler(bool radio_active)
{
        if (radio_active)
        {
                m_number_of_packets_per_interval = MAXIMUM_NUMBER_PACKET_PER_INTERVAL;

        }
        else
        {

        }
}

int main(void)
{
        // Initialize.
        log_init();
        timer_init();
        //uart_init();
        libuart_init();

        NRF_LOG_INFO("\n\nReset reason = %x\n\n", NRF_POWER->RESETREAS);

        buttons_init();
        bsp_board_init(BSP_INIT_LEDS);
        db_discovery_init();
        power_management_init();
        ble_stack_init();
        gatt_init();
        nus_c_init();
        scan_init();
        ble_radio_notification_init(APP_IRQ_PRIORITY_LOW_MID, NRF_RADIO_NOTIFICATION_DISTANCE_800US, radio_notification_handler);

        // Start execution.
        NRF_LOG_INFO("BLE Throughput Demo on BLE 2Mbps (Central)");
        scan_start();
        debug_init();

        m_data_is_sending = false;

        conn_evt_len_ext_set(true);

        throughput_measure_setup();

        // Enter main loop.
        for (;;)
        {
                idle_state_handle();
        }
}
