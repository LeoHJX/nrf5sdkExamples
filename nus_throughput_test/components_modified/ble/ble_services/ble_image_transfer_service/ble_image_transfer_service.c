/**
 * Copyright (c) 2012 - 2017, Nordic Semiconductor ASA
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
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_ITS)
#include "ble_image_transfer_service.h"
#include "ble_srv_common.h"
#include "ble.h"
#include "nrf_gpio.h"

#define NRF_LOG_MODULE_NAME ble_image_transfer_service
#if BLE_ITS_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL BLE_ITS_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR BLE_ITS_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR BLE_ITS_CONFIG_DEBUG_COLOR
#else // BLE_ITS_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL 3
#endif // BLE_ITS_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define BLE_UUID_ITS_SERVICE                 0x0001
#define BLE_UUID_ITS_RX_CHARACTERISTIC       0x0002 /**< The UUID of the RX Characteristic. */
#define BLE_UUID_ITS_TX_CHARACTERISTIC       0x0003 /**< The UUID of the TX Characteristic. */
#define BLE_UUID_ITS_IMG_INFO_CHARACTERISTIC 0x0004 /**< The UUID of the TX Command Characteristic. */
#define BLE_UUID_ITS_RX_DATA_CHARACTERISTIC  0x0005 /** < The UUID of the RX Command Characteristic. */

#define BLE_ITS_MAX_RX_CHAR_LEN BLE_ITS_MAX_DATA_LEN /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_ITS_MAX_TX_CHAR_LEN BLE_ITS_MAX_DATA_LEN /**< Maximum length of the TX Characteristic (in bytes). */

#define ITS_BASE_UUID                                                                                                  \
        {                                                                                                              \
                {                                                                                                      \
                        0x3E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E \
                }                                                                                                      \
        } /**< Used vendor specific UUID. */



volatile uint32_t file_size = 0, file_pos = 0, m_max_data_length = 20;
uint8_t *file_data;
ble_its_t *m_its;

static uint32_t count = 0;

// static ble_its_img_info_t m_image_info;
static uint32_t m_image_send_byte = 0;

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the S110 SoftDevice.
 *
 * @param[in] p_its     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_its_t *p_its, ble_evt_t const *p_ble_evt)
{
        p_its->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        ble_its_evt_t its_evt;
        its_evt.evt_type = BLE_ITS_EVT_CONNECTED;
        p_its->evt_handler(p_its, &its_evt);
}

/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the S110 SoftDevice.
 *
 * @param[in] p_its     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_its_t *p_its, ble_evt_t const *p_ble_evt)
{
        UNUSED_PARAMETER(p_ble_evt);
        ble_its_evt_t its_evt;
        its_evt.evt_type = BLE_ITS_EVT_DISCONNECTED;
        p_its->evt_handler(p_its, &its_evt);
        p_its->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the S110 SoftDevice.
 *
 * @param[in] p_its     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_its_t *p_its, ble_evt_t const *p_ble_evt)
{
        ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

        if (
                (p_evt_write->handle == p_its->tx_handles.cccd_handle) &&
                (p_evt_write->len == 2))
        {
                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                        p_its->is_notification_enabled = true;
                        NRF_LOG_INFO("Enable the TX Notification");
                        ble_its_evt_t its_evt;
                        its_evt.evt_type = BLE_ITS_EVT_ITS_TX_DATA_READY_EVT;
                        its_evt.p_data = p_evt_write->data;
                        its_evt.data_len = p_evt_write->len;
                        p_its->evt_handler(p_its, &its_evt);
                }
                else
                {
                        p_its->is_notification_enabled = false;
                }
        }
        else if (
                (p_evt_write->handle == p_its->img_info_handles.cccd_handle) &&
                (p_evt_write->len == 2))
        {
                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                        p_its->is_info_char_notification_enabled = true;
                        NRF_LOG_INFO("Enable the IMG Info Notification");
                        ble_its_evt_t its_evt;
                        its_evt.evt_type = BLE_ITS_EVT_ITS_TX_INFO_READY_EVT;
                        its_evt.p_data = p_evt_write->data;
                        its_evt.data_len = p_evt_write->len;
                        p_its->evt_handler(p_its, &its_evt);
                }
                else
                {
                        p_its->is_info_char_notification_enabled = false;
                }
        }
        else if (
                (p_evt_write->handle == p_its->rx_handles.value_handle) &&
                (p_its->evt_handler != NULL))
        {
                ble_its_evt_t its_evt;
                its_evt.evt_type = BLE_ITS_EVT_ITS_IMG_INFO_EVT;
                its_evt.p_data = p_evt_write->data;
                its_evt.data_len = p_evt_write->len;
                p_its->evt_handler(p_its, &its_evt);
        }
        else if (
                (p_evt_write->handle == p_its->rx_data_handles.value_handle) &&
                (p_its->evt_handler != NULL))
        {
                ble_its_evt_t its_evt;
                its_evt.evt_type = BLE_ITS_EVT_ITS_RX_DATA_EVT;
                NRF_LOG_INFO("BLE_ITS_EVT_ITS_RX_DATA_EVT len = %d", p_evt_write->len);
                its_evt.p_data = p_evt_write->data;
                its_evt.data_len = p_evt_write->len;
                p_its->evt_handler(p_its, &its_evt);
        }
        else
        {
                // Do Nothing. This event is not relevant for this service.
        }
}

/**@brief Function for adding TX characteristic.
 *
 * @param[in] p_its       Nordic UART Service structure.
 * @param[in] p_its_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t tx_char_add(ble_its_t *p_its, const ble_its_init_t *p_its_init)
{
        /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
        ble_gatts_char_md_t char_md;
        ble_gatts_attr_md_t cccd_md;
        ble_gatts_attr_t attr_char_value;
        ble_uuid_t ble_uuid;
        ble_gatts_attr_md_t attr_md;

        memset(&cccd_md, 0, sizeof(cccd_md));

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

        cccd_md.vloc = BLE_GATTS_VLOC_STACK;

        memset(&char_md, 0, sizeof(char_md));

        char_md.char_props.notify = 1;
        char_md.p_char_user_desc = NULL;
        char_md.p_char_pf = NULL;
        char_md.p_user_desc_md = NULL;
        char_md.p_cccd_md = &cccd_md;
        char_md.p_sccd_md = NULL;

        ble_uuid.type = p_its->uuid_type;
        ble_uuid.uuid = BLE_UUID_ITS_TX_CHARACTERISTIC;

        memset(&attr_md, 0, sizeof(attr_md));

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

        attr_md.vloc = BLE_GATTS_VLOC_STACK;
        attr_md.rd_auth = 0;
        attr_md.wr_auth = 0;
        attr_md.vlen = 1;

        memset(&attr_char_value, 0, sizeof(attr_char_value));

        attr_char_value.p_uuid = &ble_uuid;
        attr_char_value.p_attr_md = &attr_md;
        attr_char_value.init_len = sizeof(uint8_t);
        attr_char_value.init_offs = 0;
        attr_char_value.max_len = BLE_ITS_MAX_TX_CHAR_LEN;

        return sd_ble_gatts_characteristic_add(p_its->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_its->tx_handles);
        /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
}

/**@brief Function for adding TX characteristic.
 *
 * @param[in] p_its       Nordic UART Service structure.
 * @param[in] p_its_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t img_info_char_add(ble_its_t *p_its, const ble_its_init_t *p_its_init)
{
        /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
        ble_gatts_char_md_t char_md;
        ble_gatts_attr_md_t cccd_md;
        ble_gatts_attr_t attr_char_value;
        ble_uuid_t ble_uuid;
        ble_gatts_attr_md_t attr_md;

        memset(&cccd_md, 0, sizeof(cccd_md));

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

        cccd_md.vloc = BLE_GATTS_VLOC_STACK;

        memset(&char_md, 0, sizeof(char_md));

        char_md.char_props.notify = 1;
        char_md.p_char_user_desc = NULL;
        char_md.p_char_pf = NULL;
        char_md.p_user_desc_md = NULL;
        char_md.p_cccd_md = &cccd_md;
        char_md.p_sccd_md = NULL;

        ble_uuid.type = p_its->uuid_type;
        ble_uuid.uuid = BLE_UUID_ITS_IMG_INFO_CHARACTERISTIC;

        memset(&attr_md, 0, sizeof(attr_md));

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

        attr_md.vloc = BLE_GATTS_VLOC_STACK;
        attr_md.rd_auth = 0;
        attr_md.wr_auth = 0;
        attr_md.vlen = 1;

        memset(&attr_char_value, 0, sizeof(attr_char_value));

        attr_char_value.p_uuid = &ble_uuid;
        attr_char_value.p_attr_md = &attr_md;
        attr_char_value.init_len = 1 + sizeof(ble_its_cmd_data_t);
        attr_char_value.init_offs = 0;
        attr_char_value.max_len = BLE_ITS_CMD_CHAR_MAX_PAYLOAD;

        return sd_ble_gatts_characteristic_add(p_its->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_its->img_info_handles);
        /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
}

/**@brief Function for adding RX characteristic.
 *
 * @param[in] p_its       Nordic UART Service structure.
 * @param[in] p_its_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t rx_char_add(ble_its_t *p_its, const ble_its_init_t *p_its_init)
{
        ble_gatts_char_md_t char_md;
        ble_gatts_attr_t attr_char_value;
        ble_uuid_t ble_uuid;
        ble_gatts_attr_md_t attr_md;

        memset(&char_md, 0, sizeof(char_md));

        char_md.char_props.write = 1;
        char_md.char_props.write_wo_resp = 1;
        char_md.p_char_user_desc = NULL;
        char_md.p_char_pf = NULL;
        char_md.p_user_desc_md = NULL;
        char_md.p_cccd_md = NULL;
        char_md.p_sccd_md = NULL;

        ble_uuid.type = p_its->uuid_type;
        ble_uuid.uuid = BLE_UUID_ITS_RX_CHARACTERISTIC;

        memset(&attr_md, 0, sizeof(attr_md));

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

        attr_md.vloc = BLE_GATTS_VLOC_STACK;
        attr_md.rd_auth = 0;
        attr_md.wr_auth = 0;
        attr_md.vlen = 1;

        memset(&attr_char_value, 0, sizeof(attr_char_value));

        attr_char_value.p_uuid = &ble_uuid;
        attr_char_value.p_attr_md = &attr_md;
        attr_char_value.init_len = 1;
        attr_char_value.init_offs = 0;
        attr_char_value.max_len = BLE_ITS_MAX_RX_CHAR_LEN;

        return sd_ble_gatts_characteristic_add(p_its->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_its->rx_handles);
}

/**@brief Function for adding RX characteristic.
 *
 * @param[in] p_its       Nordic UART Service structure.
 * @param[in] p_its_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t rx_data_char_add(ble_its_t *p_its, const ble_its_init_t *p_its_init)
{
        ble_gatts_char_md_t char_md;
        ble_gatts_attr_t attr_char_value;
        ble_uuid_t ble_uuid;
        ble_gatts_attr_md_t attr_md;

        memset(&char_md, 0, sizeof(char_md));

        char_md.char_props.write = 1;
        char_md.char_props.write_wo_resp = 1;
        char_md.p_char_user_desc = NULL;
        char_md.p_char_pf = NULL;
        char_md.p_user_desc_md = NULL;
        char_md.p_cccd_md = NULL;
        char_md.p_sccd_md = NULL;

        ble_uuid.type = p_its->uuid_type;
        ble_uuid.uuid = BLE_UUID_ITS_RX_DATA_CHARACTERISTIC;

        memset(&attr_md, 0, sizeof(attr_md));

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

        attr_md.vloc = BLE_GATTS_VLOC_STACK;
        attr_md.rd_auth = 0;
        attr_md.wr_auth = 0;
        attr_md.vlen = 1;

        memset(&attr_char_value, 0, sizeof(attr_char_value));

        attr_char_value.p_uuid = &ble_uuid;
        attr_char_value.p_attr_md = &attr_md;
        attr_char_value.init_len = 1;
        attr_char_value.init_offs = 0;
        attr_char_value.max_len = BLE_ITS_MAX_RX_CHAR_LEN;

        return sd_ble_gatts_characteristic_add(p_its->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_its->rx_data_handles);
}

static uint32_t push_data_packets()
{
        uint32_t return_code = NRF_SUCCESS;
        uint32_t packet_length = m_max_data_length;
        uint32_t packet_size = 0;

        while (return_code == NRF_SUCCESS)
        {
                if (file_pos > file_size)
                {
                        break;
                }
                if ((file_size - file_pos) > packet_length)
                {
                        packet_size = packet_length;
                }
                else if ((file_size - file_pos) > 0)
                {
                        packet_size = file_size - file_pos;
                }

                if (packet_size > 0)
                {
                        return_code = ble_its_string_send(m_its, &file_data[file_pos], packet_size);
                        if (return_code == NRF_SUCCESS)
                        {
                                file_pos += packet_size;
                                m_image_send_byte += packet_size;
                        }
                }
                else
                {
                        file_size = 0;
                        break;
                }
        }
        return return_code;
}

static volatile bool nrf_error_resources = false;

void ble_its_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context)
{
        if ((p_context == NULL) || (p_ble_evt == NULL))
        {
                return;
        }

        ble_its_t *p_its = (ble_its_t *)p_context;

        switch (p_ble_evt->header.evt_id)
        {
        case BLE_GAP_EVT_CONNECTED:
                on_connect(p_its, p_ble_evt);
                break;

        case BLE_GAP_EVT_DISCONNECTED:
                on_disconnect(p_its, p_ble_evt);
                break;

        case BLE_GATTS_EVT_WRITE:
                on_write(p_its, p_ble_evt);
                break;

        case BLE_GATTS_EVT_HVC:
                NRF_LOG_INFO("BLE_GATTS_EVT_HVC");
                break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
        {
                //uint32_t count = p_ble_evt->evt.gatts_evt.params.hvn_tx_complete.count;
                if (file_size > 0)
                {
                        push_data_packets();
                }
                else
                {
                        ble_its_evt_t its_evt;
                        its_evt.evt_type = BLE_ITS_EVT_ITS_TX_DATA_COMPLETE_EVT;
                        p_its->evt_handler(p_its, &its_evt);
                }
                nrf_error_resources = false;
        }
        break;

        default:
                // No implementation needed.
                break;
        }
}

uint32_t ble_its_init(ble_its_t *p_its, const ble_its_init_t *p_its_init)
{
        uint32_t err_code;
        ble_uuid_t ble_uuid;
        ble_uuid128_t its_base_uuid = ITS_BASE_UUID;

        VERIFY_PARAM_NOT_NULL(p_its);
        VERIFY_PARAM_NOT_NULL(p_its_init);

        // Initialize the service structure.
        p_its->conn_handle = BLE_CONN_HANDLE_INVALID;
        p_its->evt_handler = p_its_init->evt_handler;
        p_its->is_notification_enabled = false;

        /**@snippet [Adding proprietary Service to S110 SoftDevice] */
        // Add a custom base UUID.
        err_code = sd_ble_uuid_vs_add(&its_base_uuid, &p_its->uuid_type);
        VERIFY_SUCCESS(err_code);

        ble_uuid.type = p_its->uuid_type;
        ble_uuid.uuid = BLE_UUID_ITS_SERVICE;

        // Add the service.
        err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                            &ble_uuid,
                                            &p_its->service_handle);
        /**@snippet [Adding proprietary Service to S110 SoftDevice] */
        VERIFY_SUCCESS(err_code);

        // Add the RX Characteristic.
        err_code = rx_char_add(p_its, p_its_init);
        VERIFY_SUCCESS(err_code);

        err_code = rx_data_char_add(p_its, p_its_init);
        VERIFY_SUCCESS(err_code);

        // Add the TX Characteristic.
        err_code = tx_char_add(p_its, p_its_init);
        VERIFY_SUCCESS(err_code);

        // Add the Image Info Characteristic.
        err_code = img_info_char_add(p_its, p_its_init);
        VERIFY_SUCCESS(err_code);

        return NRF_SUCCESS;
}

uint32_t ble_its_string_send(ble_its_t *p_its, uint8_t *p_string, uint16_t length)
{
        ble_gatts_hvx_params_t hvx_params;
        uint32_t err_code;

        if (nrf_error_resources)
        {
                return NRF_ERROR_RESOURCES;
        }

        VERIFY_PARAM_NOT_NULL(p_its);

        if ((p_its->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_its->is_notification_enabled))
        {
                return NRF_ERROR_INVALID_STATE;
        }

        if (length > BLE_ITS_MAX_DATA_LEN)
        {
                return NRF_ERROR_INVALID_PARAM;
        }

        memset(&hvx_params, 0, sizeof(hvx_params));
        hvx_params.handle = p_its->tx_handles.value_handle;
        hvx_params.p_data = p_string;
        hvx_params.p_len = &length;
        hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

        err_code = sd_ble_gatts_hvx(p_its->conn_handle, &hvx_params);

        if (err_code == NRF_ERROR_RESOURCES)
        {
                nrf_error_resources = true;
        }

        return err_code;
}

uint32_t ble_its_img_info_send(ble_its_t *p_its, ble_its_img_info_t *img_info)
{
        uint8_t data_buf[sizeof(ble_its_img_info_t)];
        ble_gatts_hvx_params_t hvx_params;

        VERIFY_PARAM_NOT_NULL(p_its);

        if ((p_its->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_its->is_info_char_notification_enabled))
        {
                return NRF_ERROR_INVALID_STATE;
        }

        uint16_t length = sizeof(ble_its_img_info_t);

        memcpy(&data_buf[0], img_info, sizeof(ble_its_img_info_t));

        // memcpy(&m_image_info, img_info, sizeof(ble_its_img_info_t));

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_its->img_info_handles.value_handle;
        hvx_params.p_data = data_buf;
        hvx_params.p_len = &length;
        hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

        return sd_ble_gatts_hvx(p_its->conn_handle, &hvx_params);
}

uint32_t ble_its_cmd_data_send(ble_its_t *p_its, ble_its_cmd_data_t *cmd_data)
{
        uint8_t data_buf[sizeof(ble_its_cmd_data_t)];
        ble_gatts_hvx_params_t hvx_params;

        VERIFY_PARAM_NOT_NULL(p_its);

        if ((p_its->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_its->is_info_char_notification_enabled))
        {
                return NRF_ERROR_INVALID_STATE;
        }

        uint16_t length = sizeof(ble_its_cmd_data_t);

        memcpy(&data_buf[0], cmd_data, sizeof(ble_its_cmd_data_t));

//        memcpy(&m_its_tx_cmd_data, cmd_data, sizeof(ble_its_cmd_data_t));

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_its->img_info_handles.value_handle;
        hvx_params.p_data = data_buf;
        hvx_params.p_len = &length;
        hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

        return sd_ble_gatts_hvx(p_its->conn_handle, &hvx_params);
}

uint32_t ble_its_send_file(ble_its_t *p_its, uint8_t *p_data, uint32_t data_length, uint32_t max_packet_length)
{
        uint32_t err_code;

        if ((p_its->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_its->is_notification_enabled))
        {
                return NRF_ERROR_INVALID_STATE;
        }

        if (file_size != 0)
        {
                return NRF_ERROR_BUSY;
        }

        // ble_its_img_info_t m_image_info;
        // m_image_info.file_size_bytes = data_length;
        // ble_its_img_info_send(p_its, &m_image_info);

        m_image_send_byte = 0;

        file_size = data_length;
        file_pos = 0;
        file_data = p_data;
        m_max_data_length = max_packet_length;
        m_its = p_its;

        err_code = push_data_packets();
        if (err_code == NRF_ERROR_RESOURCES)
                return NRF_SUCCESS;
        return err_code;
}

uint32_t ble_its_send_file_fragment(ble_its_t *p_its, uint8_t *p_data, uint32_t data_length)
{
        uint32_t err_code;

        if ((p_its->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_its->is_notification_enabled))
        {
                return NRF_ERROR_INVALID_STATE;
        }

        if (file_size != 0)
        {
                return NRF_ERROR_BUSY;
        }

        err_code = ble_its_string_send(p_its, p_data, data_length);
        return err_code;
}

bool ble_its_file_transfer_busy(void)
{
        return file_size != 0;
}

#endif // NRF_MODULE_ENABLED(BLE_NUS)
