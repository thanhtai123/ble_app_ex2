/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
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
 * @brief BLE Heart Rate Collector application main file.
 *
 * This file contains the source code for a sample heart rate collector.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdm.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_db_discovery.h"
#include "ble_srv_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "app_util.h"
#include "app_error.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "ble_hrs_c.h"
#include "ble_bas_c.h"
#include "app_util.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "nrf_fstorage.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_ble_scan.h"
#include "boards.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_clock.h"

#define APP_BLE_CONN_CFG_TAG 1 /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_SOC_OBSERVER_PRIO 1 /**< Applications' SoC observer priority. You shouldn't need to modify this value. */

#define LESC_DEBUG_MODE 0 /**< Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND 1                               /**< Perform bonding. */
#define SEC_PARAM_MITM 0                               /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC 1                               /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS 0                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE /**< No I/O capabilities. */
#define SEC_PARAM_OOB 0                                /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE 7                       /**< Minimum encryption key size in octets. */
#define SEC_PARAM_MAX_KEY_SIZE 16                      /**< Maximum encryption key size in octets. */

#define SCAN_DURATION_WITELIST 3000 /**< Duration of the scanning in units of 10 milliseconds. */

#define TARGET_UUID BLE_UUID_HEART_RATE_SERVICE    /**< Target device uuid that application is looking for. */
#define BLEBUTTON_START BSP_BUTTON_0               // start the advertise service
#define BLEBUTTON_STOP BSP_BUTTON_1                // stop the advertise service and disconnect gatt if it's connected
#define BLEBUTTON_SHOW BSP_BUTTON_2                // stop the advertise service and disconnect gatt if it's connected
#define BLEBUTTON_DISC BSP_BUTTON_3                // stop the advertise service and disconnect gatt if it's connected
#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50) /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

/**@brief Macro to unpack 16bit unsigned UUID from octet stream. */
#define UUID16_EXTRACT(DST, SRC) \
    do                           \
    {                            \
        (*(DST)) = (SRC)[1];     \
        (*(DST)) <<= 8;          \
        (*(DST)) |= (SRC)[0];    \
    } while (0)

NRF_BLE_GQ_DEF(m_ble_gatt_queue, /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);
NRF_BLE_GATT_DEF(m_gatt);        /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc); /**< DB discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);        /**< Scanning module instance. */

static uint16_t m_conn_handle; /**< Current connection handle. */ /**< True if whitelist has been temporarily disabled. */
static bool m_memory_access_in_progress;                          /**< Flag to keep track of ongoing operations on persistent memory. */

/**< Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t const m_scan_param =
    {
        .active = 0x01,
#if (NRF_SD_BLE_API_VERSION > 7)
        .interval_us = NRF_BLE_SCAN_SCAN_INTERVAL * UNIT_0_625_MS,
        .window_us = NRF_BLE_SCAN_SCAN_WINDOW * UNIT_0_625_MS,
#else
        .interval = NRF_BLE_SCAN_SCAN_INTERVAL,
        .window = NRF_BLE_SCAN_SCAN_WINDOW,
#endif // (NRF_SD_BLE_API_VERSION > 7)
        .scan_phys = BLE_GAP_PHY_1MBPS,
};

#define MY_BASE_UUID_C                                     \
    {                                                      \
        0x4f, 0x3a, 0x15, 0x97, 0x99, 0x86, 0x10, 0xa9,    \
            0x5c, 0x40, 0x91, 0xc4, 0x00, 0x00, 0x28, 0xfb \
    }
#define SRV_UUID_C 0x0000
static uint8_t my_uuid_type;

static void scan_start(void);
static void scan_stop(void);

/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for handling the Heart Rate Service Client and Battery Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */

/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t *p_evt)
{
    // ble_db_discovery_t *p_my_db_disc;
    ble_gatt_db_srv_t *temp_db = &p_evt->params.discovered_db;
    switch (p_evt->evt_type)
    {
    case BLE_DB_DISCOVERY_COMPLETE:
        NRF_LOG_INFO("SERV UUID: %04X", (int)temp_db->srv_uuid.uuid);
        for (int i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            NRF_LOG_INFO("CHAR %d UUID %04X", i, temp_db->charateristics[i].characteristic.uuid.uuid);
            NRF_LOG_INFO(
                "char prop:%s%s%s%s%s",
                temp_db->charateristics[i].characteristic.char_props.read ? " read," : "",
                temp_db->charateristics[i].characteristic.char_props.write ? " write," : "",
                temp_db->charateristics[i].characteristic.char_props.write_wo_resp ? " wnr," : "",
                temp_db->charateristics[i].characteristic.char_props.notify ? " noti," : "",
                temp_db->charateristics[i].characteristic.char_props.indicate ? " indi." : ""
            );
        }
        sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        break;
    case BLE_DB_DISCOVERY_AVAILABLE:
        break;
    case BLE_DB_DISCOVERY_SRV_NOT_FOUND:
        NRF_LOG_INFO("BLE_DB_DISCOVERY_SRV_NOT_FOUND!");
        break;
    case BLE_DB_DISCOVERY_ERROR:
        NRF_LOG_INFO("BLE_DB_DISCOVERY_ERROR!");
        break;
    default:
        break;
    }
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const *p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
    case PM_EVT_PEERS_DELETE_SUCCEEDED:
        // Bonds are deleted. Start scanning.
        scan_start();
        break;

    default:
        break;
    }
}

/**
 * @brief Function for shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
    case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
        // Prepare wakeup buttons.
        err_code = bsp_btn_ble_sleep_mode_prepare();
        APP_ERROR_CHECK(err_code);
        break;
    default:
        break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);

static uint16_t myble_advdata_search(uint8_t const *p_encoded_data,
                                     uint16_t data_len,
                                     uint16_t *p_offset,
                                     uint8_t ad_type)
{
    if ((p_encoded_data == NULL) || (p_offset == NULL))
    {
        return 0;
    }

    uint16_t i = 0;

    while ((i + 1 < data_len) && ((i < *p_offset) || (p_encoded_data[i + 1] != ad_type)))
    {
        // Jump to next data.
        i += (p_encoded_data[i] + 1);
    }

    if (i >= data_len)
    {
        return 0;
    }
    else
    {
        uint16_t offset = i + 2;
        uint16_t len = p_encoded_data[i] ? (p_encoded_data[i] - 1) : 0;
        if (!len || ((offset + len) > data_len))
        {
            // Malformed. Zero length or extends beyond provided data.
            return 0;
        }
        *p_offset = offset;
        return len;
    }
}

#define REPORT_BUFF_SIZE 20
static uint8_t buff_cnt = 0;
typedef struct my_ble_data_s
{
    uint8_t p_data[31]; /**< Pointer to the data buffer provided to/from the application. */
    uint16_t len;       /**< Length of the data buffer, in bytes. */
} my_ble_data_t;

typedef struct report_data_s
{
    int8_t rssi;
    ble_gap_addr_t peer_addr;
    my_ble_data_t raw_data;
    bool is_rsp_type;
} report_data_t;
report_data_t report_data_buff[REPORT_BUFF_SIZE];
static uint8_t my_adv_uuid[16] = {0x4f, 0x3a, 0x15, 0x97, 0x99, 0x86, 0x10, 0xa9,
                                  0x5c, 0x40, 0x91, 0xc4, 0x00, 0x00, 0x28, 0xfb};
typedef struct complete_data_s
{
    int8_t rssi;
    char device_name[32];
    uint8_t name_size;
    ble_gap_addr_t peer_addr;
} complete_data_t;

typedef struct complete_buff_s
{
    complete_data_t complete_data_buff[10];
    uint8_t size;
    uint8_t cnt;
} complete_buff_t;
static complete_buff_t p_buff;

void print_list_adv(complete_buff_t *p_buff)
{

    for (int i = 0; i < p_buff->size; i++)
    {
        NRF_LOG_INFO("Device  %d name: %s", i, nrf_log_push((char *)p_buff->complete_data_buff[i].device_name));
        NRF_LOG_INFO("Mac addr %02X:%02X:%02X:%02X:%02X:%02X",
            p_buff->complete_data_buff[i].peer_addr.addr[0],
            p_buff->complete_data_buff[i].peer_addr.addr[1],
            p_buff->complete_data_buff[i].peer_addr.addr[2],
            p_buff->complete_data_buff[i].peer_addr.addr[3],
            p_buff->complete_data_buff[i].peer_addr.addr[4],
            p_buff->complete_data_buff[i].peer_addr.addr[5]
        );
        NRF_LOG_INFO("RSSI: %d",(int)p_buff->complete_data_buff[i].rssi);
    }
}
static void on_adv_report(ble_gap_evt_adv_report_t const *p_adv_report, complete_buff_t *p_buff)
{
    /**@brief store data to buff:
     * return if adv unconnectable
     * data len
     * raw data
     * rssi
     * peer addr
     * data type
     */
    if (!p_adv_report->type.connectable)
    {
        return;
    }
    report_data_t *temp_report = report_data_buff + buff_cnt;
    temp_report->raw_data.len = p_adv_report->data.len;
    memcpy(temp_report->raw_data.p_data, p_adv_report->data.p_data, temp_report->raw_data.len);
    temp_report->rssi = p_adv_report->rssi;
    memcpy(&temp_report->peer_addr, &p_adv_report->peer_addr, sizeof(ble_gap_addr_t));
    temp_report->is_rsp_type = p_adv_report->type.scan_response == 1 ? true : false;
    /**
     * process if buffer full
     */
    if (++buff_cnt == REPORT_BUFF_SIZE)
    {
        buff_cnt = 0;                            // reset the buffer
        uint8_t access_i = REPORT_BUFF_SIZE - 1; // use to access buffer
        uint8_t travel_i = REPORT_BUFF_SIZE;     // use to break the loop
        uint8_t proc_stat = 0;
        uint8_t found_index;
        //uint8_t is_found_rsp;

        report_data_t *p_temp_proc_rp;
        while (travel_i > 0)
        {
            travel_i--;
            access_i = (access_i - 1 < 0) ? (REPORT_BUFF_SIZE - 1) : (access_i - 1);
            p_temp_proc_rp = report_data_buff + access_i;
            uint8_t *p_raw_data = p_temp_proc_rp->raw_data.p_data; //
            uint16_t data_len = p_temp_proc_rp->raw_data.len;
            uint16_t found_offset = 0;
            uint16_t field_len;
            switch (proc_stat)
            {
            case 0:
                /**
                 * @todo
                 * find uuid.
                 * if found: compare vs MY_UUID
                 * if match:
                 *  store id
                 *  change state find name
                 * end.
                 *
                 */
                field_len = myble_advdata_search(p_raw_data,
                                                 data_len,
                                                 &found_offset,
                                                 BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE);
                if (field_len == 0)
                {
                    break;
                }
                if (memcmp(p_raw_data + found_offset, my_adv_uuid, 16) != 0)
                {
                    break;
                }
                // found uuid
                found_index = access_i;
                proc_stat = 1;
                //is_found_rsp = p_temp_proc_rp->is_rsp_type;

                access_i++;                  // check again if it contain name
                travel_i = REPORT_BUFF_SIZE; // go all over again
                break;
            case 1:
                /**
                 * compare peer addr
                 * if match: found name
                 * store
                 */
                // NRF_LOG_HEXDUMP_INFO(p_raw_data, data_len);
                if (memcmp(&report_data_buff[found_index].peer_addr.addr, &p_temp_proc_rp->peer_addr.addr, 6) != 0)
                {
                    break;
                }
                field_len = myble_advdata_search(p_raw_data,
                                                 data_len,
                                                 &found_offset,
                                                 BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME);
                if (field_len == 0)
                {
                    // NRF_LOG_INFO("Name: not define");
                    break;
                }
                // NRF_LOG_INFO("found exit");
                // NRF_LOG_HEXDUMP_INFO(p_raw_data, data_len);
                // nrf_ble_scan_stop();
                // store
                // check if exist
                for (int i = 0; i <= p_buff->size; i++)
                {
                    if (memcmp(&p_buff->complete_data_buff[i].peer_addr.addr, &p_temp_proc_rp->peer_addr.addr, 6) == 0)
                    {
                        return;
                    }
                }
                // adding into buff
                p_buff->size++;
                uint8_t curr_index = p_buff->cnt;
                p_buff->cnt = (p_buff->cnt + 1) % 10;
                complete_data_t *curr_data = p_buff->complete_data_buff + curr_index;
                memcpy(&curr_data->peer_addr, &p_temp_proc_rp->peer_addr, sizeof(ble_gap_addr_t));
                curr_data->rssi = p_temp_proc_rp->rssi;
                // add device name
                if (field_len != 0)
                {
                    memcpy(curr_data->device_name, p_raw_data + found_offset, field_len);
                    curr_data->device_name[field_len] = 0;
                }
                curr_data->name_size = field_len;
                return;
            default:
                break;
            }
        }
    }
}
/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
    ret_code_t err_code;
    ble_gap_evt_t const *p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_ADV_REPORT:
    {
        on_adv_report(&p_gap_evt->params.adv_report, &p_buff);
    }

    break;
    case BLE_GAP_EVT_CONNECTED:
    {
        NRF_LOG_INFO("Connected.");

        // Discover peer's services.
        err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
        APP_ERROR_CHECK(err_code);

        err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
        APP_ERROR_CHECK(err_code);

        if (ble_conn_state_central_conn_count() < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
        {
            scan_start();
        }
    }
    break;

    case BLE_GAP_EVT_DISCONNECTED:
    {
        NRF_LOG_INFO("Disconnected, reason 0x%x.",
                     p_gap_evt->params.disconnected.reason);

        err_code = bsp_indication_set(BSP_INDICATE_IDLE);
        APP_ERROR_CHECK(err_code);

        if (ble_conn_state_central_conn_count() < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
        {
            scan_start();
        }
    }
    break;

    case BLE_GAP_EVT_TIMEOUT:
    {
        NRF_LOG_INFO("Connection Request timed out.");
        if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
        {
            NRF_LOG_INFO("Connection Request timed out.");
        }
    }
    break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        // Accepting parameters requested by peer.
        err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                &p_gap_evt->params.conn_param_update_request.conn_params);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
        NRF_LOG_DEBUG("PHY update request.");
        ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
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
        NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
        break;

    case BLE_GAP_EVT_AUTH_KEY_REQUEST:
        NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
        break;

    case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
        NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
        break;

    case BLE_GAP_EVT_AUTH_STATUS:
        NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                     p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                     p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                     p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                     *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                     *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
        break;

    default:
        break;
    }
}

/**@brief SoftDevice SoC event handler.
 *
 * @param[in]   evt_id      SoC event.
 * @param[in]   p_context   Context.
 */
static void soc_evt_handler(uint32_t evt_id, void *p_context)
{
    switch (evt_id)
    {
    case NRF_EVT_FLASH_OPERATION_SUCCESS:
        /* fall through */
    case NRF_EVT_FLASH_OPERATION_ERROR:

        if (m_memory_access_in_progress)
        {
            m_memory_access_in_progress = false;
            scan_start();
        }
        break;

    default:
        // No implementation needed.
        break;
    }
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

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register handlers for BLE and SoC events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    NRF_SDH_SOC_OBSERVER(m_soc_observer, APP_SOC_OBSERVER_PRIO, soc_evt_handler, NULL);
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond = SEC_PARAM_BOND;
    sec_param.mitm = SEC_PARAM_MITM;
    sec_param.lesc = SEC_PARAM_LESC;
    sec_param.keypress = SEC_PARAM_KEYPRESS;
    sec_param.io_caps = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob = SEC_PARAM_OOB;
    sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc = 1;
    sec_param.kdist_own.id = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for disabling the use of whitelist for scanning.
 */

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
    case BSP_EVENT_SLEEP:
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
        break;

    case BSP_EVENT_DISCONNECT:
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
        break;

    default:
        break;
    }
}

/**
 * @brief Database discovery collector initialization.
 */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;
    ble_uuid128_t reg_uuid_d = {MY_BASE_UUID_C};
    ret_code_t err_code;

    err_code = sd_ble_uuid_vs_add(&reg_uuid_d, &my_uuid_type);
    APP_ERROR_CHECK(err_code);

    memset(&db_init, 0, sizeof(db_init));
    ble_uuid_t reg_srv_disc_uuid = {
        .uuid = 0x0000,
        .type = my_uuid_type,
    };
    db_init.evt_handler = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);

    ble_db_discovery_evt_register(&reg_srv_disc_uuid);
}

/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;

    if (nrf_fstorage_is_busy(NULL))
    {
        m_memory_access_in_progress = true;
        return;
    }

    NRF_LOG_INFO("Starting scan.");

    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function to start scanning.
 */
static void scan_stop(void)
{
    NRF_LOG_INFO("Stop scan.");
    memset(&p_buff, 0, sizeof(complete_buff_t));
    nrf_ble_scan_stop();
}

static void disc_srv(complete_buff_t *p_buff)
{
    nrf_ble_scan_stop();
    int max_rssi_index = 0;
    int max_value = -500;
    for (int i = 0; i < p_buff->size; i++)
    {
        if (p_buff->complete_data_buff[i].rssi > max_value)
        {
            max_value = p_buff->complete_data_buff[i].rssi;
            max_rssi_index = i;
        }
    }
    sd_ble_gap_connect(&(p_buff->complete_data_buff[max_rssi_index].peer_addr),
                       &m_scan.scan_params,
                       &m_scan.conn_params,
                       APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing the power management module. */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt)
{
    switch (p_evt->evt_id)
    {
    case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED:
    {
        NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                     p_evt->conn_handle,
                     p_evt->params.att_mtu_effective);
    }
    break;

    case NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED:
    {
        NRF_LOG_INFO("Data length for connection 0x%x updated to %d.",
                     p_evt->conn_handle,
                     p_evt->params.data_length);
    }
    break;

    default:
        break;
    }
}

static void scan_evt_handler(scan_evt_t const *p_scan_evt)
{
    ret_code_t err_code;
    switch (p_scan_evt->scan_evt_id)
    {
    case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
    {
        NRF_LOG_INFO("scan_evt_connecting_error");
        err_code = p_scan_evt->params.connecting_err.err_code;
        APP_ERROR_CHECK(err_code);
    }
    break;

    case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
    {
        NRF_LOG_INFO("scan_evt_time_out");
        scan_start();
    }
    break;
    default:
        break;
    }
}

static void button_event_handler1(uint8_t pin_no, uint8_t button_action)
{
    bsp_board_led_invert(BSP_BOARD_LED_0);
    bsp_board_led_off(BSP_BOARD_LED_1);
    if (button_action != APP_BUTTON_RELEASE)
    {
        return;
    }
    switch (pin_no)
    {
    case BLEBUTTON_START:
        // nrf_ble_scan_start();
        scan_start();
        break;
    case BLEBUTTON_STOP:
        scan_stop();
        break;
    case BLEBUTTON_SHOW:
        print_list_adv(&p_buff);
        break;
    case BLEBUTTON_DISC:
        // print_list_adv(&p_buff);
        disc_srv(&p_buff);
        break;
    default:
        break;
    }
}

/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;
    bsp_board_init(BSP_INIT_BUTTONS | BSP_INIT_LEDS);
    bsp_board_led_on(BSP_BOARD_LED_0);
    bsp_board_led_on(BSP_BOARD_LED_1);
    // The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
        {
            {BLEBUTTON_STOP, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_event_handler1},
            {BLEBUTTON_START, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_event_handler1},
            {BLEBUTTON_DISC, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_event_handler1},
            {BLEBUTTON_SHOW, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_event_handler1}

        };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
    app_button_enable();
}

/**@brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initialization scanning and setting filters.
 */
static void scan_init(void)
{
    ret_code_t err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.p_scan_param = &m_scan_param;
    init_scan.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    memset(&p_buff, 0, sizeof(complete_buff_t));
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details Handle any pending log operation(s), then sleep until the next event occurs.
 */
static void idle_state_handle(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_lesc_request_handler();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_FLUSH();
    nrf_pwr_mgmt_run();
}

int main(void)
{

    // Initialize.
    log_init();
    timer_init();
    power_management_init();
    ble_stack_init();
    gatt_init();
    peer_manager_init();
    db_discovery_init();
    scan_init();

    // Start execution.
    NRF_LOG_INFO("1Heart Rate collector example started1.");
    buttons_init();
    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}
