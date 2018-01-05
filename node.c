/**
 * @brief BLE LED Button Service central and client application main file.
 *
 * This example can be a central for up to 8 peripherals.
 * The peripheral is called ble_app_blinky and can be found in the ble_peripheral
 * folder.
 */

#include "node.h"

//static ble_lbs_c_t        m_ble_lbs_c[TOTAL_LINK_COUNT];           /**< Main structures used by the LED Button client module. */
//static ble_db_discovery_t m_ble_db_discovery[TOTAL_LINK_COUNT];    /**< list of DB structures used by the database discovery module. */

/**@brief Function to handle asserts in the SoftDevice.
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
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}



/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  type       Type of data to be looked for in advertisement data.
 * @param[in]  p_advdata  Advertisement report length and pointer to report.
 * @param[out] p_typedata If data type requested is found in the data report, type data length and
 *                        pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, uint8_array_t * p_advdata, uint8_array_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->size)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == type)
        {
            p_typedata->p_data = &p_data[index + 2];
            p_typedata->size   = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t ret;

    (void) sd_ble_gap_scan_stop();

    NRF_LOG_INFO("Start scanning for device name %s.\r\n", (uint32_t)m_target_periph_name);
    ret = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(ret);
}



/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_adv_report(const ble_evt_t * const p_ble_evt)
{
    uint32_t      err_code;
    uint8_array_t adv_data;
    uint8_array_t dev_name;
    bool          do_connect = false;

    // For readibility.
    const ble_gap_evt_t * const p_gap_evt    = &p_ble_evt->evt.gap_evt;
    const ble_gap_addr_t  * const peer_addr  = &p_gap_evt->params.adv_report.peer_addr;

    // Initialize advertisement report for parsing
    adv_data.p_data = (uint8_t *)p_gap_evt->params.adv_report.data;
    adv_data.size   = p_gap_evt->params.adv_report.dlen;

    // Check if the advertisement data is relevant (0x5900)
		if (adv_data.p_data[5] == 0x59 && adv_data.p_data[6] == 0x00 && prev_count != adv_data.p_data[8]) {
				sd_ble_gap_adv_stop();

        // Copy the scanned message to the advertisement message
				prev_count = adv_data.p_data[8];
				memcpy(m_beacon_info, (uint8_t *)&(adv_data.p_data[7]), APP_BEACON_INFO_LENGTH);

				uint32_t      err_code;
				ble_advdata_t advdata;
				uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

				ble_advdata_manuf_data_t manuf_specific_data;

				manuf_specific_data.company_identifier = 0x0159;
				manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
				manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

				// Build and set advertising data.
				memset(&advdata, 0, sizeof(advdata));

				advdata.name_type             = BLE_ADVDATA_NO_NAME;
				advdata.flags                 = flags;
				advdata.p_manuf_specific_data = &manuf_specific_data;

				err_code = ble_advdata_set(&advdata, NULL);
				APP_ERROR_CHECK(err_code);

				// Initialize advertising parameters (used when starting advertising).
				memset(&m_adv_params, 0, sizeof(m_adv_params));

				m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
				m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
				m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
				m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
				m_adv_params.timeout     = 0;

				err_code = sd_ble_gap_adv_start(&m_adv_params);
				APP_ERROR_CHECK(err_code);
		}
}

/**@brief Function for handling BLE Stack events concerning central applications.
 *
 * @details This function keeps the connection handles of central applications up-to-date. It
 *          parses scanning reports, initiating a connection attempt to peripherals when a
 *          target UUID is found, and manages connection parameter update requests. Additionally,
 *          it updates the status of LEDs used to report central applications activity.
 *
 * @note Since this function updates connection handles, @ref BLE_GAP_EVT_DISCONNECTED events
 *       should be dispatched to the target application before invoking this function.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(const ble_evt_t * const p_ble_evt)
{
    ret_code_t err_code;

    // For readability.
    const ble_gap_evt_t * const p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected, initiate DB
        // discovery, update LEDs status and resume scanning if necessary.
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connection 0x%x established, starting DB discovery.\r\n",
                         p_gap_evt->conn_handle);
            APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < TOTAL_LINK_COUNT);

            APP_ERROR_CHECK(err_code);

            if (err_code != NRF_ERROR_BUSY)
            {
                APP_ERROR_CHECK(err_code);
            }

            // Update LEDs status, and check if we should be looking for more
            // peripherals to connect to.
            bsp_board_led_on(CENTRAL_CONNECTED_LED);
            if (ble_conn_state_n_centrals() == CENTRAL_LINK_COUNT)
            {
                bsp_board_led_off(CENTRAL_SCANNING_LED);
            }
            else
            {
                // Resume scanning.
                bsp_board_led_on(CENTRAL_SCANNING_LED);
                scan_start();
            }
        } break; // BLE_GAP_EVT_CONNECTED

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            uint32_t central_link_cnt; // Number of central links.

            NRF_LOG_INFO("LBS central link 0x%x disconnected (reason: 0x%x)\r\n",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);

            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);

            // Start scanning
            scan_start();

            // Update LEDs status.
            bsp_board_led_on(CENTRAL_SCANNING_LED);
            central_link_cnt = ble_conn_state_n_centrals();
            if (central_link_cnt == 0)
            {
                bsp_board_led_off(CENTRAL_CONNECTED_LED);
            }
        } break;

        case BLE_GAP_EVT_ADV_REPORT:
            on_adv_report(p_ble_evt);
            break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection request timed out.\r\n");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 * been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    uint16_t conn_handle;
    conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    ble_conn_state_on_ble_evt(p_ble_evt);

		if (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT) {

				on_adv_report(p_ble_evt);

		}
}



/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    NRF_LOG_INFO("call to ble_lbs_on_db_disc_evt for instance %d and link 0x%x!\r\n",
                    p_evt->conn_handle,
                    p_evt->conn_handle);
}


/** @brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}



/** @brief Function to sleep until a BLE event is received by the application.
 */
static void power_manage(void)
{
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


int main(void)
{
    ret_code_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);
    ble_stack_init();
    db_discovery_init();

    // Start scanning for advertisements
		scan_start();

    while (1)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            // Wait for BLE events.
            power_manage();
        }
    }
}
