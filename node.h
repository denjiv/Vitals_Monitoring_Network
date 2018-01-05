#ifndef NODE_HEADER
#define NODE_HEADER

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "app_uart.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_lbs_c.h"
#include "ble_conn_state.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define NRF_LOG_MODULE_NAME "NODE"

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE      GATT_MTU_SIZE_DEFAULT                      /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define CENTRAL_LINK_COUNT        0                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT     0                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
#define TOTAL_LINK_COUNT          CENTRAL_LINK_COUNT + PERIPHERAL_LINK_COUNT /**< Total number of links used by the application. */

#define APP_TIMER_PRESCALER       0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS      (2 + BSP_APP_TIMERS_NUMBER)                  /**< Maximum number of timers used by the application. */
#define APP_TIMER_OP_QUEUE_SIZE   2                                          /**< Size of timer operation queues. */

#define SCAN_INTERVAL             0x0004                                     /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW               0x0004                                     /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT              0x0000                                     /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL   MSEC_TO_UNITS(7.5, UNIT_1_25_MS)           /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL   MSEC_TO_UNITS(30, UNIT_1_25_MS)            /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY             0                                          /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT       MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Determines supervision time-out in units of 10 milliseconds. */

#define UUID16_SIZE               2                                          /**< Size of a UUID, in bytes. */

#define LEDBUTTON_LED             BSP_BOARD_LED_2                            /**< LED to indicate a change of state of the the Button characteristic on the peer. */

#define LEDBUTTON_BUTTON_PIN      BSP_BUTTON_0                               /**< Button that will write to the LED characteristic of the peer */
#define BUTTON_DETECTION_DELAY    APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)   /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define APP_BEACON_INFO_LENGTH          0x17                                 /**< Total length of information advertised by the Beacon. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)    /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
static ble_gap_adv_params_t m_adv_params;                                    /**< Parameters to be passed to the stack when starting advertising. */

static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH];                        /**< Last advertisement data collected from a beacon. */
static uint8_t prev_count = 0x00;                                            /**< Last index of the message. Used for filtering. */
static const char m_target_periph_name[] = "Main_Station";                   /**< Name of the device we try to connect to. This name is searched for in the scan report data*/

/** @brief Scan parameters requested for scanning and connection. */
static const ble_gap_scan_params_t m_scan_params =
{
    .active   = 0,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,

    #if (NRF_SD_BLE_API_VERSION == 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif

    #if (NRF_SD_BLE_API_VERSION == 3)
        .use_whitelist  = 0,
        .adv_dir_report = 0,
    #endif
};

/**@brief Connection parameters requested for connection. */
static const ble_gap_conn_params_t m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,
    (uint16_t)MAX_CONNECTION_INTERVAL,
    (uint16_t)SLAVE_LATENCY,
    (uint16_t)SUPERVISION_TIMEOUT
};


#endif
