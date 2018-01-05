#ifndef VITALS_HEADER
#define VITALS_HEADER

// standard
#include <stdbool.h>
#include <stdint.h>

// common
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "ble_advdata.h"
#include "bsp.h"
#include "app_timer.h"

// drivers and board specific
#include "nrf.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

#define CENTRAL_LINK_COUNT              0                                 /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           0                                 /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                 /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/
#define APP_CFG_NON_CONN_ADV_TIMEOUT    0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
#define APP_BEACON_INFO_LENGTH          0x18                              /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                              /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                              /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                              /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x0059                            /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x02                        /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x03, 0x04                        /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
                                        0x45, 0x56, 0x67, 0x78, \
                                        0x89, 0x9a, 0xab, 0xbc, \
                                        0xcd, 0xde, 0xef, 0xf0, 0x00
#define APP_TIMER_PRESCALER             0                                 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                 /**< Size of timer operation queues. */
#define TWI_INSTANCE_ID                 0                                 /**< Two wire interface identification. */
#define TEMP_ADDR                       0x5A                              /**< Temperature address. */
#define ACCEL_ADDR                      0x1C                              /**< Accelerometer address. */
#define SAMPLES_IN_BUFFER               17                                /**< Heart Rate buffer size. */
#define SAMPLE_RATE_HR		              40                                /**< Sample rate of each heart rate measurement. (ms) */

static ble_gap_adv_params_t m_adv_params;                                 /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
                         // this implementation.
};


/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);       /**< TWI driver configuration. */
static uint8_t temperature_data[2];                                             /**< Temperature data. */
static uint8_t accel_data[3];                                                   /**< Accelerometer data. */
static uint8_t state = 1;                                                       /**< State of the data. */
static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(3);               /**< Timer Instance. */
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];               /**< SAADC buffer. */
static nrf_ppi_channel_t     m_ppi_channel;                                     /**< PPI channel. */
static uint32_t              m_adc_evt_counter;                                 /**< Event Counter ADC. */
static bool motion = 0;                                                         /**< Motion flag. */
static uint8_t adc_count = 0;                                                   /**< ADC count variable. */

#endif
