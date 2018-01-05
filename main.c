
/** @file
 *
 * @defgroup ble_vitals_beacon_main main.c
 * @{
 * @ingroup ble_vitals_beacon
 * @brief Vitals Monitoring Beacon Transmitter main file.
 *
 * This file contains the source code for a Vitals Monitoring Beacon Transmitter.
 *
 */

#include "vitals.h"

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
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
    m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);

    //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    //APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for doing power management.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handing timer interupts.
  Instance is needed for the compiler.
 */
void timer_handler(nrf_timer_event_t event_type, void * p_context);

/**@brief SAADC Initialization function.
 */
void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_31250Hz;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event*/
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, SAMPLE_RATE_HR);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}

/**@brief SAADC enabler function.
 */
void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

/**@brief Temperature data extraction function.
 */
void temp_sensor (void)
{
		ret_code_t err_code;
		uint8_t read_command = 0x07;
    bool detected_device = false;

    err_code =  nrf_drv_twi_tx(&m_twi, TEMP_ADDR, &read_command, 1, true);
    err_code = nrf_drv_twi_rx(&m_twi, TEMP_ADDR, temperature_data, sizeof(temperature_data));
    if (err_code == NRF_SUCCESS)
    {
			detected_device = true;
			NRF_LOG_INFO("TWI device detected at address 0x%x.\r\n", TEMP_ADDR);
			NRF_LOG_FLUSH();
			uint16_t temp2 = ((uint16_t)temperature_data[1] << 8) | temperature_data[0];

			double temp = 0.02 * temp2;
			temp = temp - 273.15;


			NRF_LOG_INFO("Data read:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(temp));
			NRF_LOG_FLUSH();
    }

    if (!detected_device)
    {
        NRF_LOG_INFO("No device was found.\r\n");
        NRF_LOG_FLUSH();
    }
}

/**@brief Accelerometer data extraction function.
 */
void read_accel()
{
	uint8_t err_code;
	uint8_t accel_regs[3] = {0x01, 0x03, 0x05};

	//x address
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, &accel_regs[0], 1, true);
	err_code = nrf_drv_twi_rx(&m_twi, ACCEL_ADDR, accel_data, 3);
	APP_ERROR_CHECK(err_code);

	if (err_code == NRF_SUCCESS) {

			double x = accel_data[0];
			double y = accel_data[1];
			double z = accel_data[2];
			NRF_LOG_INFO("X accel: %x\r\n", accel_data[0]);
			NRF_LOG_FLUSH();
		  NRF_LOG_INFO("Y accel: %x\r\n", accel_data[1]);
			NRF_LOG_FLUSH();
		  NRF_LOG_INFO("Z accel: %x\r\n", accel_data[2]);
			NRF_LOG_FLUSH();
		  if (x > 79) x -= 256;
			if (y > 79) y -= 256;
			if (z > 79) z -= 256;
			x = x * 0.0672 * 9.8;
			y = y * 0.0672 * 9.8;
			z = z * 0.0672 * 9.8;
			NRF_LOG_INFO("X accel:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(x));
			NRF_LOG_FLUSH();
			NRF_LOG_INFO("Y accel:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(y));
			NRF_LOG_FLUSH();
			NRF_LOG_INFO("Z accel:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(z));
			NRF_LOG_FLUSH();
	}
}

/**@brief SAADC Callback function.
Collects sensor data and starts BLE advertisement.
 */
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
				ret_code_t err_code;
				nrf_gpio_pin_toggle(7);
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

				uint8_t index = 2;

				m_beacon_info[1] = temp++;
				for (uint8_t i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            m_beacon_info[index++] = p_event->data.done.p_buffer[i];
        }
				temp_sensor();
				m_beacon_info[index++] = temperature_data[1];
				m_beacon_info[index++] = temperature_data[0];
				read_accel();
				m_beacon_info[index++] = accel_data[0];
				m_beacon_info[index++] = accel_data[1];
				m_beacon_info[index++] = accel_data[2];
				sd_ble_gap_adv_stop();
				advertising_init();
				advertising_start();

        // Timeout after 30 mesurements
				if (temp%30 == 0) {
						if (motion == 0) {
								NRF_POWER->SYSTEMOFF = 1;
						}
						motion = 0;
				}


    }
}

/**@brief SAADC Initialization function.
 */
void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

}

/**@brief TWI Initialization function.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/**@brief Acceleration configuration function.
 */
void accel_config (void)
{
	ret_code_t err_code;
	uint8_t xyz_config = 0xF8;
	uint8_t trans_buf[2] = {0x2A, 0x1A};
	uint8_t status;

	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, trans_buf, sizeof(trans_buf), true); //standby, 100Hz
	APP_ERROR_CHECK(err_code);


	trans_buf[0] = 0x15;
	trans_buf[1] = xyz_config;// FF_MT_CFG, latch on, free fall, xyz on
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, trans_buf, sizeof(trans_buf), true);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, &trans_buf[0], 1, true);
	err_code = nrf_drv_twi_rx(&m_twi, ACCEL_ADDR, &status, 1);
	if (err_code != NRF_SUCCESS || status != xyz_config) {
		NRF_LOG_INFO("error configurating FF_MT_CFG\r\n");
		NRF_LOG_FLUSH();
	} else {
		NRF_LOG_INFO("FF_MT_CFG: 0x%x\r\n", status);
		NRF_LOG_FLUSH();
	}

	// FF_MT_TH, threshold = 0.063g
	trans_buf[0] = 0x17;
	trans_buf[1] = 0x10;
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, trans_buf, sizeof(trans_buf), true);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, &trans_buf[0], 1, true);
	err_code = nrf_drv_twi_rx(&m_twi, ACCEL_ADDR, &status, 1);
	if (err_code != NRF_SUCCESS || status != 0x10) {
		NRF_LOG_INFO("error configurating FF_MT_TH\r\n");
		NRF_LOG_FLUSH();
	}

	trans_buf[0] = 0x18;
	trans_buf[1] = 0x10;
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, trans_buf, sizeof(trans_buf), true);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, &trans_buf[0], 1, true);
	err_code = nrf_drv_twi_rx(&m_twi, ACCEL_ADDR, &status, 1);
	if (err_code != NRF_SUCCESS || status != 0x10) {
		NRF_LOG_INFO("error configurating FF_MT_COUNT\r\n");
		NRF_LOG_FLUSH();
	}

	trans_buf[0] = 0x0E;
	trans_buf[1] = 0x02;
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, trans_buf, sizeof(trans_buf), true);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, &trans_buf[0], 1, true);
	err_code = nrf_drv_twi_rx(&m_twi, ACCEL_ADDR, &status, 1);
	if (err_code != NRF_SUCCESS || status != 0x02) {
		NRF_LOG_INFO("error configurating XYZ_CONFIG\r\n");
		NRF_LOG_FLUSH();
	}

	trans_buf[0] = 0x2D;
	trans_buf[1] = 0x04;
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, trans_buf, sizeof(trans_buf), true);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, &trans_buf[0], 1, true);
	err_code = nrf_drv_twi_rx(&m_twi, ACCEL_ADDR, &status, 1);
	if (err_code != NRF_SUCCESS || status != 0x04) {
		NRF_LOG_INFO("error configurating CNTL_REG4\r\n");
		NRF_LOG_FLUSH();
	}

	trans_buf[0] = 0x2E;
	trans_buf[1] = 0x00;
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, trans_buf, sizeof(trans_buf), true);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, &trans_buf[0], 1, true);
	err_code = nrf_drv_twi_rx(&m_twi, ACCEL_ADDR, &status, 1);
	if (err_code != NRF_SUCCESS || status != 0x00) {
		NRF_LOG_INFO("error configurating CNTL_REG5\r\n");
		NRF_LOG_FLUSH();
	}


	trans_buf[0] = 0x2A;
	trans_buf[1] = 0x1B;
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, trans_buf, sizeof(trans_buf), true);


}

/**@brief Accelerometer motion interupt handler function.
 */
void in_pin_handler (nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	uint8_t status;
	uint8_t trans_reg = 0x16;
	motion = 1;
	nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, &trans_reg, 1, true);  // clear flags
	nrf_drv_twi_rx(&m_twi, ACCEL_ADDR, &status, 1);
}

/**@brief Accelerometer motion interupt configuration function.
 */
void gpiote_config (void)
{
	ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(25, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(25, true);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    // Initialize.
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
		nrf_gpio_cfg_output(7);
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    ble_stack_init();

		// Start Heart Rate sensor
		saadc_init();
    saadc_sampling_event_init();
    saadc_sampling_event_enable();

    // Start I2C
		twi_init();
		temp_sensor();
		accel_config();
		gpiote_config();

	  // Start execution.
		advertising_init();
		advertising_start();
    while (1)
    {
				power_manage();
		}
}


/**
 * @}
 */
