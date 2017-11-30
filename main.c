/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
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
 * @defgroup tw_scanner main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "nrf.h"
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"

#define NRF_LOG_MODULE_NAME "APP"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

 /* Number of possible TWI addresses. */
 #define TEMP_ADDR          0x5A
 #define ACCEL_ADDR         0x1C
 #define PIN_IN             22
 #define PIN_OUT            17

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);


/**
 * @brief TWI initialization.
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

void in_pin_handler (nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
	nrf_drv_gpiote_out_clear(PIN_OUT);
	NRF_LOG_INFO("motion detected\r\n");
	NRF_LOG_FLUSH();
}

void motion_config (void) {
	ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_IN, true);
}

void temp_sensor (void){
	ret_code_t err_code;
    uint8_t sample_data[2];
	uint8_t read_command = 0x07;
    bool detected_device = false;

    err_code =  nrf_drv_twi_tx(&m_twi, TEMP_ADDR, &read_command, 1, true);
	APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_twi_rx(&m_twi, TEMP_ADDR, sample_data, sizeof(sample_data));
    if (err_code == NRF_SUCCESS)
    {
		detected_device = true;
		NRF_LOG_INFO("TWI device detected at address 0x%x.\r\n", TEMP_ADDR);
		NRF_LOG_FLUSH();
		uint16_t temp2 = ((uint16_t)sample_data[1] << 8) | sample_data[0];
				
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

void accel_config (void){
	ret_code_t err_code;
	uint8_t xyz_config = 0x1E;
	uint8_t trans_buf[2] = {0x2A, 0x18};
	uint8_t status;
	
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, trans_buf, sizeof(trans_buf), true); //standby, 100Hz
	APP_ERROR_CHECK(err_code);
	

	trans_buf[0] = 0x1D;
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
	trans_buf[0] = 0x1F;
	trans_buf[1] = 0x08;
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, trans_buf, sizeof(trans_buf), true);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, &trans_buf[0], 1, true);
	err_code = nrf_drv_twi_rx(&m_twi, ACCEL_ADDR, &status, 1);
	if (err_code != NRF_SUCCESS || status != 0x08) {
		NRF_LOG_INFO("error configurating FF_MT_TH\r\n");
		NRF_LOG_FLUSH();
	}
	
	trans_buf[0] = 0x20;
	trans_buf[1] = 0x2a;
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, trans_buf, sizeof(trans_buf), true);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, &trans_buf[0], 1, true);
	err_code = nrf_drv_twi_rx(&m_twi, ACCEL_ADDR, &status, 1);
	if (err_code != NRF_SUCCESS || status != 0x2A) {
		NRF_LOG_INFO("error configurating FF_MT_COUNT\r\n");
		NRF_LOG_FLUSH();
	}
	
	trans_buf[0] = 0x2D;
	trans_buf[1] = 0x20;
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, trans_buf, sizeof(trans_buf), true);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, &trans_buf[0], 1, true);
	err_code = nrf_drv_twi_rx(&m_twi, ACCEL_ADDR, &status, 1);
	if (err_code != NRF_SUCCESS || status != 0x20) {
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
	trans_buf[1] = 0x19;
	err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, trans_buf, sizeof(trans_buf), true);

}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
	
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
	
    twi_init();
	temp_sensor();
	
	
	accel_config();
	motion_config();
	
	uint8_t status;
	uint8_t int_register = 0x0C;
	uint8_t trans_reg = 0x1E;
	
    while (true)
    {
		nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, &int_register, 1, true);
		nrf_drv_twi_rx(&m_twi, ACCEL_ADDR, &status, 1);
		if (status == 0x20) {
			nrf_drv_gpiote_out_clear(PIN_OUT);
			NRF_LOG_INFO("motion detected\r\n");
			NRF_LOG_FLUSH();
			nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, &trans_reg, 1, true);
			nrf_drv_twi_rx(&m_twi, ACCEL_ADDR, &status, 1);
		}
        /* Empty loop. */
    }
}

/** @} */
