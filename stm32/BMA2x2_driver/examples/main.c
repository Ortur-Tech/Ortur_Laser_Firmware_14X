/**
 *******************************HelloAIR*************************************
 *  HA-MOT-BMAxxx
 *  For more products, please visit:
 *  https://shop451454120.taobao.com
 */



/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
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
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "stdio.h"
#include "bma2x2.h"

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic"
static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

extern uint8_t V_BMA2x2RESOLUTION_U8;

/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *
 *  @param[in] cs           : Chip select to enable the sensor.
 *  @param[in] reg     			: Register address.
 *  @param[in] pdata 				: Pointer to the data buffer whose data has to be written.
 *  @param[in] len       		: No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t spi_write_bytes(uint8_t cs, uint8_t reg, uint8_t * pdata, uint8_t len)
{
	int8_t	ret;
	ret_code_t err_code;
	uint8_t tx_buf[len+1], rx_buf[len+1], *p, i;
	
	// Reset rx buffer and transfer done flag
	memset(tx_buf, 0, (len + 1));
  memset(rx_buf, 0, (len + 1));
	tx_buf[0] = reg & 0x7f;		//bit7=0, write	
	p = pdata;
	for(i = 0; i < len; i ++)
		{
			tx_buf[i + 1] = *p;
			p ++;
		}
	spi_xfer_done = false;
  err_code = nrf_drv_spi_transfer(&spi, tx_buf, (len + 1), rx_buf, (len + 1));	
	if(err_code != NRFX_SUCCESS)
	{
		APP_ERROR_CHECK(err_code);
		return -1;
	}
	while (!spi_xfer_done)
		{
			__WFE();
		} 
	return 0;
}

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in] cs       : Chip select to enable the sensor.
 *  @param[in] reg 			: Register address.
 *  @param[out] pdata   : Pointer to the data buffer to store the read data.
 *  @param[in] len   		: No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t spi_read_bytes(uint8_t cs, uint8_t reg, uint8_t * pdata, uint8_t len)
{
	int8_t	ret;
	ret_code_t err_code;
	uint8_t tx_buf[len+1], rx_buf[len+1], *p, i;
	
	// Reset rx buffer and transfer done flag
	memset(tx_buf, 0, (len + 1));
  memset(rx_buf, 0, (len + 1));
	tx_buf[0] = reg | 0x80;			//bit7=1, read				
	spi_xfer_done = false;
  err_code = nrf_drv_spi_transfer(&spi, tx_buf, (len + 1), rx_buf, (len + 1));	
	if(err_code != NRFX_SUCCESS)
	{
		APP_ERROR_CHECK(err_code);
		return -1;
	}
	while (!spi_xfer_done)
		{
			__WFE();
		} 
	
	p = pdata;
	for(i = 0; i < len; i ++)
		{
			*p = rx_buf[i + 1];
			p ++;
		}
	return 0;
}

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
//    NRF_LOG_INFO("Transfer completed.");
//    if (m_rx_buf[0] != 0)
//    {
//        NRF_LOG_INFO(" Received:");
//        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
//    }
}

/**
 * @brief SPI initialization.
 */
void spi_init (void)
{
    ret_code_t err_code;

//		nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
		nrf_drv_spi_config_t spi_config;
	
    spi_config.ss_pin   = SPI_SS_PIN;			//31
    spi_config.miso_pin = SPI_MISO_PIN;		//30
    spi_config.mosi_pin = SPI_MOSI_PIN;		//29
    spi_config.sck_pin  = SPI_SCK_PIN;		//26
		spi_config.irq_priority = 6;
		spi_config.orc = 0xff;
		spi_config.frequency = NRF_DRV_SPI_FREQ_1M;
		spi_config.mode = NRF_DRV_SPI_MODE_0;
		spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
	
    err_code = nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL);
    APP_ERROR_CHECK(err_code);
}

int main(void)
{
	int8_t rslt;
	struct bma2x2_t dev;
  float x, y, z;
	/* bma2x2acc_data structure used to read accel xyz data*/
	struct bma2x2_accel_data sample_xyz;
	uint8_t bw_value_u8, whoami;
	float sensitivity;
	
  APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
  NRF_LOG_DEFAULT_BACKENDS_INIT();

	NRF_LOG_INFO("\r\nHA-MOT-BMAxxx example started.\n");
  NRF_LOG_FLUSH();
	
	spi_init();
	
	/* Map the delay function pointer with the function responsible for implementing the delay */
  dev.delay_msec = nrf_delay_ms;
  /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
  dev.bus_read = spi_read_bytes;
  dev.bus_write = spi_write_bytes;
	
	rslt = bma2x2_init(&dev);
	switch (dev.chip_id)
		{
			case 0x03:
				V_BMA2x2RESOLUTION_U8 = BMA2x2_10_RESOLUTION;
				sensitivity = 64.0;			//FS=+/-8g
				NRF_LOG_INFO("chip_id=%d, the device is BMA250 or BMA222.\n",dev.chip_id);
				break;

			case 0xf9:
				V_BMA2x2RESOLUTION_U8 = BMA2x2_10_RESOLUTION;
				sensitivity = 64.0;			//FS=+/-8g
				NRF_LOG_INFO("chip_id=%d, the device is BMA250E.\n",dev.chip_id);
				break;
			
			case 0xfa:
				V_BMA2x2RESOLUTION_U8 = BMA2x2_12_RESOLUTION;
				sensitivity = 256.0;			//FS=+/-8g
				NRF_LOG_INFO("chip_id=%d, the device is BMA253 or BMA255.\n",dev.chip_id);
				break;

			case 0xfb:
				V_BMA2x2RESOLUTION_U8 = BMA2x2_14_RESOLUTION;
				sensitivity = 1024.0;			//FS=+/-8g
				NRF_LOG_INFO("chip_id=%d, the device is BMA280.\n",dev.chip_id);
				break;

			case 0x90:
				V_BMA2x2RESOLUTION_U8 = BMA2x2_12_RESOLUTION;
				sensitivity = 256.0;			//FS=+/-8g
				NRF_LOG_INFO("chip_id=%d, the device is BMA400.\n",dev.chip_id);
				break;
			
			default:
				NRF_LOG_INFO("Not support device!\n");
				NRF_LOG_FLUSH();
				while(1);
				break;
		}  
	NRF_LOG_FLUSH();
	
	rslt = bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
	if(!(rslt == 0))
	{
		NRF_LOG_INFO(" bma2x2_set_power_mode error!\n");
		NRF_LOG_FLUSH();
	}
	
	rslt = bma2x2_set_range(BMA2x2_RANGE_8G);
	if(!(rslt == 0))
	{	
		NRF_LOG_INFO(" bma2x2_set_range error!\n");
		NRF_LOG_FLUSH();
	}
	
	bw_value_u8 = 0x08;/* set bandwidth of 7.81Hz*/
	rslt = bma2x2_set_bw(bw_value_u8);
	if(!(rslt == 0))
	{
		NRF_LOG_INFO(" bma2x2_set_bw error!\n");
		NRF_LOG_FLUSH();
	}
	
  while (1)
	{
		/* accessing the bma2x2acc_data parameter by using sample_xyz*/
		/* Read the accel XYZ data*/
		rslt = bma2x2_read_accel_xyz(&sample_xyz);
		x = (float)(sample_xyz.x) / sensitivity;
		y = (float)(sample_xyz.y) / sensitivity;
		z = (float)(sample_xyz.z) / sensitivity;
		NRF_LOG_INFO("x: " NRF_LOG_FLOAT_MARKER "%6.2fg", NRF_LOG_FLOAT(x));
		NRF_LOG_INFO("y: " NRF_LOG_FLOAT_MARKER "%6.2fg", NRF_LOG_FLOAT(y));
		NRF_LOG_INFO("z: " NRF_LOG_FLOAT_MARKER "%6.2fg\n", NRF_LOG_FLOAT(z));
		NRF_LOG_FLUSH();
		dev.delay_msec(1000);
   }
}
