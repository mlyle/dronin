/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_MS5611 MS5611 Functions
 * @brief Hardware functions to deal with the altitude pressure sensor
 * @{
 *
 * @file       pios_ms5611.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2014
 * @author     dRonin, http://dRonin.org/, Copyright (C) 2017
 * @brief      MS5611 Pressure Sensor Routines
 * @see        The GNU Public License (GPL) Version 3
 *
 ******************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>
 */

/* Project Includes */
#include "pios.h"

#if defined(PIOS_INCLUDE_MS5611)

#include "pios_ms5611_priv.h"
#include "pios_semaphore.h"
#include "pios_thread.h"
#include "pios_queue.h"

/* Private constants */
#define PIOS_MS5611_OVERSAMPLING oversampling

/* MS5611 Addresses */
#define MS5611_I2C_ADDR_0x76    0x76
#define MS5611_I2C_ADDR_0x77    0x77
#define MS5611_RESET            0x1E
#define MS5611_CALIB_ADDR       0xA2  /* First sample is factory stuff */
#define MS5611_CALIB_LEN        16
#define MS5611_ADC_READ         0x00
#define MS5611_PRES_ADDR        0x40
#define MS5611_TEMP_ADDR        0x50
#define MS5611_ADC_MSB          0xF6
#define MS5611_P0               101.3250f

/* Private Variables */
uint8_t ms5611_i2c_addr;

/* Private methods */
static int32_t PIOS_MS5611_Read(uint8_t address, uint8_t * buffer, uint8_t len);
static int32_t PIOS_MS5611_WriteCommand(uint8_t command);
static bool PIOS_MS5611_Callback(void *ctx, void *output,
		int ms_to_wait, int *next_call);

/* Private types */

/* Local Types */

enum pios_ms5611_dev_magic {
	PIOS_MS5611_DEV_MAGIC = 0xefba8e1d,
};

enum conversion_type {
	NONE_CONV,
	PRESSURE_CONV,
	TEMPERATURE_CONV
};

struct ms5611_dev {
	const struct pios_ms5611_cfg * cfg;
	pios_i2c_t i2c_id;

	int64_t pressure_unscaled;
	int64_t temperature_unscaled;
	uint16_t calibration[6];
	enum conversion_type current_conversion_type;
	enum pios_ms5611_dev_magic magic;

	int32_t interleave_count;
	struct pios_semaphore *busy;
};

static struct ms5611_dev *dev;

/**
 * @brief Allocate a new device
 */
static struct ms5611_dev * PIOS_MS5611_alloc(void)
{
	struct ms5611_dev *ms5611_dev;

	ms5611_dev = (struct ms5611_dev *)PIOS_malloc(sizeof(*ms5611_dev));
	if (!ms5611_dev)
		return (NULL);

	memset(ms5611_dev, 0, sizeof(*ms5611_dev));

	ms5611_dev->magic = PIOS_MS5611_DEV_MAGIC;

	ms5611_dev->busy = PIOS_Semaphore_Create();
	PIOS_Assert(ms5611_dev->busy != NULL);

	return ms5611_dev;
}

/**
 * @brief Validate the handle to the i2c device
 * @returns 0 for valid device or <0 otherwise
 */
static int32_t PIOS_MS5611_Validate(struct ms5611_dev *dev)
{
	if (dev == NULL)
		return -1;
	if (dev->magic != PIOS_MS5611_DEV_MAGIC)
		return -2;
	if (dev->i2c_id == 0)
		return -3;
	return 0;
}

/**
 * Initialise the MS5611 sensor
 */
int32_t PIOS_MS5611_Init(const struct pios_ms5611_cfg *cfg, pios_i2c_t i2c_device)
{
	dev = (struct ms5611_dev *)PIOS_MS5611_alloc();
	if (dev == NULL)
		return -1;

	dev->i2c_id = i2c_device;
	dev->cfg = cfg;

	/* Which I2C address is being used? */
	if (dev->cfg->use_0x76_address == true)
		ms5611_i2c_addr = MS5611_I2C_ADDR_0x76;
	else
		ms5611_i2c_addr = MS5611_I2C_ADDR_0x77;

	if (PIOS_MS5611_WriteCommand(MS5611_RESET) != 0)
		return -2;

	PIOS_DELAY_WaitmS(20);

	uint8_t data[2];

	/* Calibration parameters */
	for (int i = 0; i < NELEMENTS(dev->calibration); i++) {
		PIOS_MS5611_Read(MS5611_CALIB_ADDR + i * 2, data, 2);
		dev->calibration[i] = (data[0] << 8) | data[1];
	}

	PIOS_SENSORS_RegisterCallback(PIOS_SENSOR_BARO,
			PIOS_MS5611_Callback, dev);

	return 0;
}

/**
 * Claim the MS5611 device semaphore.
 * \return 0 if no error
 * \return -1 if timeout before claiming semaphore
 */
static int32_t PIOS_MS5611_ClaimDevice(void)
{
	PIOS_Assert(PIOS_MS5611_Validate(dev) == 0);

	return PIOS_Semaphore_Take(dev->busy, PIOS_SEMAPHORE_TIMEOUT_MAX) == true ? 0 : 1;
}

/**
 * Release the MS5611 device semaphore.
 * \return 0 if no error
 */
static int32_t PIOS_MS5611_ReleaseDevice(void)
{
	PIOS_Assert(PIOS_MS5611_Validate(dev) == 0);

	return PIOS_Semaphore_Give(dev->busy) == true ? 0 : 1;
}

/**
 * @brief Return the delay for the current osr
 */
static int32_t PIOS_MS5611_GetDelay()
{
	if (PIOS_MS5611_Validate(dev) != 0)
		return 100;

	switch(dev->cfg->oversampling) {
	case MS5611_OSR_256:
		return 2;
	case MS5611_OSR_512:
		return 2;
	case MS5611_OSR_1024:
		return 3;
	case MS5611_OSR_2048:
		return 5;
	case MS5611_OSR_4096:
		return 10;
	default:
		break;
	}
	return 10;
}

int PIOS_MS5611_I2CSampleTxn(uint8_t *data, int len,
		enum conversion_type next_type)
{
	uint8_t command;
	uint8_t address = MS5611_ADC_READ;

	const struct pios_i2c_txn txn_list[] = {
		{
			.info = __func__,
			.addr = ms5611_i2c_addr,
			.rw = PIOS_I2C_TXN_WRITE,
			.len = 1,
			.buf = &address,
		},
		{
			.info = __func__,
			.addr = ms5611_i2c_addr,
			.rw = PIOS_I2C_TXN_READ,
			.len = len,
			.buf = data,
		},
		{
			.info = __func__,
			.addr = ms5611_i2c_addr,
			.rw = PIOS_I2C_TXN_WRITE,
			.len = 1,
			.buf = &command,
		 },
	};

	/* Start the conversion */
	switch (next_type) {
	default:
	case TEMPERATURE_CONV:
		command = MS5611_TEMP_ADDR + dev->cfg->oversampling;
		dev->current_conversion_type = TEMPERATURE_CONV;
		break;
	case PRESSURE_CONV:
		command = MS5611_PRES_ADDR + dev->cfg->oversampling;
		dev->current_conversion_type = PRESSURE_CONV;
		break;
	}

	int ret = PIOS_I2C_TransferAsync(dev->i2c_id, txn_list,
			NELEMENTS(txn_list), false);

	if (ret) return ret;

	bool completed = false;

	ret = PIOS_I2C_WaitAsync(dev->i2c_id, 0xfffffff, &completed);

	PIOS_Assert(completed);

	return ret;
}

/**
* Read the ADC conversion value (once ADC conversion has completed)
* \return 0 if successfully read the ADC, -1 if failed
*/
static int32_t PIOS_MS5611_ReadADC(enum conversion_type next_conv)
{
	if (PIOS_MS5611_Validate(dev) != 0)
		return -1;

	uint8_t data[3];

	enum conversion_type this_conv = dev->current_conversion_type;

	if (PIOS_MS5611_I2CSampleTxn(data, sizeof(data), next_conv)) {
		return -1;
	}

	static int64_t delta_temp;
	static int64_t temperature;

	/* Read and store the 16bit result */
	if (this_conv == TEMPERATURE_CONV) {
		uint32_t raw_temperature;

		raw_temperature = (data[0] << 16) | (data[1] << 8) | data[2];

		delta_temp = (int32_t)raw_temperature - (dev->calibration[4] << 8);
		temperature = 2000 + ((delta_temp * dev->calibration[5]) >> 23);
		dev->temperature_unscaled = temperature;

		// second order temperature compensation
		if (temperature < 2000)
			dev->temperature_unscaled -= (delta_temp * delta_temp) >> 31;

	} else if (this_conv == PRESSURE_CONV) {
		int64_t offset;
		int64_t sens;
		uint32_t raw_pressure;

		raw_pressure = (data[0] << 16) | (data[1] << 8) | (data[2] << 0);

		offset = ((int64_t)dev->calibration[1] << 16) + (((int64_t)dev->calibration[3] * delta_temp) >> 7);
		sens = (int64_t)dev->calibration[0] << 15;
		sens = sens + ((((int64_t) dev->calibration[2]) * delta_temp) >> 8);

		// second order temperature compensation
		if (temperature < 2000) {
			offset -= (5 * (temperature - 2000) * (temperature - 2000)) >> 1;
			sens -= (5 * (temperature - 2000) * (temperature - 2000)) >> 2;

			if (dev->temperature_unscaled < -1500) {
				offset -= 7 * (temperature + 1500) * (temperature + 1500);
				sens -= (11 * (temperature + 1500) * (temperature + 1500)) >> 1;
			}
		}

		dev->pressure_unscaled = ((((int64_t)raw_pressure * sens) >> 21) - offset) >> 15;
	}

	return 0;
}

/**
* Reads one or more bytes into a buffer
* \param[in] the command indicating the address to read
* \param[out] buffer destination buffer
* \param[in] len number of bytes which should be read
* \return 0 if operation was successful
* \return -1 if dev is invalid
* \return -2 if error during I2C transfer
*/
static int32_t PIOS_MS5611_Read(uint8_t address, uint8_t *buffer, uint8_t len)
{
	if (PIOS_MS5611_Validate(dev) != 0)
		return -1;

	const struct pios_i2c_txn txn_list[] = {
		{
			.info = __func__,
			.addr = ms5611_i2c_addr,
			.rw = PIOS_I2C_TXN_WRITE,
			.len = 1,
			.buf = &address,
		},
		{
			.info = __func__,
			.addr = ms5611_i2c_addr,
			.rw = PIOS_I2C_TXN_READ,
			.len = len,
			.buf = buffer,
		 }
	};

	return PIOS_I2C_Transfer(dev->i2c_id, txn_list, NELEMENTS(txn_list));
}

/**
* Writes one or more bytes to the MS5611
* \param[in] address Register address
* \param[in] buffer source buffer
* \return 0 if operation was successful
* \return -1 if dev is invalid
* \return -2 if error during I2C transfer
*/
static int32_t PIOS_MS5611_WriteCommand(uint8_t command)
{
	if (PIOS_MS5611_Validate(dev) != 0)
		return -1;

	const struct pios_i2c_txn txn_list[] = {
		{
			.info = __func__,
			.addr = ms5611_i2c_addr,
			.rw = PIOS_I2C_TXN_WRITE,
			.len = 1,
			.buf = &command,
		 },
	};

	return PIOS_I2C_Transfer(dev->i2c_id, txn_list, NELEMENTS(txn_list));
}

static bool PIOS_MS5611_Callback(void *ctx, void *output,
		int ms_to_wait, int *next_call)
{
	struct ms5611_dev *d = ctx;

	int read_adc_result;

	PIOS_Assert(d == dev);

	PIOS_MS5611_ClaimDevice();

	dev->interleave_count--;

	enum conversion_type next_conv = PRESSURE_CONV;

	if (dev->interleave_count <= 0) {
		next_conv = TEMPERATURE_CONV;

		dev->interleave_count = dev->cfg->temperature_interleaving + 1;
		if (dev->interleave_count == 1)
			dev->interleave_count = 2;
	}

	read_adc_result = PIOS_MS5611_ReadADC(next_conv);

	PIOS_MS5611_ReleaseDevice();

	*next_call = PIOS_MS5611_GetDelay();

	if (read_adc_result) {
		return false;
	}

	if (!output) {
		return true;
	}

	// Compute the altitude from the pressure and temperature and send it out
	struct pios_sensor_baro_data *data = output;

	data->temperature = ((float) dev->temperature_unscaled) / 100.0f;
	data->pressure = ((float) dev->pressure_unscaled) / 1000.0f;
	data->altitude = 44330.0f * (1.0f - powf(data->pressure / MS5611_P0, (1.0f / 5.255f)));

	return true;
}

/**
* @brief Run self-test operation.
* \return 0 if self-test succeed, -1 if failed
*/
int32_t PIOS_MS5611_Test()
{
	if (PIOS_MS5611_Validate(dev) != 0)
		return -1;

	for (int i = 0; i < 3; i++) {
		int next_call;
		if (!PIOS_MS5611_Callback(dev, NULL, 0, &next_call)) {
			return -1;
		}

		PIOS_DELAY_WaitmS(next_call);
	}

	// check range for sanity according to datasheet
	if (dev->temperature_unscaled < -4000 ||
		dev->temperature_unscaled > 8500 ||
		dev->pressure_unscaled < 1000 ||
		dev->pressure_unscaled > 120000)
		return -1;

	return 0;
}

#endif

/**
 * @}
 * @}
 */
