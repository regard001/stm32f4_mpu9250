/*
 * sd_hal_mpu9250.c
 *
 *  Created on: Dec 5, 2019
 *      Author: regard001
 */


/**
 * |----------------------------------------------------------------------
 * | Copyright (C) Sina Darvishi,2016
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */

#include "sd_hal_MPU9250.h"

/* Default I2C address */
#define MPU9250_I2C_ADDR			0xD0

/* Who I am register value */
#define MPU9250_I_AM				0x71
#define AK8963_I_AM					0x48



uint8_t myTemp;

SD_MPU9250_Result SD_MPU9250_Init(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct, SD_MPU9250_Device DeviceNumber, SD_MPU9250_Accelerometer AccelerometerSensitivity, SD_MPU9250_Gyroscope GyroscopeSensitivity)
{
	uint8_t WHO_AM_I = (uint8_t)MPU9250_WHO_AM_I;
	uint8_t temp;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t d[2];
	uint8_t buffer[21];
	for(int i=0;i<21;i++){
		buffer[i] = 0;
	}


	/* Format I2C address */
	DataStruct->Address = MPU9250_I2C_ADDR | (uint8_t)DeviceNumber;
	uint8_t address = DataStruct->Address;

	/* Check if device is connected */
	if(HAL_I2C_IsDeviceReady(Handle,address,2,5)!=HAL_OK)
	{
				return SD_MPU9250_Result_Error;
	}
	 // select clock source to gyro
	d[0] = MPU9250_PWR_MGMT_1;
	d[1] = MPU9250_CLOCK_SEL_PLL;
	HAL_I2C_Master_Transmit(Handle, address, d, 2, 1000);
	 // enable I2C master mode
	d[0] = MPU9250_USER_CTRL;
	d[1] = I2C_MST_EN;
	HAL_I2C_Master_Transmit(Handle, address, d, 2, 1000);
	// set the I2C bus speed to 400 kHz
	d[0] = I2C_MST_CTRL;
	d[1] = I2C_MST_CLK;
	HAL_I2C_Master_Transmit(Handle, address, d, 2, 1000);

	// SAJAT KOD
	// set AK8963 to Power Down
	int status;
	status = writeAK8963Register(Handle, AK8963_CNTL, AK8963_PWR_DOWN);

	// reset MPU
	d[0] = MPU9250_PWR_MGMT_1;
	d[1] = MPU9250_PWR_RESET;
	HAL_I2C_Master_Transmit(Handle, address, d, 2, 1000);
	HAL_Delay(100);
	// reset the AK8963
	status = writeAK8963Register(Handle, AK8963_CNTL2, AK8963_RESET);
	 // select clock source to gyro
	d[0] = MPU9250_PWR_MGMT_1;
	d[1] = MPU9250_CLOCK_SEL_PLL;
	HAL_I2C_Master_Transmit(Handle, address, d, 2, 1000);
	///-------

	  HAL_Delay(100);
	/* Check who am I */
	//------------------
		/* Send address */
		if(HAL_I2C_Master_Transmit(Handle, address, &WHO_AM_I, 1, 1000) != HAL_OK)
		{
			return SD_MPU9250_Result_Error;
		}

		/* Receive multiple byte */
		if(HAL_I2C_Master_Receive(Handle, address, &temp, 1, 1000) != HAL_OK)
		{
			return SD_MPU9250_Result_Error;
		}

		/* Checking */
		while(temp != MPU9250_I_AM)
		{
				/* Return error */
				return SD_MPU9250_Result_DeviceInvalid;
		}
	//------------------

	/* Wakeup MPU9250
	//------------------

		d[0] = MPU9250_PWR_MGMT_1;
		d[1] = 0x00;


		if(HAL_I2C_Master_Transmit(Handle,(uint16_t)address , (uint8_t *)d, 2, 1000) != HAL_OK)
		{
					return SD_MPU9250_Result_Error;
		}

	*/
	/* Set sample rate to 1kHz */
	SD_MPU9250_SetDataRate(I2Cx,DataStruct, SD_MPU9250_DataRate_2KHz);

	/* Config accelerometer */
	SD_MPU9250_SetAccelerometer(I2Cx,DataStruct, AccelerometerSensitivity);

	/* Config Gyroscope */
	SD_MPU9250_SetGyroscope(I2Cx,DataStruct, GyroscopeSensitivity);
	// SAJAT KOD

	// setting the sample rate divider to 0 as default
	d[0] = SMPDIV;
	d[1] = 0x00;
	HAL_I2C_Master_Transmit(Handle, address, d, 2, 1000);

	 // enable I2C master mode
	d[0] = MPU9250_USER_CTRL;
	d[1] = I2C_MST_EN;
	HAL_I2C_Master_Transmit(Handle, address, d, 2, 1000);

	// set the I2C bus speed to 400 kHz
	d[0] = I2C_MST_CTRL;
	d[1] = I2C_MST_CLK;
	HAL_I2C_Master_Transmit(Handle, address, d, 2, 1000);

	// AK8963 WHO I AM
	readAK8963Registers(Handle, AK8963_WHO_AM_I, 1, buffer);

	/* get the magnetometer calibration */
	// set AK8963 to Power Down
	status = writeAK8963Register(Handle, AK8963_CNTL, AK8963_PWR_DOWN);
	HAL_Delay(100);
	// set AK8963 to FUSE ROM access
	status = writeAK8963Register(Handle, AK8963_CNTL, AK8963_FUSE_ROM);
	HAL_Delay(100);
	// read the AK8963 ASA registers and compute magnetometer scale factors
	status = readAK8963Registers(Handle, AK8963_ASAX, 3, buffer);
	float destination[3];
	destination[0] = ((((float)buffer[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f;
	destination[1] = ((((float)buffer[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f;
	destination[2] = ((((float)buffer[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f;
	// set AK8963 to Power Down
	status = writeAK8963Register(Handle, AK8963_CNTL, AK8963_PWR_DOWN);
	HAL_Delay(100);
	// set AK8963 to 16 bit resolution, 100 Hz update rate
	status = writeAK8963Register(Handle, AK8963_CNTL, AK8963_CNT_MEAS2);
	HAL_Delay(100);
	// select clock source to gyro
	d[0] = MPU9250_PWR_MGMT_1;
	d[1] = MPU9250_CLOCK_SEL_PLL;
	HAL_I2C_Master_Transmit(Handle, address, d, 2, 1000);
	// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
	status = readAK8963Registers(Handle, AK8963_HXL, 7, buffer);
	/* Return OK */
	return SD_MPU9250_Result_Ok;



}






void SD_MPU9250_setXAccelOffset(I2C_HandleTypeDef* I2Cx, int16_t offset) {
	uint8_t offsetL;
	uint8_t offsetH;
	uint8_t data[3];

	offsetL = offset & 0xFF;
	offsetH = offset >> 8;

	data[0] = MPU9250_RA_XA_OFFS_H;
	data[1] = offsetH;
	data[2] = offsetL;
	I2C_HandleTypeDef* Handle = I2Cx;
	if(HAL_I2C_Master_Transmit(Handle, MPU9250_I2C_ADDR ,(uint8_t *)data, 3, 1000) != HAL_OK)
		{
			//return SD_MPU9250_Result_Error;
		}
}

// YA_OFFS_* register
void SD_MPU9250_setYAccelOffset(I2C_HandleTypeDef* I2Cx, int16_t offset) {
	uint8_t offsetL;
	uint8_t offsetH;
	uint8_t data[3];

	offsetL = offset & 0xFF;
	offsetH = offset >> 8;

	data[0] = MPU9250_RA_YA_OFFS_H;
	data[1] = offsetH;
	data[2] = offsetL;
	I2C_HandleTypeDef* Handle = I2Cx;
	if(HAL_I2C_Master_Transmit(Handle, MPU9250_I2C_ADDR ,(uint8_t *)data, 3, 1000) != HAL_OK)
		{
			//return SD_MPU9250_Result_Error;
		}
}

// ZA_OFFS_* register
void SD_MPU9250_setZAccelOffset(I2C_HandleTypeDef* I2Cx, int16_t offset) {
	uint8_t offsetL;
	uint8_t offsetH;
	uint8_t data[3];

	offsetL = offset & 0xFF;
	offsetH = offset >> 8;

	data[0] = MPU9250_RA_ZA_OFFS_H;
	data[1] = offsetH;
	data[2] = offsetL;
	I2C_HandleTypeDef* Handle = I2Cx;
	if(HAL_I2C_Master_Transmit(Handle, MPU9250_I2C_ADDR ,(uint8_t *)data, 3, 1000) != HAL_OK)
		{
			//return SD_MPU9250_Result_Error;
		}
}

// XG_OFFS_USR* registers
void SD_MPU9250_setXGyroOffset(I2C_HandleTypeDef* I2Cx, int16_t offset) {
	uint8_t offsetL;
	uint8_t offsetH;
	uint8_t data[3];

	offsetL = offset & 0xFF;
	offsetH = offset >> 8;

	data[0] = MPU9250_RA_XG_OFFS_USRH;
	data[1] = offsetH;
	data[2] = offsetL;
	I2C_HandleTypeDef* Handle = I2Cx;
	if(HAL_I2C_Master_Transmit(Handle, MPU9250_I2C_ADDR ,(uint8_t *)data, 3, 1000) != HAL_OK)
		{
			//return SD_MPU9250_Result_Error;
		}
}

// YG_OFFS_USR* register
void SD_MPU9250_setYGyroOffset(I2C_HandleTypeDef* I2Cx, int16_t offset) {
	uint8_t offsetL;
	uint8_t offsetH;
	uint8_t data[3];

	offsetL = offset & 0xFF;
	offsetH = offset >> 8;

	data[0] = MPU9250_RA_YG_OFFS_USRH;
	data[1] = offsetH;
	data[2] = offsetL;
	I2C_HandleTypeDef* Handle = I2Cx;
	if(HAL_I2C_Master_Transmit(Handle, MPU9250_I2C_ADDR ,(uint8_t *)data, 3, 1000) != HAL_OK)
		{
			//return SD_MPU9250_Result_Error;
		}
}

// ZG_OFFS_USR* register
void SD_MPU9250_setZGyroOffset(I2C_HandleTypeDef* I2Cx, int16_t offset) {
	uint8_t offsetL;
	uint8_t offsetH;
	uint8_t data[3];

	offsetL = offset & 0xFF;
	offsetH = offset >> 8;

	data[0] = MPU9250_RA_ZG_OFFS_USRH;
	data[1] = offsetH;
	data[2] = offsetL;
	I2C_HandleTypeDef* Handle = I2Cx;
	if(HAL_I2C_Master_Transmit(Handle, MPU9250_I2C_ADDR ,(uint8_t *)data, 3, 1000) != HAL_OK)
		{
			//return SD_MPU9250_Result_Error;
		}
}




SD_MPU9250_Result SD_MPU9250_SetDataRate(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct, uint8_t rate)
{
	uint8_t d[2];
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;
	/* Format array to send */
	d[0] = MPU9250_SMPLRT_DIV;
	d[1] = rate;

	/* Set data sample rate */
	while(HAL_I2C_Master_Transmit(Handle,(uint16_t)address,(uint8_t *)d,2,1000)!=HAL_OK);
	/*{
				return SD_MPU9250_Result_Error;
	}*/

	/* Return OK */
	return SD_MPU9250_Result_Ok;
}

SD_MPU9250_Result SD_MPU9250_SetAccelerometer(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct, SD_MPU9250_Accelerometer AccelerometerSensitivity)
{
	uint8_t temp;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;
	uint8_t regAdd =(uint8_t )MPU9250_ACCEL_CONFIG;
	uint8_t data[2];
	data[0] = regAdd;
	/* Config accelerometer */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &regAdd, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU9250_Result_Error;
	}*/
	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &temp, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU9250_Result_Error;
	}*/
	temp = (temp & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
	data[1] = temp;
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address,(uint8_t *)data, 2, 1000) != HAL_OK);
	/*{
				return SD_MPU9250_Result_Error;
	}*/

	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccelerometerSensitivity) {
		case SD_MPU9250_Accelerometer_2G:
			DataStruct->Acce_Mult = (float)1 / MPU9250_ACCE_SENS_2;
			break;
		case SD_MPU9250_Accelerometer_4G:
			DataStruct->Acce_Mult = (float)1 / MPU9250_ACCE_SENS_4;
			break;
		case SD_MPU9250_Accelerometer_8G:
			DataStruct->Acce_Mult = (float)1 / MPU9250_ACCE_SENS_8;
			break;
		case SD_MPU9250_Accelerometer_16G:
			DataStruct->Acce_Mult = (float)1 / MPU9250_ACCE_SENS_16;
			break;
		default:
			break;
		}

	/* Return OK */
	return SD_MPU9250_Result_Ok;
}

SD_MPU9250_Result SD_MPU9250_SetGyroscope(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct, SD_MPU9250_Gyroscope GyroscopeSensitivity)
{
	uint8_t temp;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;
	uint8_t regAdd =(uint8_t )MPU9250_GYRO_CONFIG;
	uint8_t data[2];
	data[0] = regAdd;

	/* Config gyroscope */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address,&regAdd, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU9250_Result_Error;
	}*/
	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &temp, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU9250_Result_Error;
	}*/
	temp = (temp & 0xE7) | (uint8_t)GyroscopeSensitivity << 3;
	data[1] = temp;
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address,(uint8_t *)data, 2, 1000) != HAL_OK);
	/*{
				return SD_MPU9250_Result_Error;
	}*/

	switch (GyroscopeSensitivity) {
			case SD_MPU9250_Gyroscope_250s:
				DataStruct->Gyro_Mult = (float)1 / MPU9250_GYRO_SENS_250;
				break;
			case SD_MPU9250_Gyroscope_500s:
				DataStruct->Gyro_Mult = (float)1 / MPU9250_GYRO_SENS_500;
				break;
			case SD_MPU9250_Gyroscope_1000s:
				DataStruct->Gyro_Mult = (float)1 / MPU9250_GYRO_SENS_1000;
				break;
			case SD_MPU9250_Gyroscope_2000s:
				DataStruct->Gyro_Mult = (float)1 / MPU9250_GYRO_SENS_2000;
				break;
			default:
				break;
		}
	/* Return OK */
	return SD_MPU9250_Result_Ok;
}

SD_MPU9250_Result SD_MPU9250_ReadAccelerometer(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct)
{
	uint8_t data[6];
	uint8_t reg = MPU9250_ACCEL_XOUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Read accelerometer data */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 6, 1000) != HAL_OK);

	/* Format */
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Return OK */
	return SD_MPU9250_Result_Ok;
}
SD_MPU9250_Result SD_MPU9250_ReadGyroscope(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct)
{
	uint8_t data[6];
	uint8_t reg = MPU9250_GYRO_XOUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Read gyroscope data */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 6, 1000) != HAL_OK);

	/* Format */
	DataStruct->Gyroscope_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Gyroscope_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Gyroscope_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Return OK */
	return SD_MPU9250_Result_Ok;
}

SD_MPU9250_Result SD_MPU9250_ReadTemperature(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct)
{
	uint8_t data[2];
	int16_t temp;
	uint8_t reg = MPU9250_TEMP_OUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Read temperature */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 2, 1000) != HAL_OK);

	/* Format temperature */
	temp = (data[0] << 8 | data[1]);
	DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);

	/* Return OK */
	return SD_MPU9250_Result_Ok;
}
SD_MPU9250_Result SD_MPU9250_ReadAll(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct)
{
	uint8_t data[21];
	int16_t temp;
	uint8_t reg = MPU9250_ACCEL_XOUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Read full raw data, 14bytes */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 21, 1000) != HAL_OK);

	/* Format accelerometer data */
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Format temperature */
	temp = (data[6] << 8 | data[7]);
	DataStruct->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);

	/* Format gyroscope data */
	DataStruct->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
	DataStruct->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
	DataStruct->Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);

	DataStruct->Mag_X = (int16_t)(data[15] << 8 | data[14]);
	DataStruct->Mag_Y = (int16_t)(data[17] << 8 | data[16]);
	DataStruct->Mag_Z = (int16_t)(data[19] << 8 | data[18]);

	/* Return OK */
	return SD_MPU9250_Result_Ok;
}
SD_MPU9250_Result SD_MPU9250_ReadAll_Raw(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct, uint8_t data[])
{
//uint8_t data[14];
//	int16_t temp;
	uint8_t reg = MPU9250_ACCEL_XOUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Read full raw data, 14bytes */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 14, 1000) != HAL_OK);
/*
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

	temp = (data[6] << 8 | data[7]);
	DataStruct->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);

	DataStruct->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
	DataStruct->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
	DataStruct->Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);
*/

	/* Return OK */
	return SD_MPU9250_Result_Ok;
}
SD_MPU9250_Result SD_MPU9250_EnableInterrupts(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct)
{
	uint8_t temp;
	uint8_t reg[2] = {MPU9250_INT_ENABLE,0x21};
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Enable interrupts for data ready and motion detect */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, reg, 2, 1000) != HAL_OK);

	uint8_t mpu_reg= MPU9250_INT_PIN_CFG;
	/* Clear IRQ flag on any read operation */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &mpu_reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &temp, 14, 1000) != HAL_OK);
	temp |= 0x10;
	reg[0] = MPU9250_INT_PIN_CFG;
	reg[1] = temp;
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, reg, 2, 1000) != HAL_OK);

	/* Return OK */
	return SD_MPU9250_Result_Ok;
}
SD_MPU9250_Result SD_MPU9250_DisableInterrupts(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct)
{
	uint8_t reg[2] = {MPU9250_INT_ENABLE,0x00};
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Disable interrupts */
	while(HAL_I2C_Master_Transmit(Handle,(uint16_t)address,reg,2,1000)!=HAL_OK);
	/* Return OK */
	return SD_MPU9250_Result_Ok;
}
SD_MPU9250_Result SD_MPU9250_ReadInterrupts(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct, SD_MPU9250_Interrupt* InterruptsStruct)
{
	uint8_t read;

	/* Reset structure */
	InterruptsStruct->Status = 0;
	uint8_t reg = MPU9250_INT_STATUS;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &read, 14, 1000) != HAL_OK);

	/* Fill value */
	InterruptsStruct->Status = read;
	/* Return OK */
	return SD_MPU9250_Result_Ok;
}


// Sajat kod

/* reads registers from the AK8963 */
int readAK8963Registers(I2C_HandleTypeDef* I2Cx, uint8_t subAddress, uint8_t count, uint8_t* dest){
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t reg = MPU9250_I2C_ADDR | 0; // | DeviceNumber
	uint8_t regaddr[2];
	regaddr[0] = I2C_SLV0_ADDR;
	regaddr[1] = AK8963_ADDRESS | I2C_READ_FLAG;
	// set slave 0 to the AK8963 and set for read
	if(HAL_I2C_Master_Transmit(Handle, (uint16_t)reg, regaddr, 2, 1000) != HAL_OK)
	{
		return -1;
	}
	regaddr[0] = I2C_SLV0_REG;
	regaddr[1] = subAddress;
  // set the register to the desired AK8963 sub address
	if(HAL_I2C_Master_Transmit(Handle, (uint16_t)reg, regaddr, 2, 1000) != HAL_OK)
	{
    return -2;
  }
  // enable I2C and request the bytes
	regaddr[0] = I2C_SLV0_CTRL;
	regaddr[1] =  I2C_SLV0_EN | count;
	if(HAL_I2C_Master_Transmit(Handle, (uint16_t)reg, regaddr, 2, 1000) != HAL_OK){
    return -3;
  }
	HAL_Delay(10); // takes some time for these registers to fill
  // read the bytes off the MPU9250 EXT_SENS_DATA registers
	uint8_t subaddr = (uint8_t)EXT_SENS_DATA_00;
	if(HAL_I2C_Master_Transmit(Handle, (uint16_t)reg, &subaddr, 1, 1000)!= HAL_OK){
		return -5;
	}
	if(HAL_I2C_Master_Receive(Handle, (uint16_t)reg, &dest[0], count, 1000) != HAL_OK)
	{
		return -4;
	}

  return 0;
}

/* writes a register to the AK8963 given a register address and data */
int writeAK8963Register(I2C_HandleTypeDef* I2Cx, uint8_t subAddress, uint8_t data){
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t buffer[1];
  // set slave 0 to the AK8963 and set for write
	uint8_t reg = MPU9250_I2C_ADDR;
	uint8_t regaddr[2];
	regaddr[0] = I2C_SLV0_ADDR;
	regaddr[1] = AK8963_ADDRESS;
	if(HAL_I2C_Master_Transmit(Handle, (uint16_t)reg, regaddr, 2, 1000) != HAL_OK)
	 {
    return -1;
  }
  // set the register to the desired AK8963 sub address
	regaddr[0] = I2C_SLV0_REG;
	regaddr[1] = subAddress;
	if(HAL_I2C_Master_Transmit(Handle, reg, regaddr, 2, 1000) != HAL_OK)
 {
    return -2;
  }
  // store the data for write
	regaddr[0] = I2C_SLV0_DO;
	regaddr[1] = data;
	if(HAL_I2C_Master_Transmit(Handle, reg, regaddr, 2, 1000) != HAL_OK)
 {
    return -3;
  }
  // enable I2C and send 1 byte
	regaddr[0] = I2C_SLV0_CTRL;
	regaddr[1] = I2C_SLV0_EN | (uint8_t)1;
	if(HAL_I2C_Master_Transmit(Handle, reg, regaddr, 2, 1000) != HAL_OK)
	 {
    return -4;
  }
	// read the register and confirm
	if(readAK8963Registers(Handle, subAddress, 1, buffer) < 0)
	 {
    return -5;
  }
	if(buffer[0] == data) {
  	return 1;
  } else{
  	return -6;
  }
}
