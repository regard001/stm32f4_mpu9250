/*
 * sd_hal_mpu9250.h
 *
 *  Created on: Dec 5, 2019
 *      Author: regard001
 */

#ifndef SRC_SD_HAL_MPU9250_H_
#define SRC_SD_HAL_MPU9250_H_


/*
 C++ detection
#ifdef __cplusplus
extern "C" {
#endif
*/

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

/**
 * @defgroup SD_MPU9250_Macros
 * @brief    Library defines
 * @{
 */

/* Default I2C used */
//#ifndef MPU9250_I2C
//#define	MPU9250_I2C                    I2C1              /*!< Default I2C */
//#define MPU9250_I2C_PINSPACK           SD_I2C_PinsPack_1 /*!< Default I2C pinspack. Check @ref SD_I2C for more information */
//#endif

/* Default I2C clock */
#ifndef MPU9250_I2C_CLOCK
#define MPU9250_I2C_CLOCK              400000            /*!< Default I2C clock speed */
#endif

/**
 * @brief  Data rates predefined constants
 * @{
 */
#define SD_MPU9250_DataRate_8KHz       0   /*!< Sample rate set to 8 kHz */
#define SD_MPU9250_DataRate_4KHz       1   /*!< Sample rate set to 4 kHz */
#define SD_MPU9250_DataRate_2KHz       3   /*!< Sample rate set to 2 kHz */
#define SD_MPU9250_DataRate_1KHz       7   /*!< Sample rate set to 1 kHz */
#define SD_MPU9250_DataRate_500Hz      15  /*!< Sample rate set to 500 Hz */
#define SD_MPU9250_DataRate_250Hz      31  /*!< Sample rate set to 250 Hz */
#define SD_MPU9250_DataRate_125Hz      63  /*!< Sample rate set to 125 Hz */
#define SD_MPU9250_DataRate_100Hz      79  /*!< Sample rate set to 100 Hz */

#define MPU9250_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU9250_RA_XA_OFFS_L_TC     0x07
#define MPU9250_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU9250_RA_YA_OFFS_L_TC     0x09
#define MPU9250_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU9250_RA_ZA_OFFS_L_TC     0x0B
#define MPU9250_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU9250_RA_XG_OFFS_USRL     0x14
#define MPU9250_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU9250_RA_YG_OFFS_USRL     0x16
#define MPU9250_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
/**
 * @}
 */

/**
 * @}
 */

// Sajat
//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value
#define AK8963_HXL 0x03
#define AK8963_PWR_DOWN 0x00
#define AK8963_CNT_MEAS1 0x12
#define AK8963_CNT_MEAS2 0x16
#define AK8963_FUSE_ROM 0x0F
#define AK8963_CNTL2 0x0B
#define AK8963_RESET 0x01




/* MPU9250 registers */
#define MPU9250_ADDR 0x68
#define MPU9250_AUX_VDDIO			0x01
#define MPU9250_SMPLRT_DIV			0x19
#define MPU9250_CONFIG				0x1A
#define MPU9250_GYRO_CONFIG			0x1B
#define MPU9250_ACCEL_CONFIG		0x1C
#define MPU9250_MOTION_THRESH		0x1F
#define MPU9250_INT_PIN_CFG			0x37
#define MPU9250_INT_ENABLE			0x38
#define MPU9250_INT_STATUS			0x3A
#define MPU9250_ACCEL_XOUT_H		0x3B
#define MPU9250_ACCEL_XOUT_L		0x3C
#define MPU9250_ACCEL_YOUT_H		0x3D
#define MPU9250_ACCEL_YOUT_L		0x3E
#define MPU9250_ACCEL_ZOUT_H		0x3F
#define MPU9250_ACCEL_ZOUT_L		0x40
#define MPU9250_TEMP_OUT_H			0x41
#define MPU9250_TEMP_OUT_L			0x42
#define MPU9250_GYRO_XOUT_H			0x43
#define MPU9250_GYRO_XOUT_L			0x44
#define MPU9250_GYRO_YOUT_H			0x45
#define MPU9250_GYRO_YOUT_L			0x46
#define MPU9250_GYRO_ZOUT_H			0x47
#define MPU9250_GYRO_ZOUT_L			0x48
#define MPU9250_MOT_DETECT_STATUS	0x61
#define MPU9250_SIGNAL_PATH_RESET	0x68
#define MPU9250_MOT_DETECT_CTRL		0x69
#define MPU9250_USER_CTRL			0x6A
#define MPU9250_PWR_MGMT_1			0x6B
#define MPU9250_PWR_MGMT_2			0x6C
#define MPU9250_FIFO_COUNTH			0x72
#define MPU9250_FIFO_COUNTL			0x73
#define MPU9250_FIFO_R_W			0x74
#define MPU9250_WHO_AM_I			0x75

#define MPU9250_PWR_RESET 0x80
#define MPU9250_CLOCK_SEL_PLL 0x01

#define EXT_SENS_DATA_00 0x49
#define I2C_MST_EN 0x20
#define I2C_MST_CLK 0x0D
#define I2C_MST_CTRL 0x24
#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_DO 0x63
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV0_EN 0x80
#define I2C_READ_FLAG 0x80

#define SMPDIV 0x19

/* Gyro sensitivities in degrees/s */
#define MPU9250_GYRO_SENS_250		((float) 131.0)
#define MPU9250_GYRO_SENS_500		((float) 65.5)
#define MPU9250_GYRO_SENS_1000		((float) 32.8)
#define MPU9250_GYRO_SENS_2000		((float) 16.4)

/* Acce sensitivities in g/s */
#define MPU9250_ACCE_SENS_2			((float) 16384)
#define MPU9250_ACCE_SENS_4			((float) 8192)
#define MPU9250_ACCE_SENS_8			((float) 4096)
#define MPU9250_ACCE_SENS_16		((float) 2048)





/**
 * @defgroup SD_MPU9250_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  MPU9250 can have 2 different slave addresses, depends on it's input AD0 pin
 *         This feature allows you to use 2 different sensors with this library at the same time
 */
typedef enum  {
	SD_MPU9250_Device_0 = 0x00, /*!< AD0 pin is set to low */
	SD_MPU9250_Device_1 = 0x02  /*!< AD0 pin is set to high */
} SD_MPU9250_Device;

/**
 * @brief  MPU9250 result enumeration
 */
typedef enum  {
	SD_MPU9250_Result_Ok = 0x00,          /*!< Everything OK */
	SD_MPU9250_Result_Error,              /*!< Unknown error */
	SD_MPU9250_Result_DeviceNotConnected, /*!< There is no device with valid slave address */
	SD_MPU9250_Result_DeviceInvalid       /*!< Connected device with address is not MPU9250 */
} SD_MPU9250_Result;

/**
 * @brief  Parameters for accelerometer range
 */
typedef enum  {
	SD_MPU9250_Accelerometer_2G = 0x00, /*!< Range is +- 2G */
	SD_MPU9250_Accelerometer_4G = 0x01, /*!< Range is +- 4G */
	SD_MPU9250_Accelerometer_8G = 0x02, /*!< Range is +- 8G */
	SD_MPU9250_Accelerometer_16G = 0x03 /*!< Range is +- 16G */
} SD_MPU9250_Accelerometer;

/**
 * @brief  Parameters for gyroscope range
 */
typedef enum {
	SD_MPU9250_Gyroscope_250s = 0x00,  /*!< Range is +- 250 degrees/s */
	SD_MPU9250_Gyroscope_500s = 0x01,  /*!< Range is +- 500 degrees/s */
	SD_MPU9250_Gyroscope_1000s = 0x02, /*!< Range is +- 1000 degrees/s */
	SD_MPU9250_Gyroscope_2000s = 0x03  /*!< Range is +- 2000 degrees/s */
} SD_MPU9250_Gyroscope;

/**
 * @brief  Main MPU9250 structure
 */
typedef struct  {
	/* Private */
	uint8_t Address;         /*!< I2C address of device. */
	float Gyro_Mult;         /*!< Gyroscope corrector from raw data to "degrees/s". Only for private use */
	float Acce_Mult;         /*!< Accelerometer corrector from raw data to "g". Only for private use */
	/* Public */
	int16_t Accelerometer_X; /*!< Accelerometer value X axis */
	int16_t Accelerometer_Y; /*!< Accelerometer value Y axis */
	int16_t Accelerometer_Z; /*!< Accelerometer value Z axis */
	int16_t Gyroscope_X;     /*!< Gyroscope value X axis */
	int16_t Gyroscope_Y;     /*!< Gyroscope value Y axis */
	int16_t Gyroscope_Z;     /*!< Gyroscope value Z axis */
	int16_t Mag_X;
	int16_t Mag_Y;
	int16_t Mag_Z;
	float   Temperature;       /*!< Temperature in degrees */
	//I2C_HandleTypeDef* I2Cx;
} SD_MPU9250;


/**
 * @brief  Interrupts union and structure
 */
typedef union {
	struct {
		uint8_t DataReady:1;       /*!< Data ready interrupt */
		uint8_t reserved2:2;       /*!< Reserved bits */
		uint8_t Master:1;          /*!< Master interrupt. Not enabled with library */
		uint8_t FifoOverflow:1;    /*!< FIFO overflow interrupt. Not enabled with library */
		uint8_t reserved1:1;       /*!< Reserved bit */
		uint8_t MotionDetection:1; /*!< Motion detected interrupt */
		uint8_t reserved0:1;       /*!< Reserved bit */
	} F;
	uint8_t Status;
} SD_MPU9250_Interrupt;


/**
 * @}
 */

/**
 * @defgroup SD_MPU9250_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes MPU9250 and I2C peripheral
 * @param  *DataStruct: Pointer to empty @ref SD_MPU9250_t structure
 * @param  DeviceNumber: MPU9250 has one pin, AD0 which can be used to set address of device.
 *          This feature allows you to use 2 different sensors on the same board with same library.
 *          If you set AD0 pin to low, then this parameter should be SD_MPU9250_Device_0,
 *          but if AD0 pin is high, then you should use SD_MPU9250_Device_1
 *
 *          Parameter can be a value of @ref SD_MPU9250_Device_t enumeration
 * @param  AccelerometerSensitivity: Set accelerometer sensitivity. This parameter can be a value of @ref SD_MPU9250_Accelerometer_t enumeration
 * @param  GyroscopeSensitivity: Set gyroscope sensitivity. This parameter can be a value of @ref SD_MPU9250_Gyroscope_t enumeration
 * @retval Initialization status:
 *            - SD_MPU9250_Result_t: Everything OK
 *            - Other member: in other cases
 */
SD_MPU9250_Result SD_MPU9250_Init(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct, SD_MPU9250_Device DeviceNumber, SD_MPU9250_Accelerometer AccelerometerSensitivity, SD_MPU9250_Gyroscope GyroscopeSensitivity);

/**
 * @brief  Sets gyroscope sensitivity
 * @param  *DataStruct: Pointer to @ref SD_MPU9250_t structure indicating MPU9250 device
 * @param  GyroscopeSensitivity: Gyro sensitivity value. This parameter can be a value of @ref SD_MPU9250_Gyroscope_t enumeration
 * @retval Member of @ref SD_MPU9250_Result_t enumeration
 */
SD_MPU9250_Result SD_MPU9250_SetGyroscope(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct, SD_MPU9250_Gyroscope GyroscopeSensitivity);

/**
 * @brief  Sets accelerometer sensitivity
 * @param  *DataStruct: Pointer to @ref SD_MPU9250_t structure indicating MPU9250 device
 * @param  AccelerometerSensitivity: Gyro sensitivity value. This parameter can be a value of @ref SD_MPU9250_Accelerometer_t enumeration
 * @retval Member of @ref SD_MPU9250_Result_t enumeration
 */
SD_MPU9250_Result SD_MPU9250_SetAccelerometer(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct, SD_MPU9250_Accelerometer AccelerometerSensitivity);

/**
 * @brief  Sets output data rate
 * @param  *DataStruct: Pointer to @ref SD_MPU9250_t structure indicating MPU9250 device
 * @param  rate: Data rate value. An 8-bit value for prescaler value
 * @retval Member of @ref SD_MPU9250_Result_t enumeration
 */
SD_MPU9250_Result SD_MPU9250_SetDataRate(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct, uint8_t rate);


/**
 * @brief  Enables interrupts
 * @param  *DataStruct: Pointer to @ref SD_MPU9250_t structure indicating MPU9250 device
 * @retval Member of @ref SD_MPU9250_Result_t enumeration
 */
SD_MPU9250_Result SD_MPU9250_EnableInterrupts(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct);

/**
 * @brief  Disables interrupts
 * @param  *DataStruct: Pointer to @ref SD_MPU9250_t structure indicating MPU9250 device
 * @retval Member of @ref SD_MPU9250_Result_t enumeration
 */
SD_MPU9250_Result SD_MPU9250_DisableInterrupts(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct);

/**
 * @brief  Reads and clears interrupts
 * @param  *DataStruct: Pointer to @ref SD_MPU9250_t structure indicating MPU9250 device
 * @param  *InterruptsStruct: Pointer to @ref SD_MPU9250_Interrupt_t structure to store status in
 * @retval Member of @ref SD_MPU9250_Result_t enumeration
 */
SD_MPU9250_Result SD_MPU9250_ReadInterrupts(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct, SD_MPU9250_Interrupt* InterruptsStruct);

/**
 * @brief  Reads accelerometer data from sensor
 * @param  *DataStruct: Pointer to @ref SD_MPU9250_t structure to store data to
 * @retval Member of @ref SD_MPU9250_Result_t:
 *            - SD_MPU9250_Result_Ok: everything is OK
 *            - Other: in other cases
 */
SD_MPU9250_Result SD_MPU9250_ReadAccelerometer(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct);

/**
 * @brief  Reads gyroscope data from sensor
 * @param  *DataStruct: Pointer to @ref SD_MPU9250_t structure to store data to
 * @retval Member of @ref SD_MPU9250_Result_t:
 *            - SD_MPU9250_Result_Ok: everything is OK
 *            - Other: in other cases
 */
SD_MPU9250_Result SD_MPU9250_ReadGyroscope(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct);

/**
 * @brief  Reads temperature data from sensor
 * @param  *DataStruct: Pointer to @ref SD_MPU9250_t structure to store data to
 * @retval Member of @ref SD_MPU9250_Result_t:
 *            - SD_MPU9250_Result_Ok: everything is OK
 *            - Other: in other cases
 */
SD_MPU9250_Result SD_MPU9250_ReadTemperature(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct);

/**
 * @brief  Reads accelerometer, gyroscope and temperature data from sensor
 * @param  *DataStruct: Pointer to @ref SD_MPU9250_t structure to store data to
 * @retval Member of @ref SD_MPU9250_Result_t:
 *            - SD_MPU9250_Result_Ok: everything is OK
 *            - Other: in other cases
 */
SD_MPU9250_Result SD_MPU9250_ReadAll(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct);

/**
 * @brief  Reads accelerometer, gyroscope and temperature data from sensor
 * @param  *DataStruct: Pointer to @ref SD_MPU9250_t structure to read address
 * @param  data[]: At least 14 long array, to store the raw data from the sensor
 * @retval Member of @ref SD_MPU9250_Result_t:
 *            - SD_MPU9250_Result_Ok: everything is OK
 *            - Other: in other cases
 */
 SD_MPU9250_Result SD_MPU9250_ReadAll_Raw(I2C_HandleTypeDef* I2Cx,SD_MPU9250* DataStruct, uint8_t data[]);
void SD_MPU9250_setXAccelOffset(I2C_HandleTypeDef* I2Cx, int16_t offset);
void SD_MPU9250_setYAccelOffset(I2C_HandleTypeDef* I2Cx, int16_t offset);
void SD_MPU9250_setZAccelOffset(I2C_HandleTypeDef* I2Cx, int16_t offset);

void SD_MPU9250_setXGyroOffset(I2C_HandleTypeDef* I2Cx, int16_t offset);
void SD_MPU9250_setYGyroOffset(I2C_HandleTypeDef* I2Cx, int16_t offset);
void SD_MPU9250_setZGyroOffset(I2C_HandleTypeDef* I2Cx, int16_t offset);

int readAK8963Registers(I2C_HandleTypeDef* I2Cx, uint8_t subAddress, uint8_t count, uint8_t* dest);
int writeAK8963Register(I2C_HandleTypeDef* I2Cx, uint8_t subAddress, uint8_t data);
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */


#endif /* SRC_SD_HAL_MPU9250_H_ */
