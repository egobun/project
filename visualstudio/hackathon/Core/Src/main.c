/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU9250_reg.h"
#include "string.h"
#include <stdio.h>
#include "string.h"
#include "i2c_er.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t counter = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
float destination_a[3] = { 0, };
float destination_g[3] = { 0, };
int16_t destination_m[3] = { 0, };
int16_t destination_m_calib[3] = {0, };

char mass[50];
uint8_t flag = 0; 
uint8_t receive_bayte; 
char buf[36] = { 0, };
uint8_t sw[4] = { 0x02, 0x00, 0xA6, 0xBD }; 
float data[9];

float calib_gyro_x = 0;
float calib_gyro_y = 0;
float calib_gyro_z = 0;
float calib_accel_x_k = 1.00;
float calib_accel_x_b = -0.2;
float calib_accel_y_k = 1.00;
float calib_accel_y_b = -0.07;
float calib_accel_z_k = 0.98;
float calib_accel_z_b = -0.37;

uint8_t flag_exti = 0;
float gyro_offcet[3][1000] = {0, };
float accel_offcet[1000] = { 0, };
uint8_t flag_gyro = 0;
uint8_t flag_accel = 0;
float accel_uncalibrate_param[6];
//###################################### // new variables for magnetometer
//float magBias[3];
//float magScale[3];
//float mx;
//float my;
//float mz;
//int16_t magCount[3];
//float mRes;
//float magCalibration[3];
//enum Mscale {
//	MFS_14BITS = 0, // 0.6 mG per LSB
//	MFS_16BITS      // 0.15 mG per LSB
//};
//uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
//uint8_t Mmode = 0x06; // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
//###################################################################
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MPU_init(void) {
 
	uint8_t R;
	uint8_t c;
 
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS_W, PWR_MGMT_1, 1, 0x00, 1, 100); // Clear sleep mode bit (6), enable all sensors
	
	HAL_Delay(100);
	R = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS_W, PWR_MGMT_1, 1, &R, 1, 100); // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	
	
	// Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
	// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	// Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
	R = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS_W, CONFIG, 1, &R, 1, 100);
	
	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	R = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS_W, SMPLRT_DIV, 1, &R, 1, 100); // Use a 200 Hz rate; the same rate set in CONFIG above
	
	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS_R, GYRO_CONFIG, 1, &c, 6, 100); // get current GYRO_CONFIG register value
	c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x02; // Clear Fchoice bits [1:0]
	c = c & ~0x18; // Clear AFS bits [4:3]
	c = c | 0 << 3; // Set full scale range for the gyro - 0=+250dps
	c = c | 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS_W, GYRO_CONFIG, 1, &c, 1, 100); // Write new GYRO_CONFIG value to register
 
   // Set accelerometer full-scale range configuration
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS_R, ACCEL_CONFIG, 1, &c, 6, 100); // get current ACCEL_CONFIG register value
	c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x18; // Clear AFS bits [4:3]
	c = c | 0 << 3; // Set full scale range for the accelerometer  - 0=2g
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS_W, ACCEL_CONFIG, 1, &c, 1, 100); // Write new ACCEL_CONFIG register value
 
	 // Set accelerometer sample rate configuration
	 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS_R, MPU9250_ACCEL_CONFIG_2, 1, &c, 6, 100); // get current ACCEL_CONFIG2 register value
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0]) 
	c = c | 0x03; // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS_W, MPU9250_ACCEL_CONFIG_2, 1, &c, 1, 100); // Write new ACCEL_CONFIG2 register value
 
	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
 
	// Configure Interrupts and Bypass Enable
   // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
   // can join the I2C bus and all can be controlled by the Arduino as master
	R = 0x22;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS_W, MPU9250_INT_PIN_CFG, 1, &R, 1, 100);
	R = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS_W, MPU9250_INT_ENABLE, 1, &R, 1, 100); // Enable data ready (bit 0) interrupt
}

void MAGN_init(float * destination) {
 
	uint8_t R;
	uint8_t rawData[3]; // x/y/z gyro calibration data stored here
 
	// First extract the factory calibration for each magnetometer axis
	R = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS_W, AK8963_CNTL, 1, &R, 1, 100); // Power down magnetometer 
	HAL_Delay(10);
	R = 0x0F;
	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS_W, AK8963_CNTL, 1, &R, 1, 100); // Enter Fuse ROM access mode
	HAL_Delay(10);
	HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS_W, AK8963_ASAX, 1, rawData, 3, 100); // Read the x-, y-, and z-axis calibration values
	destination[0] =  (float)(rawData[0] - 128) / 256.0f + 1.0f; // Return x-axis sensitivity adjustment values, etc.
	destination[1] =  (float)(rawData[1] - 128) / 256.0f + 1.0f; 
	destination[2] =  (float)(rawData[2] - 128) / 256.0f + 1.0f;
	R = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS_W, AK8963_CNTL, 1, &R, 1, 100); // Power down magnetometer 
	HAL_Delay(10);
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	R = 0 << 4 | 0x06;
	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS_W, AK8963_CNTL, 1, &R, 1, 100); // Set magnetometer data resolution and sample ODR
	HAL_Delay(10);
 
	R = 0x40;
	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS_W, AK8963_ASTC, 1, &R, 1, 100); // set self-test  
}

void MPU_get_magn(int16_t * destination) {
	uint8_t rawData[7]; // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	uint8_t c;
	uint32_t status = HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS_R, AK8963_ST1, 1, &c, 1, 10);
	if (status != HAL_OK)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 1000);
	}
	if (c >= 0x01) {
		// wait for magnetometer data ready bit to be set
		status = HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS_R, AK8963_XOUT_L, 1, rawData, 7, 10); // Read the six raw data and ST2 registers sequentially into data array
		if (status != HAL_OK)
		{
			I2C_ClearBusyFlagErratum(&hi2c1, 1000);
		}
		c = rawData[6]; // End data read by reading ST2 register
		if (!(c & 0x08)) {
			// Check if magnetic sensor overflow set, if not then report data
			destination[1] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]); // Turn the MSB and LSB into a signed 16-bit value
			destination[0] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]); // Data stored as little Endian
			destination[2] = -(int16_t)(((int16_t)rawData[5] << 8) | rawData[4]);	 
		}
	}
 
}
//#######################################################################################
//the same function as MPU_get_magn
//void readMagData(int16_t * destination) {
//
//	uint8_t readData;
//
//	HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS_R, AK8963_ST1, 1, &readData, 1, 100);
//	if ((readData & 0x01) == 0x01) {
//		uint8_t rawMagData[7]; // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
//		HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS_R, AK8963_XOUT_L, 1, &rawMagData[0], 7, 100); // Read the six raw data and ST2 registers sequentially into data array
//		uint8_t c = rawMagData[6];
//		if (!(c & 0x08)) {
//			destination[0] = ((int16_t)rawMagData[1] << 8) | rawMagData[0]; // Turn the MSB and LSB into a signed 16-bit value
//			destination[1] = ((int16_t)rawMagData[3] << 8) | rawMagData[2]; // Data stored as little Endian
//			destination[2] = ((int16_t)rawMagData[5] << 8) | rawMagData[4];
//
//			//			if (SerialDebugB) {
//			//				printf("Mag X: %d\r\n", destination[0]);
//			//				printf("Mag Y: %d\r\n", destination[1]);
//			//				printf("Mag Z: %d\r\n", destination[2]);
//			//				printf("-------------------------\r\n");
//			//			}
//		}
//	}
//}
//// the same function as MAGN_INIT
//void initAK8963(float * destination) {
//	//pre def. vars
//	uint8_t writeData;
//
//	//First extract the factory calibration for each magnetometer axis
//	// x/y/z gyro calibration data stored here
//	uint8_t rawMagCalData[3];
//
//	//Power down magnetometer
//	writeData = 0x00;
//	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS_W, AK8963_CNTL, 1, &writeData, 1, 100);
//	HAL_Delay(100);
//
//	writeData = 0x0F;
//	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS_W, AK8963_CNTL, 1, &writeData, 1, 100); // Enter Fuse ROM access mode
//	HAL_Delay(100);
//
//
//	HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS_W, AK8963_ASAX, 1, &rawMagCalData[0], 3, 100); // Read the x-, y-, and z-axis calibration values
//	destination[0] =  (float)(rawMagCalData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
//	destination[1] =  (float)(rawMagCalData[1] - 128) / 256. + 1.;
//	destination[2] =  (float)(rawMagCalData[2] - 128) / 256. + 1.;
//
////	if (SerialDebugA) {
////		printf("Mag cal off X: %f\r\n", destination[0]);
////		printf("Mag cal off Y: %f\r\n", destination[1]);
////		printf("Mag cal off Z: %f\r\n", destination[2]);
////		printf("-------------------------\r\n");
////	}
//
//	writeData = 0x00;
//	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS_W, AK8963_CNTL, 1, &writeData, 1, 100); // Power down magnetometer
//	HAL_Delay(100);
//
//	// Configure the magnetometer for continuous read and highest resolution
//	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
//	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
//	writeData = Mscale << 4 | Mmode;
//	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS_W, AK8963_CNTL, 1, &writeData, 1, 100); // Set magnetometer data resolution and sample ODR
//
//	//writeData = 0x16;
//	//HAL_I2C_Mem_Write(I2Cx, AK8963_ADDRESS, AK8963_CNTL, 1, &writeData, 1, i2c_timeout);
//	HAL_Delay(10);
//
//	//if (SerialDebugA){printf("MAG Init Succesful! \r\n"); }
//} 
//
//void calibrateMag(float * dest1, float * dest2) {
//
//	uint16_t ii = 0, sample_count = 0;
//	int32_t mag_bias[3] = { 0, 0, 0 }, mag_scale[3] = { 0, 0, 0 };
//	int16_t mag_max[3] = { -32767, -32767, -32767 }, mag_min[3] = { 32767, 32767, 32767 }, mag_temp[3] = { 0, 0, 0 };
//
//	//if (SerialDebugA){printf("Mag Calibration: Wave device in a figure eight until done!\r\n"); }
//	HAL_Delay(4000);
//
//	// shoot for ~fifteen seconds of mag data
//	if (Mmode == 0x02) sample_count = 128; // at 8 Hz ODR, new mag data is available every 125 ms
//	if (Mmode == 0x06) sample_count = 1500; // at 100 Hz ODR, new mag data is available every 10 ms
//	for (ii = 0; ii < sample_count; ii++) {
//		MPU_get_magn(mag_temp); // Read the mag data
//		for (int jj = 0; jj < 3; jj++) {
//			if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
//			if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
//		}
//		if (Mmode == 0x02) HAL_Delay(135); // at 8 Hz ODR, new mag data is available every 125 ms
//		if (Mmode == 0x06) HAL_Delay(12); // at 100 Hz ODR, new mag data is available every 10 ms
//	}
//
//	// Get hard iron correction
//	mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
//	mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
//	mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts
//
//	dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0]; // save mag biases in G for main program
//	dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];
//	dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];
//
//	// Get soft iron correction estimate
//	mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
//	mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
//	mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts
//
//	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
//	avg_rad /= 3.0;
//
//	dest2[0] = avg_rad / ((float)mag_scale[0]);
//	dest2[1] = avg_rad / ((float)mag_scale[1]);
//	dest2[2] = avg_rad / ((float)mag_scale[2]);
//
//	//if (SerialDebugA){printf("Mag Calibration done!\r\n"); }
//}
//
//void getMres() {
//	switch (Mscale)
//	{
//		// Possible magnetometer scales (and their register bit settings) are:
//		// 14 bit resolution (0) and 16 bit resolution (1)
//	case MFS_14BITS:
//		mRes = 10. * 4912. / 8190.; // Proper scale to return milliGauss
//		break;
//	case MFS_16BITS:
//		mRes = 10. * 4912. / 32760.0; // Proper scale to return milliGauss
//		break;
//	}
//}
//
//void MPU_get_magn_calib(float * destination)
//{
//	readMagData(magCount); // Read the x/y/z adc values
//	getMres();
//	
//	// Calculate the magnetometer values in milliGauss
//	// Include factory calibration per data sheet and user environmental corrections
//	mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0]; // get actual magnetometer value, this depends on scale being set
//	my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];
//	mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];
//	mx *= magScale[0];
//	my *= magScale[1];
//	mz *= magScale[2];
//}
//
//void set_data_calib()
//{
//	for (int i = 0; i < 3; i++)
//	{
//		data[i] = destination_m_calib[i];
//	}
//	for (int i = 0; i < 3; i++)
//	{
//		data[i + 3] = destination_a[i];
//	}
//	for (int i = 0; i < 3; i++)
//	{
//		if (destination_g[i] > 250){data[i + 6] = 250; }
//		if (destination_g[i] < -250){data[i + 6] = -250; }
//		data[i + 6] = destination_g[i];
//	}
//	
//}

//######################################################################################

void MPU_get_accel(float * destination) {//��� ��� �������� ������ ���������
	uint8_t rawData[6];
	uint32_t status= HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS_R, MPU9250_ACCEL_XOUT_H, 1, rawData, 6, 100);
	if (status != HAL_OK)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 1000);
	}
	destination[0] = ((float)(int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) / 16384 * (9.8))*calib_accel_x_k + calib_accel_x_b; // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((float)(int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) / 16384 * (9.8))*calib_accel_y_k + calib_accel_y_b; 
	destination[2] = ((float)(int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) / 16384 * (9.8))*calib_accel_z_k + calib_accel_z_b;
}
void MPU_get_accel_start(float * destination) {
	uint8_t rawData[6];
	uint32_t status = HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS_R, MPU9250_ACCEL_XOUT_H, 1, rawData, 6, 100);
	if (status != HAL_OK)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 1000);
	}
	destination[0] = (float)(int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) / 16384 * (9.8); // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (float)(int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) / 16384 * (9.8); 
	destination[2] = (float)(int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) / 16384 * (9.8);
}

void MPU_get_gyro(float * destination) {
 
	uint8_t rawData[6]; // x/y/z gyro register data stored here
	uint32_t status = HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS_R, MPU9250_GYRO_XOUT_H, 1, rawData, 6, 100); // Read the six raw data registers sequentially into data array
	if (status != HAL_OK)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 1000);
	}
	destination[0] = (float)(int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) / 131 - calib_gyro_x; // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (float)(int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) / 131 - calib_gyro_y; 
	destination[2] = (float)(int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) / 131 - calib_gyro_z;
} 
void MPU_get_gyro_start(float * destination) {
 
	uint8_t rawData[6]; // x/y/z gyro register data stored here
	uint32_t status = HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS_R, MPU9250_GYRO_XOUT_H, 1, rawData, 6, 100); // Read the six raw data registers sequentially into data array
	if (status != HAL_OK)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 1000);
	}
	destination[0] = (float)(int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) / 131 ; // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (float)(int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) / 131 ; 
	destination[2] = (float)(int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) / 131 ;
}
 
 
//void MPU_get_magn(int16_t * destination){
//	uint8_t rawData[7]; // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
//	uint8_t c;
//	uint32_t status = HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS_R, AK8963_ST1, 1, &c, 1, 10);
//	if (status != HAL_OK)
//	{
//		I2C_ClearBusyFlagErratum(&hi2c1, 1000);
//	}
//	if (c >= 0x01) {
//		// wait for magnetometer data ready bit to be set
//		status = HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS_R, AK8963_XOUT_L, 1, rawData, 7, 10); // Read the six raw data and ST2 registers sequentially into data array
//		if (status != HAL_OK)
//		{
//			I2C_ClearBusyFlagErratum(&hi2c1, 1000);
//		}
//		c = rawData[6]; // End data read by reading ST2 register
//		if (!(c & 0x08)) {
//			// Check if magnetic sensor overflow set, if not then report data
//			destination[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]); // Turn the MSB and LSB into a signed 16-bit value
//			destination[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]); // Data stored as little Endian
//			destination[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]);	 
//		}
//	}
// 
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{ 
	if (huart == &huart1)
	{
		//HAL_UART_Transmit(&huart4,&receive_bayte, 1, 100);
		if (receive_bayte == 'r') {
			flag = 1;
			//snprintf(buf, 20, "%d %d %d %d ", sw[0], sw[1], sw[2], sw[3]);
			//HAL_UART_Transmit(&huart4, (uint8_t *)buf, sizeof(buf), 1000);
		}
		if (receive_bayte == 's')
		{
			flag = 0;
			
		} 
	}
	HAL_UART_Receive_IT(&huart1, &receive_bayte, 1);
} 

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		
		// ����� ���������� �����-�� ����, ���������� �� ��������� ��������
	}     
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	flag_exti = 1;
	HAL_TIM_Base_Start_IT(&htim1);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (HAL_GPIO_ReadPin(btn_GPIO_Port, btn_Pin) == 0)
	{
		flag_accel = 1;
		HAL_TIM_Base_Stop_IT(&htim1);
	}
	else
	{
		flag_gyro = 1;
		HAL_TIM_Base_Stop_IT(&htim1);
	}
}

void set_data()
{
	for (int i = 0; i < 3; i++)
	{
		data[i] = destination_m[i];
	}
	for (int i = 0; i < 3; i++)
	{
		data[i + 3] = destination_a[i];
	}
	for (int i = 0; i < 3; i++)
	{
		if (destination_g[i] > 250){data[i + 6] = 250;}
		if (destination_g[i] < -250){data[i + 6] = -250; }
		data[i + 6] = destination_g[i];
	}
	
}
void gyro_calibration()
{
	HAL_Delay(2000);
	HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
	for (int i = 0; i < 800; i++)
	{
		MPU_get_gyro_start(destination_g);
		gyro_offcet[0][i] = destination_g[0];
		gyro_offcet[1][i] = destination_g[1];
		gyro_offcet[2][i] = destination_g[2];
		HAL_Delay(10);
	}
	for (int i = 0; i < 800; i++)
	{
		calib_gyro_x += gyro_offcet[0][i];
		calib_gyro_y += gyro_offcet[1][i];
		calib_gyro_z += gyro_offcet[2][i];
		if (i == 799)
		{
			calib_gyro_x = calib_gyro_x / 800;
			calib_gyro_y = calib_gyro_y / 800;
			calib_gyro_z = calib_gyro_z / 800;
		}
	}
	HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
}

float get_accel_average(int j)
{
	uint8_t p = 0;
	switch (j)
	{
	case 0:
		p = 0;
		break;
	case 1:
		p = 0;
		break;
	case 2:
		p = 1;
		break;
	case 3:
		p = 1;
		break;
	case 4:
		p = 2;
		break;
	case 5:
		p = 2;
		break;
	}
	for (int i = 0; i < 800; i++)
	{

		MPU_get_accel_start(destination_a);
		
		accel_offcet[i] = destination_a[p];
		HAL_Delay(10);
	}
	float average;
	for (int i = 0; i < 800; i++)
	{
		average += accel_offcet[i];
		
		if (i == 799)
		{
			average = average / 800;
		}
	}
	return average;
}
void accel_calibration()
{
	HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
	HAL_Delay(2000);
	switch (counter)
	{
		case 1:
		accel_uncalibrate_param[counter-1] = get_accel_average(counter - 1);
		HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
			break;
		case 2:
		accel_uncalibrate_param[counter - 1] = get_accel_average(counter - 1);
		HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
			break;
		case 3:
		accel_uncalibrate_param[counter - 1] = get_accel_average(counter - 1);
		HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
			break;
		case 4:
		accel_uncalibrate_param[counter - 1] = get_accel_average(counter - 1);
		HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
			break;
		case 5:
		accel_uncalibrate_param[counter - 1] = get_accel_average(counter - 1);
		HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
			break;
		case 6:
		accel_uncalibrate_param[counter - 1] = get_accel_average(counter - 1);
		HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
			break;
	}
	if (counter == 6)
	{
		calib_accel_x_k = 2. * 9.81 / (accel_uncalibrate_param[0] - accel_uncalibrate_param[1]);
		calib_accel_x_b = 9.81 - calib_accel_x_k *accel_uncalibrate_param[0];
		
		calib_accel_y_k = 2. * 9.81 / (accel_uncalibrate_param[2] - accel_uncalibrate_param[3]);
		calib_accel_y_b = 9.81 - calib_accel_y_k *accel_uncalibrate_param[2];
		
		calib_accel_z_k = 2. * 9.81 / (accel_uncalibrate_param[4] - accel_uncalibrate_param[5]);
		calib_accel_z_b = 9.81 - calib_accel_z_k *accel_uncalibrate_param[4];
		
	}
	
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_SR_UIF);
	//������������� �������������, ��������� � ������������ 
	MPU_init();
	MAGN_init((float*)destination_m); // for uncallibrated magnetometer data
	
	//######################################################################################
	//Get magnetometer calibration from AK8963 ROM
	//initAK8963(magCalibration); // Initialize device for active mode read of magnetometer
//	for (uint8_t k = 0; k < 5; k++)
//	{
//		HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
//		//HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
//		HAL_Delay(500);
//		HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
//		HAL_Delay(500);
//	}
//	HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
//	calibrateMag(magBias, magScale);
//	HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
//
//	HAL_Delay(1000);
	//######################################################################################
	
	//���������� ������ � �������������, ��������� � ������������
	MPU_get_gyro(destination_g);
	MPU_get_accel(destination_a);
	MPU_get_magn(destination_m);
	
	set_data();
	HAL_UART_Receive_IT(&huart1, &receive_bayte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (flag == 1)
	  {

		  HAL_UART_Transmit_IT(&huart1, sw, 4); 
		  HAL_Delay(10); 
		  HAL_UART_Transmit_IT(&huart1, (uint8_t*)data, sizeof(data)); 
		  HAL_Delay(10); 
		 
	  }
	  else
	  {
		  MPU_get_gyro(destination_g);
		  MPU_get_accel(destination_a);
		  
		  MPU_get_magn(destination_m); // to get uncallibrate magnetometer data
		  set_data();
		  
		  //######################################################################################
	  //MPU_get_magn_calib((float*)destination_m_calib); // to get callibrated data 
	  //set_data_calib();
		  //######################################################################################
	  }
	  if (flag_exti == 1)
	  {
		  if (flag_gyro == 1)
		  {
			  flag_exti = 0;
			  gyro_calibration();
		  }
		  if (flag_accel == 1)
		  {
			  flag_exti = 0;
			  counter++;
			  accel_calibration();
			  if (counter == 6)
			  {
				  counter = 0;
			  }
		  }
		  flag_gyro = 0;
		  flag_accel = 0;
	  }
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : led_Pin */
  GPIO_InitStruct.Pin = led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : btn_Pin */
  GPIO_InitStruct.Pin = btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(btn_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
