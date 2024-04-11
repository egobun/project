#include "MPU.h"
#include "usart.h"
#include "i2c.h"
//#include "freertos.c"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "math.h"



int16_t data[14];

char msg[10];
typedef struct
{

	int16_t AccelX;
	int16_t AccelY;
	int16_t AccelZ;
	double aX;
	double aY;
	double aZ;

	int16_t GyroX;
	int16_t GyroY;
	int16_t GyroZ;
	double gX;
	double gY;
	double gZ;

	int16_t temp;
	int Temperature;

} MPU6050znach;

void InitMPU6050(void) {     

	uint8_t data;
	data = 0;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_Address, PWR_MGMT_1_REG, 1, &data, 1, time);

	data = 0x07;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_Address, SMPLRT_DIV_REG, 1, &data, 1, time);
 
	data = 0x18;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_Address, ACCEL_CONFIG_REG, 1, &data, 1, time);

	data = 0x18;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_Address, GYRO_CONFIG_REG, 1, &data, 1, time);

	data = 0x1;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_Address, INT_ENABLE_REG, 1, &data, 1, time);

}

MPU6050znach z;


void MPU6050Read(void)
{
	///////////////////////////склейка/////////////////
	z.AccelX = (int16_t)(data[0] << 8 | data[1]);
	z.AccelY = (int16_t)(data[2] << 8 | data[3]);
	z.AccelZ = (int16_t)(data[4] << 8 | data[5]);
	z.temp = (int16_t)(data[6] << 8 | data[7]);
	z.GyroX = (int16_t)(data[8] << 8 | data[9]);
	z.GyroY = (int16_t)(data[10] << 8 | data[11]);
	z.GyroZ = (int16_t)(data[12] << 8 | data[13]);

	/////////////////////////////обработка////////////////////
	z.aX = z.AccelX / 2048.0;
	z.aY = z.AccelY / 2048.0;
	z.aZ = z.AccelZ / 2048.0;
	z.Temperature = (int)((int16_t)z.temp / (float)340.0 + (float)36.53);
	z.gX = z.GyroX / 131.0;
	z.gY = z.GyroY / 131.0;
	z.gZ = z.GyroZ / 131.0;

	/////////////////////////вычисление////////////////////
	int pitch;
	int pitch_sqrt = sqrt(z.aY * z.aY +  z.aZ * z.aZ);
	pitch = atan2(-z.aX, pitch_sqrt);

	/////////////////////////вывод/////////////////
	snprintf(msg, sizeof(msg), "%d", pitch);
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n", strlen("\r\n"), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n", strlen("\r\n"), HAL_MAX_DELAY);
}