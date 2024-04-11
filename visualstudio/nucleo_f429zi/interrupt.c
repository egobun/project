#include "interrupt.h"
#include "tim.h"
#include "i2c.h"
#include "MPU.h"
#include "tim.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal.h"
uint32_t zntime;
extern int16_t data[14];

   


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) 
{
	if (htim->Instance == TIM1)
	{
		HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_Address, ACCEL_XOUT_H_REG, 1, data, 14);
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	MPU6050Read();
}