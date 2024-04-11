#pragma once

#define MPU6050_Address 0x68
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define GYRO_CONFIG_REG 0x1B
#define INT_ENABLE_REG 0x38
#define time 500

void InitMPU6050(void);
void MPU6050Read(void);