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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU9250_reg.h"
#include "string.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_TxPacketConfig TxConfig;
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart4;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
float destination_a[3] = { 0, };
float destination_g[3] = { 0, };
int16_t destination_m[3] = { 0, };
uint8_t counter = 0;
char mass[50];
uint8_t flag = 0; 
uint8_t flag_t = 1;
uint8_t receive_bayte ; 
char buf[36] = { 0, };
uint8_t sw[4] = { 0x02, 0x00, 0xA6, 0xBD }; 
float data[9];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_I2C1_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_UART4_Init(void);
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
	c = c| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
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

void MPU_get_accel(float * destination) {
 
 
	uint8_t rawData[6];
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS_R, MPU9250_ACCEL_XOUT_H, 1, rawData, 6,100);
	destination[0] = (float)(int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) / 16384 * 9.8; // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (float)(int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) / 16384 * 9.8; 
	destination[2] = (float)(int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) / 16384 * 9.8;
}

 
void MPU_get_gyro(float * destination) {
 
	uint8_t rawData[6]; // x/y/z gyro register data stored here
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS_R, MPU9250_GYRO_XOUT_H, 1, rawData, 6,100); // Read the six raw data registers sequentially into data array
	destination[0] = (float)(int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) / 131; // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (float)(int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) / 131; 
	destination[2] = (float)(int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) / 131;
}
 
 
void MPU_get_magn(int16_t * destination) {
	uint8_t rawData[7]; // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	uint8_t c;
	HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS_R, AK8963_ST1, 1, &c, 1,10);
	if (c >= 0x01) {
		// wait for magnetometer data ready bit to be set
		HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS_R, AK8963_XOUT_L, 1, rawData, 7,10); // Read the six raw data and ST2 registers sequentially into data array
		c = rawData[6]; // End data read by reading ST2 register
		if (!(c & 0x08)) {
			// Check if magnetic sensor overflow set, if not then report data
			destination[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]); // Turn the MSB and LSB into a signed 16-bit value
			destination[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]); // Data stored as little Endian
			destination[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]);
				 
		}
	}
 
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{ 
	if (huart == &huart4)
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
	HAL_UART_Receive_IT(&huart4, &receive_bayte, 1);
} 

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart4)
	{
		counter++;
		flag_t = 1;
		// можно установить какой-то флаг, сообщающий об окончании отправки
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
		data[i + 6] = destination_g[i];
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
  MX_ETH_Init();
  MX_I2C1_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
	
	//uint32_t a = 3181772803; 
	

	
	//HAL_UART_Receive_IT(&huart4, &receive_bayte, 1); 
 
	
	MPU_init();
	MAGN_init((float*)destination_m);
	MPU_get_gyro(destination_g);
	MPU_get_accel(destination_a);
	MPU_get_magn(destination_m);
	set_data();
	HAL_UART_Receive_IT(&huart4, &receive_bayte, 1);
	

 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 
//	  strcpy(mass, "MAX ADC\r\n\0");
//	  HAL_UART_Transmit(&huart4, (uint8_t*)mass, strlen(mass), 100); 
//	  HAL_Delay(100); 
	  //counter++;
	  if (flag == 1)
	  {
		  // flag = 0;
		  
		  
			 // flag_t = 0;
		  HAL_UART_Transmit_IT(&huart4, sw, 4); 
		  HAL_Delay(10); 
		  // flag_t = 0;
		  HAL_UART_Transmit_IT(&huart4, (uint8_t*)data, sizeof(data)); 
		  HAL_Delay(10); 
		  //HAL_UART_Receive_IT(&huart4, &receive_bayte, 1);
		  
		  
		  //		  HAL_UART_Transmit_IT(&huart4, sw, 4); 
		  //		  HAL_Delay(10); 
		  //		  HAL_UART_Transmit_IT(&huart4,(uint8_t*)data, sizeof(data)); 
		  //		  HAL_Delay(10); 
		  //		  snprintf(buf, 36, "%f ", destination_a[0]);
		  //		  HAL_UART_Transmit(&huart4, (uint8_t*)buf, sizeof(buf), 100);
		  //		  HAL_Delay(100);
		 
	  }
	  else
	  {
		  MPU_get_gyro(destination_g);
		  MPU_get_accel(destination_a);
		  MPU_get_magn(destination_m);
		  set_data();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

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
