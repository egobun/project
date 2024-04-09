#include "main.h"
extern uint8_t position;
extern uint8_t time;
/*void rotation(){
	switch (position) {
				case 0:
					
					
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0);
					HAL_Delay(1);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);
				
					//HAL_Delay(1);
				
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,1);
					HAL_Delay(1);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);
				
					//HAL_Delay(1);
					
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,1);
					HAL_Delay(1);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,1);
					//HAL_Delay(1);
				
					position = 1;
				
					break;
				case 1:
				
					
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0);
				HAL_Delay(1);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);
				//HAL_Delay(1);
				
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,1);
				HAL_Delay(1);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,1);
				//HAL_Delay(1);
				
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,1);
					HAL_Delay(1);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,0);
					//HAL_Delay(1);
				
					position = 2;
					break;
				case 2:
				
					
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,1);
				HAL_Delay(1);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);
				//HAL_Delay(1);
				
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,0);
				HAL_Delay(1);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,1);
				//HAL_Delay(1);
				
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,1);
					HAL_Delay(1);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,0);
					//HAL_Delay(1);
				
					position = 3;
					break;
				case 3:
			
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,1);
				HAL_Delay(1);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0);
				//HAL_Delay(1);
				
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,0);
				HAL_Delay(1);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,1);
				//HAL_Delay(1);
				
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,1);
					HAL_Delay(1);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,1);
					//HAL_Delay(1);
				
					position = 4;
				case 4:
			
					
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,1);
				HAL_Delay(1);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0);
				//HAL_Delay(1);
				
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,1);
				HAL_Delay(1);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,1);
				//HAL_Delay(1);
				
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,0);
					HAL_Delay(1);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,1);
					//HAL_Delay(1);
				
					position = 5;
				case 5:
			
					
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,1);
				HAL_Delay(1);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);
				//HAL_Delay(1);
				
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,1);
				HAL_Delay(1);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);
				//HAL_Delay(1);
				
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,0);
					HAL_Delay(1);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,1);
					//HAL_Delay(1);
				
					position = 0;
		}
			HAL_Delay(time);
	}*/