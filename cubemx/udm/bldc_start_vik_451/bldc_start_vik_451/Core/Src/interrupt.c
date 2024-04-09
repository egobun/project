#include "main.h"
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim9;
extern uint32_t counter;
extern uint16_t current_number;
extern uint32_t tim6_counter_period;
extern uint8_t H1;
extern uint8_t H2;
extern uint8_t H3;
extern uint32_t speed;
extern uint32_t current_position;
extern uint32_t count_overflow;
extern uint32_t mean_rate;
extern int32_t timing;
uint32_t PWM_max = 2560;
uint32_t PWM_min = 0;
float power_percent = 0.5;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
				if(htim->Instance == TIM6)
				{
					//counter++;
					TIM6->ARR = tim6_counter_period;
					if(current_position == 6){
						current_position = 0;
					}
					/*
					if(counter<100){
						TIM8->CCER |= 0x550;//Enable A B
						TIM8->CCER &= ~0x005;
						TIM8->CCR3 = PWM_max*(1-power_percent);
						TIM8->CCR2 = PWM_max*(1-power_percent);
					  TIM8->CCR1 = PWM_max*(1-power_percent);
					}
					else{
						TIM8->CCER |= 0x055;//Enable A B
						TIM8->CCER &= ~0x500;
						TIM8->CCR3 = PWM_max*(1-power_percent);
						TIM8->CCR2 = PWM_max*(1-power_percent);
					  TIM8->CCR1 = PWM_max*(1-power_percent);
					}
					*/
					
				switch(current_position){
					case 0:
						counter++;
						TIM8->CCER |= 0x055;//Enable A B
						TIM8->CCER &= ~0x500;//Disable C
						TIM8->CCR1 = PWM_max*(1-power_percent);
						TIM8->CCR2 = PWM_max;
					break;
					case 1:
						TIM8->CCER |= 0x505;//Enable A C
						TIM8->CCER &= ~0x050;//Disable B
						TIM8->CCR1 = PWM_max*(1-power_percent);
						TIM8->CCR3 = PWM_max;
					break;
					case 2:
						TIM8->CCER |= 0x550;//Enable B C
						TIM8->CCER &= ~0x005;//Disable A
						TIM8->CCR2 = PWM_max*(1-power_percent);
						TIM8->CCR3 = PWM_max;
					break;
					case 3:
						TIM8->CCER |= 0x055;//Enable A B
						TIM8->CCER &= ~0x500;//Disable C
						TIM8->CCR1 = PWM_max;
						TIM8->CCR2 = PWM_max*(1-power_percent);
					break;
			    case 4:
						TIM8->CCER |= 0x505;//Enable A C
						TIM8->CCER &= ~0x050;//Disable B
						TIM8->CCR1 = PWM_max;
						TIM8->CCR3 = PWM_max*(1-power_percent);
					break;
					case 5:		
						TIM8->CCER |= 0x550;//Enable B C
						TIM8->CCER &= ~0x005;//Disable A
						TIM8->CCR2 = PWM_max;
						TIM8->CCR3 = PWM_max*(1-power_percent);
					break;
				}
				current_position++;
			}
				
				if(htim->Instance == TIM5)
        {
                count_overflow++;
        }		
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM5)
        {
					if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // RISING ? LOW ?? HIGH
                {
									TIM5->CNT = 0;
									count_overflow = 0;
                }

                else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // FALLING ? HIGH ?? LOW
                {
                        speed = HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_2) + (65535 * count_overflow);
												mean_rate = (int)(17.95*1000000/speed);
												
                }
				}
				if(htim->Instance == TIM2)
        {
					if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // RISING ? LOW ?? HIGH
					{
						H2 = 1;
					}
					else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // FALLING ? HIGH ?? LOW
					{
						H2 = 0;
					}
				}
				if(htim->Instance == TIM9)
        {
					if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // RISING ? LOW ?? HIGH
					{
						H3 = 1;
					}
					else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // FALLING ? HIGH ?? LOW
					{
						H3 = 0;
					}
				}
}