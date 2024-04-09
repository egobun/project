#include "main.h"
#include "math.h"

//TIM HALL
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim9;

//TIM table
extern TIM_HandleTypeDef htim6;

//TIM PWM
extern TIM_HandleTypeDef htim7;

//extern variables
extern int32_t current_number;
extern uint32_t tim6_counter_period;
extern uint8_t H1;
extern uint8_t H2;
extern uint8_t H3;
extern float speed;
//extern uint32_t power;
extern uint32_t count_overflow;
extern float mean_rate;
extern int32_t timing;
extern int32_t direction;
extern uint32_t PMSM_PWM;
extern uint8_t TIM8_PSC;
extern uint8_t TIM6_PSC;
extern uint8_t IsOK;
extern float help;
extern float rate;
extern float prev_rate;
extern float mean_rate_prev;
extern float mean_rate_help;
extern float real_speed;
extern float prev_real_speed;
extern uint16_t set_timing;
extern uint32_t set_PMSM_PWM;
extern float set_PMSM_PWM_b;
extern int32_t new_direction;
extern int16_t trigger_position;
extern uint32_t min_rate;

static const uint8_t PMSM_SINTABLE [PMSM_SINTABLESIZE][3] =
{
		{0,       0,      221},
		{8,       0,      225},
		{17,      0,      229},
		{25,      0,      232},
		{33,      0,      236},
		{42,      0,      239},
		{50,      0,      241},
		{58,      0,      244},
		{66,      0,      246},
		{74,      0,      248},
		{82,      0,      250},
		{90,      0,      252},
		{98,      0,      253},
		{105,     0,      254},
		{113,     0,      254},
		{120,     0,      255},
		{128,     0,      255},
		{135,     0,      255},
		{142,     0,      254},
		{149,     0,      254},
		{155,     0,      253},
		{162,     0,      252},
		{168,     0,      250},
		{174,     0,      248},
		{180,     0,      246},
		{186,     0,      244},
		{192,     0,      241},
		{197,     0,      239},
		{202,     0,      236},
		{207,     0,      232},
		{212,     0,      229},
		{217,     0,      225},
		{221,     0,      221},
		{225,     0,      217},
		{229,     0,      212},
		{232,     0,      207},
		{236,     0,      202},
		{239,     0,      197},
		{241,     0,      192},
		{244,     0,      186},
		{246,     0,      180},
		{248,     0,      174},
		{250,     0,      168},
		{252,     0,      162},
		{253,     0,      155},
		{254,     0,      149},
		{254,     0,      142},
		{255,     0,      135},
		{255,     0,      127},
		{255,     0,      120},
		{254,     0,      113},
		{254,     0,      105},
		{253,     0,      98},
		{252,     0,      90},
		{250,     0,      82},
		{248,     0,      74},
		{246,     0,      66},
		{244,     0,      58},
		{241,     0,      50},
		{239,     0,      42},
		{236,     0,      33},
		{232,     0,      25},
		{229,     0,      17},
		{225,     0,      8},
		{221,     0,      0},
		{225,     8,      0},
		{229,     17,     0},
		{232,     25,     0},
		{236,     33,     0},
		{239,     42,     0},
		{241,     50,     0},
		{244,     58,     0},
		{246,     66,     0},
		{248,     74,     0},
		{250,     82,     0},
		{252,     90,     0},
		{253,     98,     0},
		{254,     105,    0},
		{254,     113,    0},
		{255,     120,    0},
		{255,     127,    0},
		{255,     135,    0},
		{254,     142,    0},
		{254,     149,    0},
		{253,     155,    0},
		{252,     162,    0},
		{250,     168,    0},
		{248,     174,    0},
		{246,     180,    0},
		{244,     186,    0},
		{241,     192,    0},
		{239,     197,    0},
		{236,     202,    0},
		{232,     207,    0},
		{229,     212,    0},
		{225,     217,    0},
		{221,     221,    0},
		{217,     225,    0},
		{212,     229,    0},
		{207,     232,    0},
		{202,     236,    0},
		{197,     239,    0},
		{192,     241,    0},
		{186,     244,    0},
		{180,     246,    0},
		{174,     248,    0},
		{168,     250,    0},
		{162,     252,    0},
		{155,     253,    0},
		{149,     254,    0},
		{142,     254,    0},
		{135,     255,    0},
		{128,     255,    0},
		{120,     255,    0},
		{113,     254,    0},
		{105,     254,    0},
		{98,      253,    0},
		{90,      252,    0},
		{82,      250,    0},
		{74,      248,    0},
		{66,      246,    0},
		{58,      244,    0},
		{50,      241,    0},
		{42,      239,    0},
		{33,      236,    0},
		{25,      232,    0},
		{17,      229,    0},
		{8,       225,    0},
		{0,       221,    0},
		{0,       225,    8},
		{0,       229,    17},
		{0,       232,    25},
		{0,       236,    33},
		{0,       239,    42},
		{0,       241,    50},
		{0,       244,    58},
		{0,       246,    66},
		{0,       248,    74},
		{0,       250,    82},
		{0,       252,    90},
		{0,       253,    98},
		{0,       254,    105},
		{0,       254,    113},
		{0,       255,    120},
		{0,       255,    128},
		{0,       255,    135},
		{0,       254,    142},
		{0,       254,    149},
		{0,       253,    155},
		{0,       252,    162},
		{0,       250,    168},
		{0,       248,    174},
		{0,       246,    180},
		{0,       244,    186},
		{0,       241,    192},
		{0,       239,    197},
		{0,       236,    202},
		{0,       232,    207},
		{0,       229,    212},
		{0,       225,    217},
		{0,       221,    221},
		{0,       217,    225},
		{0,       212,    229},
		{0,       207,    232},
		{0,       202,    236},
		{0,       197,    239},
		{0,       192,    241},
		{0,       186,    244},
		{0,       180,    246},
		{0,       174,    248},
		{0,       168,    250},
		{0,       162,    252},
		{0,       155,    253},
		{0,       149,    254},
		{0,       142,    254},
		{0,       135,    255},
		{0,       128,    255},
		{0,       120,    255},
		{0,       113,    254},
		{0,       105,    254},
		{0,       98,     253},
		{0,       90,     252},
		{0,       82,     250},
		{0,       74,     248},
		{0,       66,     246},
		{0,       58,     244},
		{0,       50,     241},
		{0,       42,     239},
		{0,       33,     236},
		{0,       25,     232},
		{0,       17,     229},
		{0,       8,      225}
};



void change_rotation(){
				if(direction != new_direction){
								if(rate != rate_for_changing_direction)
									prev_rate = rate;
								rate = rate_for_changing_direction;
								if(real_speed < min_rate){
									direction = new_direction;
									rate = prev_rate;
								}
				}
}
void set_timings_PMSM_PWM(){
				set_timing = (int)(set_timing_k * rate - set_timing_b);
				set_PMSM_PWM = (int)(set_PMSM_PWM_k * rate + set_PMSM_PWM_b);
}
void correct_timings(){
				if(set_timing >= set_timing_min && set_timing <= set_timing_max){
								if(timing < set_timing)
									timing ++;
								if(timing > set_timing)
									timing --;
								}
}
void correct_set_PMSM_PWM_parametrs(){
				if(tim6_counter_period > tim6_counter_period_high){
					set_PMSM_PWM_b = set_PMSM_PWM_b_max;
				}
				else
					set_PMSM_PWM_b = set_PMSM_PWM_b_regular;
}
void correct_PMSM_PWM_parametrs(){
				if(set_PMSM_PWM >= set_PMSM_PWM_min && set_PMSM_PWM < set_PMSM_PWM_max){
								if(PMSM_PWM < set_PMSM_PWM)
									PMSM_PWM ++;
								if(PMSM_PWM > set_PMSM_PWM)
									PMSM_PWM --;
								}
}
void protection_against_incorrect_parameters(){
			if(set_PMSM_PWM < set_PMSM_PWM_min || set_PMSM_PWM >= set_PMSM_PWM_max)
						set_PMSM_PWM = set_PMSM_PWM_middle;
					if(set_timing < set_timing_min || set_timing > set_timing_max)
						set_timing = set_timing_min;
					if(timing < set_timing_min || timing > set_timing_max)
						timing = set_timing_min;	
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
				if(htim->Instance == TIM6)
				{ 
					if(IsOK == 1){
						tim6_counter_period = (int)(help/mean_rate*7/pairs);
					}
					TIM6->ARR = tim6_counter_period;
					
					TIM6->PSC = TIM6_PSC;
					TIM8->PSC = TIM8_PSC;
					TIM8->ARR = power;
					
					if(direction == clockwise){
						trigger_position = trigger_position_for_clockwise;
						current_number++;
						TIM8->CCR1 = (int)(power - PMSM_PWM*PMSM_SINTABLE[current_number][0]/max_sin_value);
						TIM8->CCR2 = (int)(power - PMSM_PWM*PMSM_SINTABLE[current_number][1]/max_sin_value);
						TIM8->CCR3 = (int)(power - PMSM_PWM*PMSM_SINTABLE[current_number][2]/max_sin_value);
						if(current_number == PMSM_SINTABLE_max_index){
							current_number = -1;
						}
					}
					else if(direction == counterclockwise){
						trigger_position = trigger_position_for_counterclockwise;
						if(current_number == -1)
							current_number = 1;
						current_number--;
						TIM8->CCR1 = (int)(power - PMSM_PWM*PMSM_SINTABLE[current_number][0]/255);
						TIM8->CCR2 = (int)(power - PMSM_PWM*PMSM_SINTABLE[current_number][1]/255);
						TIM8->CCR3 = (int)(power - PMSM_PWM*PMSM_SINTABLE[current_number][2]/255);
						if(current_number == 0){
							current_number = PMSM_SINTABLE_max_index + 1;
						}
					}
				}
				if(htim->Instance == TIM5)
        {
           count_overflow++;
        }
				
				if(htim->Instance == TIM7)
				{
					//Changing the direction of rotation
					change_rotation();
					//Setting PWM filling and timings
					set_timings_PMSM_PWM();
					//Adjusting timings to the set value
					correct_timings();
					//Adjusting PMSM_PWM to the set value
					correct_PMSM_PWM_parametrs();
					//Change of coefficients of set_PMSM_PWM in marginal values
					correct_set_PMSM_PWM_parametrs();
					//Protection against setting incorrect parameters
					protection_against_incorrect_parameters();
				}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
				if(htim->Instance == TIM5)
        {
					if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // RISING ? LOW ?? HIGH
                {
									H1 = HALL_UP;
									TIM5->CNT = 0;
									count_overflow = 0;
									
									if((timing + trigger_position) >= 0 && (timing + trigger_position) < PMSM_SINTABLESIZE && direction == clockwise){
										current_number = timing + trigger_position;
									}
									if((PMSM_SINTABLE_max_index - timing - trigger_position) >= 0 && (PMSM_SINTABLE_max_index - timing - trigger_position) < PMSM_SINTABLESIZE && direction == counterclockwise){
										current_number = PMSM_SINTABLE_max_index - timing - trigger_position;
									}
								}
                else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // FALLING ? HIGH ?? LOW
                {
									H1 = HALL_DOWN;
									speed = HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_2) + (htim5.Init.Period * count_overflow);
									
									mean_rate_prev = mean_rate;
									mean_rate = (speed_to_mean_rate/speed);
									prev_real_speed = real_speed;
									real_speed = mean_rate * calculation_speed_correct_k + calculation_speed_correct_b;
                }
				}
				if(htim->Instance == TIM2)
        {
					if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // RISING ? LOW ?? HIGH
					{
						H2 = HALL_UP;
					}
					else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // FALLING ? HIGH ?? LOW
					{
						H2 = HALL_DOWN;
					}
				}
				if(htim->Instance == TIM9)
        {
					if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // RISING ? LOW ?? HIGH
					{
						H3 = HALL_UP;
					}
					else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // FALLING ? HIGH ?? LOW
					{
						H3 = HALL_DOWN;
					}
				}
}