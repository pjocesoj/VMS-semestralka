#ifndef MONITOR_H_
#define MONITOR_H_

#include "main.h"

//TIM
int tim16 = 0;
int tim17 = 0;
int tim2 = 0;
int tim2_ch2 = 0;
int tim2_ch4 = 0;

//mereni otacek
uint16_t adc3_new = 0; //ADC3 raw hodnota
uint16_t adc3_old = 0; //ADC3 v t-1

//do H nepaatri
void TIM2_monitor()
{
	HAL_GPIO_TogglePin(LD8_GPIO_Port, LD8_Pin);
	tim2 = HAL_GPIO_ReadPin(LD8_GPIO_Port, LD8_Pin);
	tim2 += 9;

	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
	tim2_ch2 = 1 + 3;

	HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
	tim2_ch4 = 1 + 6;
}

void TIM16_monitor()
{
	HAL_GPIO_TogglePin(LD9_GPIO_Port, LD9_Pin);
	tim16 = HAL_GPIO_ReadPin(LD9_GPIO_Port, LD9_Pin);
}

void TIM17_monitor()
{
	HAL_GPIO_TogglePin(LD10_GPIO_Port, LD10_Pin);
	tim17 = HAL_GPIO_ReadPin(LD10_GPIO_Port, LD10_Pin);
}

void PWM_monitor(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		tim2_ch2 = 3;
	}
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	{
		HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
		tim2_ch4 = 6;
	}
}
#endif
