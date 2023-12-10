#include "GPIO_helper.h"

Pin_struct constructor(GPIO_TypeDef* gpio,uint16_t cislo)
{
	Pin_struct ret;
	ret.sbernice=gpio;
	ret.pin=cislo;

	return ret;
}

void pisPin(Pin_struct kam,_Bool hodnota)
{
	HAL_GPIO_WritePin(kam.sbernice,kam.pin,hodnota);
}
_Bool ctiPin(Pin_struct odkud)
{
	return HAL_GPIO_ReadPin(odkud.sbernice,odkud.pin);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	GPIO_PinState B1_old=HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
	if ( B1_old== 1)
	{
		zmenSmer();
	}
}
