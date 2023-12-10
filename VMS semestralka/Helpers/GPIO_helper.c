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
