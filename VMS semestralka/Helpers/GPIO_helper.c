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

Pin_struct kruh[8];
int8_t kruh_index=0;
void defineKruh()
{
	kruh[0]=constructor(LD3_GPIO_Port, LD3_Pin);
	kruh[1]=constructor(LD5_GPIO_Port, LD5_Pin);
	kruh[2]=constructor(LD7_GPIO_Port, LD7_Pin);
	kruh[3]=constructor(LD9_GPIO_Port, LD9_Pin);
	kruh[4]=constructor(LD10_GPIO_Port, LD10_Pin);
	kruh[5]=constructor(LD8_GPIO_Port, LD8_Pin);
	kruh[6]=constructor(LD6_GPIO_Port, LD6_Pin);
	kruh[7]=constructor(LD4_GPIO_Port, LD4_Pin);
}

void otocKruhem(int8_t smer)
{
	pisPin(kruh[kruh_index], 0);

	kruh_index+=smer;
	if(kruh_index>=8){kruh_index=0+smer-1;}
	if(kruh_index<0){kruh_index=7+(smer+1);}

	pisPin(kruh[kruh_index], 1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	GPIO_PinState B1_old=HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
	if ( B1_old== 1)
	{
		zmenSmer();
	}
}
