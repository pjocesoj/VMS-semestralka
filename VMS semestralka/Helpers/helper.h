#ifndef MY_HELPER
#define MY_HELPER
//#include "stm32f3xx_hal.h"
#include "main.h"

typedef struct
{
	GPIO_TypeDef* sbernice;//GPIOA
	uint16_t pin;//GPIO_PIN_1
}Pin_struct;

Pin_struct constructor(GPIO_TypeDef* gpio,uint16_t cislo);

void pisPin(Pin_struct kam,_Bool hodnota);
_Bool ctiPin(Pin_struct odkud);

void defineKruh();
void otocKruhem(int8_t smer);

#endif
