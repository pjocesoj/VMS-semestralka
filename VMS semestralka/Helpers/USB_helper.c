#include "USB_helper.h"

void dec_ascii(uint16_t dec, char ret[],uint8_t len)
{
	uint16_t temp = dec;
	uint8_t a=0;
	uint16_t rad = 1;

	for(int i=1;i<len;i++){rad*=10;}

	for(uint8_t i=0;i<len;i++)
	{
		a=temp/rad;
		ret[i]=a+48;//48 = '0' v ASCII
		temp=temp-a*rad;

		rad/=10;
	}
}
