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

void posliUSB(uint8_t ADC, uint16_t RPM)
{
	char bufferADC[3]={0};
	dec_ascii(ADC, bufferADC,3);

	char bufferRPM[5]={0};
	dec_ascii(RPM, bufferRPM,5);

	uint8_t bufferUSB[19]={'A','D','C',':',bufferADC[0],bufferADC[1],bufferADC[2],'\t',
			'\t',bufferRPM[0],bufferRPM[1],bufferRPM[2],bufferRPM[3],bufferRPM[4],'R','P','M','\r','\n'};

	uint8_t len=strlen(bufferUSB);//z nejakeho duvodu vraci 25 misto 20
	CDC_Transmit_FS(bufferUSB,19);
}
