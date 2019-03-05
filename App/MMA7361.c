#include "include.h"
#include "MMA7361.h"

//*************管脚说明***********************
/*
  AX -> PB0 -> ADC0_SE8
  AY -> PB1 -> ADC0_SE9
  AZ -> PB2 -> ADC0_SE12
  GX -> PB3 -> ADC0_SE13
  SL -> E1 高
  GS -> E2 高
*/
unsigned char BUF[20];       //接收数据缓存区
				 //IIC

int32 AX,AY,AZ;
float ax,ay,az;


//*************初始化MPU6050***********************
int Init_MMA7361(void)
{
  //初始化 MMA7361 所用的 ADC通道
  adc_init(ADC0_SE9); 
  adc_init(ADC0_SE12); 
  adc_init(ADC0_SE8);

  return 1;
}


void READ_MMA7361(void)
{
   AX =	adc_once(ADC0_SE8, ADC_12bit);
   AX -= (1184+3560)/2;
   ax = AX*2/(float)(3560-1184);		                //读取计算AX数据，3040和1040是实际测得的，下面的也一样

   AY =	adc_once(ADC0_SE9, ADC_12bit);
   AY -= (900+2930)/2;
   ay = AY*2/(float)(2930-900);		                //读取计算AY数据

   AZ =	adc_once(ADC0_SE12, ADC_12bit);
   AZ -= (3415+1080)/2;
   az = AZ*2/(float)(3415-1080);	                 	//读取计算AZ数据
  
}


