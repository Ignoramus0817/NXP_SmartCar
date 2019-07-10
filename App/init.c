#include "init.h"

//初始化所有模块
void init_all(uint32 INIT_SPEED, uint32 INIT_ANGLE, uint8 imgbuff[])
{ 
  //电机
  //CH1-L1X CH3-L2 CH0-R1X CH2-R2 
  ftm_pwm_init(FTM0, FTM_CH1, 10*1000, INIT_SPEED);
  ftm_pwm_init(FTM0, FTM_CH2, 10*1000, INIT_SPEED);
  
  //舵机
  ftm_pwm_init(FTM1, FTM_CH0, 100, INIT_ANGLE); 
  
  //LED(24/19/6/9)
  gpio_init(PTA19, GPO, 1);
  gpio_init(PTA9, GPO, 1);
  
  //编码器
  ftm_input_init(FTM2, FTM_CH0, FTM_Falling, FTM_PS_2);
  ftm_input_init(FTM2, FTM_CH1, FTM_Falling, FTM_PS_2);
  
  //摄像头
  uart_init(UART0, 9600);
  uart_init(UART5, 9600);
  camera_init(imgbuff); 
  
  //电磁
  adc_init(ADC1_SE8);
  adc_init(ADC1_SE9);
}