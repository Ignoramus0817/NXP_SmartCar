/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       main.c
 * @brief      ɽ��K60 ƽ̨������
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-08-28
 */
////D.Va

#pragma optimize=none

#define FRONT 0
#define BACK 1
#define LEFT 2
#define RIGHT 3

#include "common.h"
#include "include.h"
#include "math.h"
#include "IMG_GET.h"

// ����ͷ�߶ȴ�ԼΪ17.5- 18.0 cm 
//imgbuffΪ����ͷ��ֵͼ��洢һά����һ�ֽ�8���أ�1��0�׻��෴
//imgΪ����ͷ��ѹ��Ҷ�ͼ��
uint8 imgbuff[CAMERA_SIZE];      
uint8 img[CAMERA_H][CAMERA_W]; 

//����������ֵ�������������ƣ�ռ�ձȣ���λ���֮��
uint32 ANGLE_UPPER_LIMIT = 850;
uint32 ANGLE_LOWER_LIMIT = 450;

//�ٶȼ�����Ƕȳ�ʼֵ��ռ�ձȣ���λ���֮��
//450����- 850 ���ң�
uint32 INIT_ANGLE = 650;
uint32 INIT_SPEED = 2000;

//��ʼ��
void init_all();

//�����ٶȿ���
void speed_adj(uint32 speed_next);
void turn(int angle_change, uint32 angle_rate);

//track detecting
int turn_error(uint8 img[][CAMERA_W]);
int get_P(int error);


////������main����
//void main(void)
//{
//  uint32 spin;
//  uint32 speed;
//  init_all();
//  ftm_pwm_duty(FTM0, FTM_CH3, 800);
////  ftm_pwm_duty(FTM0, FTM_CH2, 2000);
//  ftm_pwm_duty(FTM1, FTM_CH0, 1310);
//} 

//��ʼ������ģ��
void init_all()
{
  //���
  ftm_pwm_init(FTM0, FTM_CH3, 10*1000, INIT_SPEED);
  ftm_pwm_init(FTM0, FTM_CH2, 10*1000, INIT_SPEED);
  
  //���
  ftm_pwm_init(FTM1, FTM_CH0, 100, INIT_ANGLE); 
  
  //����ͷ
  uart_init(UART5, 9600);
  camera_init(imgbuff);                                  
}

//�ٶȵ���������Ϊ���ٶ�
void speed_adj(uint32 speed_next)
{
    ftm_pwm_duty(FTM0, FTM_CH3, speed_next);
    ftm_pwm_duty(FTM0, FTM_CH2, speed_next);
}

//ת�򣬲���1Ϊ�Ƕȱ���������2Ϊԭ�Ƕ�
void turn(int angle_change, uint32 angle_rate)
{
  uint32 angle;
  int temp = (int)(angle_rate) + angle_change;
  angle = (uint32)(temp);
  if(angle > ANGLE_UPPER_LIMIT)
    angle = ANGLE_UPPER_LIMIT;
  if(angle < ANGLE_LOWER_LIMIT)
    angle = ANGLE_LOWER_LIMIT;
  ftm_pwm_duty(FTM1, FTM_CH0, angle);
}

//��������ͷ���ݣ�����ͼ�����飬����ͼ�����ߺ��������ߵĲ�ֵ�����ڼ���Pֵ
int turn_error(uint8 img[][CAMERA_W])
{
  //ͼ���е���Ϊ��30�е�39�У�x_comp���ڴ���������������
  int x_base = 39, y_ref = 30, x_comp;
//  //��ת���㷨
//  if(img[y_ref][x_base] == 0xFF){
//    int i = x_base, j = x_base;
//    for(i ; i > 0 ; i--){
//      if(img[y_ref][i] == 0xFF && img[y_ref][i-1] == 0x00)
//        break;
//    }
//    for(j ; j < CAMERA_W; j++){
//      if(img[y_ref][j] == 0xFF && img[y_ref][j+1] ==0x00)
//        break;
//    }
//    x_comp = (i + j) / 2;
//  }
//  else if(img[y_ref][x_base] == 0x00){
//    int i = 0, j = 0;
//    for(i ; i < x_base ; i++){
//      if(img[y_ref][x_base-i] == 0x00 && img[y_ref][x_base-i-1] == 0xFF)
//        break;
//    }
//    for(j ; j < CAMERA_W-x_base; j++){
//      if(img[y_ref][x_base+j] == 0x00 && img[y_ref][x_base+j+1] ==0xFF)
//        break;
//    }
//    if(i < j){
//      int k = x_base - i;
//      for(k; k > 0; k --){
//        if(img[y_ref][k] == 0xFF && img[y_ref][k-1] == 0x00)
//          break;
//      }
//      x_comp = (k + x_base - i) / 2;
//    }
//    else if (i > j){
//      int k = x_base + j;
//      for(k; k > 0; k --){
//        if(img[y_ref][k] == 0xFF && img[y_ref][k+1] == 0x00)
//          break;
//      }
//      x_comp = (k + x_base + j) / 2;
//    }
//  }
//  
//  return (x_comp - x_base);
  
//��ת���㷨 
  //���ҳ�ʼ���ؾ�Ϊ��
  if(img[y_ref][0] == 0xFF && img[y_ref][CAMERA_W-1] == 0xFF){
    int i = 0, j = 0;
    for(i; i < 80; i++){
      if(img[y_ref][i] == 0xFF && img[y_ref][i+1] == 0x00)
        break;
    }
    for(j; j <80; j++){
      if(img[y_ref][CAMERA_W-j] == 0xFF && img[y_ref][CAMERA_W-j-1] == 0x00)
        break;
    }
    if(i - j > 0){
      for(int k = 0; k < 80; k++){
        if(img[y_ref][k] == 0xFF && img[y_ref][k+1] == 0x00)
          x_comp = k / 2;
      }
    }
    else if (i - j < 0){
      for(int k = 0; k < 80; k++){
        if(img[y_ref][CAMERA_W-k] == 0xFF && img[y_ref][CAMERA_W-k-1] == 0x00)
          x_comp = (k + 79) / 2;
      }
    }
    else
      x_comp = x_base;
  }
  //���ҳ�ʼ���ؾ�Ϊ��
  else if(img[y_ref][0] == 0x00 && img[y_ref][CAMERA_W-1] == 0x00){
    for(int i = 0; i < 80; i++){
      if(img[y_ref][i] == 0xFF && img[y_ref][i-1] == 0x00)
        x_comp = i;
      if(img[y_ref][i] == 0xFF && img[y_ref][i+1] == 0x00)
        x_comp = (x_comp + i) / 2;
    }
  }
  //����Һ�
  else if(img[y_ref][0] == 0xFF && img[y_ref][CAMERA_W-1] == 0x00){
    for(int i = 0; i < 80; i++){
      if(img[y_ref][i] == 0xFF && img[y_ref][i+1] == 0x00)
        x_comp = i / 2;
    }
  }
  //����Ұ�
  else if(img[y_ref][0] == 0x00 && img[y_ref][CAMERA_W-1] == 0xFF){
    for(int i = 0; i < 80; i++){
      if(img[y_ref][i] == 0xFF && img[y_ref][i-1] == 0x00)
        x_comp = (i + 79) / 2;
    }
  }
  return (x_comp - x_base);
}

//����Pֵ������turn_error�ķ���ֵ������turn�����õ���turn_change
int get_P(int error)
{
  int output = 0, ref, P;
  ref = abs(error);
  if(ref >= 0 && ref < 1)
    P = 0;
  else if(ref >= 1 && ref < 20)
    P = 18;
  else if(ref >= 20 && ref < 80)
    P = 20;
  output = error * P;
  return output;
}


//ѭ��ֱ�в���main����
void main(void){
  int turn_err, turn_change, abs_error, i = 0;
  uint32 pre_turn = INIT_ANGLE, speed_next = 0;
  
  init_all();
  speed_adj(INIT_SPEED);                                    //�������
  while(1){
    img_get(imgbuff, img); 
    turn_err = turn_error(img);
    abs_error = abs(turn_err);
    turn_change = get_P(turn_err);
//    uart_putchar(UART0, '@');
//    uart_putbuff(UART0, img[30], 80);
//    uart_putchar(UART0, turn_err);
//    uart_putchar(UART0, turn_change);
//    uart_putchar(UART0, '@');
    
    //ÿ4��ѭ������һ���ٶ�
    if(i == 4){
      //�ֶε����ٶȣ� ֱ������������٣��ֶ����������С
      if(abs_error >= 0 && abs_error <2)
        speed_next = INIT_SPEED + 300;
      else if (abs_error >= 2 && abs_error < 5)
        speed_next = INIT_SPEED + 300;
      else if (abs_error >= 5 && abs_error < 20)
        speed_next = INIT_SPEED - 170;
      else if(abs_error >= 20 && abs_error < 30)
        speed_next = INIT_SPEED - 200;
      else if(abs_error >= 30 && abs_error < 40)
        speed_next = INIT_SPEED - 220;
      else
        speed_next = INIT_SPEED;
      
      i = 0;
    }
    //�����ٶȲ�ת��
    speed_adj(speed_next);
    turn(turn_change, INIT_ANGLE);
    
    //�ӳ�100ms�ٴ�ѭ��
    DELAY_US(100);
    i += 1;
  }
}

