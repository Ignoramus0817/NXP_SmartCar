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
uint32 ANGLE_UPPER_LIMIT = 840;
uint32 ANGLE_LOWER_LIMIT = 460;

//�ٶȼ�����Ƕȳ�ʼֵ��ռ�ձȣ���λ���֮��
//450����- 850 ���ң�
uint32 INIT_ANGLE = 650;
uint32 INIT_SPEED = 2300;

int enter_flag1 = 0, enter_flag2 = 0, enter_flag = 0, exit_flag1 = 0, exit_flag = 0;
int wall_flag = 1, stop_flag = 0, island_flag = 0, decelerate_flag = 0, turn_flag = 0;;

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
  //CH1-L1, CH3-L2, CH2-R1, CH4-R2
  ftm_pwm_init(FTM0, FTM_CH1, 10*1000, INIT_SPEED);
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
    ftm_pwm_duty(FTM0, FTM_CH1, speed_next);
    ftm_pwm_duty(FTM0, FTM_CH2, speed_next);
}

//ת�򣬲���1Ϊ�Ƕȱ���������2Ϊԭ�Ƕ�
void turn(int angle_change, uint32 angle_rate)
{
  uint32 angle;
  int temp = (int)(angle_rate) + angle_change;
  angle = abs(temp);
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
  int x_base = 39, y_ref = 35, x_comp;
  //y_upper = 19-21, y_lower = 40-42
  int y_upper, y_lower;
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
  
  //�ж��Ƿ���ǽ
  for(int i = 0;i < CAMERA_W; i++){
    if(img[0][i] == 0x00)
      wall_flag = 0;
    if(img[2][i] == 0x00 && img[3][i] == 0x00)
      stop_flag += 1;
  }
  
  
  //�뻷��
  for(y_upper = 19; y_upper < 21; y_upper ++){
    if(img[y_upper][0] == 0x00 && img[y_upper][CAMERA_W-1] == 0xFF)
      enter_flag1 += 1;
  }
  for(y_lower = 40; y_lower < 44; y_lower ++){
    if(img[y_lower][0] == 0xFF && img[y_lower][CAMERA_W-1] == 0x00)
      enter_flag2 = 1;
  }
  if(enter_flag1 == 2 && enter_flag2 == 1){
    island_flag = 1;
    for(int i = 15; i < 60; i++){
      if(img[i][39] == 0x00)
        island_flag = 0;
    }
    if(img[19][59] == 0x00 && img[19][69] == 0x00)
      island_flag = 0;
    if(img[30][0] == 0xFF)
      island_flag = 0;

    //��ź�ɫ���򳤶�
    int length[23], max_length = 0, i, j;
    for(int k = 0; k < 23; k++)
      length[k] = 0;
    //ɨ��20-42�е��Ҳ������ҵ�ÿһ�к�ɫ����ĳ��ȣ�����length��
    for(i = 0; i < 23; i++){
      for(j = 79; j > 0; j--){
        if(img[i][79] == 0xFF)
          break;
        if(img[i + 20][j] == 0x00 && img[i + 20][j - 1] == 0xFF)
          break;
      }
      int temp = 0;
      if(j > 39)
        temp = CAMERA_W - j - 1;
      length[i] = temp;
      temp = 0;
    }
    
    
    //����ĺ�ɫ����
    max_length = length[0];
    for(int k = 0; k < 22;k ++){
      if(max_length < length[k+1])
        max_length = length[k+1];
    }
    
    
    //���ݺ�ɫ����ĳ������ж��Ƿ�Ӧ�ý��뻷��
    if(max_length <= 20 && max_length >= 5)
      turn_flag = 0;
    else if(max_length <= 5 && max_length > 0)
      turn_flag = 1;

    if(island_flag == 1 && turn_flag == 1){
      enter_flag = 1;
      exit_flag = 0;
    }
  }
   //������
  y_upper = 16;
  y_lower = 27;
  for(int i = 0; i < CAMERA_W; i++){
    if(img[y_upper][i] == 0x00 && img[y_lower][i] == 0xFF)
      exit_flag1 += 1;
  }
  if(exit_flag1 >= 70){
    enter_flag = 0;
    exit_flag = 1;
//    island_flag3 = 0;
  }
  //��ת���㷨 
  if(enter_flag == 1 && wall_flag == 0){
    for(int i = 0; i < 80; i++){
      if(img[y_ref][i] == 0xFF && img[y_ref][i-1] == 0x00)
        x_comp = (i + 100) / 2;
    }
  }
  else if(exit_flag == 1){
    x_comp = 80;
  }
  else{
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
  }

  return (x_comp - x_base);
}

//����Pֵ������turn_error�ķ���ֵ������turn�����õ���turn_change
int get_P(int error)
{
  int output = 0, ref, P;
  ref = abs(error);
//  if(ref >= 0 && ref < 5)
//    P = 0;
//  else if(ref >= 5 && ref < 20)
//    P = 18;
//  else if(ref >= 20 && ref < 25)
//    P = 23;
//  else if(ref >= 25 && ref < 30)
//    P = 37;
//  else if(ref >= 30 && ref < 60)
//    P = 40;
//  else if(ref >= 60 && ref < 80)
//    P = 45;
  if(ref >= 0 && ref < 5)
    P = 0;
  else if(ref >= 5 && ref < 8)
    P = 17;
  else if(ref >= 8 && ref < 10)
    P = 18;
  else if(ref >=10 && ref < 15)
    P = 19;
  else if(ref >= 15 && ref < 25)
    P = 20;
  else if(ref >= 25 && ref < 30)
    P = 21;
  else if(ref >= 30 && ref < 60)
    P = 40;
  else if(ref >= 60 && ref < 80)
    P = 45;
  output = error * P;
  return output;
}


//ѭ��ֱ�в���main����
void main(void){
  int turn_err, turn_change, abs_error, i = 0;
  uint32 pre_turn = INIT_ANGLE, speed_next = 0, speed_pre = 0;
  uint32 turn_angle = 0;
  
  init_all();
  speed_adj(INIT_SPEED);                                    //�������
  while(1){
    int j = 180;
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
        speed_next = INIT_SPEED + 500;
      else if (abs_error >= 2 && abs_error < 5)
        speed_next = INIT_SPEED + 500;
//      else if (abs_error >= 5 && abs_error < 10){
//        if(speed_pre >= 2500)
//          speed_next = INIT_SPEED - 700;
//        else
//          speed_next = INIT_SPEED - 200;
//      }
      else if (abs_error >= 5 && abs_error < 20){
        if(speed_pre >= 2500)
          speed_next = INIT_SPEED - 500;
        else
          speed_next = INIT_SPEED - 550;
      }
      else if(abs_error >= 20 && abs_error < 30){
        if(speed_pre >= 2500)
          speed_next = INIT_SPEED - 600;
        else
          speed_next = INIT_SPEED - 600;
      }
      else if(abs_error >= 30 && abs_error < 40){
        if(speed_pre >= 2500)
          speed_next = INIT_SPEED - 700;
        else
          speed_next = INIT_SPEED - 630;
      }
      else
        speed_next = INIT_SPEED;
      
      i = 0;
    }
    speed_pre = speed_next;
    
    if(island_flag == 1 && j > 1){
      turn_change = 0;
      speed_next = 1050;
      j --;
    }
   
    if(turn_flag == 1 && island_flag == 1){
      for(int i = 0; i < 8; i++){
        turn_angle += 30;
        turn(turn_angle, INIT_ANGLE);
        DELAY_MS(80);
      }
    }
    //�����ٶȲ�ת��
    speed_adj(speed_next);
    if(turn_flag == 0)
      turn(turn_change, INIT_ANGLE);
    //�ӳ��ٴ�ѭ��
    if(enter_flag == 1)
      DELAY_US(500);
    DELAY_US(100);
    i += 1;
  
  enter_flag1 = 0;
  enter_flag2 = 0;
  enter_flag = 0;
  exit_flag1 = 0;
  exit_flag = 0;
  wall_flag = 1;
  stop_flag = 0;
  turn_flag = 0;
  island_flag = 0;
  decelerate_flag = 0;
  }
}