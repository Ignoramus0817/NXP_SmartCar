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
uint32 ANGLE_LOWER_LIMIT = 430;

//�ٶȼ�����Ƕȳ�ʼֵ��ռ�ձȣ���λ���֮��
//450����- 850 ���ң�
uint32 INIT_ANGLE = 635;
uint32 INIT_SPEED = 3000;

int enter_flag1 = 0, enter_flag2 = 0, enter_flag = 0, exit_flag1 = 0, exit_flag = 0;
int wall_flag = 1, stop_flag = 0, island_flag = 0, decelerate_flag = 0, turn_flag = 0;
int out_flag = 0;

//��ʼ��
void init_all();
void clear_all();

//�����ٶȿ���
void speed_adj(uint32 speed_next);
void turn(int angle_change, uint32 angle_rate);
void auto_stop(void);

//track detecting
int turn_error(uint8 img[][CAMERA_W]);
int get_P(int error);

//��ʼ������ģ��
void init_all()
{
  //���
  //CH1-L1X CH3-L2 CH0-R1X CH2-R2 
  ftm_pwm_init(FTM0, FTM_CH3, 10*1000, INIT_SPEED);
  ftm_pwm_init(FTM0, FTM_CH2, 10*1000, INIT_SPEED);
  
  //���
  ftm_pwm_init(FTM1, FTM_CH0, 100, INIT_ANGLE); 
  
  //����ͷ
  uart_init(UART5, 9600);
  camera_init(imgbuff);     
}

//�������flag
void clear_all(){
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
  out_flag = 0;
}

//�ٶȵ���������Ϊ���ٶ�
void speed_adj(uint32 speed_next)
{
    ftm_pwm_duty(FTM0, FTM_CH3, speed_next);
    ftm_pwm_duty(FTM0, FTM_CH2, speed_next);
}

//���ٰ�����ٶ�
void speed_adj_res(uint32 speed_left, uint32 speed_right){
    ftm_pwm_duty(FTM0, FTM_CH3, speed_left);
    ftm_pwm_duty(FTM0, FTM_CH2, speed_right);
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
  
  //�ж��Ƿ���ǽ
  for(int i = 0;i < CAMERA_W; i++){
    if(img[0][i] == 0x00)
      wall_flag = 0;
    if(img[2][i] == 0x00 && img[3][i] == 0x00)
      stop_flag += 1;
  }
  
  int cross_flag = 0;
  //ʮ��·������
  for(int i = 20; i <= 21; i++){
    if(img[i][0] == 0x00 && img[i][CAMERA_W-1] == 0x00 && img[i][x_base] == 0xFF)
      cross_flag += 0;
  }
  if(img[26][0] == 0xFF && img[26][CAMERA_W-1] == 0xFF)
    cross_flag += 1;
  if(cross_flag == 2)
    y_ref = 20;
  
  
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
      if(img[y_ref][x_base] == 0x00)
        y_ref = 58;
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
        if(img[y_ref][i] == 0xFF && img[y_ref][i+1] == 0x00){
          x_comp = i / 2;
          break;
        }
      }
    }
    //����Ұ�
    else if(img[y_ref][0] == 0x00 && img[y_ref][CAMERA_W-1] == 0xFF){
      for(int i = 0; i < 80; i++){
        if(img[y_ref][i] == 0xFF && img[y_ref][i-1] == 0x00){
          x_comp = (i + 79) / 2;
          break;
        }
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
  
  if(ref >= 0 && ref < 2)
    P = 0;
  else if(ref >= 2 && ref < 5)
    P = 3;
  else if(ref >= 5 && ref < 7)
    P = 4;
  else if(ref >= 7 && ref < 9)
    P = 8;
  else if(ref >= 9 && ref < 11)
    P = 9;
  else if(ref >= 11 && ref < 13)
    P = 10;
  else if(ref >= 13 && ref < 15)
    P = 11;
  else if(ref >= 15 && ref < 25)
    P = 19;
  else if(ref >= 25)
    P = 20;
  output = error * P;
  return output;
}

//�Զ�ͣ��
int track_lost(uint8 img[][CAMERA_W]){
  int count = 0, line_count = 0;
  for(int i = 0; i < 60; i++){
    for(int j = 0; j < 80; j++){
      if(img[i][j] == 0x00)
        count += 1;
    }
    if(count >= 70)
      line_count += 1;
    count = 0;
  }
  if(line_count >= 50)
    return 1;
  else
    return 0;
}

void main(void){
  int turn_err, turn_change, abs_error, i = 0;
  uint32 pre_turn = INIT_ANGLE, speed_next = 0, speed_pre_left = 0, speed_pre_right = 0;
  uint32 speed_left = INIT_SPEED, speed_right = INIT_SPEED;
  uint32 turn_angle = 0;
  
  init_all();
   speed_adj(INIT_SPEED);                                    //�������
  while(1){
    int j = 180;
    img_get(imgbuff, img); 
    turn_err = turn_error(img);
    out_flag = track_lost(img);
    abs_error = abs(turn_err);
    turn_change = get_P(turn_err);
    
    int speed_diff = 0;
    //�ǲ���ע��
    if(i == 2){
      //�ֶε����ٶȣ� ֱ������������٣��ֶ����������С
      if(abs_error >= 0 && abs_error < 2){
        speed_left = INIT_SPEED + 500;
        speed_right = INIT_SPEED + 500;
      }
      else if(abs_error >= 15 && abs_error < 20)
        speed_diff = 17;
      else if(abs_error >= 20 && abs_error < 25)
        speed_diff = 18;
      else if(abs_error >= 25 && abs_error < 30)
        speed_diff = 19;
      else if(abs_error >= 30 && abs_error < 35)
        speed_diff = 20;
      else if(abs_error >= 35 && abs_error < 40)
        speed_diff = 21;
      else if(abs_error >= 40)
        speed_diff = 22;
      
      i = 0;
    }
    
    int speed_varA = 100, speed_varB = 30;
    if(turn_err > 0){
      speed_left = INIT_SPEED + speed_diff * speed_varB;
      speed_right = INIT_SPEED - speed_diff * speed_varA;
    }
    else{
      speed_left = INIT_SPEED - speed_diff * speed_varA;
      speed_right = INIT_SPEED + speed_diff * speed_varB;
    }
    
    if(speed_left < 0)
      speed_left = 0;
    if(speed_right < 0)
      speed_right = 0;

    speed_pre_left = speed_left;
    speed_pre_right = speed_right;

  
    if(island_flag == 1 && j > 1){
      turn_change = 0;
      speed_left = 1050;
      speed_right = 1050;
      j --;
    }
    
    if(turn_flag == 1 && island_flag == 1){
      for(int i = 0; i < 8; i++){
        turn_angle += 30;
        turn(turn_angle, INIT_ANGLE);
        DELAY_MS(80);
      }
    }
   
    speed_adj_res(speed_left, speed_right);
    
    if(turn_flag == 0)
      turn(turn_change, INIT_ANGLE);
    
    //�ӳ��ٴ�ѭ��
    if(enter_flag == 1)
      DELAY_US(400);
    if(island_flag == 0)
      DELAY_US(80);
    i += 1;
    
    clear_all();
    }
  }