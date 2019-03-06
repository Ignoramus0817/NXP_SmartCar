/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       main.c
 * @brief      山外K60 平台主程序
 * @author     山外科技
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

// 摄像头高度大约为17.5- 18.0 cm 
//imgbuff为摄像头二值图像存储一维矩阵，一字节8像素，1黑0白或相反
//img为摄像头解压后灰度图像
uint8 imgbuff[CAMERA_SIZE];      
uint8 img[CAMERA_H][CAMERA_W]; 

//舵机右左最大值，用作输入限制（占空比，单位万分之）
uint32 ANGLE_UPPER_LIMIT = 850;
uint32 ANGLE_LOWER_LIMIT = 450;

//速度及舵机角度初始值（占空比，单位万分之）
//450（左）- 850 （右）
uint32 INIT_ANGLE = 650;
uint32 INIT_SPEED = 2000;

//初始化
void init_all();

//方向及速度控制
void speed_adj(uint32 speed_next);
void turn(int angle_change, uint32 angle_rate);

//track detecting
int turn_error(uint8 img[][CAMERA_W]);
int get_P(int error);


////测试用main函数
//void main(void)
//{
//  uint32 spin;
//  uint32 speed;
//  init_all();
//  ftm_pwm_duty(FTM0, FTM_CH3, 800);
////  ftm_pwm_duty(FTM0, FTM_CH2, 2000);
//  ftm_pwm_duty(FTM1, FTM_CH0, 1310);
//} 

//初始化所有模块
void init_all()
{
  //电机
  ftm_pwm_init(FTM0, FTM_CH3, 10*1000, INIT_SPEED);
  ftm_pwm_init(FTM0, FTM_CH2, 10*1000, INIT_SPEED);
  
  //舵机
  ftm_pwm_init(FTM1, FTM_CH0, 100, INIT_ANGLE); 
  
  //摄像头
  uart_init(UART5, 9600);
  camera_init(imgbuff);                                  
}

//速度调整，输入为新速度
void speed_adj(uint32 speed_next)
{
    ftm_pwm_duty(FTM0, FTM_CH3, speed_next);
    ftm_pwm_duty(FTM0, FTM_CH2, speed_next);
}

//转向，参数1为角度变量，参数2为原角度
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

//处理摄像头数据，传入图像数组，返回图像中线和赛道中线的差值，用于计算P值
int turn_error(uint8 img[][CAMERA_W])
{
  //图像中点设为第30行第39列，x_comp用于储存计算的赛道中线
  int x_base = 39, y_ref = 30, x_comp;
//  //新转向算法
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
  
//旧转向算法 
  //左右初始像素均为白
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
  //左右初始像素均为黑
  else if(img[y_ref][0] == 0x00 && img[y_ref][CAMERA_W-1] == 0x00){
    for(int i = 0; i < 80; i++){
      if(img[y_ref][i] == 0xFF && img[y_ref][i-1] == 0x00)
        x_comp = i;
      if(img[y_ref][i] == 0xFF && img[y_ref][i+1] == 0x00)
        x_comp = (x_comp + i) / 2;
    }
  }
  //左白右黑
  else if(img[y_ref][0] == 0xFF && img[y_ref][CAMERA_W-1] == 0x00){
    for(int i = 0; i < 80; i++){
      if(img[y_ref][i] == 0xFF && img[y_ref][i+1] == 0x00)
        x_comp = i / 2;
    }
  }
  //左黑右白
  else if(img[y_ref][0] == 0x00 && img[y_ref][CAMERA_W-1] == 0xFF){
    for(int i = 0; i < 80; i++){
      if(img[y_ref][i] == 0xFF && img[y_ref][i-1] == 0x00)
        x_comp = (i + 79) / 2;
    }
  }
  return (x_comp - x_base);
}

//计算P值，输入turn_error的返回值，返回turn函数用到的turn_change
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


//循迹直行测试main函数
void main(void){
  int turn_err, turn_change, abs_error, i = 0;
  uint32 pre_turn = INIT_ANGLE, speed_next = 0;
  
  init_all();
  speed_adj(INIT_SPEED);                                    //启动电机
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
    
    //每4次循环调整一次速度
    if(i == 4){
      //分段调整速度， 直道加速弯道减速，分段依据弯道大小
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
    //调整速度并转弯
    speed_adj(speed_next);
    turn(turn_change, INIT_ANGLE);
    
    //延迟100ms再次循环
    DELAY_US(100);
    i += 1;
  }
}

