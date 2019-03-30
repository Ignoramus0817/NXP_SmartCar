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
uint32 ANGLE_UPPER_LIMIT = 840;
uint32 ANGLE_LOWER_LIMIT = 430;
uint32 SPEED_UPPER_LIMIT = 4300;

//速度及舵机角度初始值（占空比，单位万分之）
//450（左）- 850 （右）
uint32 INIT_ANGLE = 635;
uint32 INIT_SPEED = 3500;

int exit_flag1 = 0, exit_flag2 = 0, exit_flag = 0;
int wall_flag = 1, island_flag = 0, turn_flag = 0;
int cross_flag = 0;

//初始化
void init_all();
void clear_all();

//方向及速度控制
void turn(int angle_change, uint32 angle_rate);

//track detecting
int turn_error(uint8 img[][CAMERA_W]);
int get_P(int error);

//初始化所有模块
void init_all()
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
  uart_init(UART5, 9600);
  camera_init(imgbuff); 
}

////编码器测试
//void FTM2_INPUT_IRQHandler(void){
//  uint8 s = FTM2_STATUS;
//  uint8 CHn;
//  int count_L = 0, count_R = 0;
//  
//  FTM2_STATUS = 0x00;
//  
//  CHn = 0;
//  if( s & (1 << CHn) )
//    count_L++;
//  
//  CHn = 1;
//  if( s & (1 << CHn) )
//    count_R++;
//  
//  printf("L: %d; R: %d\n", count_L, count_R);
//}
//
//void main(void){
//  init_all();
//  set_vector_handler(FTM2_VECTORn, FTM2_INPUT_IRQHandler);
//  enable_irq(FTM2_IRQn);
//  while(1);
//}

//清除所有flag
void clear_all(){
  exit_flag1 = 0;
  exit_flag2 = 0;
  exit_flag = 0;
  cross_flag = 0;
  turn_flag = 0;
  wall_flag = 1;
}

//调整速度
void speed_adj_res(uint32 speed_left, uint32 speed_right){
  if(speed_left >= SPEED_UPPER_LIMIT)
    speed_left = INIT_SPEED;;
    if(speed_right >= SPEED_UPPER_LIMIT)
      speed_right = INIT_SPEED;
    ftm_pwm_duty(FTM0, FTM_CH1, speed_left);
    ftm_pwm_duty(FTM0, FTM_CH2, speed_right);
}

//转向，参数1为角度变量，参数2为原角度
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

//处理摄像头数据，传入图像数组，返回图像中线和赛道中线的差值，用于计算P值
int turn_error(uint8 img[][CAMERA_W])
{
  //图像中点设为第30行第39列，x_comp用于储存计算的赛道中线
  int x_base = 39, y_ref = 35, x_comp;
  
  //判断是否有墙
  for(int i = 0;i < CAMERA_W; i++){
    if(img[0][i] == 0x00)
      wall_flag = 0;
  }
  
  //十字路口问题
  int cross_flag_u = 0;
  //上22/25
  if(img[25][0] == 0x00 && img[25][CAMERA_W-1] == 0x00 && img[25][x_base] == 0xFF)
    cross_flag_u += 1;
  if(img[22][0] == 0x00 && img[22][CAMERA_W-1] == 0x00 && img[22][x_base] == 0xFF)
    cross_flag_u += 1;
  //下33,37
  int cross_flag_l1 = 1, cross_flag_l2 = 1;
  for(int i = 0; i <= CAMERA_W; i++){
    if(img[33][i] == 0x00)
      cross_flag_l1 = 0;
    if(img[37][i] == 0x00)
      cross_flag_l2 = 0;
  }
  if( cross_flag_u == 2 && (cross_flag_l1 == 1 || cross_flag_l2 == 1) ){
    y_ref = 20;
    cross_flag = 1;
  }
  // y_ref调整
  int count = 0;
  for(int m = 0; m < CAMERA_W - 1; m ++){
    if( (img[y_ref][m] == 0x00 && img[y_ref][m+1] == 0xFF) ||
       (img[y_ref][m] == 0xFF && img[y_ref][m+1] == 0x00) )
      count += 1;
  }
  if(count >= 3)
    y_ref = 50;
  
  //入环岛 
  if(cross_flag == 0){
    //l25, diff < 5 
    //right_start为右侧起点（行数）
    int right_start = 0, height_bound = 35;
    for(int i = 0; i < height_bound; i++){
      if(img[i][79] == 0x00 && img[i+1][79] == 0xFF){
        right_start = i;
        break;
      }
    }
    //left_end为左侧终点（行数）
    int left_end = right_start, right_secondary = 0;
    for(int i = 0;i < 79 - 25; i ++){
      if(img[left_end][79 - i] == 0xFF){
        for(int j = 0; j < left_end; j++){
          if(img[left_end - j][79 - i] == 0x00){
            left_end = left_end - j;
            break;
          }
          else{
            if(img[left_end - j][79 - i + 1] == 0xFF){
              island_flag = 0;
              goto position1;
            }
          }
        }
      }
    }

    if(right_start - left_end <= 5)
      island_flag = 0;
    else if(right_start <= 25){
      island_flag = 1;
      exit_flag = 0;
    }
    if(img[right_start + 2][60] == 0x00)
      island_flag = 0;
    
    if( !((img[24][0] == 0x00 && img[24][CAMERA_W-1] == 0xFF) && 
        (img[25][0] == 0x00 && img[25][CAMERA_W-1] == 0xFF)) )
      island_flag = 0;;
  }
  else
    island_flag = 0;
  
  int length[40], max_length = 0, i, j;
  for(int k = 0; k < 40; k++)
    length[k] = 0;
  //扫描20-59行的右侧区域，找到每一列黑色区域的长度，存在length中
  for(i = 0; i < 40; i++){
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
  
  
  //找最长的黑色长度
  max_length = length[0];
  for(int k = 0; k < 29;k ++){
    if(max_length < length[k+1])
      max_length = length[k+1];
  }
  
  
  //根据黑色区域的长度来判断是否应该进入环岛
  if(max_length <= 20 && max_length >= 5)
    turn_flag = 0;
  else if(max_length <= 5 && max_length >= 0)
    turn_flag = 1;
  
  //出环岛
  //y_upper = 15 - 27;
  //y_lower = 30 - 59;
position1:
  for(int i = 15; i < 27; i++){
    int count = 0;
    for(int j = 0; j < CAMERA_W; j++){
      if(img[i][j] == 0x00)
        count++;
    }
    if(count >= 75)
      exit_flag1 += 1;
  }
  for(int i = 30; i < 60; i++){
    int count = 0;
    for(int j = 0; j < CAMERA_W; j++){
      if(img[i][j] == 0xFF)
        count ++;
    }
    if(count >= 75)
      exit_flag2 += 1;
  }
  
  if(exit_flag1 >= 6 && exit_flag2 >= 27){
    island_flag = 0;
    exit_flag = 1;
  }
  
  //旧转向算法 
  if(island_flag == 1 && turn_flag == 1)
    x_comp = 39 + 79 / 2;
  else if(exit_flag == 1){
    x_comp = 80;
  }
  else{
    //左右初始像素均为白
    if(img[y_ref][0] == 0xFF && img[y_ref][CAMERA_W-1] == 0xFF){
      int i = 0, j = 0;
      for(; i < 79; i++){
        if(img[y_ref][i] == 0xFF && img[y_ref][i+1] == 0x00)
          break;
      }
      for(; j < 79; j++){
        if(img[y_ref][CAMERA_W-j] == 0xFF && img[y_ref][CAMERA_W-j-1] == 0x00)
          break;
      }
      if(i - j > 0){
        for(int k = 0; k < 79; k++){
          if(img[y_ref][k] == 0xFF && img[y_ref][k+1] == 0x00)
            x_comp = (k - 30) / 2;
        }
      }
      else if (i - j < 0){
        for(int k = 0; k < 79; k++){
          if(img[y_ref][CAMERA_W-k] == 0xFF && img[y_ref][CAMERA_W-k-1] == 0x00)
            x_comp = (k + 79 + 30) / 2;
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
        if(img[y_ref][i] == 0xFF && img[y_ref][i+1] == 0x00){
          x_comp = i / 2;
          break;
        }
      }
    }
    //左黑右白
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

//计算P值，输入turn_error的返回值，返回turn函数用到的turn_change
int get_P(int error)
{
  int output = 0, ref, P;
  ref = abs(error);
  
  if(ref >= 0 && ref < 2)
    P = 0;
  else if(ref >= 2 && ref < 4)
    P = 1;
  else if(ref >= 4 && ref < 7)
    P = 7;
  else if(ref >= 7 && ref < 10)
    P = 14;
  else if(ref >= 10 && ref < 11)
    P = 20;
  else if(ref >= 11 && ref < 13)
    P = 25;
  else if(ref >= 13 && ref < 15)
    P = 27;
  else if(ref >= 15 && ref < 25)
    P = 26;
  else if(ref >= 25)
    P = 27;
  output = error * P;
  
  return output;
}

void main(void){
  int i = 0, j = 1000;
  int turn_err, turn_change, abs_error;
  uint32 speed_pre_left = 0, speed_pre_right = 0;
  uint32 speed_left = INIT_SPEED, speed_right = INIT_SPEED;
//  uint32 turn_angle = 0;
  
  init_all();
  speed_adj_res(INIT_SPEED, INIT_SPEED);                                    //启动电机
  while(1){
    img_get(imgbuff, img); 
    turn_err = turn_error(img);
    abs_error = abs(turn_err);
    turn_change = get_P(turn_err);
    
    int speed_diff_A = 0, speed_diff_B = 0;
    //非差速注释
    if(i == 2){
      //分段调整速度， 直道加速弯道减速，分段依据弯道大小
      if(abs_error >= 0 && abs_error < 2){
        speed_left = INIT_SPEED + 500;
        speed_right = INIT_SPEED + 500;
      }
      else if(abs_error >= 2 && abs_error < 5){
        speed_diff_A = 0;
        speed_diff_B = 0;
      }
      else if(abs_error >= 5 && abs_error < 7){
        speed_diff_A = 10;
        speed_diff_B = 2;
      }
      else if(abs_error >= 7 && abs_error < 10){
        speed_diff_A = 16;
        speed_diff_B = 4;
      }
      else if(abs_error >= 10 && abs_error < 15){
        speed_diff_A = 21;
        speed_diff_B = 11;
      }
      else if(abs_error >= 15 && abs_error < 20){
        speed_diff_A = 23;
        speed_diff_B = 22;
      }
      else if(abs_error >= 20 && abs_error < 25){
        speed_diff_A = 22;
        speed_diff_B = 50;
      }
      else if(abs_error >= 25 && abs_error < 30){
        speed_diff_A = 22;
        speed_diff_B = 70;
      }
      else if(abs_error >= 30 && abs_error < 35){
        speed_diff_A = 22;
        speed_diff_B = 90;
      }
      else if(abs_error >= 35){
        speed_diff_A = 22;
        speed_diff_B = 100;
      }
      i = 0;
    }
    
    int speed_varA = 0, speed_varB = 0;
    if(speed_pre_left >= 3500 || speed_pre_right >= 3500){
      speed_varA = 150;
      speed_varB = -10;
    }
    else{
      speed_varA = 135;
      speed_varB = 10;
    }
    if(turn_err > 0){
      speed_left = INIT_SPEED + speed_diff_B * speed_varB;
      speed_right = INIT_SPEED - speed_diff_A * speed_varA;
    }
    else{
      speed_left = INIT_SPEED - speed_diff_A * speed_varA;
      speed_right = INIT_SPEED + speed_diff_B * speed_varB;
    }
    
    if(speed_left > 10000)
      speed_left = 0;
    if(speed_right > 10000)
      speed_right = 0;
    
    speed_pre_left = speed_left;
    speed_pre_right = speed_right;
    
    
    if(island_flag == 1 && j > 0){
      speed_pre_left = 1600;
      speed_pre_right = 1600;
      j --;
    }
    if(island_flag == 0)
      j = 1000;
    
    if(island_flag == 1 && turn_flag == 1)
      gpio_set(PTA19, 0);
    else
      gpio_set(PTA19, 1);
    if(island_flag == 1)
      gpio_set(PTA9, 0);
    else
      gpio_set(PTA9, 1);
    
    speed_adj_res(speed_left, speed_right);
    turn(turn_change, INIT_ANGLE);
  
    if(island_flag == 0)
      DELAY_US(30);
    i += 1;
    
    clear_all();
  }
}