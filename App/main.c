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
//430（左）- 850 （右）
uint32 INIT_ANGLE = 640;
uint32 INIT_SPEED = 3500;

int exit_flag1 = 0, exit_flag2 = 0, exit_flag = 0, enable_flag = 1;
int wall_flag = 1, island_flag = 0, turn_flag = 0;
int cross_flag = 0;
int peak_counter = 0, valley_counter = 0, peak_start = 0, valley_start = 0;

//初始化
void init_all();
void clear_all();

//方向及速度控制
void turn(int angle_change, uint32 angle_rate);

//track detecting
int turn_error(uint8 img[][CAMERA_W]);
int get_PID(int error, int pre_error);

//初始化所有模块
void init_all()
{
  //蓝牙主机串口号15，名字ZWO，密码4321， 波特率9600
  //使用蓝牙发送数据，在MK60_conf.h中修改VCAN_PORT至UART0
  //使用有线发送，修改VCAN_PORT至UART5
  
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
  
  //电感
  adc_init(ADC1_SE9);
  adc_init(ADC1_SE8);
  
  //摄像头
  uart_init(UART0, 9600);
  uart_init(UART5, 9600);
  camera_init(imgbuff); 
}

uint16 LEM = 0, REM = 0, send[2] = {0, 0}; 
// 电感中断处理函数, 偏左时REM较大
void PIT0_IRQHandler(void){
  LEM = adc_once(ADC1_SE8, ADC_12bit);
  REM = adc_once(ADC1_SE9, ADC_12bit);
  send[0] = LEM;
  send[1] = REM;
  
//  printf("L:%d, R:%d\n", LEM, REM);
  
  if(LEM - REM >= 1400 && LEM >= 1800)
    peak_start = 1;
  if(peak_start == 1)
    peak_counter += 1;
  if(peak_start == 1 && LEM <= 1000)
    valley_start = 1;
  if(valley_start == 1)
    valley_counter += 1;
  if(valley_start == 1 && LEM - REM >= 1400){
    peak_start = 0;
    valley_start = 0;
  }
//  vcan_sendware(send, sizeof(send));
  PIT_Flag_Clear(PIT0);
}

//void main(void){
//  init_all();
//  pit_init_us(PIT0, 50);
//  set_vector_handler(PIT0_VECTORn, PIT0_IRQHandler);
//  enable_irq(PIT0_IRQn);
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
    speed_left = INIT_SPEED;
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
  //上16/17
  if(img[16][0] == 0x00 && img[16][CAMERA_W-1] == 0x00 && img[16][x_base] == 0xFF)
    cross_flag_u += 1;
  if(img[17][0] == 0x00 && img[17][CAMERA_W-1] == 0x00 && img[17][x_base] == 0xFF)
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
    y_ref = 17;
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
    int left_end = right_start;
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
        count ++;
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
  
  if(exit_flag1 >= 4 && exit_flag2 >= 23){
    island_flag = 0;
    exit_flag = 1;
    peak_counter = 0;
    valley_counter = 0;
    enable_flag = 0;
  }
  
  if(enable_flag == 0){
    island_flag = 0;
    turn_flag = 0;
  }
    
  //旧转向算法 
  if(island_flag == 1 && turn_flag == 1){
    x_comp = 39 + 89 / 2;
//    printf("1\n");
  }
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
int get_PID(int error, int pre_error)
{
  int output = 0, ref, P, D;
  int diff = error - pre_error;
  ref = abs(error);
  
  if(ref >= 0 && ref < 2)
    P = 0;
  else if(ref >= 2 && ref < 4)
    P = 2;
  else if(ref >= 4 && ref < 5)
    P = 8;
  else if(ref >= 5 && ref < 6)
    P = 15;
  else if(ref >= 6 && ref < 8)
    P = 18;
  else if(ref >= 8 && ref < 10)
    P = 19;
  else if(ref >= 10 && ref < 11)
    P = 20;
  else if(ref >= 11 && ref < 13)
    P = 25;
  else if(ref >= 13 && ref < 15)
    P = 27;
  else if(ref >= 15)
    P = 14;
  
  if( abs(diff) >= 0 && abs(diff) < 4)
    D = 0;
  else if(abs(diff) >= 4)
    D = 10;
  
  output = error * P + diff * D;
  
  return output;
}

//使用电感计算转向角度
float induc_turn_error(uint16 LIND, uint16 RIND, int pre_err_ratio){
  float err_ratio = 0;
  err_ratio = 0 - (float)( (LIND - RIND) ) / (float)((LIND + RIND));
//  printf("%f\n", err_ratio);
  return err_ratio;
}

int induc_PID(float err_ratio, float pre_err_ratio){
  int P = 400, D = 400, output = 0;
  if(err_ratio >= 0 && err_ratio < 0.1)
    P = 50;
  else if(err_ratio >= 0.1 && err_ratio < 0.3)
    P = 400;
  else if(err_ratio >= 0.3)
    P = 600;
  output = err_ratio * P + (err_ratio - pre_err_ratio) * D;
}

void main(void){
  float err_ratio = 0, pre_err_ratio = 0;
  int i = 0, j = 1000;
  int turn_err, turn_change_c, turn_change_i, abs_error, pre_turn_err = 0;
  uint32 speed_pre_left = 0, speed_pre_right = 0;
  uint32 speed_left = INIT_SPEED, speed_right = INIT_SPEED;
  
  init_all();
  speed_adj_res(INIT_SPEED, INIT_SPEED);                                    //启动电机
  pit_init_ms(PIT0, 10);
  set_vector_handler(PIT0_VECTORn, PIT0_IRQHandler);
  enable_irq(PIT0_IRQn);
  while(1){
    img_get(imgbuff, img); 
    turn_err = turn_error(img);
    turn_change_c = get_PID(turn_err, pre_turn_err);
    abs_error = abs(turn_err);
    err_ratio = induc_turn_error(LEM, REM, pre_turn_err);
    turn_change_i = induc_PID(err_ratio, pre_err_ratio);
    
    int speed_p_acc = 0, speed_p_dea = 0;
    //非差速注释
     if(i == 2){
      //分段调整速度， 直道加速弯道减速，分段依据弯道大小
      if(abs_error >= 0 && abs_error < 2){
        speed_left = INIT_SPEED + 300;
        speed_right = INIT_SPEED + 300;
      }
      else if(abs_error >= 2 && abs_error < 4){
        speed_p_dea = 0;
        speed_p_acc = 0;
      }
      else if(abs_error >= 4 && abs_error < 6){
        speed_p_dea = (3000 * 0.9) / 5;
        speed_p_acc = (3000 * 0.1) / 7;
      }
      else if(abs_error >= 6 && abs_error < 8){
        speed_p_dea = (3500 * 0.857) / 5;
        speed_p_acc = (3500 * 0.143) / 7;                       // 1 / 7
      }
      else if(abs_error >= 8 && abs_error < 10){
        speed_p_dea = (3700 * 0.857) / 7;
        speed_p_acc = (3700 * 0.143) / 10;                      // 1 / 7
      }
      else if(abs_error >= 10 && abs_error < 11){
        speed_p_dea = (3900 * 0.875) / 10;
        speed_p_acc = (3900 * 0.125) / 15;                      // 1 / 8
      }
      else if(abs_error >= 11 && abs_error < 13){
        speed_p_dea = (4000 * 0.875) / 15;
        speed_p_acc = (4000 * 0.125) / 20;                      // 1 / 8
      }
      else if(abs_error >= 13 && abs_error < 15){
        speed_p_dea = (4200 * 0.833) / 20;
        speed_p_acc = (4200 * 0.167) / 25;                      // 1 / 6
      }
      else if(abs_error >= 15){
        speed_p_dea = (4200 * 0.833) / 5;
        speed_p_acc = (4200 * 0.167) / 7;
      }
      i = 0;
    }
    
    int D_acc = 50, D_dea = 200;
    if(turn_err > 2){
      speed_left = INIT_SPEED + speed_p_acc * abs_error + (turn_err - pre_turn_err) * D_acc;
      speed_right = INIT_SPEED - speed_p_dea * abs_error - (turn_err - pre_turn_err) * D_dea;
    }
    else if(turn_err < -2){
      speed_left = INIT_SPEED - speed_p_dea * abs_error - (turn_err - pre_turn_err) * D_dea;
      speed_right = INIT_SPEED + speed_p_acc * abs_error + (turn_err - pre_turn_err) * D_acc;
    }
    
    if(speed_left > 10000)
      speed_left = 0;
    if(speed_right > 10000)
      speed_right = 0;
    
    //存储上次循环的速度
    speed_pre_left = speed_left;
    speed_pre_right = speed_right;
    
//    //环岛
//    if(island_flag == 1 && j > 0){
//      speed_pre_left = 1600;
//      speed_pre_right = 1600;
//      j --;
//    }
//    if(island_flag == 0)
//      j = 1000;
    
//    if( !(peak_counter > 0 && peak_counter <= 30 && valley_counter > 0) ){
//      island_flag = 0;
//      turn_flag = 0;
//    }
//    else
//      printf("2\n");
    
    //改变速度和舵机输出
    speed_adj_res(speed_left, speed_right);
//    if(cross_flag == 1)
//      turn( (turn_change_c + turn_change_i) / 2, INIT_ANGLE);
//    else
//      turn(turn_change_c, INIT_ANGLE);
    turn(turn_change_c, INIT_ANGLE);
    pre_turn_err = turn_err;
    pre_err_ratio = err_ratio;
  
    i += 1;
    
    clear_all();
  }
}