#define FRONT 0
#define BACK 1
#define LEFT 2
#define RIGHT 3

#include "common.h"
#include "include.h"
#include "math.h"
#include "IMG_GET.h"

//1. 要通过蓝牙发送数据，将MK60_conf.h文件第85行中VCAN_PORT值设为UART0，通过串口发送数据则为UART5。
//2. 要发送数值，直接printf即可（建议写在397行后），要发送图像到上位机，将IMG_GET.C文件第20行取消注释，实际上赛道时应注释掉。
//3. 电机差速位于main.c第559至第611行。
//4. 舵机转向PID位于main.c第482至第517行。

// 摄像头高度大约为17.5- 18.0 cm 
//imgbuff为摄像头二值图像存储一维矩阵，一字节8像素，1黑0白或相反
//img为摄像头解压后灰度图像
uint8 imgbuff[CAMERA_SIZE];      
uint8 img[CAMERA_H][CAMERA_W]; 

//舵机右左最大值，用作输入限制（占空比，单位万分之）
uint32 ANGLE_UPPER_LIMIT = 820;
uint32 ANGLE_LOWER_LIMIT = 440;
uint32 SPEED_UPPER_LIMIT = 2500;

//速度及舵机角度初始值（占空比，单位万分之）
//430（左）- 850 （右）
uint32 INIT_ANGLE = 630;
uint32 INIT_SPEED = 3500;

//摄像头参考线纵坐标
int Y_REF_STD = 30;
int Y_CHANGED = 0;

int exit_flag1 = 0, exit_flag2 = 0, exit_flag = 0, enable_flag = 1;
int ellipse_flag = 0, island_flag = 0, turn_flag = 0;
int cross_flag = 0, out_flag = 0, reEnter_flag = 0;
int enter_counter = 0, exit_counter = 0;
int cut_off_in = 0, cut_off_out = 0, camera_flag = 0, inductor_flag = 0, cut_off2 = 0;
int object_flag = 0;

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
  gpio_init(PTA24, GPO, 1);
  gpio_init(PTA19, GPO, 1);
  gpio_init(PTA6, GPO, 1);
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
double optimal_data_L = 0, optimal_data_R = 0, p_L = 0, p_R = 0;
// 电感中断处理函数, 偏左时REM较大
void PIT0_IRQHandler(void){
  send[0] = LEM;
  send[1] = REM;
  //  vcan_sendware(send, sizeof(send));
  PIT_Flag_Clear(PIT0);
}

//清除所有flag
void clear_all(){
  exit_flag1 = 0;
  exit_flag2 = 0;
  exit_flag = 0;
  cross_flag = 0;
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
  int x_base = 39, x_comp = 39;
  int y_ref = Y_REF_STD;
  
//  //十字路口问题
//  int cross_flag_u = 0;
//  //上16/17
//  if(img[16][0] == 0x00 && img[16][CAMERA_W-1] == 0x00 && img[16][x_base] == 0xFF)
//    cross_flag_u += 1;
//  if(img[17][0] == 0x00 && img[17][CAMERA_W-1] == 0x00 && img[17][x_base] == 0xFF)
//    cross_flag_u += 1;
//  //下33,37
//  int cross_flag_l1 = 1, cross_flag_l2 = 1;
//  for(int i = 0; i <= CAMERA_W; i++){
//    if(img[33][i] == 0x00)
//      cross_flag_l1 = 0;
//    if(img[37][i] == 0x00)
//      cross_flag_l2 = 0;
//  }
//  if( cross_flag_u == 2 && (cross_flag_l1 == 1 || cross_flag_l2 == 1) ){
//    y_ref = 17;
//    cross_flag = 1;
//  }
  
  int black_pixels = 0;
  for(int i = 30; i <= 59; i++){
    for(int j = 0; j < CAMERA_W; j++){
      if(img[i][j] == 0x00)
        black_pixels += 1;
    }
  }
  if(black_pixels == 0){
    int black_left = 0, black_right = 0;
    for(int i = 30; i >= 0; i--){
      if(img[i][0 + 7] == 0x00){
        black_left = i;
        break;
      }
    }
    for(int i = 30; i >= 0; i--){
      if(img[i][CAMERA_W - 1 - 7] == 0x00){
        black_right = i;
        break;
      }
    }
    int upper_cross = 0, lower_cross = 0;
    if(black_left >= black_right){
      lower_cross = black_left;
      upper_cross = black_right;
    }
    else{
      lower_cross = black_right;
      upper_cross = black_left;
    }
    int black_lines = 0;
//    for(int i = upper_cross; i >= 0; i--){
//      int white_pixels = 0;
//      for(int j = 0; j < CAMERA_W; j++){
//        if(img[i][j] == 0xFF)
//          white_pixels += 1;
//      }
//      if(white_pixels == 0)
//        black_lines += 1;
//    }
    if(black_lines <= 5){
      if(upper_cross <= 7)
        y_ref = lower_cross;
      else
        y_ref = upper_cross;
    }
    Y_CHANGED = y_ref;
    INIT_SPEED = 4000;
//    while(img[y_ref][0] == 0xFF || img[y_ref][CAMERA_W - 1] == 0xFF)
//      y_ref += 1;
  }
  else{
    y_ref = Y_REF_STD;
    Y_CHANGED = 0;
  }
  
  // y_ref调整
  int count = 0;
  for(int m = 0; m < CAMERA_W - 1; m ++){
    if( (img[y_ref][m] == 0x00 && img[y_ref][m + 1] == 0xFF) ||
       (img[y_ref][m] == 0xFF && img[y_ref][m + 1] == 0x00) )
      count += 1;
  }
  if(count >= 3)
    y_ref = 50;
  
  
  // 不是十字
  if(cross_flag == 0){
    // 障碍物判断
    int object_upper_edge[6] = {0, 0, 0, 0, 0, 0};
    int object_lower_edge[6] = {0, 0, 0, 0, 0, 0};
    int object_upper_detected[6] = {0, 0, 0, 0, 0, 0};
    int object_lower_detected[6] = {0, 0, 0, 0, 0, 0};
    
    for(int i = CAMERA_H - 1; i > 0 ; i --){
      for(int j = 1; j <= 6; j ++){
        if(img[i - 1][30 + 3 * j] == 0x00 && img[i][30 + 3 * j] == 0xFF){
          if(object_lower_detected[j - 1] == 0){
            object_lower_edge[j - 1] = i - 1;
            object_lower_detected[j - 1] = 1;
          }
        }
      }
    }
    int max_lower = object_lower_edge[0], min_lower = object_lower_edge[0];
    for(int i = 0; i < 6; i++){
      if(object_lower_edge[i] > max_lower)
        max_lower = object_lower_edge[i];
      if(object_lower_edge[i] < min_lower)
        min_lower = object_lower_edge[i];
    }
    if( max_lower > 20 && min_lower > 20 && max_lower - min_lower <= 2){
      for(int i = min_lower; i >= 0 ; i --){
        for(int j = 1; j <= 6; j ++){
          if(img[i][30 + 3 * j] == 0xFF && img[i + 1][30 + 3 * j] == 0x00){
            if(object_upper_detected[j - 1] == 0){
              object_upper_edge[j - 1] = i + 1;
              object_upper_detected[j - 1] = 1;
            }
          }
        }
      }
      
      int max_upper = object_upper_edge[0], min_upper = object_upper_edge[0];
      for(int i = 0; i < 6; i++){
        if(object_upper_edge[i] > max_upper)
          max_upper = object_upper_edge[i];
        if(object_upper_edge[i] < min_upper)
          min_upper = object_upper_edge[i];
      }
      if(max_upper > 0 && min_upper > 0 && max_upper - min_upper <= 1 && img[30][0] == 0x00 && img[30][CAMERA_W - 1] == 0x00){
        object_flag = 1;
        gpio_set(PTA6, 0);
      }   
    }
    else
      object_flag = 0;
    // end of 障碍物判断
    
    // 进入断路
    int left_x = 1, left_y = 30, right_x = CAMERA_W - 2, right_y = 30;
    int v_plain_flag = 0;
    if( cut_off_in == 0 && object_flag == 0){
      if(img[30][0] == 0x00 && img[30][CAMERA_W - 1] == 0x00){
        int cut_off_v = 0, cut_off_black = 0;
        while((left_y > 0) && (right_y > 0)){
          if(v_plain_flag == 0){
            if(img[left_y][left_x] == 0xFF && img[left_y][left_x - 1] == 0x00){
              while(img[left_y][left_x] == 0xFF && left_y > 0){
                if(img[left_y][left_x - 1] == 0xFF)
                  break;
                left_y -= 1;
              }
              if(img[left_y][left_x - 1] == 0xFF){
                cut_off_v = 0;
                cut_off_black = 0;
                cut_off2 = 0;
                break;
              }
              if(left_y > 0)
                left_x += 1;
            }
            else{
              int v_temp_l = left_x;
              while(img[left_y][left_x] == 0x00){
                left_x += 1;
                if(left_x == right_x){
                  if(v_temp_l <= right_x - 15)
                    v_plain_flag = 0;
                  else
                    v_plain_flag = 1;
                  break;
                }
              }
            }
          }
          if(v_plain_flag == 0){
            if(img[right_y][right_x] == 0xFF && img[right_y][right_x + 1] == 0x00){
              while(img[right_y][right_x] == 0xFF){
                if(img[right_y][right_x + 1] == 0xFF)
                  break;
                right_y -= 1;
              }
              if(img[right_y][right_x + 1] == 0xFF){
                cut_off_v = 0;
                cut_off_black = 0;
                cut_off2 = 0;
                break;
              }
              if(right_y > 0)
                right_x -= 1;
            }
            else{
              int v_temp_r = right_x;
              while(img[right_y][right_x] == 0x00){
                right_x -= 1;
                if(right_x == left_x){
                  if(v_temp_r >= left_x + 15)
                    v_plain_flag = 0;
                  else
                    v_plain_flag = 1;
                  break;
                }
              }
            }
          }
          if( (abs(left_x - right_x) <= 1) && v_plain_flag == 1){
            cut_off_v = 1;
            break;
          }
          else
            cut_off_v = 0;
          if(left_y >= 28 && right_y >= 28)
            cut_off2 = 1;
          else
            cut_off2 = 0;
        }
        
        int v_black_line = 0;
        int start_y = 0;
        if(left_y > right_y)
          start_y = right_y;
        else
          start_y = left_y;
        
        for(int i = start_y - 1; i > left_y - 5; i --){
          int j = 0;
          for(int k = 0; k < CAMERA_W; k++){
            if(img[i][k] == 0x00)
              j += 1;
          }
          if(j >= 78)
            v_black_line += 1;
        }
        if(v_black_line >= 3)
          cut_off_black = 1;
        else
          cut_off_black = 0;
        
        if((cut_off_v == 1 && cut_off_black == 1)|| cut_off2 == 1){
          cut_off_in = 1;
          cut_off_out = 0;
          v_black_line = 0;
          cut_off2 = 0;
        }
      }
      else
        cut_off_in = 0;
    }
      // end of 进入断路
      
      // 离开断路
      int cut_off_out_lines = 0, cut_off_ref_pixels = 0;
      for(int i = 33; i < 60; i ++){
        int cut_off_out_pixels = 0;
        for(int j = 0; j < CAMERA_W; j++){
          if(img[i][j] == 0x00)
            cut_off_out_pixels += 1;
        }
        if(cut_off_out_pixels >= 78)
          cut_off_out_lines += 1;
      }
      for(int i = 0; i < CAMERA_W; i ++){
        if(img[32][i] == 0x00)
          cut_off_ref_pixels += 1;
      }
      if(cut_off_out_lines >= 25 && cut_off_ref_pixels < 75 && img[30][0] == 0x00 && img[30][CAMERA_W - 1] == 0x00){
        cut_off_in = 0;
        cut_off_out = 1;
        cut_off_out_lines = 0;
      }
    
    if(cut_off_in == 1 && cut_off_out == 0){
      camera_flag = 0;
      inductor_flag = 1;
    }
    else if(cut_off_in == 0){
      camera_flag = 1;
      inductor_flag = 0;
    }
    // end of 离开断路
    
    //环岛判断条件
    if(enable_flag == 0)
      goto position1;
    //l25, diff < 5 
    //right_start为右侧起点（行数）
    //    for(int i = 0; i < height_bound; i++){
    //      if(img[i][79] == 0x00 && img[i+1][79] == 0xFF){
    //        right_start = i;
    //        break;
    //      }
    //    }
    int right_start = 0, left_start = 0;
    int temp1 = 0, temp2 = 0;
//    if(img[30][CAMERA_W - 1] == 0x00){
//      goto position1;
//    }
//    else{
      for(int i = 0; i <= 20; i++){
        if(img[30 - i][CAMERA_W - 1] == 0x00 && img[30 - i + 1][CAMERA_W - 1] == 0xFF){
          temp1 = 30 - i;
          break;
        }
      }
      for(int i = 1; i <= temp1; i++){
        if(img[temp1 - i][CAMERA_W - 1] == 0x00 && img[temp1 - i + 1][CAMERA_W - 1] == 0xFF){
          temp2 = temp1 - i;
          break;
        }
      }
      if(temp2 == 0){
        temp2 = temp1;
      }
      right_start = temp2;
//    }
      
      // 新搜索算法
      int left_y_i = 30, right_y_i = right_start;
      int left_x_i = 1, right_x_i = 78;
      if(right_start < 10)
        goto position1;
      while((left_y_i > 0) && (right_y_i > 0)){
          if(img[left_y_i][left_x_i] == 0xFF && img[left_y_i][left_x_i - 1] == 0x00){
            while(img[left_y_i][left_x_i] == 0xFF && left_y_i > 0){
              if(img[left_y_i][left_x_i - 1] == 0xFF)
                goto position1;
              left_y_i -= 1;
            }
            if(left_y_i > 0)
              left_x_i += 1;
          }
          else{
            while(img[left_y_i][left_x_i] == 0x00){
              left_x_i += 1;
              if(left_x_i == right_x_i)
                break;
            }
            if(left_x_i == right_x_i)
              break;
          }
          if(img[right_y_i][right_x_i] == 0xFF && img[right_y_i][right_x_i + 1] == 0x00){
            while(img[right_y_i][right_x_i] == 0xFF && right_y_i > 0){
              if(img[right_y_i][right_x_i + 1] == 0xFF)
                goto position1;
              right_y_i -= 1;
            }
            if(right_y_i > 0)
              right_x_i -= 1;
          }
          else{
            while(img[right_y_i][right_x_i] == 0x00){
              right_x_i -= 1;
              if(right_x_i == left_x_i)
                break;
            }
            if(right_x_i == left_x_i)
              break;
          }
      }
      if( left_y_i > 0 && right_y_i > 0 && abs(right_y_i - right_start) > 5 && abs(left_y_i - right_y_i) <= 2)
        island_flag = 1;
      
    //left_start为左侧分界处
    if(img[30][0] == 0x00){
      for(int i = 30; i < 60; i++){
        if(img[i][0] == 0x00 && img[i + 1][0] == 0xFF){
          left_start = i;
          break;
        }
      }
    }
    else{
      for(int i = 30; i >= 0; i--){
        if(img[i][0] == 0x00 && img[i + 1][0] == 0xFF){
          left_start = i;
          break;
        }
      }
    }
    
//    //改搜索算法
//    //left_end为左侧终点（行数）
//    int left_end = right_start;
//    for(int i = 0;i < 79 - 25; i ++){
//      if(img[left_end][79 - i] == 0xFF){
//        for(int j = 0; j < left_end; j++){
//          if(img[left_end - j][79 - i] == 0x00){
//            left_end = left_end - j;
//            break;
//          }
//          else{
//            if(img[left_end - j][79 - i + 1] == 0xFF){
//              island_flag = 0;
//              goto position1;
//            }
//          }
//        }
//      }
//    }
    
    
    int left_end_disturb = right_start;
    for(int i = 0;i < 10; i ++){
      if(img[left_end_disturb][79 - i] == 0xFF){
        for(int j = 0; j < left_end_disturb; j++){
          if(img[left_end_disturb - j][79 - i] == 0x00){
            left_end_disturb = left_end_disturb - j;
            break;
          }
        }
      }
    }
     
// // 改搜索算法
//    if(right_start - left_end <= 5)
//      island_flag = 0;
//    else if(right_start >= 10 && right_start <= 25){
//      island_flag = 1;
//      exit_flag = 0;
//    }
    if(img[right_start + 2][60] == 0x00)
      island_flag = 0;
    
//    //区别十字和环岛
//    if( abs(left_start - right_start) <= 3)
//      island_flag = 0;
    
    if( !((img[right_start + 1][0] == 0x00 && img[right_start + 1][CAMERA_W-1] == 0xFF) && 
          (img[right_start + 2][0] == 0x00 && img[right_start + 2][CAMERA_W-1] == 0xFF)) )
      island_flag = 0;
    
//    if(right_start - left_end_disturb >= 2)
//      island_flag = 0;
    
//    //转弯干扰dirty(改进：黑白跳变)
//    int disturb = 0;
//    for(int i = 30; i >= 0; i --){
//      if(img[i][39] == 0x00){
//        disturb = i;
//        break;
//      }
//    }
//    if(disturb >= 12)
//      island_flag = 0;
    
    int length[40], i, j;
    for(int k = 0; k < 40; k++)
      length[k] = 0;
    //扫描30-59行的右侧区域，找到每一列黑色区域的长度，存在length中
    for(i = 0; i < 30; i++){
      for(j = 79; j > 0; j--){
        if(img[i][79] == 0xFF)
          break;
        if(img[i + 30][j] == 0x00 && img[i + 30][j - 1] == 0xFF)
          break;
      }
      int temp = 0;
      if(j > 39)
        temp = CAMERA_W - j;
      length[i] = temp;
      temp = 0;
    }
    //end of 环岛判断条件
    
    //转入判断条件
    //找最长的黑色长度
    int temp_var_l = length[0];
    for(int k = 0; k < 29;k ++){
      if(temp_var_l < length[k+1])
        temp_var_l = length[k+1];
    }  
    
    if(island_flag == 1 && temp_var_l >= 6)
      ellipse_flag = 1;
//    printf("island_flag = %d, temp_var = %d, ellipse_flag = %d \n", island_flag, temp_var_l, ellipse_flag);
    
    if(ellipse_flag == 0)
      island_flag = 0;
    //根据黑色区域的长度来判断是否应该进入环岛
    if(temp_var_l < 2)
      turn_flag = 1;   
    
    //end of 转入判断条件
    if(island_flag == 1)
      gpio_set(PTA19, 0);
    else
      gpio_set(PTA19, 1);
    if(island_flag == 1 && turn_flag ==  1){
      gpio_set(PTA9, 0);
      enter_counter = 5;
    }
//    else
//      gpio_set(PTA9, 1);
    //end of 入环岛
    
    //出环岛
    //y_upper = 15 - 27;
    //y_lower = 30 - 59;
  position1:
    int right_bound = 0, left_bound = 0;
    int upper_bound = 0, lower_bound = 0;
    for(int i = 59; i >= 0; i--){
      if(img[i][CAMERA_W - 1] == 0x00){
        right_bound = i;
        break;
      }
    }
    for(int i = 59; i >= 0; i--){
      if(img[i][0] == 0x00){
        left_bound = i;
        break;
      }
    }
    if(left_bound <= right_bound){
      upper_bound = left_bound;
      lower_bound = right_bound;
    }
    else{
      upper_bound = right_bound;
      lower_bound = left_bound;
    }
    
    
    for(int i = 0; i < upper_bound; i++){
      int count = 0;
      for(int j = 0; j < CAMERA_W; j++){
        if(img[upper_bound - i][j] == 0x00)
          count ++;
      }
      if(count >= 75)
        exit_flag1 += 1;
    }
    if(lower_bound >= 24 && lower_bound < 30){ 
      for(int i = lower_bound + 1; i < 60; i++){
        int count = 0;
        for(int j = 0; j < CAMERA_W; j++){
          if(img[i][j] == 0x00)
            count ++;
        }
        if(count == 0)
          exit_flag2 += 1;
      }
    }
    
    int enter_white = 0;
    for(int i = 0; i < CAMERA_W; i++){
      if(img[upper_bound][i] == 0xFF)
        enter_white += 1;
    } 
//    if(enter_white == 0 && out_flag == 1 && abs(upper_bound - lower_bound) <= 8 && exit_flag1 >= 5 && exit_flag2 == 59 - lower_bound){
    if(enter_white == 0 && abs(upper_bound - lower_bound) <= 8 && exit_flag1 >= 5 && exit_flag2 == 59 - lower_bound){
      island_flag = 0;
      turn_flag = 0;
      exit_flag = 1; 
      exit_counter = 20;
      enable_flag = 0;
      reEnter_flag = 1;
      for(int i = 30; i >= 0; i--){
        if(img[i][CAMERA_W - 1] == 0x00){
          Y_REF_STD = i - 1;
          break;
        }
      }
      gpio_set(PTA24, 0);
    }
  }
  //end of 出环岛 
  
  //激活环岛判断
  if(reEnter_flag == 1 && img[30][0] == 0x00 && img[30][CAMERA_W-1] == 0x00){
    enable_flag = 1;
    ellipse_flag = 0;
    island_flag = 0;
    turn_flag = 0;
    exit_flag = 0;
    out_flag = 0;
    Y_REF_STD = 30;
    gpio_set(PTA24, 1);
  }
//  else
//    gpio_set(PTA9, 1);
  
  if( enter_counter == 1){
    Y_REF_STD = 10;
  }
  //end of 激活环岛判断
  
  //转向算法 
  if( enter_counter >= 1 && (island_flag == 1 && turn_flag == 1) ){
    x_comp = 48;
    out_flag = 1;
    reEnter_flag = 0;
    enter_counter -= 1;
    //    printf("1\n");
  }
  else if(exit_counter >= 2 || exit_flag == 1){
    x_comp = 700;
    out_flag = 0;
    exit_counter -= 1;
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
  //end of 转向算法
  return (x_comp - x_base);
}


//计算P值，输入turn_error的返回值，返回turn函数用到的turn_change
//error, pre_error分别为此次和上次循环中，摄像头计算赛道中线与摄像头视野中线的差值
int get_PID(int error, int pre_error)
{
  int output = 0, ref, P, D;
  int diff = error - pre_error;
  ref = abs(error);
  
  if(Y_CHANGED == 0 || Y_CHANGED > 15){
    if(ref >= 0 && ref < 2)
      P = 0;
    // 3
    else if(ref >= 2 && ref < 4)
      P = 3;
    // 5
    else if(ref >= 4 && ref < 5)
      P = 5;
    // 9
    else if(ref >= 5 && ref < 6)
      P = 9;
    // 12
    else if(ref >= 6 && ref < 8)
      P = 12;
    // 13
    else if(ref >= 8 && ref < 9)
      P = 13;
    // 18
    else if(ref >= 9 && ref < 11)
      P = 18;
    // 19
    else if(ref >= 11 && ref < 13)
      P = 19;
    // 18
    else if(ref >= 13 && ref < 15)
      P = 18;
    // 17
    else if(ref >= 15)
      P = 17;
    
    // 50
    if( abs(diff) > 0 && abs(diff) <= 1)
      D = 50;
    // 80
    else if( abs(diff) > 1 && abs(diff) <= 3)
      D = 80;
    // 100
    else if( abs(diff) > 3 && abs(diff) <= 5)
      D = 100;
    // 100
    else if(abs(diff) > 5)
      D = 100;
  }
  else if(Y_CHANGED >= 1 && Y_CHANGED <= 15){
    if(ref >= 0 && ref < 2)
      P = 0;
    else if(ref >= 2 && ref < 4)
      P = 7;
    else if(ref >= 4 && ref < 5)
      P = 8;
    else if(ref >= 5 && ref < 6)
      P = 9;
    else if(ref >= 6 && ref < 8)
      P = 17;
    else if(ref >= 8 && ref < 9)
      P = 18;
    else if(ref >= 9 && ref < 11)
      P = 19;
    else if(ref >= 11 && ref < 13)
      P = 19;
    else if(ref >= 13 && ref < 15)
      P = 18;
    else if(ref >= 15)
      P = 17;
    
    if( abs(diff) > 0 && abs(diff) <= 1)
      D = 60;
    else if( abs(diff) > 1 && abs(diff) <= 3)
      D = 100;
    else if( abs(diff) > 3 && abs(diff) <= 5)
      D = 120;
    else if(abs(diff) > 5)
      D = 100;
  }
  //printf("ref: %d\n", ref);
  output = error * P + diff * D;
  
  return output;
}

double filter_Kalman(const double new_data,double Q,double R, double *x_last, double *p_last){
    double x_mid;
    double p_mid;
    double p_now;

    double kg;

    x_mid=*x_last;
    p_mid=*p_last+Q;
    
    kg=p_mid/(p_mid+R);                 //kg为 kalman filter，R 为噪声
    double x_now =x_mid+kg*(new_data-x_mid);   //估计出的最优值
    p_now=(1-kg)*p_mid;                 //最优值对应的covariance
    *p_last = p_now;                     //更新covariance 值
    *x_last = x_now;                     //更新系统状态值

    return x_now;
}

//使用电感计算转向角度
float induc_turn_error(int pre_err_ratio){
  float error = 0;
//  LEM = adc_once(ADC1_SE9, ADC_12bit);
//  REM = adc_once(ADC1_SE8, ADC_12bit);
  LEM = (uint16)filter_Kalman(adc_once(ADC1_SE9, ADC_12bit), 0.00001, 0.1, &optimal_data_L, &p_L);
  LEM = (uint16)filter_Kalman(adc_once(ADC1_SE8, ADC_12bit), 0.00001, 0.1, &optimal_data_R, &p_R);
  error = (float)( (LEM - REM) ) / (float)((LEM+ REM));
  //  printf("%f\n", err_ratio);
  return error;
}

int ind_PID(float error, float error_pre){
  int P_L = 450, D_L = 0;
  int P_R = 350, D_R = 0;
  int P = 350, D = 0;
  int output = 0;
  float abs_error = fabs(error);
  
//  if( abs_error < 0.1)
//    P = 0;
//  else if( abs_error >= 0.1 && abs_error < 0.2)
//    P = 320;
//  else if( abs_error >= 0.2 && abs_error < 0.3)
//    P = 330;
//  else if( abs_error >= 0.3 && abs_error < 0.4)
//    P = 340;
//  else if( abs_error > 0.4)
//    P = 350;
  
  if( abs_error < 0.1){
    P_L = 0;
    D_L = 0;
    P_R = 0;
    D_R = 0;
  }
  else if( abs_error >= 0.1 && abs_error < 0.2){
    P_L = 100;
    D_L = 0;
    P_R = 150;
    D_R = 0;
  }
  else if( abs_error >= 0.2 && abs_error < 0.3){
    P_L = 180;
    D_L = 0;
    P_R = 270;
    D_R = 0;
  }
  else if( abs_error >= 0.3 && abs_error < 0.4){
    P_L = 270;
    D_L = 0; 
    P_R = 330;
    D_R = 0;
  }
  else if( abs_error >= 0.4 && abs_error < 0.5){
    P_L = 300;
    D_L = 0;
    P_R = 340;
    D_R = 0;
  }
  else if( abs_error >= 0.5 && abs_error < 0.6){
    P_L = 340;
    D_L = 0;
    P_R = 380;
    D_R = 0;
  }
  else if( abs_error >= 0.6){
    P_L = 380;
    D_L = 0;
    P_R = 420;
    D_R = 0;
  }
  
  if(error >= 0){
    P = P_R;
    D = D_R;
  }
  else{
    P = P_L;
    D = D_L;
  }
  output = P * error + D * (error - error_pre);
//  printf("%d\n", output);
  return(output); 
}



void main(void){
  int i = 0, j = 20;
  int turn_err = 0, turn_change_c = 0, turn_change_i = 0, abs_error, pre_turn_err = 0;
  float turn_err_i = 0, pre_turn_err_i = 0;
  uint32 speed_left = INIT_SPEED, speed_right = INIT_SPEED;
  
  init_all();
  speed_adj_res(INIT_SPEED, INIT_SPEED);                                    //启动电机
  pit_init_ms(PIT0, 5);
  set_vector_handler(PIT0_VECTORn, PIT0_IRQHandler);
  enable_irq(PIT0_IRQn);
  while(1){
    img_get(imgbuff, img); 
    turn_err = turn_error(img);
    turn_change_c = get_PID(turn_err, pre_turn_err);
    abs_error = abs(turn_err);
    turn_err_i = induc_turn_error(pre_turn_err_i);
    turn_change_i = ind_PID(turn_err_i, pre_turn_err_i);
    
//    int speed_p_acc = 0, speed_p_dea = 0;
//    //差速转向
//    if(i == 2){
//      //分段调整速度， 直道加速弯道减速，分段依据弯道大小，abs_error为摄像头计算赛道中线与摄像头视野中线的差值
//      if(abs_error >= 0 && abs_error < 2){
//        speed_p_dea = 0;
//        speed_p_acc = 0;
//      }
//      else if(abs_error >= 2 && abs_error < 4){
//        speed_p_dea = 0;
//        speed_p_acc = 0;
//      }
//      //speed_p_acc, speed_p_dea分别为加减速p值
//      else if(abs_error >= 4 && abs_error < 6){
//        speed_p_dea = (3800 * 0.92) / 4;
//        speed_p_acc = (3800 * 0.08) / 6;
//      }
//      else if(abs_error >= 6 && abs_error < 8){
//        speed_p_dea = (3900 * 0.94) / 6;
//        speed_p_acc = (3900 * 0.06) / 8;                       // 1 / 7
//      }
//      else if(abs_error >= 8 && abs_error < 10){
//        speed_p_dea = (4100 * 0.95) / 8;
//        speed_p_acc = (4100 * 0.05) / 10;                      // 1 / 7
//      }
//      else if(abs_error >= 10 && abs_error < 11){
//        speed_p_dea = (4500 * 0.95) / 10;
//        speed_p_acc = (4500 * 0.05) / 11;                      // 1 / 8
//      }
//      else if(abs_error >= 11 && abs_error < 13){
//        speed_p_dea = (4700 * 0.88) / 11;
//        speed_p_acc = (4700 * 0.12) / 13;                      // 1 / 8
//      }
//      else if(abs_error >= 13 && abs_error < 15){
//        speed_p_dea = (4900 * 0.85) / 13;
//        speed_p_acc = (4900 * 0.15) / 15;                      // 1 / 6
//      }
//      else if(abs_error >= 15){
//        speed_p_dea = (4900 * 0.82) / 15;
//        speed_p_acc = (4900 * 0.18) / 15;
//      }
//      i = 0;
//    }
//    // D_acc, D_dea为加减速D值
//    int D_acc = 60, D_dea = 200;
//    if(turn_err > 2){
//      speed_left = INIT_SPEED + speed_p_acc * abs_error + (turn_err - pre_turn_err) * D_acc;
//      speed_right = INIT_SPEED - speed_p_dea * abs_error - (turn_err - pre_turn_err) * D_dea;
//    }
//    else if(turn_err < -2){
//      speed_left = INIT_SPEED - speed_p_dea * abs_error - (turn_err - pre_turn_err) * D_dea;
//      speed_right = INIT_SPEED + speed_p_acc * abs_error + (turn_err - pre_turn_err) * D_acc;
//    }
//    //end of 差速转向
    
    //输入保护，以免电机全速输出
    if(speed_left >= 10000)
      speed_left = 0;
    if(speed_right >= 10000)
      speed_right = 0;
    
    if(island_flag == 1 || out_flag == 1 || exit_flag == 1){
      INIT_SPEED = 3500;
      INIT_SPEED = 3500;
    }
    else{
      INIT_SPEED = 3500;
      INIT_SPEED = 3500;
    }
    
       // 转向减速
    if(abs_error > 0 && abs_error <= 4)
      INIT_SPEED = 3500;
    else if(abs_error > 4 && abs_error <= 6)
      INIT_SPEED = 2700;
    else if(abs_error > 6 && abs_error <= 9)
      INIT_SPEED = 2300;
    else if(abs_error > 9)
      INIT_SPEED = 2000;
    
    if(cut_off_in == 1)
      INIT_SPEED = 1500;
    if(cut_off_out ==  1)
      INIT_SPEED = 3500;
    
    //改变速度和舵机输出
    if(exit_counter <= 5 && exit_counter > 0 && j > 0){
      turn_change_c = 0;
      j--;
    }
 
    if(object_flag == 1){
      INIT_SPEED = 1500;
      speed_adj_res(INIT_SPEED, INIT_SPEED);
      int turn_change_temp = -200;
      for(int i = 0; i < 10; i ++){
        turn(turn_change_temp, INIT_ANGLE);
        turn_change_temp += 20;
        DELAY_MS(45);
      }
      turn_change_temp = 200;
      for(int i = 0; i < 10; i ++){
        turn(turn_change_temp, INIT_ANGLE);
        turn_change_temp -= 20;
        DELAY_MS(45);
      }
      INIT_SPEED = 3500;
    }
    speed_adj_res(speed_left, speed_right);
    turn(turn_change_c * camera_flag + turn_change_i * inductor_flag, INIT_ANGLE);
//    printf("turn_err = %d", turn_err);
//    printf("turn_change_c = %d\n", turn_change_c);
//    turn(turn_change_i, INIT_ANGLE);
//    turn(turn_change_c, INIT_ANGLE);
    
    pre_turn_err = turn_err;
    pre_turn_err_i = turn_err_i;
    
    i += 1;
    
    clear_all();
  }
}