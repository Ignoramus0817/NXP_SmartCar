#include "init.h"
#include "speed.h"
#include "tracking.h"
#include "inductor.h"


//全局变量定义

//摄像头高度大约为17.5- 18.0 cm 
//imgbuff为摄像头二值图像存储一维矩阵，一字节8像素，1黑0白或相反
//img为摄像头解压后灰度图像
uint8 imgbuff[CAMERA_SIZE];      
uint8 img[CAMERA_H][CAMERA_W]; 

//舵机右左最大值，用作输入限制（占空比，单位万分之）
uint32 ANGLE_UPPER_LIMIT = 860;
uint32 ANGLE_LOWER_LIMIT = 410;
uint32 SPEED_UPPER_LIMIT = 5000;

//速度及舵机角度初始值（占空比，单位万分之）
//430（左）- 850 （右）
uint32 INIT_ANGLE = 637;
uint32 INIT_SPEED = 5000;

//摄像头参考线纵坐标
int Y_REF_STD = 30;
int Y_CHANGED = 0;

/*
 * 变量说明
 * exit_flag1:出环岛时上方黑色行数
 * exit_flag2:出环岛时下方白色行数
 * exit_flag: 出环岛条件全部满足时置1（用于指示转弯算法应进入出环岛分支）
 * enable_flag: 环岛检测使能
 * wall_flag: 检测摄像头前方是否探测到墙壁
 * island_flag & turn_flag; 环岛标志位 & 转入标志位
 * out_flag: 已开始进环岛标志位（用于使能出环岛算法）
 * reEnter_flag: 可重新检测环岛标志位（已出环岛，重新激活环岛检测，一切环岛相关标志位清空）
 * enter_counter:进环岛循环计数（为了多次循环维持同一角度）
 * exit_counter:出环岛循环计数
 */
int exit_flag1 = 0, exit_flag2 = 0, exit_flag = 0, enable_flag = 1;
int wall_flag = 1, island_flag = 0, turn_flag = 0;
int out_flag = 0, reEnter_flag = 0;
int enter_counter = 0, exit_counter = 0;
int cut_off = 0;;
uint16 LEM = 0, REM = 0;

//清除所有flag
void clear_all(){
  exit_flag1 = 0;
  exit_flag2 = 0;
  exit_flag = 0;
  wall_flag = 1;
}

void pit_5ms(){
  uint16 send[2];
  send[0] = LEM;
  send[1] = REM;
//  vcan_sendware(send, sizeof(send));
  PIT_Flag_Clear(PIT0);
}

void main(void){
  //打开开关延迟2S启动
//  DELAY_MS(2000);
  //环岛转向循环延迟
  int i = 0, j = 20;
  /*
   * 变量说明
   * turn_err：由摄像头处理算法计算得到的赛道中线与镜头中线的差值
   * turn_change_c：摄像头处理算法得到的最终舵机的偏转角度
   * abs_error：turn_error的绝对值
   * pre_turn_error：上一次循环的turn_error
   * speed_left/right：用于传入加减速函数的左右电机的占空比
   * speed：由差速算法计算得到的左右电机的占空比
   */
  int turn_err, turn_change_c, abs_error, pre_turn_err = 0;
  uint32 speed_left = INIT_SPEED, speed_right = INIT_SPEED;
  uint32 *speed;
  float error_i, turn_change_i, pre_error_i = 0;
  
  //初始化所有模块
  init_all(INIT_SPEED, INIT_ANGLE, imgbuff);
  //启动电机
  speed_adj_res(INIT_SPEED, INIT_SPEED, INIT_SPEED, SPEED_UPPER_LIMIT);
  pit_init_ms(PIT0, 5);
  set_vector_handler(PIT0_VECTORn, pit_5ms);
  enable_irq(PIT0_IRQn);
  
  //进入赛道循环
  while(1){
    //获取摄像头图像，二维数组[60][80]存储在img中
    img_get(imgbuff, img); 
    //使用摄像头处理算法（需用地址传递传入所有flag)
    turn_err = turn_error(img,
                          &exit_flag1, &exit_flag2, &exit_flag, &enable_flag,
                          &wall_flag, &island_flag, &turn_flag, &out_flag, &reEnter_flag,
                          &enter_counter, &exit_counter,
                          &Y_REF_STD, &Y_CHANGED
                          );
    error_i = get_ind(&LEM, &REM);
    //用PID算法+摄像头数据计算舵机角度
    turn_change_c = get_PID(turn_err, pre_turn_err, Y_CHANGED);
    //用PID算法+舵机数据计算舵机角度
    turn_change_i = ind_PID(error_i, pre_error_i);
    //取turn_error绝对值以改变差速的区间
    abs_error = abs(turn_err);
    //用PID算法计算左右电机的占空比，存在数组speed里
    speed = speed_control(abs_error, turn_err, pre_turn_err, i, INIT_SPEED);
    //将数组的值转存至变量，方便后面使用
    speed_left = speed[0];
    speed_right = speed[1];
    
    //环岛内减速至3200
    if(island_flag == 1 || out_flag == 1 || exit_flag == 1){
      speed_left = 3200;
      speed_right = 3200;
    }
    else{
      speed_left = 5000;
      speed_right = 5000;
    }
    
    if(is_cutoff()){
      speed_left = 3200;
      speed_right = 3200;
    }
    
    //出环岛时直行一段时间
    if(exit_counter <= 5 && exit_counter > 0 && j > 0){
      turn_change_c = 0;
      j--;
    }
    
    //使用摄像头数据改变速度和舵机输出（舵机、电机占空比实际改变的地方）
    speed_adj_res(speed_left, speed_right, init_speed, speed_upper_limit);
    turn(turn_change_c, INIT_ANGLE, ANGLE_UPPER_LIMIT, ANGLE_LOWER_LIMIT);
      
//    turn(turn_change_i, INIT_ANGLE, ANGLE_UPPER_LIMIT, ANGLE_LOWER_LIMIT);
    
    //转入环岛时延时10毫秒
    if(island_flag == 1 && turn_flag == 1)
      DELAY_MS(10);
    
    //记录本次循环的turn_err
    pre_turn_err = turn_err;
    pre_error_i = error_i;
    //i加一，2次循环改变一次差速
    i += 1;
    
//    DELAY_MS(5);
    clear_all();
  }
}