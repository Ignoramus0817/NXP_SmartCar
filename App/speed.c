#include "init.h"
#include "speed.h"

uint32 *speed_control(int abs_error, int turn_err, int pre_turn_err, int i, uint32 INIT_SPEED){
  uint32 speed_left, speed_right, speed[2];
  int speed_p_acc = 0, speed_p_dea = 0;
  //差速转向
  if(i == 2){
    //分段调整速度， 直道加速弯道减速，分段依据弯道大小，abs_error为摄像头计算赛道中线与摄像头视野中线的差值
    if(abs_error >= 0 && abs_error < 2){
      speed_p_dea = 0;
      speed_p_acc = 0;
    }
    else if(abs_error >= 2 && abs_error < 4){
      speed_p_dea = 0;
      speed_p_acc = 0;
    }
    //speed_p_acc, speed_p_dea分别为加减速p值
    else if(abs_error >= 4 && abs_error < 6){
      speed_p_dea = (4800 * 0.95) / 4;
      speed_p_acc = (4800 * 0.05) / 6;
    }
    else if(abs_error >= 6 && abs_error < 8){
      speed_p_dea = (4900 * 0.97) / 6;
      speed_p_acc = (4900 * 0.03) / 8;                       // 1 / 7
    }
    else if(abs_error >= 8 && abs_error < 10){
      speed_p_dea = (5200 * 0.96) / 8;
      speed_p_acc = (5200 * 0.04) / 10;                      // 1 / 7
    }
    else if(abs_error >= 10 && abs_error < 11){
      speed_p_dea = (5500 * 0.91) / 10;
      speed_p_acc = (5500 * 0.09) / 11;                      // 1 / 8
    }
    else if(abs_error >= 11 && abs_error < 13){
      speed_p_dea = (5800 * 0.86) / 11;
      speed_p_acc = (5800 * 0.14) / 13;                      // 1 / 8
    }
    else if(abs_error >= 13 && abs_error < 15){
      speed_p_dea = (6100 * 0.82) / 13;
      speed_p_acc = (6100 * 0.18) / 15;                      // 1 / 6
    }
    else if(abs_error >= 15){
      speed_p_dea = (6100 * 0.82) / 15;
      speed_p_acc = (6100 * 0.18) / 15;
    }
    i = 0;
  }
  // D_acc, D_dea为加减速D值
  int D_acc = 60, D_dea = 200;
  if(turn_err > 2){
    speed_left = INIT_SPEED + speed_p_acc * abs_error + (turn_err - pre_turn_err) * D_acc;
    speed_right = INIT_SPEED - speed_p_dea * abs_error - (turn_err - pre_turn_err) * D_dea;
  }
  else if(turn_err < -2){
    speed_left = INIT_SPEED - speed_p_dea * abs_error - (turn_err - pre_turn_err) * D_dea;
    speed_right = INIT_SPEED + speed_p_acc * abs_error + (turn_err - pre_turn_err) * D_acc;
  }
  
  //输入保护，以免电机全速输出
  if(speed_left >= 10000)
    speed_left = 0;
  if(speed_right >= 10000)
    speed_right = 0;
  
  speed[0] = speed_left;
  speed[1] = speed_right;
  
  return(speed);
}

//计算P值，输入turn_error的返回值，返回turn函数用到的turn_change
//error, pre_error分别为此次和上次循环中，摄像头计算赛道中线与摄像头视野中线的差值
int get_PID(int error, int pre_error, int Y_CHANGED){
  int output = 0, ref, P, D;
  int diff = error - pre_error;
  ref = abs(error);
  
  if(Y_CHANGED == 0 || Y_CHANGED > 15){
    if(ref >= 0 && ref < 2)
      P = 0;
    else if(ref >= 2 && ref < 4)
      P = 3;
    else if(ref >= 4 && ref < 5)
      P = 5;
    else if(ref >= 5 && ref < 6)
      P = 10;
    else if(ref >= 6 && ref < 8)
      P = 13;
    else if(ref >= 8 && ref < 9)
      P = 14;
    else if(ref >= 9 && ref < 11)
      P = 19;
    else if(ref >= 11 && ref < 13)
      P = 19;
    else if(ref >= 13 && ref < 15)
      P = 18;
    else if(ref >= 15)
      P = 17;
    
    if( abs(diff) > 0 && abs(diff) <= 1)
      D = 50;
    else if( abs(diff) > 1 && abs(diff) <= 3)
      D = 80;
    else if( abs(diff) > 3 && abs(diff) <= 5)
      D = 100;
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
  output = error * P + diff * D;
  
  return output;
}

//调整速度
void speed_adj_res(uint32 speed_left, uint32 speed_right, uint32 INIT_SPEED, uint32 SPEED_UPPER_LIMIT){
  if(speed_left >= SPEED_UPPER_LIMIT)
    speed_left = INIT_SPEED;
  if(speed_right >= SPEED_UPPER_LIMIT)
    speed_right = INIT_SPEED;
  ftm_pwm_duty(FTM0, FTM_CH1, speed_left);
  ftm_pwm_duty(FTM0, FTM_CH2, speed_right);
}