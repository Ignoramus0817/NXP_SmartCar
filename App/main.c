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
uint32 SPEED_UPPER_LIMIT = 4300;

//�ٶȼ�����Ƕȳ�ʼֵ��ռ�ձȣ���λ���֮��
//450����- 850 ���ң�
uint32 INIT_ANGLE = 635;
uint32 INIT_SPEED = 3500;

int enter_flag1 = 0, enter_flag2 = 0, enter_flag = 0, exit_flag1 = 0, exit_flag2 = 0, exit_flag = 0;
int wall_flag = 1, stop_flag = 0, island_flag = 0, decelerate_flag = 0, turn_flag = 0;
int out_flag = 0, enable_island_flag = 1, enable_turn_flag = 1;

//��ʼ��
void init_all();
void clear_all();

//�����ٶȿ���
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
  ftm_pwm_init(FTM0, FTM_CH1, 10*1000, INIT_SPEED);
  ftm_pwm_init(FTM0, FTM_CH2, 10*1000, INIT_SPEED);
  
  //���
  ftm_pwm_init(FTM1, FTM_CH0, 100, INIT_ANGLE); 
  
  //LED(24/19/6/9)
  gpio_init(PTA19, GPO, 1);
  gpio_init(PTA9, GPO, 1);
  
  //������
  ftm_input_init(FTM2, FTM_CH0, FTM_Falling, FTM_PS_2);
  ftm_input_init(FTM2, FTM_CH1, FTM_Falling, FTM_PS_2);
  
  //����ͷ
  uart_init(UART5, 9600);
  camera_init(imgbuff); 
}

////����������
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

//�������flag
void clear_all(){
  enter_flag1 = 0;
  enter_flag2 = 0;
  enter_flag = 0;
  exit_flag1 = 0;
  exit_flag2 = 0;
  exit_flag = 0;
  wall_flag = 1;
  stop_flag = 0;
//  turn_flag = 0;
//  island_flag = 0;
  decelerate_flag = 0;
  out_flag = 0;
}

//���ٰ�����ٶ�
void speed_adj_res(uint32 speed_left, uint32 speed_right){
  if(speed_left >= SPEED_UPPER_LIMIT)
    speed_left = INIT_SPEED;;
    if(speed_right >= SPEED_UPPER_LIMIT)
      speed_right = INIT_SPEED;
    ftm_pwm_duty(FTM0, FTM_CH1, speed_left);
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
  
  int cross_flag_u = 0;
  //ʮ��·������
  //��20-21
  for(int i = 20; i <= 21; i++){
    if(img[i][0] == 0x00 && img[i][CAMERA_W-1] == 0x00 && img[i][x_base] == 0xFF)
      cross_flag_u = 1;
  }
  //��33,37
  int cross_flag_l1 = 1, cross_flag_l2 = 1;
  for(int i = 0; i <= CAMERA_W; i++){
    if(img[33][i] == 0x00)
      cross_flag_l1 = 0;
    if(img[37][i] == 0x00)
      cross_flag_l2 = 0;
  }
  if( cross_flag_u == 1 && (cross_flag_l1 == 1 || cross_flag_l2 == 1) )
    y_ref = 20;
  
  // y_ref����
  int count = 0;
  for(int m = 0; m < CAMERA_W - 1; m ++){
    if( (img[y_ref][m] == 0x00 && img[y_ref][m+1] == 0xFF) ||
       (img[y_ref][m] == 0xFF && img[y_ref][m+1] == 0x00) )
      count += 1;
  }
  if(count >= 3)
    y_ref = 50;
  
  if(enable_island_flag == 1){
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
      for(int i = 13; i < 60; i++){
        if(img[i][39] == 0x00)
          island_flag = 0;
      }
      if(img[19][59] == 0x00 && img[19][69] == 0x00)
        island_flag = 0;
      if(img[30][0] == 0xFF)
        island_flag = 0;
      
      //l25, diff < 5 
      //right_startΪ�Ҳ���㣨������
      int right_start = 0;
      for(int i = 0; i < 19; i++){
        if(img[19 - i][79] == 0x00){
          right_start = 19 - i;
          break;
        }
      }
      //left_endΪ����е㣨������
      int left_end = right_start;
      for(int i = 0;i < 79 - 25; i ++){
        if(img[left_end][79 - i] == 0xFF){
          for(int j = 0; j < left_end; j++){
            if(img[left_end - j][79 - i] == 0x00){
              left_end = left_end - j;
              break;
            }
          }
        }
      }
      if(right_start - left_end <= 5)
        island_flag = 0;
      
      if(island_flag == 1)
        enable_island_flag = 0;
    }
  }
  
  if(enable_turn_flag == 1){  
    //��ź�ɫ���򳤶�
    int length[40], max_length = 0, i, j;
    for(int k = 0; k < 40; k++)
      length[k] = 0;
    //ɨ��20-59�е��Ҳ������ҵ�ÿһ�к�ɫ����ĳ��ȣ�����length��
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
    
    
    //����ĺ�ɫ����
    max_length = length[0];
    for(int k = 0; k < 29;k ++){
      if(max_length < length[k+1])
        max_length = length[k+1];
    }
    
    
    //���ݺ�ɫ����ĳ������ж��Ƿ�Ӧ�ý��뻷��
    if(max_length <= 20 && max_length >= 5)
      turn_flag = 0;
    else if(max_length <= 5 && max_length > 0)
      turn_flag = 1;
    
    int count_L = 0, count_R = 0;
    for(int i = 10; i < 25; i++){
      if( (img[i][50] == 0x00 && img[i+1][50] == 0xFF) ||
         (img[i][50] == 0xFF && img[i+1][50] == 0x00) )
        count_L ++;
      if( (img[i][60] == 0x00 && img[i+1][60] == 0xFF) ||
         (img[i][60] == 0xFF && img[i+1][60] == 0x00) )
        count_R ++;
    }
    if(count_L >= 3 || count_R >= 3)
      turn_flag = 0;
    
    if(island_flag == 1 && turn_flag == 1){
      enter_flag = 1;
      exit_flag = 0;
    }
    
    if(enter_flag == 1)
      enable_turn_flag = 0;
  }
  
  if(island_flag == 1 && turn_flag == 1){
    enter_flag = 1;
    exit_flag = 0;
  }
  
  //������
  //y_upper = 15 - 27;
  //y_lower = 30 - 59;
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
    enter_flag = 0;
    turn_flag = 0;
    island_flag = 0;
    exit_flag = 1;
  }
  if(island_flag == 1 && turn_flag == 1)
    y_ref = 19;
  
  //��ת���㷨 
  if(enter_flag == 1 && wall_flag == 0){
    for(int i = 0; i < 80; i++){
      if(img[y_ref][i] == 0xFF && img[y_ref][i-1] == 0x00)
        x_comp = (i + 79 - 1) / 2;
    }
  }
  else if(exit_flag == 1){
    x_comp = 80;
  }
  else{
    //���ҳ�ʼ���ؾ�Ϊ��
    if(img[y_ref][0] == 0xFF && img[y_ref][CAMERA_W-1] == 0xFF){
      int i = 0, j = 0;
      for(i; i < 79; i++){
        if(img[y_ref][i] == 0xFF && img[y_ref][i+1] == 0x00)
          break;
      }
      for(j; j < 79; j++){
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
  
  if(enter_flag == 1)
    output -= 25;
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
  uint32 speed_pre_left = 0, speed_pre_right = 0;
  uint32 speed_left = INIT_SPEED, speed_right = INIT_SPEED;
  uint32 turn_angle = 0;
  
  init_all();
  speed_adj_res(INIT_SPEED, INIT_SPEED);                                    //�������
  while(1){
    int j = 180;
    img_get(imgbuff, img); 
    turn_err = turn_error(img);
    out_flag = track_lost(img);
    abs_error = abs(turn_err);
    turn_change = get_P(turn_err);
    
    int speed_diff_A = 0, speed_diff_B = 0;
    //�ǲ���ע��
    if(i == 2){
      //�ֶε����ٶȣ� ֱ������������٣��ֶ����������С
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
    
    
    //    if(island_flag == 1 && j > 1){
    //      turn_change = 0;
    //      speed_left = 1050;
    //      speed_right = 1050;
    //      j --;
    //    }
    
    if(island_flag == 1)
      gpio_set(PTA9, 0);
    if(enter_flag == 1){
      gpio_set(PTA19, 0);
//      for(int i = 0; i < 3; i++){
//        turn_angle += 10;
//        turn(turn_angle, INIT_ANGLE);
//      }
    }
    else
      gpio_set(PTA19, 1);
    if(exit_flag == 1)
      gpio_set(PTA9, 1);
    
    speed_adj_res(speed_left, speed_right);
    
    //    if(enter_flag == 0)
    turn(turn_change, INIT_ANGLE);
    
    //�ӳ��ٴ�ѭ��
//    if(enter_flag == 1)
//      DELAY_US(380);
    if(island_flag == 0)
      DELAY_US(30);
    i += 1;
    
    clear_all();
  }
}