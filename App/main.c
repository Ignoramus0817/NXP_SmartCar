#include "common.h"
#include "include.h"
#include "math.h"
#include "IMG_GET.h"

#define TRIG_PIN PTC14
#define ECHO_PIN PTC15
//1. Ҫͨ�������������ݣ���MK60_conf.h�ļ���85����VCAN_PORTֵ��ΪUART0��ͨ�����ڷ���������ΪUART5��
//2. Ҫ������ֵ��ֱ��printf���ɣ�����д��397�к󣩣�Ҫ����ͼ����λ������IMG_GET.C�ļ���20��ȡ��ע�ͣ�ʵ��������ʱӦע�͵���
//3. �������λ��main.c��559����611�С�
//4. ���ת��PIDλ��main.c��482����517�С�

// ����ͷ�߶ȴ�ԼΪ17.5- 18.0 cm 
//imgbuffΪ����ͷ��ֵͼ��洢һά����һ�ֽ�8���أ�1��0�׻��෴
//imgΪ����ͷ��ѹ��Ҷ�ͼ��
uint8 imgbuff[CAMERA_SIZE];      
uint8 img[CAMERA_H][CAMERA_W]; 

//����������ֵ�������������ƣ�ռ�ձȣ���λ���֮��
uint32 ANGLE_UPPER_LIMIT = 820;
uint32 ANGLE_LOWER_LIMIT = 440;
uint32 SPEED_UPPER_LIMIT = 5000;

//�ٶȼ�����Ƕȳ�ʼֵ��ռ�ձȣ���λ���֮��
//430����- 850 ���ң�
uint32 INIT_ANGLE = 630;
uint32 INIT_SPEED = 3300;

//����ͷ�ο���������
int Y_REF_STD = 30;
int Y_CHANGED = 0;

// ʮ��
int cross_flag = 0;
// ����ʹ�ܼ��ټ���
int enable_flag = 1, reEnter_flag = 0;
// ��·
int cut_off_in = 0, cut_off_out = 0, cut_off2 = 0;
// ����ͷ�����
int camera_flag = 0, inductor_flag = 0;
int inductor_counter = 0;
// �ϰ���
int object_flag = 0, object_lower_max = 0, object_lower_min = 0;
int obstacle_distance = 0, time_var = 0;
int object_distance = 0;
// ����������
int exit_flag1 = 0, exit_flag2 = 0;
// ������
int exit_flag_R = 0, exit_flag_L = 0;
// ����������
int out_flag_R = 0, out_flag_L = 0;
// ��Բ�ж�
int ellipse_flag_R = 0, island_flag_R = 0, turn_flag_R = 0;
int ellipse_flag_L = 0, island_flag_L = 0, turn_flag_L = 0;
// ���뻷��������
int enter_counter_R = 0, exit_counter_R = 0;
int enter_counter_L = 0, exit_counter_L = 0;
// ����
float speed_l = 0, speed_r = 0, speed = 0, pre_speed = 0;
float speed_l_true = 0, speed_r_true = 0;
int speed_flag = 0;
// ����
int exclusive_flag = 0;
// �µ�
int ramp_flag = 0;

void speed_get();

//��ʼ������ģ��
void init_all(){
  //�����������ں�15������ZWO������4321�� ������9600
  //ʹ�������������ݣ���MK60_conf.h���޸�VCAN_PORT��UART0
  //ʹ�����߷��ͣ��޸�VCAN_PORT��UART5
  
  // ���
  // CH1-L1X CH3-L2 CH0-R1X CH2-R2 
  ftm_pwm_init(FTM0, FTM_CH1, 10*1000, INIT_SPEED);
  ftm_pwm_init(FTM0, FTM_CH2, 10*1000, INIT_SPEED);
  
  //  ftm_pwm_init(FTM0, FTM_CH3, 10*1000, INIT_SPEED);
  //  ftm_pwm_init(FTM0, FTM_CH0, 10*1000, INIT_SPEED);
  
  // ���
  ftm_pwm_init(FTM1, FTM_CH0, 100, INIT_ANGLE); 
  
  // LED
  /* 1����·
  * 2���жϻ���
  * 3���ϰ���
  * 4�����
  */
  gpio_init(PTA24, GPO, 1);
  gpio_init(PTA19, GPO, 1);
  gpio_init(PTA6, GPO, 1);
  gpio_init(PTA9, GPO, 1);
  
  // ���뿪��
  gpio_init(PTE26, GPI, 0);
  port_init_NoALT(PTE26, PULLUP);
  
  gpio_init(PTE27, GPI, 1);
  port_init_NoALT(PTE27, PULLUP);
  
  gpio_init(PTE24, GPI, 1);
  port_init_NoALT(PTE24, PULLUP);
  
  gpio_init(PTE25, GPI, 1);
  port_init_NoALT(PTE25, PULLUP);
  
  // ���������ģ��
  gpio_init(TRIG_PIN, GPO, 0);
  port_init_NoALT(TRIG_PIN,PULLUP);
  gpio_init(ECHO_PIN, GPI, 0);
  port_init_NoALT(ECHO_PIN,PULLUP);
  
  // ������
  ftm_input_init(FTM2, FTM_CH0, FTM_Falling, FTM_PS_2);
  ftm_input_init(FTM2, FTM_CH1, FTM_Falling, FTM_PS_2);
  
  // ���
  adc_init(ADC1_SE9);
  adc_init(ADC1_SE8);
  
  // ����ͷ
  uart_init(UART0, 9600);
  uart_init(UART5, 9600);
  camera_init(imgbuff); 
}

// �յ�ͣ��
int dest_flag = 0;
int destination(uint8 img[][CAMERA_W]){
  // 30��55��������Ŀ
  int jump_38 = 0, jump_50 = 0;
  // ���30��������Ŀ
  for(int i = 0; i < CAMERA_W - 1; i++){
    if( (img[38][i] == 0x00 && img[38][i + 1] == 0xFF) || (img[38][i] == 0xFF && img[38][i + 1] == 0x00) )
      jump_38 += 1;
  }
  if(jump_38 >= 12){
    dest_flag = 1;
    //    gpio_set(PTA19, 0);
  }
  // ��30���������ֹ�15��������������55��
  if(dest_flag == 1){
    for(int i = 0; i < CAMERA_W - 1; i++){
      if( (img[50][i] == 0x00 && img[50][i + 1] == 0xFF) || (img[50][i] == 0xFF && img[50][i + 1] == 0x00) )
        jump_50 += 1;
    }
  }
  
  //  printf("38L: %d, 50L: %d", jump_38, jump_50);
  // ��55��ͬ������15�����������򷵻���
  if(jump_50 >= 12){
    //    gpio_set(PTA19, 1);
    return(1);
  }
  else
    return(0);
}

void distance(void)
{
  gpio_set(TRIG_PIN, 1);               //������������
  pit_delay_us(PIT1,20);       
  gpio_set(TRIG_PIN ,0);           //����һ��20us�ĸߵ�ƽ����
  
  
  while(gpio_get(ECHO_PIN) == 0);             //�ȴ���ƽ��ߣ��͵�ƽһֱ�ȴ�
  pit_time_start(PIT1); //��ʼ��ʱ
  while(gpio_get(ECHO_PIN) == 1);              //�ȴ���ƽ��ͣ��ߵ�ƽһֱ�ȴ�         
  
  time_var = pit_time_get(PIT1);    //ֹͣ��ʱ����ȡ��ʱʱ��
  obstacle_distance = time_var*(331.4+0.607*10)/2000;  //�����¶Ȳ���
} 

uint16 LEM = 0, REM = 0, send[2] = {0, 0}; 
double optimal_data_L = 0, optimal_data_R = 0, p_L = 0, p_R = 0;
// ����жϴ�����, ƫ��ʱREM�ϴ�
void PIT0_IRQHandler(void){
  send[0] = LEM;
  send[1] = REM;
  //  vcan_sendware(send, sizeof(send));
  speed_get();  
  PIT_Flag_Clear(PIT0);
}

//�������flag
void clear_all(){
  exit_flag1 = 0;
  exit_flag2 = 0;
  exit_flag_R = 0;
  cross_flag = 0;
}

// ����ռ�ձ�
void speed_adj_res(uint32 speed_left, uint32 speed_right){  
  // ��������
  if(speed_left >= SPEED_UPPER_LIMIT)
    speed_left = INIT_SPEED;
  if(speed_right >= SPEED_UPPER_LIMIT)
    speed_right = INIT_SPEED;
  
  ftm_pwm_duty(FTM0, FTM_CH1, speed_left);
  ftm_pwm_duty(FTM0, FTM_CH2, speed_right);
}

// �����ٶ�
uint32 speed_control(float current_speed, float target_speed, uint32 duty_cycle, uint32 step, uint32 pre_speed_input, uint32 pre_target_speed){
  uint32 speed_input = 0;
  
  if(fabs(target_speed - pre_target_speed) < 0.01){
    if(current_speed < target_speed)
      speed_input = pre_speed_input + step;
    else if(current_speed > target_speed)
      speed_input = pre_speed_input - step;
    else
      speed_input = pre_speed_input;
  }
  else{
    if(current_speed < target_speed)
      speed_input = duty_cycle + step;
    else if(current_speed > target_speed)
      speed_input = duty_cycle - step;
    else
      speed_input = duty_cycle;
  }
  
  if(speed_input >= 10000)
    speed_input = 3500;  
  //  printf("s = %f, c = %f, t = %f, d: %d\n",speed, current_speed, target_speed, speed_input);
  //  printf("%d \n", speed_input);
  ftm_pwm_duty(FTM0, FTM_CH1, speed_input);
  ftm_pwm_duty(FTM0, FTM_CH2, speed_input);
  return(speed_input);
}

//ת�򣬲���1Ϊ�Ƕȱ���������2Ϊԭ�Ƕ�
void turn(int angle_change, uint32 angle_rate){
  uint32 angle;
  int temp = (int)(angle_rate) + angle_change;
  angle = abs(temp);
  // ��������
  if(angle > ANGLE_UPPER_LIMIT)
    angle = ANGLE_UPPER_LIMIT;
  if(angle < ANGLE_LOWER_LIMIT)
    angle = ANGLE_LOWER_LIMIT;
  ftm_pwm_duty(FTM1, FTM_CH0, angle);
}

//��������ͷ���ݣ�����ͼ�����飬����ͼ�����ߺ��������ߵĲ�ֵ�����ڼ���Pֵ
int turn_error(uint8 img[][CAMERA_W]){
  
  
  //ͼ���е���Ϊ��30�е�39�У�x_comp���ڴ���������������
  int x_base = 39, x_comp = 39;
  int y_ref = Y_REF_STD;
  
  // 30��Ϊȫ�׻������Ҿ�Ϊ�����õ��
  int white_pixels_30 = 0, black_pixels_30 = 0;
  for(int i = 0; i < CAMERA_W; i ++){
    if(img[30][i] == 0xFF)
      white_pixels_30 += 1;
    if(img[30][i] == 0x00)
      black_pixels_30 = 0;
  }
  
  // �ϰ����ж�
  int object_upper_edge[6] = {0, 0, 0, 0, 0, 0};                 // �ϰ����Ϸ���Ե��������
  int object_lower_edge[6] = {0, 0, 0, 0, 0, 0};                 // �ϰ����·���Ե��������
  int object_upper_detected[6] = {0, 0, 0, 0, 0, 0};             // �������飬�����������±�Ե
  int object_lower_detected[6] = {0, 0, 0, 0, 0, 0};             // �������飬�����������±�Ե
  
  // ȡ6�������ϰ����·���Ե
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
  // ȡ������6���е������Сֵ
  int max_lower = object_lower_edge[0], min_lower = object_lower_edge[0];
  for(int i = 0; i < 6; i++){
    if(object_lower_edge[i] > max_lower)
      max_lower = object_lower_edge[i];
    if(object_lower_edge[i] < min_lower)
      min_lower = object_lower_edge[i];
  }
  // ʹ��ȫ�ֱ����洢�ײ���Եֵ
  object_lower_max = max_lower;
  object_lower_min = min_lower;
  // end of �ϰ����ж�
  
  int left_y = 0, left_x = 0, right_x = 0, right_y = 0, v_start = 0;
  int v_x = 0, v_y = 0;
  int v_plain_flag = 0;
  
  // �����·
  if(PTE27_IN == 1){
    if( cut_off_in == 0 && object_flag == 0){
      
      for(int i = 79; i > 0; i--){
        if(img[i][0] == 0xFF && img[i - 1][0] == 0x00){
          left_y = i;
          break;
        }
      }
      for(int i = 79; i > 0; i--){
        if(img[i][CAMERA_W - 1] == 0xFF && img[i - 1][CAMERA_W - 1] == 0x00){
          right_y = i;
          break;
        }
      }
      if(left_y > right_y)
        v_start = right_y;
      else
        v_start = left_y;
      
      if(v_start >= 30 && v_start < 79){
        int white_v[80];
        for(int i = 0; i < 80; i++){
          white_v[i] = 0;
        }
        for(int i = 0; i < 80; i++){
          if(img[v_start][i] == 0x00)
            continue;
          for(int j = v_start; j > 0; j--){
            if(img[j][i] == 0x00 && img[j + 1][i] == 0xFF){
              white_v[i] = v_start - j + 1;
              break;
            }
          }
        }
        int v_max_length = 0, v_max_first = 0, v_max_last = 0;
        for(int i = 0; i < 80; i++){
          if(v_max_length < white_v[i]){
            v_max_length = white_v[i];
            v_max_first = i;
          }
        }
        for(int i = 0; i < 80; i++){
          if(v_max_length == white_v[i])
            v_max_last = i;
        }
        v_x = (v_max_first + v_max_last) / 2;
        v_y = v_start - v_max_length + 1;
        object_distance = abs(30 - v_y);
        
        int v_black_lines = 0;
        for(int i = v_y - 1; i > v_y - 2; i --){
          int v_black_pixels = 0;
          for(int j = 0; j < CAMERA_W; j ++){
            if(img[i][j] == 0x00)
              v_black_pixels += 1;
          }
          if(v_black_pixels >= 77)
            v_black_lines += 1;
        }
        
        if(PTE26_IN == 1){
          if( 30 - v_y <= 6 && v_black_lines >= 1){
            cut_off_in = 1;
            cut_off_out = 0;
            exclusive_flag = 1;
          }
        }
        else{
          if( abs(object_lower_max - object_lower_min) >= 2 ){
            if( 30 - v_y <= 11 && 30 - v_y >= 10){
              distance();  
              if(obstacle_distance < 32000){
                object_flag = 1;
                exclusive_flag = 1;
              }
            }
            else if( 30 - v_y <= 6 && v_black_lines >= 1){
              cut_off_in = 1;
              cut_off_out = 0;
              exclusive_flag = 1;
            }
          }
          else if(abs(object_lower_max - object_lower_min < 2)){
            if( 30 - object_lower_max >= 10 && 30 - object_lower_max <= 11  ){
              distance();
              if(obstacle_distance < 32000){
                object_flag = 1;
                exclusive_flag = 1;
              }
            }
            else if( 30 - v_y <= 6 && v_black_lines >= 1){
              cut_off_in = 1;
              cut_off_out = 0;
              exclusive_flag = 1;
            }
          }
        }
      }
      else
        cut_off_in = 0;
    }
  }
  else{
    // �ϰ����±߽����� max_lower > 20 && min_lower > 20
    if(max_lower - min_lower <= 2){
      // �������±�Ե����������±�Ե��ʼ�������ϱ�Ե
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
      // ȡ�ϱ�Ե6���е������Сֵ
      int max_upper = object_upper_edge[0], min_upper = object_upper_edge[0];
      for(int i = 0; i < 6; i++){
        if(object_upper_edge[i] > max_upper)
          max_upper = object_upper_edge[i];
        if(object_upper_edge[i] < min_upper)
          min_upper = object_upper_edge[i];
      }
      // �ϰ����ϱ߽�����
      if(max_upper > 0 && min_upper > 0 && max_upper - min_upper <= 1 &&
         max_lower - max_upper >= 11 && max_lower - max_upper <= 18 &&
           img[30][0] == 0x00 && img[30][CAMERA_W - 1] == 0x00 &&
             exclusive_flag == 0){
               // �����ϱ߽���������λ
               object_flag = 1;  
               // �ϰ��ﻥ����1
               exclusive_flag = 1;
             }
    }
    else
      object_flag = 0;
    
    left_x = 1, right_x = 78, left_y = 30, right_y = 30;
    // ����·δ��1�Ҳ����ϰ��������·�ж�
    if( cut_off_in == 0 && object_flag == 0){
      // ��30�������Ϊ0��ʼ�ж϶�·
      if(img[30][0] == 0x00 && img[30][CAMERA_W - 1] == 0x00){
        // ������·ǰ��v�ּ��м���ܳ��ֵ�ƽ������
        int cut_off_v = 0, cut_off_black = 0;
        while((left_y > 0) && (right_y > 0)){
          // ������ƽ�������򲻽��к�������
          if(v_plain_flag == 0){
            // ���ֺڰ����䣬������ָΪ��ɫ����
            if(img[left_y][left_x] == 0xFF && img[left_y][left_x - 1] == 0x00){
              // ������Ϊ��ɫ����һֱ����ֱ��������ɫ
              while(img[left_y][left_x] == 0xFF && left_y > 0){
                // �������ְ�ɫ����Ϊ���ݣ�����ѭ��
                if(img[left_y][left_x - 1] == 0xFF)
                  break;
                if(left_y > 0)
                  left_y -= 1;
              }
              // ��������������Ϊ�ף�������
              if(img[left_y][left_x - 1] == 0xFF){
                cut_off_v = 0;
                cut_off_black = 0;
                cut_off2 = 0;
                break;
              }
              // ������ɫ����
              if(left_y > 0 && left_x < 79)
                left_x += 1;
            }
            else{
              // ��������ɫ����֮ǰһֱ�����ƶ�
              int v_temp_l = left_x;
              while(img[left_y][left_x] == 0x00){
                // ����������ʱ���ò���㣬���Ҳ�����֮�����ж��Ƿ����ƽ������
                if(left_x == right_x){
                  if(v_temp_l <= right_x - 15)
                    v_plain_flag = 0;
                  else
                    v_plain_flag = 1;
                  break;
                }
                if(left_x < 79)
                  left_x += 1;
              }
            }
          }
          // �����������ȫ�Գ�
          if(v_plain_flag == 0){
            if(img[right_y][right_x] == 0xFF && img[right_y][right_x + 1] == 0x00){
              while(img[right_y][right_x] == 0xFF){
                if(img[right_y][right_x + 1] == 0xFF)
                  break;
                if(right_y > 0)
                  right_y -= 1;
              }
              if(img[right_y][right_x + 1] == 0xFF){
                cut_off_v = 0;
                cut_off_black = 0;
                cut_off2 = 0;
                break;
              }
              if(right_y > 0 && right_x > 0)
                right_x -= 1;
            }
            else{
              int v_temp_r = right_x;
              while(img[right_y][right_x] == 0x00){
                if(right_x == left_x){
                  if(v_temp_r >= left_x + 15)
                    v_plain_flag = 0;
                  else
                    v_plain_flag = 1;
                  break;
                }
                if(right_x > 0)
                  right_x -= 1;
              }
            }
          }
          // ����������V�ζ��������������ƽ�����֣�һ������ƶ�������һ������
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
        // �������С��һ��Ϊ��㣬����������ɫ����
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
        // ��ɫ������������
        if(v_black_line >= 3)
          cut_off_black = 1;
        else
          cut_off_black = 0;
        
        // ����ɫ������v�־��������������·flag��1
        // �ڶ������뿪�أ����ڿ��ƶ�·����
        if( ((cut_off_v == 1 && cut_off_black == 1)|| cut_off2 == 1) && exclusive_flag == 0 ){
          cut_off_in = 1;
          // ��·������1
          exclusive_flag = 1;
          cut_off_out = 0;
          v_black_line = 0;
          cut_off2 = 0;
        }
      }
      else
        cut_off_in = 0;
    }
    // end of �����·
  }
  
  // �뿪��·
  // ����33�����º�ɫ����
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
  // ����32�к�ɫ���ظ���
  for(int i = 0; i < CAMERA_W; i ++){
    if(img[32][i] == 0x00)
      cut_off_ref_pixels += 1;
  }
  // ����ɫ��������25��32�к�ɫ������С��75����30�����Ҿ�Ϊ��ɫ�������·
  // &&(img[30][0] == 0x00 || img[30][CAMERA_W - 1] == 0x00)
  if(PTE27_IN == 1){
    if(cut_off_out_lines >= 25 && cut_off_ref_pixels < 75){
      cut_off_in = 0;
      cut_off_out = 1;
      // ���ö�·����
      exclusive_flag = 0;
      cut_off_out_lines = 0;
    }
  }
  else{
    if( cut_off_out_lines >= 25 && cut_off_ref_pixels < 75 && (img[30][0] == 0x00 || img[30][CAMERA_W - 1] == 0x00) ){
      cut_off_in = 0;
      cut_off_out = 1;
      // ���ö�·����
      exclusive_flag = 0;
      cut_off_out_lines = 0;
    }
  }
  
  if(cut_off_in == 1 && cut_off_out == 0){
    camera_flag = 0;
    inductor_flag = 1;
  }
  else if(cut_off_in == 0){
    camera_flag = 1;
    inductor_flag = 0;
  }
  // end of �뿪��·
  
  //ת���㷨 
  //���ҳ�ʼ���ؾ�Ϊ��
  if(img[y_ref][0] == 0xFF && img[y_ref][CAMERA_W-1] == 0xFF){
    int i = 0, j = 0, y_ref_black = 0;
    for(; i < 79; i++){
      if(img[y_ref][i] == 0xFF && img[y_ref][i+1] == 0x00)
        break;
    }
    for(; j < 79; j++){
      if(img[y_ref][CAMERA_W-j] == 0xFF && img[y_ref][CAMERA_W-j-1] == 0x00)
        break;
    }
    for(; i < 79; i++){
      if(img[y_ref][i] == 0x00)
        y_ref_black += 1;
    }
    // iΪ��೤�ȣ�jΪ�Ҳ೤�ȣ�ԭ+30
    if(i - j > 0){
      for(int k = 0; k < 79; k++){
        if(img[y_ref][k] == 0xFF && img[y_ref][k+1] == 0x00)
          x_comp = k / 2;
      } 
    }
    else if (i - j < 0){
      for(int k = 0; k < 79; k++){
        if(img[y_ref][CAMERA_W-k] == 0xFF && img[y_ref][CAMERA_W-k-1] == 0x00)
          x_comp = (k + 79) / 2;
      }
    }
    else
      x_comp = x_base;
    
    if(cut_off_in != 1 && object_flag != 1){
      camera_flag = 0;
      inductor_flag = 1;
      inductor_counter = 10;
    }
  }
  //���ҳ�ʼ���ؾ�Ϊ��
  else if(img[y_ref][0] == 0x00 && img[y_ref][CAMERA_W-1] == 0x00){
    if(img[y_ref][40] == 0xFF){
      int i = 0, j = 0;
      for(i = 40; i > 0; i--){
        if(img[y_ref][i] == 0x00 && img[y_ref][i + 1] == 0xFF)
          break;
      }
      for(j = 40; j < 80; j++){
        if(img[y_ref][j] == 0x00 && img[y_ref][j - 1] == 0xFF)
          break;
      }
      x_comp = (i + j) / 2;
    }
    else{
      if(cut_off_in != 1 && object_flag != 1){
        camera_flag = 0;
        inductor_flag = 1;
        inductor_counter = 10;
      }
    }
  }
  //����Һ�
  else if(img[y_ref][0] == 0xFF && img[y_ref][CAMERA_W-1] == 0x00){
    if(img[y_ref][40] == 0x00){
      int i = 0;
      for(i = 40; i > 0; i--){
        if(img[y_ref][i - 1] == 0xFF && img[y_ref][i] == 0x00)
          break;
      }
      if( i >= 15)
        x_comp = i / 2;
      else{
        if(cut_off_in != 1 && object_flag != 1){
          camera_flag = 0;
          inductor_flag = 1;
          inductor_counter = 10;
        }
      }
    }
    else{
      int i = 0, j = 0;
      for(i = 40; i > 0; i--){
        if(img[y_ref][i - 1] == 0x00 && img[y_ref][i] == 0xFF)
          break;
      }
      for(j = 40; j < 80; j++){
        if(img[y_ref][j] == 0x00 && img[y_ref][j - 1] == 0xFF)
          break;
      }
      x_comp = (i + j) / 2;
    }
  }
  //����Ұ�
  else if(img[y_ref][0] == 0x00 && img[y_ref][CAMERA_W-1] == 0xFF){
    if(img[y_ref][40] == 0x00){
      int i = 0;
      for(i = 40; i < 80; i++){
        if(img[y_ref][i] == 0xFF && img[y_ref][i - 1] == 0x00)
          break;
      }
      if( 79 - i >= 15)
        x_comp = (i + 79) / 2;
      else{
        if(cut_off_in != 1 && object_flag != 1){
          camera_flag = 0;
          inductor_flag = 1;
          inductor_counter = 10;
        }
      }
    }
    else{
      int i = 0, j = 0;
      for(i = 40; i > 0; i--){
        if(img[y_ref][i - 1] == 0x00 && img[y_ref][i] == 0xFF)
          break;
      }
      for(j = 40; j < 80; j++){
        if(img[y_ref][j] == 0x00 && img[y_ref][j - 1] == 0xFF)
          break;
      }
      x_comp = (i + j) / 2;
    }
  }
  //printf("%d\n", y_ref);
  //end of ת���㷨
  
  if(cut_off_in != 1 && object_flag != 1){
    if(white_pixels_30 >= 77){
      camera_flag = 0;
      inductor_flag = 1;
      inductor_counter = 10;
    }
    if(inductor_counter > 0){
      camera_flag = 0;
      inductor_flag = 1;
    }
    else{
      camera_flag = 1;
      inductor_flag = 0;
    }
  }
  
  if(inductor_flag == 1)
    inductor_counter -= 1;
  
//  printf("x_comp: %d\n", x_comp);
  return (x_comp - x_base);
}

// ����Pֵ������turn_error�ķ���ֵ������turn�����õ���turn_change
// error, pre_error�ֱ�Ϊ�˴κ��ϴ�ѭ���У�����ͷ������������������ͷ��Ұ���ߵĲ�ֵ
int get_PID(int error, int pre_error){
  int output = 0, ref, P = 0, D = 0;
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
      P = 9;
    // 13
    else if(ref >= 8 && ref < 9)
      P = 10;
    // 18
    else if(ref >= 9 && ref < 11)
      P = 11;
    // 19
    else if(ref >= 11 && ref < 13)
      P = 11;
    // 18
    else if(ref >= 13 && ref < 15)
      P = 12;
    // 17
    else if(ref >= 15)
      P = 12;
    
    // 50
    if( abs(diff) > 0 && abs(diff) <= 1)
      D = 20;
    // 80
    else if( abs(diff) > 1 && abs(diff) <= 3)
      D = 15;
    // 100
    else if( abs(diff) > 3 && abs(diff) <= 5)
      D = 10;
    // 100
    else if(abs(diff) > 5)
      D = 5;
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

// ʹ�õ�м���ת��Ƕ�
float induc_turn_error(int pre_err_ratio){
  float error = 0;
  LEM = adc_once(ADC1_SE9, ADC_12bit);
  REM = adc_once(ADC1_SE8, ADC_12bit);
  error = (float)( (LEM - REM) ) / (float)((LEM+ REM));
  //printf("%f\n", error);
  return error;
}

// �����е�PIDֵ
int ind_PID(float error, float error_pre){
  int P_L = 450, D_L = 0;
  int P_R = 350, D_R = 0;
  int P = 350, D = 0;
  int output = 0;
  float diff_i = error - error_pre;
  float P_error = 0, D_pre = 0;
  float abs_error = fabs(error);
  
  if(PTE24_IN == 1 && PTE25_IN == 1){
    if( abs_error < 0.1){
      P_L = 0;
      P_R = 0;
    }
    else if( abs_error >= 0.1 && abs_error < 0.2){
      P_L = 100;
      P_R = 100;
    }
    else if( abs_error >= 0.2 && abs_error < 0.3){
      P_L = 180;
      P_R = 180;
    }
    else if( abs_error >= 0.3 && abs_error < 0.4){
      P_L = 270;
      P_R = 270;
    }
    else if( abs_error >= 0.4 && abs_error < 0.5){
      P_L = 270;
      P_R = 270;
    }
    else if( abs_error >= 0.5 && abs_error < 0.6){
      P_L = 270;
      P_R = 270;
    }
    else if( abs_error >= 0.6){
      P_L = 300;
      P_R = 300;
    }
    
    if( fabs(diff_i) > 0 && fabs(diff_i) <= 0.05)
      D = 60;
    else if( fabs(diff_i) > 0.05 && fabs(diff_i) <= 0.1)
      D = 300;
    else if( fabs(diff_i) > 0.1 && fabs(diff_i) <= 0.2)
      D = 400;
    else if(fabs(diff_i) > 0.2)
      D = 400;
  }
  else if(PTE24_IN == 1 && PTE25_IN == 0){
    P_L = 270;
    P_R = 270;
    D = 220;
  }
  else if(PTE24_IN == 0 && PTE25_IN == 1){
    P_L = 360;
    P_R = 360;
    D = 210;
  }
    //    // ��δʹ��
    //    else if(PTE24_IN == 0 && PTE25_IN == 0){}
  
  if(error >= 0){
    P = P_R;
    //D = D_R;
  }
  else{
    P = P_L;
    //D = D_L;
  }
  
  output = P * error + D * (error - error_pre);
  P_error = P * error;
  D_pre = D * (error - error_pre);
  //  printf("P = %f  D = %f \n",P_error,D_pre);
  // printf("out = %f diff = %f \n",output,diff_i);
  return(output); 
}

// ����������
void speed_input_IRQHandler(){
  uint8 s = FTM2_STATUS;
  uint8 CHn_l;
  uint8 CHn_r;
  
  FTM2_STATUS = 0X00;
  
  CHn_l = 0;
  CHn_r = 1;
  
  if(s & (1 << CHn_l) ){
    if(gpio_get(PTA13))
      speed_l -= 1;
    else
      speed_l += 1;
  }
  if(s & (1 << CHn_r) ){
    if(gpio_get(PTA12))
      speed_r -= 1;
    else
      speed_r += 1;
  }
}

// �ٶȼ���
void speed_cal(){
  pre_speed = speed;
  speed_l_true = speed_l * 0.03 / 10;
  speed_r_true = speed_r * 0.03 / 10;
  speed = (speed_l_true + speed_r_true) / 2;
}
//�ٶȻ�ȡ
void speed_get(){
  static uint8 i = 0;
  FTM_IRQ_DIS(FTM2, FTM_CH0);
  FTM_IRQ_DIS(FTM2, FTM_CH1);
  
  if(i == 0){
    speed_cal();
    speed_l = 0;
    speed_r = 0;
  }
  
  i = (i + 1) % 10;
  
  FTM_IRQ_EN(FTM2, FTM_CH0);
  FTM_IRQ_EN(FTM2, FTM_CH1);
}

//void main(void){
//  init_all();
//  while(1){
//    distance();
//    printf("%d\n", obstacle_distance);
//  }
//}

// ������������ѭ��
void main(void){
  int i = 0, j = 20;
  int turn_err = 0, turn_change_c = 0, turn_change_i = 0, abs_error, pre_turn_err = 0;
  float turn_err_i = 0, pre_turn_err_i = 0;
  uint32 speed_left = INIT_SPEED, speed_right = INIT_SPEED;
  uint32 duty_cycle = INIT_SPEED, step = 200;       // �ϴ������ռ�ձ�
  uint32 pre_speed_input = 0;
  float target_speed = 0, pre_target_speed = 0;               // Ŀ���ٶ�
  
  init_all();
  speed_adj_res(INIT_SPEED, INIT_SPEED);  
  
  // ���õ���ж�
  pit_init_ms(PIT0, 5);
  set_vector_handler(PIT0_VECTORn, PIT0_IRQHandler);
  enable_irq(PIT0_IRQn);
  // �����ٶ��ж�
  set_vector_handler(FTM2_VECTORn, speed_input_IRQHandler);
  enable_irq(FTM2_IRQn);
  
  while(1){
    img_get(imgbuff, img); 
//    distance();
//    printf("distance: %d\n", obstacle_distance);
    
    // ͣ��
    if(destination(img) == 1){
      goto STOP;
    }
    turn_err = turn_error(img);
    turn_change_c = get_PID(turn_err, pre_turn_err);
    abs_error = abs(turn_err);
    turn_err_i = induc_turn_error(pre_turn_err_i);
    turn_change_i = ind_PID(turn_err_i, pre_turn_err_i);
    
    //�ı��ٶȺͶ�����
    if(exit_counter_R <= 5 && exit_counter_R > 0 && j > 0){
      turn_change_c = 0;
      j--;
    }
    
    // �ƹ��ϰ���
    if(object_flag == 1){
      if((abs(object_lower_max - object_lower_min) >= 2 && object_distance < 9) || (abs(object_lower_max - object_lower_min) < 2 && 30 - object_lower_max < 9)){
        target_speed = 1.8;
        duty_cycle = 2300;
        speed_adj_res(duty_cycle, duty_cycle);
        int turn_change_temp = -300;
        for(int i = 0; i < 10; i ++){
          //        speed_control(speed, target_speed, duty_cycle, step, pre_speed_input, pre_target_speed);
          turn(turn_change_temp, INIT_ANGLE);
          //        printf("%d\n", turn_change_temp);
          turn_change_temp += 30;
          DELAY_MS(50);
        }
        turn_change_temp = 300;
        for(int i = 0; i < 10; i ++){
          //        speed_control(speed, target_speed, duty_cycle, step, pre_speed_input, pre_target_speed);
          turn(turn_change_temp, INIT_ANGLE);
          //        printf("%d\n", turn_change_temp);
          turn_change_temp -= 30;
          DELAY_MS(80);
        }
        target_speed = 2;
        duty_cycle = 2500;
        object_flag = 0;
        object_distance = 1000;
        //      speed_control(speed, target_speed, duty_cycle, step, pre_speed_input, pre_target_speed);
        // �����ϰ��ﻥ��
        exclusive_flag = 0; 
      }
    }
    
    // Ŀ���ٶȸ�ֵ�� ����ͷ�ٶȹ̶�
    if(camera_flag == 1){
      if( (island_flag_L == 1 && turn_flag_L == 0) || (island_flag_R == 1 && turn_flag_R == 0) ){
        target_speed = 2;
        duty_cycle = 2200;
      }
      else{
        if(abs_error > 0 && abs_error <= 4){
          target_speed = 2.2;
          duty_cycle = 3200;
        }
        else if(abs_error > 4 && abs_error <= 6){
          target_speed = 2;
          duty_cycle = 3000;
        }
        else if(abs_error > 6 && abs_error <= 9){
          target_speed = 2;
          duty_cycle = 2900;
        }
        else if(abs_error > 9){
          target_speed = 2;
          duty_cycle = 2700;
        }
      }
    }
    else if(inductor_flag == 1){
      // ��������ʱ�������ĸ����뿪�ص�״̬�����ٶ�
      if(PTE24_IN == 1 && PTE25_IN == 1){
        target_speed = 2;
        duty_cycle = 1800;
      }
      else if(PTE24_IN == 1 && PTE25_IN == 0){
        target_speed = 2;
        duty_cycle = 1800;
      }
      else if(PTE24_IN == 0 && PTE25_IN == 1){
        if(fabs(turn_err_i) > 0 && fabs(turn_err_i) <= 0.1)
          duty_cycle = 2300;
        else if(fabs(turn_err_i) > 0.1 && fabs(turn_err_i) <= 0.2)
          duty_cycle = 2100;
        else if(fabs(turn_err_i) > 0.2 && fabs(turn_err_i) <= 0.3)
          duty_cycle = 1900;
        else if(fabs(turn_err_i) > 0.3 && fabs(turn_err_i) <= 0.4)  
          duty_cycle = 1800;
        else if(fabs(turn_err_i) > 0.4)
          duty_cycle = 1600;
      }
        //      //��δʹ��
        //      else if(PTE24_IN == 0 && PTE25_IN == 0){}
    }
    
    // PID��λ 24/19/6/9
    /* 1����·
    * 2���жϻ���
    * 3���ϰ���
    * 4�����
    */
    if(cut_off_in == 1)
      gpio_set(PTA24, 0);
    if(cut_off_out == 1)
      gpio_set(PTA24, 1);
    
    if(island_flag_L == 1 || island_flag_R == 1)
      gpio_set(PTA19, 0);
    if(exit_flag_L == 1 || exit_flag_R == 1)
      gpio_set(PTA19, 1);
    
    //    if(turn_flag_R == 1 || turn_flag_L == 1)
    //      gpio_set(PTA6, 0);
    //    if(exit_flag_L == 1 || exit_flag_R == 1)
    //      gpio_set(PTA6, 1);
    
    if(object_flag == 1)
      gpio_set(PTA6, 0);
    else
      gpio_set(PTA6, 1);
    
    if(inductor_flag == 1)
      gpio_set(PTA9, 0);
    else
      gpio_set(PTA9, 1);   
    
//    // �µ����
//    if(speed_flag == 1){
//      if(speed <= 0.6 && exclusive_flag == 0){
//        ramp_flag = 1;
//        inductor_flag = 1;
//        camera_flag = 0;
//        step = 1500;
//        //      gpio_set(PTA19, 0);
//      }
//    }
//    if(speed >= 1.3 && ramp_flag == 1){
//      ramp_flag = 0;
//      inductor_flag = 0;
//      camera_flag = 1;
//      step = 2000;
//      //      gpio_set(PTA19, 1);
//    }
//    
//    // �ٶȼ�⣬�տ�ʼ����ʱ�Ȳ����ٶ�����
//    if( speed >= 1)
//      speed_flag = 1;
    
//    printf("S = %f\n",speed);
    // �ٶȸı�
//    if(ramp_flag == 1)
//      pre_speed_input = speed_control(speed, target_speed, duty_cycle, step, pre_speed_input, pre_target_speed);
//    else
    
    if(inductor_flag == 1)
     printf("%d\n", obstacle_distance); 
    
    speed_adj_res(duty_cycle, duty_cycle);
    
    //    printf("s:%f  t: %f\n", speed,  target_speed);
    //    printf("c = %f, t = %f, pre_t = %f, d: %d, s: %d\n",speed, target_speed, pre_target_speed, pre_speed_input, step);
//    if(ramp_flag == 1)
//      turn(0, INIT_ANGLE);
//    else
    
    turn(turn_change_c * camera_flag + turn_change_i * inductor_flag, INIT_ANGLE);
    
    pre_turn_err = turn_err;
    pre_turn_err_i = turn_err_i;
    pre_target_speed = target_speed;
    
    i += 1;
    
    clear_all();
  }
  
STOP:
  turn(0, INIT_ANGLE);
  speed_adj_res(0, 0);
}