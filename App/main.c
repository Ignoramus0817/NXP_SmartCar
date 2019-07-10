#include "init.h"
#include "speed.h"
#include "tracking.h"
#include "inductor.h"


//ȫ�ֱ�������

//����ͷ�߶ȴ�ԼΪ17.5- 18.0 cm 
//imgbuffΪ����ͷ��ֵͼ��洢һά����һ�ֽ�8���أ�1��0�׻��෴
//imgΪ����ͷ��ѹ��Ҷ�ͼ��
uint8 imgbuff[CAMERA_SIZE];      
uint8 img[CAMERA_H][CAMERA_W]; 

//����������ֵ�������������ƣ�ռ�ձȣ���λ���֮��
uint32 ANGLE_UPPER_LIMIT = 860;
uint32 ANGLE_LOWER_LIMIT = 410;
uint32 SPEED_UPPER_LIMIT = 5000;

//�ٶȼ�����Ƕȳ�ʼֵ��ռ�ձȣ���λ���֮��
//430����- 850 ���ң�
uint32 INIT_ANGLE = 637;
uint32 INIT_SPEED = 5000;

//����ͷ�ο���������
int Y_REF_STD = 30;
int Y_CHANGED = 0;

/*
 * ����˵��
 * exit_flag1:������ʱ�Ϸ���ɫ����
 * exit_flag2:������ʱ�·���ɫ����
 * exit_flag: ����������ȫ������ʱ��1������ָʾת���㷨Ӧ�����������֧��
 * enable_flag: �������ʹ��
 * wall_flag: �������ͷǰ���Ƿ�̽�⵽ǽ��
 * island_flag & turn_flag; ������־λ & ת���־λ
 * out_flag: �ѿ�ʼ��������־λ������ʹ�ܳ������㷨��
 * reEnter_flag: �����¼�⻷����־λ���ѳ����������¼������⣬һ�л�����ر�־λ��գ�
 * enter_counter:������ѭ��������Ϊ�˶��ѭ��ά��ͬһ�Ƕȣ�
 * exit_counter:������ѭ������
 */
int exit_flag1 = 0, exit_flag2 = 0, exit_flag = 0, enable_flag = 1;
int wall_flag = 1, island_flag = 0, turn_flag = 0;
int out_flag = 0, reEnter_flag = 0;
int enter_counter = 0, exit_counter = 0;
int cut_off = 0;;
uint16 LEM = 0, REM = 0;

//�������flag
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
  //�򿪿����ӳ�2S����
//  DELAY_MS(2000);
  //����ת��ѭ���ӳ�
  int i = 0, j = 20;
  /*
   * ����˵��
   * turn_err��������ͷ�����㷨����õ������������뾵ͷ���ߵĲ�ֵ
   * turn_change_c������ͷ�����㷨�õ������ն����ƫת�Ƕ�
   * abs_error��turn_error�ľ���ֵ
   * pre_turn_error����һ��ѭ����turn_error
   * speed_left/right�����ڴ���Ӽ��ٺ��������ҵ����ռ�ձ�
   * speed���ɲ����㷨����õ������ҵ����ռ�ձ�
   */
  int turn_err, turn_change_c, abs_error, pre_turn_err = 0;
  uint32 speed_left = INIT_SPEED, speed_right = INIT_SPEED;
  uint32 *speed;
  float error_i, turn_change_i, pre_error_i = 0;
  
  //��ʼ������ģ��
  init_all(INIT_SPEED, INIT_ANGLE, imgbuff);
  //�������
  speed_adj_res(INIT_SPEED, INIT_SPEED, INIT_SPEED, SPEED_UPPER_LIMIT);
  pit_init_ms(PIT0, 5);
  set_vector_handler(PIT0_VECTORn, pit_5ms);
  enable_irq(PIT0_IRQn);
  
  //��������ѭ��
  while(1){
    //��ȡ����ͷͼ�񣬶�ά����[60][80]�洢��img��
    img_get(imgbuff, img); 
    //ʹ������ͷ�����㷨�����õ�ַ���ݴ�������flag)
    turn_err = turn_error(img,
                          &exit_flag1, &exit_flag2, &exit_flag, &enable_flag,
                          &wall_flag, &island_flag, &turn_flag, &out_flag, &reEnter_flag,
                          &enter_counter, &exit_counter,
                          &Y_REF_STD, &Y_CHANGED
                          );
    error_i = get_ind(&LEM, &REM);
    //��PID�㷨+����ͷ���ݼ������Ƕ�
    turn_change_c = get_PID(turn_err, pre_turn_err, Y_CHANGED);
    //��PID�㷨+������ݼ������Ƕ�
    turn_change_i = ind_PID(error_i, pre_error_i);
    //ȡturn_error����ֵ�Ըı���ٵ�����
    abs_error = abs(turn_err);
    //��PID�㷨�������ҵ����ռ�ձȣ���������speed��
    speed = speed_control(abs_error, turn_err, pre_turn_err, i, INIT_SPEED);
    //�������ֵת�����������������ʹ��
    speed_left = speed[0];
    speed_right = speed[1];
    
    //�����ڼ�����3200
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
    
    //������ʱֱ��һ��ʱ��
    if(exit_counter <= 5 && exit_counter > 0 && j > 0){
      turn_change_c = 0;
      j--;
    }
    
    //ʹ������ͷ���ݸı��ٶȺͶ���������������ռ�ձ�ʵ�ʸı�ĵط���
    speed_adj_res(speed_left, speed_right, init_speed, speed_upper_limit);
    turn(turn_change_c, INIT_ANGLE, ANGLE_UPPER_LIMIT, ANGLE_LOWER_LIMIT);
      
//    turn(turn_change_i, INIT_ANGLE, ANGLE_UPPER_LIMIT, ANGLE_LOWER_LIMIT);
    
    //ת�뻷��ʱ��ʱ10����
    if(island_flag == 1 && turn_flag == 1)
      DELAY_MS(10);
    
    //��¼����ѭ����turn_err
    pre_turn_err = turn_err;
    pre_error_i = error_i;
    //i��һ��2��ѭ���ı�һ�β���
    i += 1;
    
//    DELAY_MS(5);
    clear_all();
  }
}