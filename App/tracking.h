#include "init.h"

//Ѳ��
int turn_error(uint8 img[][CAMERA_W],
               int *exit_flag1, int *exit_flag2, int *exit_flag, int *enable_flag,
               int *wall_flag, int *island_flag, int *turn_flag,
               int *out_flag, int *reEnter_flag,
               int *enter_counter, int *exit_counter,
               int *Y_REF_STD, int *Y_CHANGED
              );

//ת��ʵ�ʸı���ռ�ձȣ�
void turn(int angle_change, uint32 angle_rate, uint32 ANGLE_UPPER_LIMIT, uint32 ANGLE_LOWER_LIMIT);