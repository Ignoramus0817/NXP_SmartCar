#include "init.h"

uint32 *speed_control(int abs_error, int turn_err, int pre_turn_err, int i, uint32 INIT_SPEED);
int get_PID(int error, int pre_error, int Y_CHANGED);
void speed_adj_res(uint32 speed_left, uint32 speed_right, uint32 INIT_SPEED, uint32 SPEED_UPPER_LIMIT);