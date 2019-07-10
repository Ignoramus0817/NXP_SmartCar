#include "init.h"
#include "tracking.h"

//转向，参数1为角度变量，参数2为原角度
void turn(int angle_change, uint32 angle_rate, uint32 ANGLE_UPPER_LIMIT, uint32 ANGLE_LOWER_LIMIT){
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
int turn_error(uint8 img[][CAMERA_W],
               int *exit_flag1, int *exit_flag2, int *exit_flag, int *enable_flag,
               int *wall_flag, int *island_flag, int *turn_flag,
               int *out_flag, int *reEnter_flag,
               int *enter_counter, int *exit_counter,
               int *Y_REF_STD, int *Y_CHANGED){
  //图像中点设为第30行第39列，x_comp用于储存计算的赛道中线
  int x_base = 39, x_comp = 39;
  int y_ref = *Y_REF_STD;
  
  //判断是否有墙
  for(int i = 0;i < CAMERA_W; i++){
    if(img[0][i] == 0x00)
      wall_flag = 0;
  }
  
  //十字路口检测及调整ref值
  //检测第30行之下是否为全白
  int black_pixels = 0;
  for(int i = 30; i <= 59; i++){
    for(int j = 0; j < CAMERA_W; j++){
      if(img[i][j] == 0x00)
        black_pixels += 1;
    }
  }
  if(black_pixels == 0){
    //若30行以下为全白，则从左右30行8/32列开始向上搜索黑色像素
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
    //比较两侧黑色像素第一次出现的行数，较小值赋与upper，较大值赋与lower
    int upper_cross = 0, lower_cross = 0;
    if(black_left >= black_right){
      lower_cross = black_left;
      upper_cross = black_right;
    }
    else{
      lower_cross = black_right;
      upper_cross = black_left;
    }
    //假如上方黑色行数不大于7，则用下方黑色行数作为行数基准线，否则用上方行数
    if(upper_cross <= 7)
      y_ref = lower_cross;
    else
      y_ref = upper_cross;
    //保存修改过的y_ref值
    *Y_CHANGED = y_ref;
  }
  else{
    //若30行以下非全白，则恢复y_ref值
    y_ref = *Y_REF_STD;
    *Y_CHANGED = 0;
  }
  
  // y_ref调整，若y_ref行横向出现黑白跳变，调整y_ref至50行
  int count = 0;
  for(int m = 0; m < CAMERA_W - 1; m ++){
    if( (img[y_ref][m] == 0x00 && img[y_ref][m + 1] == 0xFF) ||
       (img[y_ref][m] == 0xFF && img[y_ref][m + 1] == 0x00) )
      count += 1;
  }
  if(count >= 3)
    y_ref = 50;
  
  
  //入环岛 
  //若enable_flag为0，则直接跳转至出环岛处
  if(enable_flag == 0)
    goto position1;
  
  //right/left_start为左右两侧黑白跳变的行坐标
  int right_start = 0, left_start = 0;
  //从右侧中线开始向上寻找白—>黑跳变，记录跳变处黑色的坐标，若出现两次则记录上方的值，存在right_start中
  int temp1 = 0, temp2 = 0;
  if(img[30][CAMERA_W - 1] == 0x00){
    goto position1;
  }
  else{
    for(int i = 0; i <= 30; i++){
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
  }
  
  //从左侧中线开始搜白->黑跳变（若30行为白则向上，若30行为黑则向下）
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
  
  //从right_start开始，记录黑色边界的行坐标，从79列搜索至25列，最终left_end存放行坐标
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
  
  //排除干扰*
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
  
  //V字条件，左右黑色边界行坐标之差大于5，且右边界行坐标在10到25之间
  if(right_start - left_end <= 5)
    island_flag = 0;
  else if(right_start >= 10 && right_start <= 25){
    *island_flag = 1;
    *exit_flag = 0;
  }
  //避免环岛处提前转弯
  if(img[right_start + 2][60] == 0x00)
    island_flag = 0;
  
  //区别十字和环岛
  if( abs(left_start - right_start) <= 3)
    island_flag = 0;
  
  if( !((img[right_start + 1][0] == 0x00 && img[right_start + 1][CAMERA_W-1] == 0xFF) && 
        (img[right_start + 2][0] == 0x00 && img[right_start + 2][CAMERA_W-1] == 0xFF)) )
    island_flag = 0;
  
  if(right_start - left_end_disturb >= 2)
    island_flag = 0;
  
  //转弯干扰dirty(改进：黑白跳变)
  int disturb = 0;
  for(int i = 30; i >= 0; i --){
    if(img[i][39] == 0x00){
      disturb = i;
      break;
    }
  }
  if(disturb >= 12)
    island_flag = 0;
  
  //转入环岛判断
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
  int temp_var = 0;
  //    if(right_start < 18){
  if( max_length <= 20 && max_length > 5 )
    temp_var = 0;
  //    }
  else
    temp_var = 1;
  *turn_flag = temp_var;
  //入环岛计数器赋初值
  if(*island_flag == 1 && *turn_flag ==  1)
    *enter_counter = 15;
  
  //出环岛
position1:
  //计算图像两侧黑白交界处的行坐标，存放在left/right_bound中，并比较它们的高低，存放在upper/lower_bound中
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
  
  //计算上方黑色和下方白色的行数
  for(int i = 0; i < upper_bound; i++){
    int count = 0;
    for(int j = 0; j < CAMERA_W; j++){
      if(img[upper_bound - i][j] == 0x00)
        count ++;
    }
    if(count >= 75)
      exit_flag1 += 1;
  }
  if(lower_bound >= 22 && lower_bound < 30){ 
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
  /*
   * 出环岛需要满足的条件（按照判断条件顺序）：
   * upper_bound一行全黑
   * 已经进入环岛
   * 左右黑白分界线行数之差不超过8
   * 上方黑色行数超过5
   * 下方全白
   */
  if(enter_white == 0 && *out_flag == 1 && abs(upper_bound - lower_bound) <= 8 && *exit_flag1 >= 5 && *exit_flag2 == 59 - lower_bound){
    *island_flag = 0;
    *turn_flag = 0;
    *exit_flag = 1; 
    *exit_counter = 20;
    *enable_flag = 0;
    *reEnter_flag = 1;
    for(int i = 30; i >= 0; i--){
      if(img[i][CAMERA_W - 1] == 0x00){
        *Y_REF_STD = i - 1;
        break;
      }
    }
  }
  
  //已经出环岛，激活环岛判断，并清空所有有关环岛的标志位
  if(*reEnter_flag == 1 && img[30][0] == 0x00 && img[30][CAMERA_W-1] == 0x00){
    *enable_flag = 1;
    *island_flag = 0;
    *turn_flag = 0;
    *exit_flag = 0;
    *out_flag = 0;
    *Y_REF_STD = 30;
    gpio_set(PTA19, 1);
  }
  
  //出环岛时将基准线上移至10
  if(*enter_counter == 1){
    *Y_REF_STD = 10;
    gpio_set(PTA19, 1);
  }
  
  //转向算法 
  //入环岛分支
  if(*enter_counter >= 1 && (*island_flag == 1 && *turn_flag == 1) ){
    x_comp = 39 + 140 + 89 / 2;
    *out_flag = 1;
    *reEnter_flag = 0;
    *enter_counter -= 1;
  }
  //出环岛分支
  else if(*exit_counter >= 2 || *exit_flag == 1){
    x_comp = 600;
    *out_flag = 0;
    gpio_set(PTA19, 0);
    *exit_counter -= 1;
  }
  //正常转向分支
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

int tracking_PID(int camera, int pre_camera, int inductor, int pre_inductor){
  float P = 0, D = 0;
  int use, pre_use;
  int output = 0;
  if(is_cutoff()){
    P = 0;
    D = 0;
    use = inductor;
    pre_use = pre_inductor;
  }
  else{
    P = 0;
    D = 0;
    use = camera;
    pre_use = pre_camera;
  }
  output = use * P + (use - pre_use) * D;
  return(output);
}

int is_cutoff(uint8 img[][CAMERA_W]){
  if(1)
    return(true);
  else
    return(false);
}