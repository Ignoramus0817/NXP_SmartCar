#include "inductor.h"

float get_ind(uint16 *LEM, uint16 *REM){
  float error = 0;
  float output = 0;
  *LEM = adc_once(ADC1_SE9, ADC_12bit);
  *REM = adc_once(ADC1_SE8, ADC_12bit);
  error = ((float)(*LEM - *REM)) / ((float)(*LEM + *REM));
  printf("%f ", error);
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
  printf("%d\n", output);
  return(output); 
}