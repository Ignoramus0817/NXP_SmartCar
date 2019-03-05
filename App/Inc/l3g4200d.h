/**
 * @file DigitalSensor.h
 * @version 0.1[By Ranger]
 * @date 2013-09-24
 * @L3G4200D三轴陀螺仪传感器设备驱动程序
 */
#ifndef __DIGITALSENSOR__
#define __DIGITALSENSOR__
//==========L3G4200D 寄存器地址==================//
#define L3G4200D_WHO_AM_I      0x0F  
#define L3G4200D_CTRL_REG1     0x20  
#define L3G4200D_CTRL_REG2     0x21  
#define L3G4200D_CTRL_REG3     0x22  
#define L3G4200D_CTRL_REG4     0x23  
#define L3G4200D_CTRL_REG5     0x24  
#define L3G4200D_REFERENCE     0x25  
#define L3G4200D_OUT_TEMP      0x26  
#define L3G4200D_STATUS_REG    0x27  
#define L3G4200D_OUT_X_L       0x28  
#define L3G4200D_OUT_X_H       0x29  
#define L3G4200D_OUT_Y_L       0x2A  
#define L3G4200D_OUT_Y_H       0x2B  
#define L3G4200D_OUT_Z_L       0x2C  
#define L3G4200D_OUT_Z_H       0x2D  
#define L3G4200D_FIFO_CTRL_REG 0x2E  
#define L3G4200D_FIFO_SRC_REG  0x2F

#define L3G4200D_INT1_CFG      0x30  
#define L3G4200D_INT1_SRC      0x31  
#define L3G4200D_INT1_THS_XH   0x32  
#define L3G4200D_INT1_THS_XL   0x33  
#define L3G4200D_INT1_THS_YH   0x34  
#define L3G4200D_INT1_THS_YL   0x35  
#define L3G4200D_INT1_THS_ZH   0x36  
#define L3G4200D_INT1_THS_ZL   0x37  
#define L3G4200D_INT1_DURATION 0x38  
#define L3G4200D_ADDR 	       0x69


extern int16 xResult,yResult,zResult;
extern uint8 temp[6];
extern float gx,gy,gz;
void L3G4200D_Init(void);
void L3G4200D_Read(void);



#endif 