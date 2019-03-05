#ifndef _MMA7361_H_
#define _MMA7361_H_
#include "include.h"
//AX 1095 3170
//AY 720 2790
//AZ 3290 1210
extern int32 GX,GY,GZ,T,AX,AY,AZ;
extern float GX_F,GY_F,GZ_F,T_F,AX_F,AY_F,AZ_F,ax,ay,az;

extern unsigned char BUF[20];       //接收数据缓存区

int Init_MMA7361(void);
void READ_MMA7361(void);


#endif

