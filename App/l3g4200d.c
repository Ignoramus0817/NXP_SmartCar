



#include "include.h"
#include "l3g4200d.h"

/*关于L3G4200D的接线 
SCL为时钟线，接单片机SCK
SDO为数据输出，接单片机SIN
SDA为数据输入，接单片机SOUT
CS为片选信号，接单片机CS
单片机各引脚在PORT_cfg.h中查询
*/

int16 xResult,yResult,zResult;//采集到的数据
float gx,gy,gz;//处理后得到的角速度值
uint8 temp[6];//存储数据

/*

SPI读取函数，先发送8位reg
读取的数据存储在dat中
*/


uint8 spi_readreg(uint8 reg, uint8 *dat)
{

    uint8 buff[2];

    buff[0] = reg;          //先发送寄存器

    spi_mosi(SPI1, SPI_PCS0, buff, buff, 2); //发送buff数据，并保存到buff里

    *dat = buff[1];                         //提取第二个数据

    /*返回状态寄存器的值*/
    return buff[0];
}

/*
SPI写函数，将dat数据发送到从机上，对4200D而言，读取时RW位是0
因此reg就是要写的寄存器地址（详细参考数据手册P24-25）
*/
  
uint8 spi_writereg(uint8 reg, uint8 dat)
{
    uint8 buff[2];

    buff[0] = reg;          //先发送寄存器
    buff[1] = dat;          //再发送数据

    spi_mosi(SPI1, SPI_PCS0, buff, buff, 2); //发送buff里数据，并采集到 buff里

    /*返回状态寄存器的值*/
    return buff[0];
}

/*
L3G4200D初始化函数
*/
void L3G4200D_Init(){
  //初始化spi0模块用cs0片选号，k60做主机，波特率1250*1000
  spi_init(SPI1,SPI_PCS0, MASTER,1250*1000);
  //初始化l3g4200d芯片，包括调它的精度之类的
  lptmr_delay_ms(1500);
  spi_writereg(L3G4200D_CTRL_REG1,0x0F);      //800Hz,110
  lptmr_delay_us(100);      
  spi_writereg(L3G4200D_CTRL_REG2,0x03);      //高通滤波器，1Hz
  lptmr_delay_us(100); 
  spi_writereg(L3G4200D_CTRL_REG3,0xA8);      //输出信号引脚配置
  lptmr_delay_us(100);      
  spi_writereg(L3G4200D_CTRL_REG4,0x10);      //精度+—500dps  事实更新
  lptmr_delay_us(100); 
  spi_writereg(L3G4200D_CTRL_REG5,0x0F);      //滤波器
  lptmr_delay_us(100);      

}



void L3G4200D_Read(void){
  //和L3g4200D通讯，将数据储存到temp数组中
  
  spi_readreg(0xA8,temp);
  spi_readreg(0xA9,temp+1);
  spi_readreg(0xAA,temp+2);
  spi_readreg(0xAB,temp+3);
  spi_readreg(0xAC,temp+4);
  spi_readreg(0xAD,temp+5);
  xResult=(int16)((temp[1]<<8)|temp[0]);
  gx=xResult*17.5/1000;
  //从temp数组中得到x,y,z轴的数据
  //下面还有对数据的处理得到角速度的过程，请自行完成
  yResult=(int16)((temp[3]<<8)|temp[2]);
  gy=yResult*17.5/1000;
  zResult=(int16)((temp[5]<<8)|temp[4]);
  gz=zResult*17.5/1000;
}

