# Zimbabwe-Ostrich
Group ZWO's code in 2019 NXP Smart Car Race



1. 要通过蓝牙发送数据，将MK60_conf.h文件第85行中VCAN_PORT值设为UART0，通过串口发送数据则为UART5。

2. 要发送数值，直接printf即可（建议写在397行后），要发送图像到上位机，将IMG_GET.C文件第20行取消注释，实际上赛道时应注释掉。
3. 电机差速位于main.c第559至第611行。
4. 舵机转向PID位于main.c第482至第517行。