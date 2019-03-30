#include "IMG_GET.h"

void PORTA_IRQHandler();
void DMA0_IRQHandler();

void  img_get(uint8 *imgbuff, uint8 img[][CAMERA_W])
{
    //配置中断服务函数
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);    //设置PORTA的中断服务函数为 PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);      //设置DMA0的中断服务函数为 DMA0_IRQHandler


        //获取图像
        camera_get_img();                                   //摄像头获取图像

        /***********  提供两种方式可供用户自行选择【二值化模式】、【灰度模式】  ************/
        /***  强调一点：所谓的灰度模式，实际上只有2个像素值，即已经二值化好的灰度模式。  ***/

        //发送图像到上位机
//        vcan_sendimg(imgbuff, CAMERA_SIZE);                  //发送到上位机
        img_extract(img, imgbuff,CAMERA_SIZE);                  //解压图像
//        vcan_sendimg(img, CAMERA_W * CAMERA_H);                  //发送到上位机
}

/*!
 *  @brief      PORTA中断服务函数
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
    uint8  n = 0;    //引脚号
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if 0             //鹰眼直接全速采集，不需要行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif
}

/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}