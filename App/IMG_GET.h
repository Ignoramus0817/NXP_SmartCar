#include "common.h"
#include "include.h"
#include "math.h"

void PORTA_IRQHandler();
void DMA0_IRQHandler();
void  img_get(uint8 *imgbuff, uint8 img[][CAMERA_W]);