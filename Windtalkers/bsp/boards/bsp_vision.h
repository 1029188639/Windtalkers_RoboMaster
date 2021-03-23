#ifndef BSP_VISION_H
#define BSP_VISION_H
#include "struct_typedef.h"

extern void vision_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void vision_unable(void);
extern void vision_restart(uint16_t dma_buf_num);
#endif
