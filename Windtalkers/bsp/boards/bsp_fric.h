#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"

#define FRIC_UP 1500
#define FRIC_DOWN 1400	//最大值
#define FRIC_OFF 1000   //最小值

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
