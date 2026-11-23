#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include "zf_common_headfile.h"

// Initializes IPS200, MT9V03X and the four menu keys.
// Return value is the return value of mt9v03x_init(): 0 means success.
uint8 System_Init(void);

// 主循环持续调用：所有页面均无每轮固定等待；按键按真实时间消抖，图像处理与屏幕刷新分别调度。
void System_Task(void);

#endif
