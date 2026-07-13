#ifndef ACCELERATOR_H
#define ACCELERATOR_H

#include "zf_common_typedef.h"

// IMU 安装方向配置：0=X轴，1=Y轴，2=Z轴。
// 小车水平放置时，选择垂直于车身平面的轴作为偏航轴。
#define ACCELERATOR_YAW_AXIS             ( 2 )

// 调试时向右转动车身：若读取的偏航角速度为负值，将此符号改为 -1.0f。
#define ACCELERATOR_YAW_SIGN             ( 1.0f )

#define ACCELERATOR_STABILIZATION_MS      ( 500U )
#define ACCELERATOR_DISCARD_SAMPLES       ( 100U )
#define ACCELERATOR_CALIBRATION_SAMPLES   ( 400U )
#define ACCELERATOR_CALIBRATION_DELAY_MS ( 5U )
#define ACCELERATOR_CALIBRATION_MAX_RANGE_DPS ( 5.0f )
#define ACCELERATOR_CALIBRATION_MAX_BIAS_DPS  ( 20.0f )
#define ACCELERATOR_YAW_FILTER_ALPHA     ( 0.20f )
#define ACCELERATOR_YAW_DEADBAND_DPS     ( 0.50f )

uint8 Accelerator_Init(void);
void Accelerator_Update(void);
uint8 Accelerator_IsReady(void);
float Accelerator_GetYawRate(void);

#endif
