#ifndef SERVO_H
#define SERVO_H

#include "zf_common_headfile.h"

// 逐飞助手的参数通道
#define SERVO_PARAM_KP_CHANNEL          ( 1 )
#define SERVO_PARAM_KD_CHANNEL          ( 2 )
#define SERVO_PARAM_TARGET_CHANNEL      ( 3 )
#define SERVO_PARAM_CENTER_CHANNEL      ( 4 )

void Servo_Reset(void);                                 // 舵机重置
void Servo_SetPosition(float position);                 // 设置舵机位置
void Servo_SetTargetPosition(float target);             // 设置舵机目标位置
void Servo_PositionPdUpdate(void);                      // 舵机位置PD更新
void Servo_AssistantUpdate(volatile float *lastParamChannel, volatile float *lastParamValue);       // 逐飞助手参数更新

#endif
