#ifndef SERVO_H
#define SERVO_H

#include "zf_common_headfile.h"

// 逐飞助手的参数通道
#define SERVO_PARAM_KP_CHANNEL          ( 1 )
#define SERVO_PARAM_KD_CHANNEL          ( 2 )
#define SERVO_PARAM_TARGET_CHANNEL      ( 3 )
#define SERVO_PARAM_CENTER_CHANNEL      ( 4 )
#define SERVO_PARAM_DIRECT_CHANNEL      ( 5 )
#define SERVO_PARAM_MODE_CHANNEL        ( 6 )

void Servo_Reset(void);                                 // 舵机重置
void Servo_SetPosition(float position);                 // 设置舵机位置
void Servo_SetTargetPosition(float target);             // 设置舵机目标位置
void Servo_PositionPdUpdate(void);                      // 舵机位置PD更新
void Servo_AssistantUpdate(volatile float *lastParamChannel, volatile float *lastParamValue);       // 逐飞助手参数更新
float Servo_GetTargetPosition(void);                    // 获取舵机目标位置
float Servo_GetTargetCommand(void);                     // 获取逐飞助手发送的舵机目标位置命令
float Servo_GetFeedbackPosition(void);                  // 获取舵机反馈位置
float Servo_GetOutputDuty(void);                        // 获取舵机输出占空比
float Servo_GetPwmDuty(void);                           // 获取舵机PWM占空比

float Servo_GetTuneMode(void);

#endif
