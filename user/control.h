#ifndef CONTROL_H
#define CONTROL_H

#include "init.h"

void control_reset();                                     // 重置pid状态
void control_Init();                                      // 初始化
void control_pit_handler();                               // pid计算和处理
void control_task();                                      // 控制任务
void control_update();                                    // 从逐飞更新参数
void control_sendOscData();                               // 发送示波器数据
float control_limit(float value, float low, float high);  // 限幅函数
void Motor1_SetSpeed(float dutyPercent);                  // 设置电机1速度
void Motor2_SetSpeed(float dutyPercent);                  // 设置电机2速度

#endif
