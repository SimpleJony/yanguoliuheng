#ifndef CONTROL_H
#define CONTROL_H

#include "init.h"

void control_reset();                                     // 清除pid的状态
void control_Init();                                      // 控制模块初始化
void control_pit_handler();                               // 控制周期中断服务程序
void control_task();                                      // 主循环任务
void control_update();                                    // 从逐飞助手中更新数据
void control_sendOscData();                               // 发送示波器数据
float control_limit(float value, float low, float high);  // 浮点数限幅函数
void Motor1_SetSpeed(float dutyPercent);                  // 设置电机1的速度
void Motor2_SetSpeed(float dutyPercent);                  // 设置电机2的速度

#endif