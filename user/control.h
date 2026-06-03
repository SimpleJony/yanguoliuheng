#ifndef CONTROL_H
#define CONTROL_H

#include "init.h"

void control_reset(void);
void control_Init(void);
void control_pit_handler(void);
void control_task(void);
void control_line_follow_update(void);
void control_set_target_speed(float motor1Target, float motor2Target);
void control_update(void);
void control_sendOscData(void);
float control_limit(float value, float low, float high);
void Motor1_SetSpeed(float dutyPercent);
void Motor2_SetSpeed(float dutyPercent);

#endif
