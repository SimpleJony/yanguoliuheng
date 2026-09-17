#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include "zf_common_headfile.h"

// Left/right motor speed PID values, using Motor.c's integer convention.
typedef struct
{
    int16 kp;
    int16 ki;
    int16 kd;
    int16 motor_speed;   // Speed limit in encoder counts per 5 ms, 0..1000
    uint8 motor_enabled; // Runtime only: 0 off, 1 on
} system_pid_config_t;

extern system_pid_config_t system_pid_config;
extern uint8 system_run_requested;

// Initializes IPS200, MT9V03X and the four menu keys.
// Return value is the return value of mt9v03x_init(): 0 means success.
uint8 System_Init(void);

// Call continuously from main. One call takes about 10 ms, excluding LCD refresh.
void System_Task(void);

// Placeholder entrance for the future car-running program.
void System_Run_Entry(void);

// Clear the request if the application has consumed it.
void System_Clear_Run_Request(void);

#endif
