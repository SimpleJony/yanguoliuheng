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
    int16 servo_kp;      // Steering gain: 100 = 1.00x, 0..1000
} system_pid_config_t;

extern system_pid_config_t system_pid_config;
extern uint8 system_run_requested;

// Initializes IPS200, MT9V03X and the four menu keys.
// Return value is the return value of mt9v03x_init(): 0 means success.
uint8 System_Init(void);

// Call continuously from main. Includes 10 ms delay, frame processing and LCD.
void System_Task(void);

// Enter camera/search/pure-pursuit mode; valid frames enable the motor loop.
void System_Run_Entry(void);

// Stop both motors, leave tracking mode and return to the main menu.
void System_Clear_Run_Request(void);

#endif
