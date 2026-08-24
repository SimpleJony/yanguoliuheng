#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "zf_common_headfile.h"

#define motorPwm_L PWMD_CH4_P53
#define motorDir_L IO_P52
#define motorPwm_R PWMD_CH2_P51
#define motorDir_R IO_P50
#define servoPwm   PWME_CH1P_PA0
#define servoMid   1145
#define motorL     0
#define motorR     1

typedef struct {
    int16 Kp, Ki, Kd;
    uint8 which;
    int16 attitude_coeff; // 抑制转向的系数
    int16 last_error;
    int16 llast_error;
    int16 base;
    int16 max_out;
} PID;

volatile extern int16 setSpeed, handle, encoderL, encoderR;
extern int16 enableDifSpeed, currentSpeed;
void PID_Init(PID *pid, uint8 which, int16 Kp, int16 Ki, int16 Kd, int16 attitude_coeff,int16 max_out);
int16 steerPIDCalculate(PID *pid, int16 error);
int16 motorPIDCalculate(PID *pid, int16 target, int16 actual);
void alongLine(Point roadLine[], int16 count, int16 aimPos);
void setServoDuty(int8 d);
void Motor_Init(int16 _servoP, int16 _aimPos, int16 _curveCoeff, int16 _speedLimit);
int8 getAimPos();

#endif //__MOTOR_H__