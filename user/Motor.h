#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "zf_common_headfile.h"

#define motorPwm_L PWMD_CH4_P53
#define motorDir_L IO_P52
#define motorPwm_R PWMD_CH2_P51
#define motorDir_R IO_P50
#define servoPwm   PWME_CH1P_PA0
#define servoMid   3260
#define motorL     0
#define motorR     1
#define MOTOR_SERVO_KP_MAX 1000

typedef struct {
    int16 Kp, Ki, Kd;
    uint8 which;
    int16 attitude_coeff; // 抑制转向的系数
    int16 last_error;
    int16 llast_error;
    int16 base;
    int16 max_out;
} PID;

extern volatile int16 setSpeed, handle, encoderL, encoderR, currentSpeed;
extern volatile int16 enableDifSpeed;
void PID_Init(volatile PID *pid, uint8 which, int16 Kp, int16 Ki, int16 Kd, int16 attitude_coeff,int16 max_out);
int16 steerPIDCalculate(volatile PID *pid, int16 error);
int16 motorPIDCalculate(volatile PID *pid, int16 target, int16 actual);
void alongLine(Point roadLine[], int16 count, int16 aimPos);
/* Nearest available point to the configured rear-axle lookahead radius.
 * Returns -1 if calibration/forward points are unavailable. */
int16 Motor_SelectTarget(Point roadLine[], int16 count);
void setServoDuty(int8 d);
void Motor_Init(int16 _servoP, int16 _aimPos, int16 _curveCoeff, int16 _speedLimit);
void Motor_SetSpeedPID(int16 kp, int16 ki, int16 kd);
// 100 means 1.00x steering gain; output remains limited by the servo PID.
void Motor_SetServoKp(int16 kp);
typedef struct {
    int16 pixel_error; // Image centre minus target column, positive left.
    int16 angle10;     // Pure pursuit command in tenths of a degree, before Kp.
    int16 pwm;         // Last duty value written by the timer, not angle feedback.
} motor_steering_state_t;
void Motor_GetSteering(motor_steering_state_t *state);
void Motor_SetSpeedLimit(int16 speed);
void Motor_SetEnabled(uint8 enabled);
/* Invalid image path: hold actual steering PWM; reduce speed once only when
 * RUN_FIXED_SPEED_MODE is disabled.
 * Processed lost frames refresh the watchdog only during a live run. */
void Motor_RejectPath(void);
// Run mode requires processed camera frames; the 5 ms ISR stops stale commands.
void Motor_SetRunMode(uint8 enabled);
uint8 Motor_IsEnabled(void);
uint16 Motor_GetTicks(void);
int8 getAimPos();

#endif //__MOTOR_H__
