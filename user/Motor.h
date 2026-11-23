/* 电机、舵机与路径控制接口。图像 Point 为行/列，SearchPoint 为毫米坐标，调用时不可混用。 */
#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "zf_common_headfile.h"
#include "search_geometry.h"

#define motorPwm_L PWMB_CH1_P74 /* 左轮PWM通道 */
#define motorDir_L IO_P75 /* 左轮方向引脚 */
#define motorPwm_R PWMB_CH3_P76 /* 右轮PWM通道 */
#define motorDir_R IO_P77 /* 右轮方向引脚 */
#define servoPwm   PWME_CH1P_PA0 /* 舵机PWM通道 */
#define servoMid   3260 /* 舵机中位PWM值 */
#define motorL     0 /* 左轮PID标识 */
#define motorR     1 /* 右轮PID标识 */
#define MOTOR_SERVO_KP_MAX 1000 /* 舵机比例系数上限 */

/* 平均目标/反馈速度，单位为编码器计数/5ms。 */
extern volatile int16 setSpeed, currentSpeed;
/* 当前图像路径跟踪：count是点数，aimPos是目标索引，Point.x为行、y为列。 */
void alongLine(Point roadLine[], int16 count, int16 aimPos);
/* Nearest available point to the configured rear-axle lookahead radius.
 * Returns -1 if calibration/forward points are unavailable. */
/* 按后轴毫米瞄点距离选择图像路径索引，失败返回-1。 */
int16 Motor_SelectTarget(Point roadLine[], int16 count);
/* 初始化控制器及硬件，设置舵机增益。 */
void Motor_Init(int16 servo_kp);
/* 同时设置左右轮速度PID参数。 */
void Motor_SetSpeedPID(int16 kp, int16 ki, int16 kd);
// 100 means 1.00x steering gain; output remains limited by the servo PID.
/* 设置舵机比例增益并清空上一误差。 */
void Motor_SetServoKp(int16 kp);
typedef struct {
    int16 pixel_error; // 图像中心列减目标列的像素误差，左偏为正。
    int16 angle10;     // 纯追踪转角，单位0.1度，尚未乘舵机增益。
    int16 pwm;         // 定时器最近写入的舵机PWM，不是实测角度。
    int16 gyro_rate10; // 有效滤波角速度，0.1度/秒，右转正；失效为0。
    int16 gyro_pwm;    // 当前陀螺修正PWM，正值向左；与视觉舵量相加。
} motor_steering_state_t;
/* 原子读取当前转向诊断数据到state。 */
void Motor_GetSteering(motor_steering_state_t *state);
/* 设置菜单直线/手动测试速度0～1000；弯道和圆环速度独立配置。 */
void Motor_SetStraightSpeed(int16 speed);
/* 请求驱动或停车；Run下启用还需满足有效路径及看门狗条件。 */
void Motor_SetEnabled(uint8 enabled);
/* 处理失线帧，已有运行路径时保持转向并按配置降速。 */
void Motor_RejectPath(void);
// Run mode requires processed camera frames; the 5 ms ISR stops stale commands.
/* 切换Run模式并先停车；Run模式启用摄像头看门狗。 */
void Motor_SetRunMode(uint8 enabled);
/* 返回当前驱动使能状态。 */
uint8 Motor_IsEnabled(void);
/* 原子读取5ms计数器，计数会自然回绕。 */
uint16 Motor_GetTicks(void);

#endif //__MOTOR_H__
