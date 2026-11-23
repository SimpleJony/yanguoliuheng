/* IMU 公共状态、转向角速度样本和俯仰角；MDEG 表示毫度。 */
#ifndef __IMU_H__
#define __IMU_H__
#include "zf_common_headfile.h"
/* Reference project pitch mounting correction, in millidegrees (2500 = 2.5 deg). */
#define IMU_INSTALL_OFFSET_MDEG 1500 /* 俯仰安装角修正，毫度 */
/* Startup calibration: keep the car still; raw gyro counts, not degrees. */
#define IMU_CAL_SAMPLES 200U /* 每轮静止零偏校准的样本数 */
#define IMU_CAL_ATTEMPTS 3U /* 零偏校准最多尝试轮数 */
#define IMU_CAL_MAX_ABS_RAW 100L /* 校准时三轴原始计数绝对值上限 */
#define IMU_CAL_MAX_SPAN_RAW 60L /* 校准时任一轴原始计数极差上限 */
#define IMU_STATUS_MISSING 0U /* 设备尚未就绪/初始化失败 */
#define IMU_STATUS_CALIBRATING 1U /* 静止校准中 */
#define IMU_STATUS_READY 2U /* 校准完成可使用 */
#define IMU_STATUS_CAL_FAILED 3U /* 零偏校准失败 */
#define IMU_CAL_FAIL_ABS  0x01U /* 三轴任一原始计数绝对值超限 */
#define IMU_CAL_FAIL_SPAN 0x02U /* 三轴任一原始计数在采样期间波动过大 */
/* 校准状态，取值为 IMU_STATUS_*，用于区分未连接、校准中、就绪和校准失败。 */
extern volatile uint8 imu_cal_status;
/* 最后一轮校准的失败原因位，可同时包含 ABS 和 SPAN；重新初始化或校准成功时清零。 */
extern volatile uint8 imu_cal_failure;
/* 当前安装：Z轴右转为正、左转为负，舵机PWM正偏移向左。
 * 正的陀螺反馈因此抵消右摆。系数单位为 PWM/(10度/秒)，独立于 Servo Kp。
 * 6 对应 0.6 PWM/(度/秒)；0 可关闭抑摆。 */
#ifndef IMU_STEERING_GYRO_COEFF
#define IMU_STEERING_GYRO_COEFF 5
#endif
#define IMU_STEERING_DEADBAND10 15          /* ±1.5度/秒软死区：超出后减去死区，避免台阶 */
#define IMU_STEERING_MAX_PWM 48            /* 陀螺修正上限，约占总舵量1/8 */
#define IMU_STEERING_SLEW_PWM 4            /* 每5ms修正最多变化4 PWM */
#define IMU_STEERING_STALE_TICKS 6U        /* 30ms无新IMU样本则撤掉修正 */
#define IMU_STEERING_CURVE_START10 50      /* 视觉转角5度以内使用完整阻尼 */
#define IMU_STEERING_CURVE_FULL10 200      /* 视觉转角20度起使用最小同向阻尼 */
#define IMU_STEERING_CURVE_PERCENT 35      /* 正常同向转弯时保留35%；反向摆动不削弱 */
#if IMU_STEERING_GYRO_COEFF < 0 || IMU_STEERING_GYRO_COEFF > 30
#error IMU_STEERING_GYRO_COEFF must be 0..30 PWM per 10 degrees/second.
#endif
#if IMU_STEERING_MAX_PWM < 1 || IMU_STEERING_SLEW_PWM < 1 || IMU_STEERING_STALE_TICKS < 1 || IMU_STEERING_STALE_TICKS > 255
#error Invalid IMU steering correction limits.
#endif
#if IMU_STEERING_CURVE_START10 < 0 || IMU_STEERING_CURVE_FULL10 <= IMU_STEERING_CURVE_START10 || IMU_STEERING_CURVE_PERCENT < 0 || IMU_STEERING_CURVE_PERCENT > 100
#error Invalid IMU steering curve attenuation.
#endif
/* IMU 是否已连接且完成零偏校准；为 0 时定时中断不读取传感器。 */
extern volatile uint8 imu_ready;
typedef struct {
    int16 rate10;   /* 快速滤波后的Z轴角速度，0.1度/秒，右转为正 */
    uint8 sequence; /* 每次发布新样本递增，允许回绕；读数相同也算新样本 */
    uint8 valid;    /* 校准就绪且至少完成过一次采样 */
} imu_steering_sample_t;
/* 原子读取整份样本；不会触发SPI读取，由电机5ms中断调用。 */
void IMU_GetSteeringSample(imu_steering_sample_t *sample);
/* 对外提供的整数俯仰角，单位为度。 */
extern volatile int16 pitch;

/* 停车后初始化并静止校准；允许在 Image 页手动重试，返回0成功、1设备初始化失败、2零偏校准失败。 */
uint8 IMU_Init(void);
#endif
