#ifndef __IMU_H__
#define __IMU_H__
#include "zf_common_headfile.h"
/* Reference project pitch mounting correction, in millidegrees (2500 = 2.5 deg). */
#define IMU_INSTALL_OFFSET_MDEG 0
/* Startup calibration: keep the car still; raw gyro counts, not degrees. */
#define IMU_CAL_SAMPLES 200U
#define IMU_CAL_ATTEMPTS 3U
#define IMU_CAL_MAX_ABS_RAW 100L
#define IMU_CAL_MAX_SPAN_RAW 60L
#define IMU_STATUS_MISSING 0U
#define IMU_STATUS_CALIBRATING 1U
#define IMU_STATUS_READY 2U
#define IMU_STATUS_CAL_FAILED 3U
extern volatile uint8 imu_cal_status;
extern volatile int16 imu_gyro_z_bias;
/* Reference Motor.c uses +2 * bias-corrected gyro Z / 100 in the steering PID.
 * Raw left-negative/right-positive matches this car's positive-left PWM. */
#define IMU_STEERING_GYRO_COEFF 1
extern volatile uint8 imu_ready;
extern volatile int16 imu_steering_gyro_z;
extern volatile int16 pitch;
uint8 IMU_Init(void);
#endif
