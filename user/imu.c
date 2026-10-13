/* Keep the sensor/control path at O3, as with the motor interrupt. */
#if defined(__C251__)
#pragma OPTIMIZE(3,SPEED)
#endif
#include "imu.h"
#include "Motor.h"

volatile uint8 imu_ready;
volatile uint8 imu_cal_status;
volatile int16 imu_gyro_x_bias, imu_gyro_y_bias;
volatile int16 imu_gyro_z_bias;
volatile uint8 imu_steering_slew_percent = 100;
volatile int16 imu_steering_gyro_z;
volatile int16 pitch;
static int32 pitch_mdeg;
static int32 xy_motion_filtered;

static void imu_update_steering_motion(int32 gyro_x, int32 gyro_y)
{
    int32 motion;
    gyro_x = func_abs(gyro_x);
    gyro_y = func_abs(gyro_y);
    motion = gyro_x > gyro_y ? gyro_x : gyro_y;
    /* Rectify before filtering so alternating shake does not cancel out. */
    xy_motion_filtered += (motion - xy_motion_filtered) / 4;
    motion = func_limit_ab(xy_motion_filtered - IMU_XY_MOTION_DEADBAND,
        0L, IMU_XY_MOTION_FULL - IMU_XY_MOTION_DEADBAND);
    /* One-byte publication: the motor ISR cannot read a torn percentage. */
    imu_steering_slew_percent = (uint8)(100L - motion *
        (100 - IMU_XY_SLEW_MIN_PERCENT) /
        (IMU_XY_MOTION_FULL - IMU_XY_MOTION_DEADBAND));
}

/* Reference Imu.c: TIM0, 5 ms, 0.997 complementary filter.
 * All products are widened before multiplication for C251's 16-bit int.
 * Only this ISR reads the sensor once initialized; no yaw/ring integration. */
static void IMU_pitHandler(void)
{
    int32 gyro_x, acc_pitch, predicted;
    if (!imu_ready) return;
    imu660rb_get_acc();
    imu660rb_get_gyro();
    /* Subtract in 32 bits before clamping: signed 16-bit extremes can overflow. */
    imu_steering_gyro_z = (int16)func_limit_ab(
        (int32)imu660rb_gyro_z - imu_gyro_z_bias, -32768L, 32767L);
    imu_update_steering_motion((int32)imu660rb_gyro_x - imu_gyro_x_bias,
        (int32)imu660rb_gyro_y - imu_gyro_y_bias);
    gyro_x = imu660rb_gyro_x;
    if (func_abs(gyro_x) < 100L) gyro_x = 0;
    acc_pitch = atan2_int(-(int32)imu660rb_acc_y,
        func_abs((int32)imu660rb_acc_z)) * 1000L - IMU_INSTALL_OFFSET_MDEG;
    predicted = pitch_mdeg + gyro_x * 6993L / 20000L;
    /* Equivalent complementary weights, avoiding a large 997 * angle product. */
    pitch_mdeg = predicted + 3L * (acc_pitch - predicted) / 1000L;
    pitch = (int16)(pitch_mdeg / 1000L);
}

/* Foreground startup only; imu_ready=0 prevents the timer from accessing SPI.
 * Check all gyro axes so pitching/rolling during calibration also rejects it.
 * This rejects obvious motion, but cannot distinguish very slow steady rotation
 * from sensor bias: the car must physically remain stationary. */
static uint8 imu_calibrate_gyro(void)
{
    uint16 sample;
    uint8 axis, attempt, stable;
    int16 raw[3], low[3], high[3];
    int32 sum[3];
    for(attempt = 0; attempt < IMU_CAL_ATTEMPTS; ++attempt) {
        stable = 1;
        for(axis = 0; axis < 3; ++axis) {
            sum[axis] = 0;
            low[axis] = 32767;
            high[axis] = -32767 - 1;
        }
        for(sample = 0; sample < IMU_CAL_SAMPLES; ++sample) {
            system_delay_ms(5);
            imu660rb_get_gyro();
            raw[0] = imu660rb_gyro_x;
            raw[1] = imu660rb_gyro_y;
            raw[2] = imu660rb_gyro_z;
            for(axis = 0; axis < 3; ++axis) {
                if(func_abs((int32)raw[axis]) > IMU_CAL_MAX_ABS_RAW) stable = 0;
                if(raw[axis] < low[axis]) low[axis] = raw[axis];
                if(raw[axis] > high[axis]) high[axis] = raw[axis];
                sum[axis] += raw[axis];
            }
        }
        for(axis = 0; axis < 3; ++axis)
            if((int32)high[axis] - low[axis] > IMU_CAL_MAX_SPAN_RAW) stable = 0;
        if(stable) {
            imu_gyro_x_bias = (int16)(sum[0] / (int32)IMU_CAL_SAMPLES);
            imu_gyro_y_bias = (int16)(sum[1] / (int32)IMU_CAL_SAMPLES);
            imu_gyro_z_bias = (int16)(sum[2] / (int32)IMU_CAL_SAMPLES);
            return 0;
        }
    }
    return 1;
}

uint8 IMU_Init(void)
{
    bit irq_state;
    Motor_SetEnabled(0);
    imu_ready = 0;
    imu_cal_status = IMU_STATUS_MISSING;
    imu_steering_gyro_z = 0;
    imu_steering_slew_percent = 100;
    xy_motion_filtered = 0;
    imu_gyro_x_bias = imu_gyro_y_bias = 0;
    imu_gyro_z_bias = 0;
    pitch = 0;
    pitch_mdeg = 0;
    /* Keep menus accessible if the sensor is absent instead of hanging boot. */
    if (imu660rb_init()) return 1;
    imu_cal_status = IMU_STATUS_CALIBRATING;
    system_delay_ms(500);
    if(imu_calibrate_gyro()) {
        /* Bounded retries; failed calibration never feeds an untrusted bias
         * into steering. Restart with the car still to try again. */
        imu_cal_status = IMU_STATUS_CAL_FAILED;
        return 2;
    }
    irq_state = EA;
    EA = 0;
    imu_cal_status = IMU_STATUS_READY;
    imu_ready = 1;
    pit_ms_init(TIM0_PIT, 5, IMU_pitHandler);
    EA = irq_state;
    return 0;
}
