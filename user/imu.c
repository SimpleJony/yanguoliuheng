/* IMU 转向阻尼和俯仰角：静止标定零偏后，由 TIM0 每5ms采样。圆环状态仅由视觉判断。 */
/* Keep the sensor/control path at O3, as with the motor interrupt. */
#if defined(__C251__)
#pragma OPTIMIZE(3,SPEED)
#endif
#include "imu.h"
#include "Motor.h"

/* IMU 是否已连接且完成零偏校准；为 0 时定时中断不读取传感器。 */
volatile uint8 imu_ready;
/* 校准状态，取值为 IMU_STATUS_*，用于区分未连接、校准中、就绪和校准失败。 */
volatile uint8 imu_cal_status;
/* 最后一轮校准失败的原因位；用于屏幕区分原始读数过高和采样期间波动过大。 */
volatile uint8 imu_cal_failure;
/* 静止时 Z 轴陀螺仪原始计数均值，用于消除零偏；不是角度或度每秒。 */
static int16 imu_gyro_z_bias;
/* 对外提供的整数俯仰角，单位为度。 */
volatile int16 pitch;
/* 互补滤波内部的俯仰角，单位为毫度（1000 毫度=1 度），用于保留积分精度。 */
static int32 pitch_mdeg;
/* 转向阻尼快速滤波。
 * Q8保留低速小数；一阶滤波 alpha=1/2，每5ms更新，无新增浮点滤波。 */
static int32 steering_rate_q8;
static volatile int16 steering_rate10;
static volatile uint8 steering_sequence, steering_sample_valid;

/* Reference Imu.c: TIM0, 5 ms, 0.997 complementary filter.
 * All products are widened before multiplication for C251's 16-bit int.
 * Only this ISR reads the sensor once initialized. */
/* TIM0 的 5 ms 回调：采样 IMU、扣除 Z 零偏、滤波角速度并更新俯仰角。
 * raw_rate：Z 轴角速度（度/秒）；gyro_x：X 轴原始计数；acc_pitch：加速度推算俯仰（毫度）；predicted：陀螺积分预测俯仰。 */
static void IMU_pitHandler(void)
{
    float raw_rate;
    int32 gyro_x, acc_pitch, predicted;
    int16 rate10, gyro_z;
    bit irq_state;
    if (!imu_ready) return;
    imu660rb_get_acc();
    imu660rb_get_gyro();
    /* Subtract in 32 bits before clamping: signed 16-bit extremes can overflow. */
    gyro_z = (int16)func_limit_ab(
        (int32)imu660rb_gyro_z - imu_gyro_z_bias, -32768L, 32767L);
    raw_rate = imu660rb_gyro_transition(gyro_z);
    rate10 = (int16)(func_limit(raw_rate, 2000.0f) * 10.0f);
    steering_rate_q8 += ((int32)rate10 * 256L - steering_rate_q8) / 2L;
    rate10 = (int16)(steering_rate_q8 / 256L);
    irq_state = EA;
    EA = 0;
    steering_rate10 = rate10;
    ++steering_sequence;
    steering_sample_valid = 1;
    EA = irq_state;
    gyro_x = imu660rb_gyro_x;
    if (func_abs(gyro_x) < 100L) gyro_x = 0;
    acc_pitch = atan2_int(-(int32)imu660rb_acc_y,
        func_abs((int32)imu660rb_acc_z)) * 1000L - IMU_INSTALL_OFFSET_MDEG;
    predicted = pitch_mdeg + gyro_x * 6993L / 20000L;
    /* Equivalent complementary weights, avoiding a large 997 * angle product. */
    pitch_mdeg = predicted + 3L * (acc_pitch - predicted) / 1000L;
    pitch = (int16)(pitch_mdeg / 1000L);
}

/* Foreground calibration only; imu_ready=0 prevents the timer from accessing SPI.
 * Check all gyro axes so pitching/rolling during calibration also rejects it.
 * This rejects obvious motion, but cannot distinguish very slow steady rotation
 * from sensor bias: the car must physically remain stationary. */
/* 静止采样三轴，检查绝对值和波动范围，计算 Z 轴零偏；返回 0 成功、1 超过重试次数仍未通过。
 * sample/axis/attempt：样本、轴、尝试索引；failure：本轮失败原因位；raw/low/high：三轴当前/最低/最高原始计数；sum_z：Z 轴累加值。 */
static uint8 imu_calibrate_z(void)
{
    uint16 sample;
    uint8 axis, attempt, failure;
    int16 raw[3], low[3], high[3];
    int32 sum_z;
    for(attempt = 0; attempt < IMU_CAL_ATTEMPTS; ++attempt) {
        sum_z = 0;
        failure = 0;
        for(axis = 0; axis < 3; ++axis) {
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
                if(func_abs((int32)raw[axis]) > IMU_CAL_MAX_ABS_RAW)
                    failure |= IMU_CAL_FAIL_ABS;
                if(raw[axis] < low[axis]) low[axis] = raw[axis];
                if(raw[axis] > high[axis]) high[axis] = raw[axis];
            }
            sum_z += raw[2];
        }
        for(axis = 0; axis < 3; ++axis)
            if((int32)high[axis] - low[axis] > IMU_CAL_MAX_SPAN_RAW)
                failure |= IMU_CAL_FAIL_SPAN;
        imu_cal_failure = failure;
        if(!failure) {
            imu_gyro_z_bias = (int16)(sum_z / (int32)IMU_CAL_SAMPLES);
            return 0;
        }
    }
    return 1;
}

/* 先停车，再初始化传感器和静止零偏校准，成功后注册 TIM0 回调；返回 0 成功、1 设备缺失、2 校准失败。
 * irq_state 保存进入临界区前的总中断使能状态；仅在状态发布和定时器注册期间关中断。 */
uint8 IMU_Init(void)
{
    bit irq_state;
    Motor_SetEnabled(0);
    imu_ready = 0;
    imu_cal_status = IMU_STATUS_MISSING;
    imu_cal_failure = 0;
    imu_gyro_z_bias = 0;
    pitch = 0;
    pitch_mdeg = 0;
    steering_rate_q8 = 0;
    steering_rate10 = 0;
    steering_sequence = steering_sample_valid = 0;
    /* Keep menus accessible if the sensor is absent instead of hanging boot. */
    if (imu660rb_init()) return 1;
    imu_cal_status = IMU_STATUS_CALIBRATING;
    system_delay_ms(500);
    if(imu_calibrate_z()) {
        /* 失败后保持 IMU 禁用；放稳车辆后，可在 Image 页按 SELECT 重试。 */
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

void IMU_GetSteeringSample(imu_steering_sample_t *sample)
{
    bit irq_state = EA;
    EA = 0;
    sample->rate10 = steering_rate10;
    sample->sequence = steering_sequence;
    sample->valid = imu_ready && steering_sample_valid;
    EA = irq_state;
}
