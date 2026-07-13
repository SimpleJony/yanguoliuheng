#include "accelerator.h"
#include "zf_common_headfile.h"

static volatile uint8 acceleratorReady = 0;
static volatile float acceleratorYawRate = 0.0f;
static float acceleratorYawBias = 0.0f;

static int16 Accelerator_GetYawRaw(void)
{
#if (ACCELERATOR_YAW_AXIS == 0)
    return imu660rb_gyro_x;
#elif (ACCELERATOR_YAW_AXIS == 1)
    return imu660rb_gyro_y;
#elif (ACCELERATOR_YAW_AXIS == 2)
    return imu660rb_gyro_z;
#else
    #error "ACCELERATOR_YAW_AXIS must be 0, 1 or 2"
#endif
}

uint8 Accelerator_Init(void)
{
    uint16 sample = 0;
    float biasSum = 0.0f;
    float sampleRate = 0.0f;
    float sampleMin = 0.0f;
    float sampleMax = 0.0f;

    acceleratorReady = 0;
    acceleratorYawRate = 0.0f;
    acceleratorYawBias = 0.0f;

    if (imu660rb_init())
    {
        return 1;
    }

    // 等待芯片和供电稳定，再丢弃刚启动时的旧数据。
    system_delay_ms(ACCELERATOR_STABILIZATION_MS);
    for (sample = 0; sample < ACCELERATOR_DISCARD_SAMPLES; sample++)
    {
        imu660rb_get_gyro();
        system_delay_ms(ACCELERATOR_CALIBRATION_DELAY_MS);
    }

    // 校准期间车辆必须保持静止，连续采样计算陀螺仪零偏。
    for (sample = 0; sample < ACCELERATOR_CALIBRATION_SAMPLES; sample++)
    {
        imu660rb_get_gyro();
        sampleRate = ACCELERATOR_YAW_SIGN
                   * imu660rb_gyro_transition(Accelerator_GetYawRaw());

        if (0U == sample)
        {
            sampleMin = sampleRate;
            sampleMax = sampleRate;
        }
        else
        {
            if (sampleRate < sampleMin)
            {
                sampleMin = sampleRate;
            }
            if (sampleRate > sampleMax)
            {
                sampleMax = sampleRate;
            }
        }

        biasSum += sampleRate;
        system_delay_ms(ACCELERATOR_CALIBRATION_DELAY_MS);
    }

    acceleratorYawBias = biasSum / (float)ACCELERATOR_CALIBRATION_SAMPLES;

    // 校准期间发生明显转动，或零偏异常时拒绝启用反馈。
    if (((sampleMax - sampleMin) > ACCELERATOR_CALIBRATION_MAX_RANGE_DPS)
        || (acceleratorYawBias > ACCELERATOR_CALIBRATION_MAX_BIAS_DPS)
        || (acceleratorYawBias < -ACCELERATOR_CALIBRATION_MAX_BIAS_DPS))
    {
        acceleratorYawBias = 0.0f;
        return 2;
    }

    acceleratorReady = 1;

    return 0;
}

void Accelerator_Update(void)
{
    float rawYawRate = 0.0f;

    if (!acceleratorReady)
    {
        acceleratorYawRate = 0.0f;
        return;
    }

    imu660rb_get_gyro();
    rawYawRate = ACCELERATOR_YAW_SIGN
               * imu660rb_gyro_transition(Accelerator_GetYawRaw())
               - acceleratorYawBias;

    if ((rawYawRate < ACCELERATOR_YAW_DEADBAND_DPS)
        && (rawYawRate > -ACCELERATOR_YAW_DEADBAND_DPS))
    {
        rawYawRate = 0.0f;
    }

    acceleratorYawRate += ACCELERATOR_YAW_FILTER_ALPHA
                        * (rawYawRate - acceleratorYawRate);

    if ((acceleratorYawRate < 0.05f)
        && (acceleratorYawRate > -0.05f))
    {
        acceleratorYawRate = 0.0f;
    }
}

uint8 Accelerator_IsReady(void)
{
    return acceleratorReady;
}

float Accelerator_GetYawRate(void)
{
    return acceleratorYawRate;
}
