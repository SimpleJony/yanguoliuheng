#include "servo.h"
#include "init.h"

#define SERVO_KP_MIN                    ( 0.0f )
#define SERVO_KP_MAX                    ( 200.0f )
#define SERVO_KD_MIN                    ( 0.0f )
#define SERVO_KD_MAX                    ( 200.0f )
#define SERVO_POSITION_MIN              ( -10000.0f )
#define SERVO_POSITION_MAX              ( 10000.0f )
#define SERVO_CENTER_MIN                ( (float)SERVO_DUTY_MIN )
#define SERVO_CENTER_MAX                ( (float)SERVO_DUTY_MAX )

static volatile float servoKp = 8.0f;                                   // 舵机Kp
static volatile float servoKd = 2.0f;                                   // 舵机Kd
static volatile float servoTargetCommand = 0.0f;                        // 逐飞助手发送的目标位置命令
static volatile float servoTargetPosition = 0.0f;                       // 经过斜坡处理后的目标位置
static volatile float servoFeedbackPosition = 0.0f;                     // 舵机反馈位置            
static volatile float servoError = 0.0f;                                // 舵机当前位置误差
static volatile float servoErrorLast = 0.0f;                            // 舵机上次位置误差
static volatile float servoOutputDuty = 0.0f;                           // 舵机输出占空比
static volatile float servoPwmDuty = (float)SERVO_DUTY_CENTER;          // 舵机PWM占空比
static volatile float servoCenterDuty = (float)SERVO_DUTY_CENTER;       // 舵机中心位置占空比

static float Servo_Limit(float value, float low, float high)
{
    if (value > high)
    {
        value = high;
    }
    if (value < low)
    {
        value = low;
    }
    return value;
}

static void Servo_SetPwmDuty(float duty)
{
    duty = Servo_Limit(duty, (float)SERVO_DUTY_MIN, (float)SERVO_DUTY_MAX);
    servoPwmDuty = duty;
    pwm_set_duty(SERVO_PWM, (uint32)duty);
}

static float Servo_RampTarget(float current, float target)
{
    float error = target - current;

    if (error > SERVO_TARGET_RAMP_STEP)
    {
        current += SERVO_TARGET_RAMP_STEP;
    }
    else if (error < -SERVO_TARGET_RAMP_STEP)
    {
        current -= SERVO_TARGET_RAMP_STEP;
    }
    else
    {
        current = target;
    }

    return current;
}

void Servo_Reset(void)
{
    bit irq_backup = EA;
    EA = 0;

    servoTargetPosition = 0.0f;
    servoTargetCommand = 0.0f;
    servoFeedbackPosition = 0.0f;
    servoError = 0.0f;
    servoErrorLast = 0.0f;
    servoOutputDuty = 0.0f;
    servoCenterDuty = (float)SERVO_DUTY_CENTER;
    servoPwmDuty = servoCenterDuty;

    EA = irq_backup;

    Servo_SetPwmDuty(servoCenterDuty);
}

void Servo_SetPosition(float position)
{
    servoFeedbackPosition = Servo_Limit(position, SERVO_POSITION_MIN, SERVO_POSITION_MAX);
}

void Servo_SetTargetPosition(float target)
{
    servoTargetCommand = Servo_Limit(target, SERVO_POSITION_MIN, SERVO_POSITION_MAX);
}

void Servo_PositionPdUpdate(void)
{
    float kpTemp = servoKp;
    float kdTemp = servoKd;
    float targetTemp = 0.0f;
    float feedbackTemp = servoFeedbackPosition;
    float errorTemp = 0.0f;
    float outputTemp = 0.0f;
    float dutyTemp = 0.0f;

    servoTargetPosition = Servo_RampTarget(servoTargetPosition, servoTargetCommand);
    targetTemp = servoTargetPosition;

    errorTemp = targetTemp - feedbackTemp;
    outputTemp = kpTemp * errorTemp + kdTemp * (errorTemp - servoErrorLast);
    outputTemp = Servo_Limit(outputTemp, -SERVO_OUTPUT_LIMIT, SERVO_OUTPUT_LIMIT);
    dutyTemp = servoCenterDuty + SERVO_DIRECTION_SIGN * outputTemp;

    servoError = errorTemp;
    servoErrorLast = errorTemp;
    servoOutputDuty = outputTemp;

    Servo_SetPwmDuty(dutyTemp);
}

void Servo_AssistantUpdate(volatile float *lastParamChannel, volatile float *lastParamValue)
{
    if (seekfree_assistant_parameter_update_flag[SERVO_PARAM_KP_CHANNEL - 1])
    {
        servoKp = Servo_Limit(seekfree_assistant_parameter[SERVO_PARAM_KP_CHANNEL - 1], SERVO_KP_MIN, SERVO_KP_MAX);
        *lastParamChannel = (float)SERVO_PARAM_KP_CHANNEL;
        *lastParamValue = servoKp;
        seekfree_assistant_parameter_update_flag[SERVO_PARAM_KP_CHANNEL - 1] = 0;
    }

    if (seekfree_assistant_parameter_update_flag[SERVO_PARAM_KD_CHANNEL - 1])
    {
        servoKd = Servo_Limit(seekfree_assistant_parameter[SERVO_PARAM_KD_CHANNEL - 1], SERVO_KD_MIN, SERVO_KD_MAX);
        *lastParamChannel = (float)SERVO_PARAM_KD_CHANNEL;
        *lastParamValue = servoKd;
        seekfree_assistant_parameter_update_flag[SERVO_PARAM_KD_CHANNEL - 1] = 0;
    }

    if (seekfree_assistant_parameter_update_flag[SERVO_PARAM_TARGET_CHANNEL - 1])
    {
        servoTargetCommand = Servo_Limit(seekfree_assistant_parameter[SERVO_PARAM_TARGET_CHANNEL - 1],
                                         SERVO_POSITION_MIN,
                                         SERVO_POSITION_MAX);
        *lastParamChannel = (float)SERVO_PARAM_TARGET_CHANNEL;
        *lastParamValue = servoTargetCommand;
        seekfree_assistant_parameter_update_flag[SERVO_PARAM_TARGET_CHANNEL - 1] = 0;
    }

    if (seekfree_assistant_parameter_update_flag[SERVO_PARAM_CENTER_CHANNEL - 1])
    {
        servoCenterDuty = Servo_Limit(seekfree_assistant_parameter[SERVO_PARAM_CENTER_CHANNEL - 1],
                                      SERVO_CENTER_MIN,
                                      SERVO_CENTER_MAX);
        Servo_SetPwmDuty(servoCenterDuty);
        *lastParamChannel = (float)SERVO_PARAM_CENTER_CHANNEL;
        *lastParamValue = servoCenterDuty;
        seekfree_assistant_parameter_update_flag[SERVO_PARAM_CENTER_CHANNEL - 1] = 0;
    }
}

float Servo_GetTargetPosition(void)
{
    return servoTargetPosition;
}

float Servo_GetTargetCommand(void)
{
    return servoTargetCommand;
}

float Servo_GetFeedbackPosition(void)
{
    return servoFeedbackPosition;
}

float Servo_GetOutputDuty(void)
{
    return servoOutputDuty;
}

float Servo_GetPwmDuty(void)
{
    return servoPwmDuty;
}
