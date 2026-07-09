#include "servo.h"
#include "init.h"

#if (SEEKFREE_ASSISTANT_TUNE_TARGET == ASSISTANT_TUNE_SERVO)
#define SERVO_DEFAULT_TUNE_MODE         ( 1.0f )
#else
#define SERVO_DEFAULT_TUNE_MODE         ( 0.0f )
#endif

static volatile float servoDirectOutput = 0.0f;
static volatile float servoDirectCommand = 0.0f;
static volatile float servoTuneMode = SERVO_DEFAULT_TUNE_MODE;
static volatile float servoCenterCommand = (float)SERVO_DUTY_CENTER;
static volatile float servoOutputCommand = 0.0f;

#define SERVO_KP_MIN                    ( 0.0f )
#define SERVO_KP_MAX                    ( 200.0f )
#define SERVO_KD_MIN                    ( 0.0f )
#define SERVO_KD_MAX                    ( 200.0f )
#define SERVO_POSITION_MIN              ( -90.0f )
#define SERVO_POSITION_MAX              ( 90.0f )
#define SERVO_CENTER_MIN                ( (float)SERVO_DUTY_MIN )
#define SERVO_CENTER_MAX                ( (float)SERVO_DUTY_MAX )

static volatile float servoKp = 8.0f;                                   // 閼稿灚婧€Kp
static volatile float servoKd = 2.0f;                                   // 閼稿灚婧€Kd
static volatile float servoTargetCommand = 0.0f;                        // 闁劙顥ｉ崝鈺傚閸欐垿鈧胶娈戦惄顔界垼娴ｅ秶鐤嗛崨鎴掓姢
static volatile float servoTargetPosition = 0.0f;                       // 缂佸繗绻冮弬婊冩蒋婢跺嫮鎮婇崥搴ｆ畱閻╊喗鐖ｆ担宥囩枂
static volatile float servoFeedbackPosition = 0.0f;                     // 閼稿灚婧€閸欏秹顩担宥囩枂            
static volatile float servoError = 0.0f;                                // 閼稿灚婧€瑜版挸澧犳担宥囩枂鐠囶垰妯?
static volatile float servoErrorLast = 0.0f;                            // 閼稿灚婧€娑撳﹥顐兼担宥囩枂鐠囶垰妯?
static volatile float servoOutputDuty = 0.0f;                           // 閼稿灚婧€鏉堟挸鍤崡鐘碘敄濮?
static volatile float servoPwmDuty = (float)SERVO_DUTY_CENTER;          // 閼稿灚婧€PWM閸楃姷鈹栧В?
static volatile float servoCenterDuty = (float)SERVO_DUTY_CENTER;       // 閼稿灚婧€娑擃厼绺炬担宥囩枂閸楃姷鈹栧В?

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

static float Servo_RampTarget(float current, float target, float step)
{
    float error = target - current;

    if (error > step)
    {
        current += step;
    }
    else if (error < -step)
    {
        current -= step;
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
    servoOutputCommand = 0.0f;
    servoCenterDuty = (float)SERVO_DUTY_CENTER;
    servoCenterCommand = servoCenterDuty;
    servoPwmDuty = servoCenterDuty;
    servoDirectOutput = 0.0f;
    servoDirectCommand = 0.0f;
    servoTuneMode = SERVO_DEFAULT_TUNE_MODE;

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

    servoCenterDuty = Servo_RampTarget(servoCenterDuty, servoCenterCommand, SERVO_CENTER_RAMP_STEP);
    servoDirectOutput = Servo_RampTarget(servoDirectOutput, servoDirectCommand, SERVO_DIRECT_RAMP_STEP);

    if (servoTuneMode >= 0.5f)
    {
        outputTemp = Servo_Limit(servoDirectOutput, -SERVO_OUTPUT_LIMIT, SERVO_OUTPUT_LIMIT);
        dutyTemp = servoCenterDuty + SERVO_DIRECTION_SIGN * outputTemp;
        servoError = servoTargetPosition - servoFeedbackPosition;
        servoErrorLast = servoError;
        servoOutputDuty = outputTemp;
        Servo_SetPwmDuty(dutyTemp);
        return;
    }

    servoTargetPosition = Servo_RampTarget(servoTargetPosition, servoTargetCommand, SERVO_TARGET_RAMP_STEP);
    targetTemp = servoTargetPosition;

    errorTemp = targetTemp - feedbackTemp;
    servoOutputCommand = kpTemp * errorTemp + kdTemp * (errorTemp - servoErrorLast);
    servoOutputCommand = Servo_Limit(servoOutputCommand, -SERVO_OUTPUT_LIMIT, SERVO_OUTPUT_LIMIT);
    outputTemp = Servo_RampTarget(servoOutputDuty, servoOutputCommand, SERVO_OUTPUT_RAMP_STEP);
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
        servoCenterCommand = Servo_Limit(seekfree_assistant_parameter[SERVO_PARAM_CENTER_CHANNEL - 1],
                                         SERVO_CENTER_MIN,
                                         SERVO_CENTER_MAX);
        *lastParamChannel = (float)SERVO_PARAM_CENTER_CHANNEL;
        *lastParamValue = servoCenterCommand;
        seekfree_assistant_parameter_update_flag[SERVO_PARAM_CENTER_CHANNEL - 1] = 0;
    }

    if (seekfree_assistant_parameter_update_flag[SERVO_PARAM_DIRECT_CHANNEL - 1])
    {
        servoDirectCommand = Servo_Limit(seekfree_assistant_parameter[SERVO_PARAM_DIRECT_CHANNEL - 1],
                                         -SERVO_OUTPUT_LIMIT,
                                         SERVO_OUTPUT_LIMIT);
        servoTuneMode = 1.0f;
        *lastParamChannel = (float)SERVO_PARAM_DIRECT_CHANNEL;
        *lastParamValue = servoDirectCommand;
        seekfree_assistant_parameter_update_flag[SERVO_PARAM_DIRECT_CHANNEL - 1] = 0;
    }

    if (seekfree_assistant_parameter_update_flag[SERVO_PARAM_MODE_CHANNEL - 1])
    {
        servoTuneMode = (seekfree_assistant_parameter[SERVO_PARAM_MODE_CHANNEL - 1] >= 0.5f) ? 1.0f : 0.0f;
        *lastParamChannel = (float)SERVO_PARAM_MODE_CHANNEL;
        *lastParamValue = servoTuneMode;
        seekfree_assistant_parameter_update_flag[SERVO_PARAM_MODE_CHANNEL - 1] = 0;
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

float Servo_GetTuneMode(void)
{
    return servoTuneMode;
}
