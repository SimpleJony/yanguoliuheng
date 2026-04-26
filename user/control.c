#include "control.h"

// PID参数
static volatile float pid_Kp = 0.35f;
static volatile float pid_Ki = 0.02f;
static volatile float pid_Kd = 0.00f;

// 控制要用到的变量
static volatile float targetSpeed1 = 150.0f;
static volatile float targetSpeed2 = 150.0f;
static volatile float feedbackSpeed1;
static volatile float feedbackSpeed2;
static volatile float errorMotor1;
static volatile float errorMotor1_k_1;
static volatile float errorMotor1_k_2;
static volatile float deltaOutput1;
static volatile float output1;
static volatile float errorMotor2;
static volatile float errorMotor2_k_1;
static volatile float errorMotor2_k_2;
static volatile float deltaOutput2;
static volatile float output2;

// 编码器计数
static volatile int16 encoderCount1 = 0;
static volatile int16 encoderCount2 = 0;

// 示波器发送周期
static volatile float oscSendTick = 0;

void control_reset(){
    bit irq_backup = EA;
    EA = 0;

    errorMotor1 = 0.0f;
    errorMotor1_k_1 = 0.0f;
    errorMotor1_k_2 = 0.0f;
    deltaOutput1 = 0.0f;
    output1 = 0.0f;

    errorMotor2 = 0.0f;
    errorMotor2_k_1 = 0.0f;
    errorMotor2_k_2 = 0.0f;
    deltaOutput2 = 0.0f;
    output2 = 0.0f;

    // 恢复中断状态
    EA = irq_backup;

    Motor1_SetSpeed(0.0f);
    Motor2_SetSpeed(0.0f);
}

void control_Init(){
    control_reset();
    pit_ms_init(CONTROL_PIT_CH, CONTROL_PERIOD_MS, control_pit_handler);
}

void control_task(){
    // 解析逐飞助手下发数据（在线调参）
    seekfree_assistant_data_analysis();
    control_update();

    if (oscSendTick >= OSCILLOSCOPE_SEND_PERIOD_MS)
    {
        oscSendTick = 0;
        control_sendOscData();
    }
    else
    {
        oscSendTick += CONTROL_TASK_PERIOD_MS;
    }

    system_delay_ms(CONTROL_TASK_PERIOD_MS);
}

void control_pit_handler(){
    float errorTemp1 = 0.0f;
    float deltaOutputTemp1 = 0.0f;
    float outputTemp1 = 0.0f;
    float errorTemp2 = 0.0f;
    float deltaOutputTemp2 = 0.0f;
    float outputTemp2 = 0.0f;

    // 读取当前控制周期内的脉冲计数，并立即清零，供下一个周期重新累计
    encoderCount1 = encoder_get_count(SPEED_ENCODER1);
    encoderCount2 = encoder_get_count(SPEED_ENCODER2);

    encoder_clear_count(SPEED_ENCODER1);
    encoder_clear_count(SPEED_ENCODER2);

    // 按控制周期换算为“每秒计数值”
    // 例如: CONTROL_PERIOD_MS = 10ms，则换算系数为 100
    feedbackSpeed1 = (float)encoderCount1 * (1000.0f / CONTROL_PERIOD_MS);
    feedbackSpeed2 = (float)encoderCount2 * (1000.0f / CONTROL_PERIOD_MS);

    errorTemp1 = targetSpeed1 - feedbackSpeed1;
    errorTemp2 = targetSpeed2 - feedbackSpeed2;

    // 电机1增量式PID
    deltaOutputTemp1 = pid_Kp * (errorTemp1 - errorMotor1_k_1)
                     + pid_Ki * errorTemp1
                     + pid_Kd * (errorTemp1 - 2.0f * errorMotor1_k_1 + errorMotor1_k_2);

    outputTemp1 = output1 + deltaOutputTemp1;
    outputTemp1 = control_limit(outputTemp1, -(float)MAX_DUTY, (float)MAX_DUTY);

    errorMotor1 = errorTemp1;
    deltaOutput1 = deltaOutputTemp1;
    output1 = outputTemp1;
    errorMotor1_k_2 = errorMotor1_k_1;
    errorMotor1_k_1 = errorTemp1;

    // 电机2增量式PID
    deltaOutputTemp2 = pid_Kp * (errorTemp2 - errorMotor2_k_1)
                     + pid_Ki * errorTemp2
                     + pid_Kd * (errorTemp2 - 2.0f * errorMotor2_k_1 + errorMotor2_k_2);

    outputTemp2 = output2 + deltaOutputTemp2;
    outputTemp2 = control_limit(outputTemp2, -(float)MAX_DUTY, (float)MAX_DUTY);

    errorMotor2 = errorTemp2;
    deltaOutput2 = deltaOutputTemp2;
    output2 = outputTemp2;
    errorMotor2_k_2 = errorMotor2_k_1;
    errorMotor2_k_1 = errorTemp2;

    Motor1_SetSpeed(outputTemp1);
    Motor2_SetSpeed(outputTemp2);
}

float control_limit(float value, float low, float high)
{
    if (value > high) value = high;
    if (value < low) value = low;
    return value;
}

void control_update(){
    bit irq_backup = EA;
    EA = 0;

    if(seekfree_assistant_parameter_update_flag[0])
    {
        pid_Kp = control_limit(seekfree_assistant_parameter[0], 0.0f, 20.0f);
        seekfree_assistant_parameter_update_flag[0] = 0;
    }
    if(seekfree_assistant_parameter_update_flag[1])
    {
        pid_Ki = control_limit(seekfree_assistant_parameter[1], 0.0f, 10.0f);
        seekfree_assistant_parameter_update_flag[1] = 0;
    }
    if(seekfree_assistant_parameter_update_flag[2])
    {
        pid_Kd = control_limit(seekfree_assistant_parameter[2], 0.0f, 20.0f);
        seekfree_assistant_parameter_update_flag[2] = 0;
    }
    if(seekfree_assistant_parameter_update_flag[3])
    {
        targetSpeed1 = control_limit(seekfree_assistant_parameter[3], -5000.0f, 5000.0f);
        seekfree_assistant_parameter_update_flag[3] = 0;
    }
    if(seekfree_assistant_parameter_update_flag[4])
    {
        targetSpeed2 = control_limit(seekfree_assistant_parameter[4], -5000.0f, 5000.0f);
        seekfree_assistant_parameter_update_flag[4] = 0;
    }

    // 恢复中断状态
    EA = irq_backup;
}

void control_sendOscData(){
    float local_targetSpeed1 = 0.0f;
    float local_targetSpeed2 = 0.0f;
    float local_feedbackSpeed1 = 0.0f;
    float local_feedbackSpeed2 = 0.0f;
    float local_output1 = 0.0f;
    float local_output2 = 0.0f;
    float local_error1 = 0.0f;
    float local_error2 = 0.0f;
    bit irq_backup = EA;
    EA = 0;

    local_targetSpeed1 = targetSpeed1;
    local_targetSpeed2 = targetSpeed2;
    local_feedbackSpeed1 = feedbackSpeed1;
    local_feedbackSpeed2 = feedbackSpeed2;
    local_output1 = output1;
    local_output2 = output2;
    local_error1 = errorMotor1;
    local_error2 = errorMotor2;

    EA = irq_backup;

    // CH1~CH8: 目标1 反馈1 输出1 误差1 目标2 反馈2 输出2 误差2
    seekfree_assistant_oscilloscope_data.dat[0] = local_targetSpeed1;
    seekfree_assistant_oscilloscope_data.dat[1] = local_feedbackSpeed1;
    seekfree_assistant_oscilloscope_data.dat[2] = local_output1;
    seekfree_assistant_oscilloscope_data.dat[3] = local_error1;
    seekfree_assistant_oscilloscope_data.dat[4] = local_targetSpeed2;
    seekfree_assistant_oscilloscope_data.dat[5] = local_feedbackSpeed2;
    seekfree_assistant_oscilloscope_data.dat[6] = local_output2;
    seekfree_assistant_oscilloscope_data.dat[7] = local_error2;
    seekfree_assistant_oscilloscope_data.channel_num = 8;

    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}

void Motor1_SetSpeed(float dutyPercent)
{
    uint32 pwmDuty = 0;

    if (dutyPercent >= 0.0f)
    {
        gpio_set_level(DIR_1, GPIO_HIGH);
    }
    else
    {
        gpio_set_level(DIR_1, GPIO_LOW);
        dutyPercent = -dutyPercent;
    }

    dutyPercent = control_limit(dutyPercent, 0.0f, (float)MAX_DUTY);
    // 百分比占空比 -> PWM 寄存器刻度值
    pwmDuty = (uint32)(dutyPercent * ((float)PWM_DUTY_MAX / 100.0f));

    pwm_set_duty(PWM_1, pwmDuty);
}

void Motor2_SetSpeed(float dutyPercent)
{
    uint32 pwmDuty = 0;

    if (dutyPercent >= 0.0f)
    {
        gpio_set_level(DIR_2, GPIO_HIGH);
    }
    else
    {
        gpio_set_level(DIR_2, GPIO_LOW);
        dutyPercent = -dutyPercent;
    }

    dutyPercent = control_limit(dutyPercent, 0.0f, (float)MAX_DUTY);
    // 百分比占空比 -> PWM 寄存器刻度值
    pwmDuty = (uint32)(dutyPercent * ((float)PWM_DUTY_MAX / 100.0f));

    pwm_set_duty(PWM_2, pwmDuty);
}