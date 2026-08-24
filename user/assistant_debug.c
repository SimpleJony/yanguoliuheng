#include "assistant_debug.h"

// 上位机参数通道号，和逐飞助手调参界面的 1~8 号通道一一对应
#define CH_SERVO_KP       0
#define CH_SERVO_KD       1
#define CH_SERVO_GYRO     2
#define CH_MOTOR_KP       3
#define CH_MOTOR_KI       4
#define CH_MOTOR_KD       5
#define CH_SET_SPEED      6
#define CH_STEER_DUTY     7

#define DEBUG_SERVO_MAX_OUT   400
#define DEBUG_SCOPE_CHANNEL   8

static void assistant_debug_apply_parameter(void);
static void assistant_debug_send_scope(void);

void assistant_debug_init(void)
{
    // 1. 初始化舵机、电机、编码器，并启动 5ms 控制中断
    Motor_Init(200, 25, 4000, 300);

    // 2. 给 PID 一套可用的默认值。
    //    Motor_Init 里部分参数是 0（电机会不动、舵机输出会被限到 0），这里补上。
    servoPid.Kp = 200;
    servoPid.Kd = 50;
    servoPid.attitude_coeff = 2;
    servoPid.max_out = DEBUG_SERVO_MAX_OUT;

    motorLPid.Kp = 3000; motorLPid.Ki = 50; motorLPid.Kd = 50; motorLPid.max_out = PWM_DUTY_MAX;
    motorRPid.Kp = 3000; motorRPid.Ki = 50; motorRPid.Kd = 50; motorRPid.max_out = PWM_DUTY_MAX;

    // 3. 初始化逐飞助手，使用调试串口（默认 USB CDC）
    seekfree_assistant_init();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
}

void assistant_debug_poll(void)
{
    static uint8 scope_tick = 0;

    // 处理上位机发来的调参数据
    seekfree_assistant_data_analysis();
    assistant_debug_apply_parameter();

    // 每两次 poll 发一帧示波器，约 100Hz，避免占用过多带宽
    if (++scope_tick >= 2)
    {
        scope_tick = 0;
        assistant_debug_send_scope();
    }

    system_delay_ms(5);
}

static void assistant_debug_apply_parameter(void)
{
    // 转向 PD
    if (seekfree_assistant_parameter_update_flag[CH_SERVO_KP])
    {
        servoPid.Kp = (int16)seekfree_assistant_parameter[CH_SERVO_KP];
        seekfree_assistant_parameter_update_flag[CH_SERVO_KP] = 0;
    }
    if (seekfree_assistant_parameter_update_flag[CH_SERVO_KD])
    {
        servoPid.Kd = (int16)seekfree_assistant_parameter[CH_SERVO_KD];
        seekfree_assistant_parameter_update_flag[CH_SERVO_KD] = 0;
    }
    if (seekfree_assistant_parameter_update_flag[CH_SERVO_GYRO])
    {
        servoPid.attitude_coeff = (int16)seekfree_assistant_parameter[CH_SERVO_GYRO];
        seekfree_assistant_parameter_update_flag[CH_SERVO_GYRO] = 0;
    }

    // 轮速 PID：左右轮先共用一组参数
    if (seekfree_assistant_parameter_update_flag[CH_MOTOR_KP])
    {
        motorLPid.Kp = motorRPid.Kp = (int16)seekfree_assistant_parameter[CH_MOTOR_KP];
        seekfree_assistant_parameter_update_flag[CH_MOTOR_KP] = 0;
    }
    if (seekfree_assistant_parameter_update_flag[CH_MOTOR_KI])
    {
        motorLPid.Ki = motorRPid.Ki = (int16)seekfree_assistant_parameter[CH_MOTOR_KI];
        seekfree_assistant_parameter_update_flag[CH_MOTOR_KI] = 0;
    }
    if (seekfree_assistant_parameter_update_flag[CH_MOTOR_KD])
    {
        motorLPid.Kd = motorRPid.Kd = (int16)seekfree_assistant_parameter[CH_MOTOR_KD];
        seekfree_assistant_parameter_update_flag[CH_MOTOR_KD] = 0;
    }

    // 手动命令：单独调电机时改 setSpeed，单独调舵机时改 duty
    if (seekfree_assistant_parameter_update_flag[CH_SET_SPEED])
    {
        setSpeed = (int16)seekfree_assistant_parameter[CH_SET_SPEED];
        seekfree_assistant_parameter_update_flag[CH_SET_SPEED] = 0;
    }
    if (seekfree_assistant_parameter_update_flag[CH_STEER_DUTY])
    {
        duty = (int16)seekfree_assistant_parameter[CH_STEER_DUTY];
        seekfree_assistant_parameter_update_flag[CH_STEER_DUTY] = 0;
    }
}

static void assistant_debug_send_scope(void)
{
    seekfree_assistant_oscilloscope_data.channel_num = DEBUG_SCOPE_CHANNEL;
    seekfree_assistant_oscilloscope_data.dat[0] = (float)encoderL;
    seekfree_assistant_oscilloscope_data.dat[1] = (float)encoderR;
    seekfree_assistant_oscilloscope_data.dat[2] = (float)setSpeed;
    seekfree_assistant_oscilloscope_data.dat[3] = (float)currentSpeed;
    seekfree_assistant_oscilloscope_data.dat[4] = (float)duty;
    seekfree_assistant_oscilloscope_data.dat[5] = (float)servoOut;
    seekfree_assistant_oscilloscope_data.dat[6] = (float)motorOutL;
    seekfree_assistant_oscilloscope_data.dat[7] = (float)motorOutR;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}
