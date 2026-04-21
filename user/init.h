#ifndef __INIT_H__
#define __INIT_H__

#include "zf_common_headfile.h"

// 电机方向与 PWM 引脚定义
#define MAX_DUTY                        ( 50 )
#define DIR_1                           ( IO_P50 )
#define PWM_1                           ( PWMD_CH2_P51 )
#define DIR_2                           ( IO_P52 )
#define PWM_2                           ( PWMD_CH4_P53 )
#define DIR_3                           ( IO_P75 )
#define PWM_3                           ( PWMB_CH1_P74 )
#define DIR_4                           ( IO_P77 )
#define PWM_4                           ( PWMB_CH3_P76 )

// 速度闭环采样编码器定义（电机1）
#define SPEED_ENCODER                   ( PWMA_ENCODER )
#define SPEED_ENCODER_PULSE_PIN         ( PWMA_ENCODER_CH1P_P60 )
#define SPEED_ENCODER_DIR_PIN           ( PWMA_ENCODER_CH2P_P62 )

// PID 控制周期与示波器发送周期
#define CONTROL_PIT_CH                  ( TIM1_PIT )
#define CONTROL_PERIOD_MS               ( 10 )
#define CONTROL_TASK_PERIOD_MS          ( 5 )
#define OSCILLOSCOPE_SEND_PERIOD_MS     ( 20 )

// 配置网络信息
#define WIFI_SSID          "Xiaomi 17 Pro"
#define WIFI_PASSWORD      "jony123456"

void initAllModules();
void Motor_Init();
void Encoder_Init();
void SeekfreeAssistant_Init();
void Screen_Init();
void WifiSPI_Init();

#endif
    