#ifndef __INIT_H__
#define __INIT_H__

#include "zf_common_headfile.h"

// 电机方向与 PWM 引脚定义
#define MAX_DUTY                        ( 100 )
#define DIR_1                           ( IO_P52 )
#define PWM_1                           ( PWMD_CH4_P53 )
#define DIR_2                           ( IO_P50 )
#define PWM_2                           ( PWMD_CH2_P51 )
#define DIR_3                           ( IO_P75 )
#define PWM_3                           ( PWMB_CH1_P74 )
#define DIR_4                           ( IO_P77 )
#define PWM_4                           ( PWMB_CH3_P76 )

#define SERVO_PWM                       ( PWME_CH1P_PA0 )
#define SERVO_PWM_FREQ                  ( 50 )
#define SERVO_DUTY(angle)               ((float)PWM_DUTY_MAX / (1000.0f / (float)SERVO_PWM_FREQ) * (0.5f + (float)(angle) / 90.0f))
#define SERVO_DUTY_CENTER               ( 815.0f )
#define SERVO_DUTY_MIN                  ( SERVO_DUTY(0.0f) )
#define SERVO_DUTY_MAX                  ( SERVO_DUTY(180.0f) )
#define SERVO_OUTPUT_MIN                ( -100.0f )
#define SERVO_OUTPUT_MAX                ( 85.0f )
#define SERVO_SAFE_DUTY_MIN             ( 730.0f )
#define SERVO_SAFE_DUTY_MAX             ( 915.0f )
#define SERVO_DIRECTION_SIGN            ( -1.0f )
#define SERVO_TARGET_RAMP_STEP          ( 8.0f )
#define SERVO_CENTER_RAMP_STEP          ( 2.0f )
#define SERVO_DIRECT_RAMP_STEP          ( 2.0f )
#define SERVO_OUTPUT_RAMP_STEP          ( 16.0f )

#if ((SERVO_PWM_FREQ < 50) || (SERVO_PWM_FREQ > 300))
    #error "SERVO_PWM_FREQ must be 50-300"
#endif

// 速度闭环采样编码器定义
#define SPEED_ENCODER1                   ( PWMC_ENCODER )
#define SPEED_ENCODER1_PULSE_PIN         ( PWMC_ENCODER_CH1P_P40 )
#define SPEED_ENCODER1_DIR_PIN           ( PWMC_ENCODER_CH2P_P42 )
#define SPEED_ENCODER2                   ( PWMA_ENCODER )
#define SPEED_ENCODER2_PULSE_PIN         ( PWMA_ENCODER_CH1P_P60 )
#define SPEED_ENCODER2_DIR_PIN           ( PWMA_ENCODER_CH2P_P62 )

// PID 控制周期与示波器发送周期
#define CONTROL_PIT_CH                  ( TIM1_PIT )
#define CONTROL_PERIOD_MS               ( 10 )
#define CONTROL_TASK_PERIOD_MS          ( 5 )
#define OSCILLOSCOPE_SEND_PERIOD_MS     ( 20 )

#define ASSISTANT_TUNE_MOTOR            ( 0 )
#define ASSISTANT_TUNE_SERVO            ( 1 )
#define ASSISTANT_TUNE_NONE             ( 2 )
#define SEEKFREE_ASSISTANT_TUNE_TARGET  ( ASSISTANT_TUNE_NONE )

// 配置网络信息
#define SEEKFREE_ASSISTANT_USE_WIFI_SPI     ( 1 )
#define WIFI_SSID                           "Xiaomi 17 Pro"
#define WIFI_PASSWORD                       "jony123456"
#define WIFI_TRANSPORT_TYPE                 "TCP"
#define WIFI_TARGET_IP                      "10.28.25.145"
#define WIFI_TARGET_PORT                    "8086"
#define WIFI_LOCAL_PORT                     "8086"
#define WIFI_CONNECT_RETRY_MAX              ( 5 )

void initAllModules();
void Motor_Init();
void Servo_Init();
void Encoder_Init();
void Camera_Init();
void SeekfreeAssistant_Init();
void SeekfreeAssistantCamera_Init();
void Screen_Init();
uint8 WifiSPI_Init();

#endif
    
