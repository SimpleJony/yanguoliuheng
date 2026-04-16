#ifndef __INIT_H__
#define __INIT_H__

#include "zf_common_headfile.h"

// 电机的方向引脚和PWM引脚定义
#define MAX_DUTY            ( 50 ) // 最大 MAX_DUTY% 占空比
#define DIR_1               ( IO_P50 ) 
#define PWM_1               ( PWMD_CH2_P51 )                     
#define DIR_2               ( IO_P52 ) 
#define PWM_2               ( PWMD_CH4_P53 )                      
#define DIR_3               ( IO_P75 )
#define PWM_3               ( PWMB_CH1_P74 )                      
#define DIR_4               ( IO_P77 )
#define PWM_4               ( PWMB_CH3_P76 )

// 配置网络信息
#define WIFI_SSID          "Xiaomi 17 Pro"
#define WIFI_PASSWORD      "jony123456"

void initAllModules();
void Motor_Init();
void WifiSPI_Init();

#endif
    