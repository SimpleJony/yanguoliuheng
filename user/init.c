#include "init.h"

void initAllModules(){
    clock_init(SYSTEM_CLOCK_96M); 			 // 时钟配置及系统初始化<务必保留>
    debug_init();                       	 // 调试串口信息初始化
    Motor_Init();                            // 电机初始化
    Encoder_Init();                          // 编码器初始化
    SeekfreeAssistant_Init();                // 逐飞助手初始化
    //Screen_Init();                         // 屏幕初始化 需要我再开
}

void Motor_Init(){
    gpio_init(DIR_1, GPO, GPIO_HIGH, GPO_PUSH_PULL);   // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_1, 17000, 0);                         // PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(DIR_2, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_2, 17000, 0);                      

    gpio_init(DIR_3, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_3, 17000, 0);                      

    gpio_init(DIR_4, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_4, 17000, 0);                      
}

void Encoder_Init(){
    encoder_quad_init(SPEED_ENCODER1, SPEED_ENCODER1_PULSE_PIN, SPEED_ENCODER1_DIR_PIN);
    encoder_quad_init(SPEED_ENCODER2, SPEED_ENCODER2_PULSE_PIN, SPEED_ENCODER2_DIR_PIN);
}

void SeekfreeAssistant_Init(){
    seekfree_assistant_init();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
}

void Screen_Init(){
    ips200_set_dir(IPS200_PORTAIT); // 设置为竖屏模式
    ips200_init(); // 初始化屏幕
}

void WifiSPI_Init(){
    uint8 wifi_spi_test_buffer[256] = "this is wifi spi test buffer";
    uint16 data_length = 0;
    while(wifi_spi_init(WIFI_SSID, WIFI_PASSWORD))
    {
        printf("\r\n connect wifi failed, try again. \r\n");
        system_delay_ms(100);                                               // 连接失败 等待 100ms
    }
    data_length = wifi_spi_send_buffer(wifi_spi_test_buffer, sizeof(wifi_spi_test_buffer));
    if(!data_length)
    {
        printf("\r\n send success. \r\n");
    }
    else
    {
        printf("\r\n %d bytes data send failed. \r\n", data_length);
    }
}
