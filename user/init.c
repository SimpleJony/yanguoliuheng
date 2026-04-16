#include "init.h"

void initAllModules(){
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化
    Motor_Init();                             // 电机初始化

    ips200_set_dir(IPS200_PORTAIT); // 设置为竖屏模式
    ips200_init(); // 初始化屏幕
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

void WifiSPI_Init(){
    uint8 wifi_spi_test_buffer[256] = "this is wifi spi test buffer";
    uint8 wifi_spi_recv_buffer[256] = "recv buffer : ";
    uint8 wifi_spi_get_data_buffer[256] = {0};
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