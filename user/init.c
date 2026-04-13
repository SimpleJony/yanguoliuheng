#include "init.h"

#ifdef WIFI_SPI
// 配置网络信息
#define WIFI_SSID_TEST          "Xiaomi 17 Pro"
#define WIFI_PASSWORD_TEST      "jony123456"

uint8 wifi_spi_test_buffer[] = "this is wifi spi test buffer";
uint8 wifi_spi_recv_buffer[] = "recv buffer : ";
uint8 wifi_spi_get_data_buffer[256] = {0};
uint16 data_length = 0;
#endif

void initAllModules(){
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化

    ips200_set_dir(IPS200_PORTAIT); // 设置为竖屏模式
    ips200_init(); // 初始化屏幕

    #ifdef WIFI_SPI
    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
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
    #endif
}