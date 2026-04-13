#include "init.h"

void initAllModules(){
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化
    
    ips200_set_dir(IPS200_PORTAIT); // 设置为竖屏模式
    ips200_init(); // 初始化屏幕
}