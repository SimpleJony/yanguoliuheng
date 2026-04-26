#include "zf_common_headfile.h"
#include "init.h"
#include "control.h"

void main(){
    initAllModules(); // 初始化所有模块
    control_Init();   // 启动闭环控制

    while(1)
    {
        control_task();
    }
}