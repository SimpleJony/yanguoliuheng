#include "zf_common_headfile.h"
#include "init.h"
#include "control.h"

void main()
{
    initAllModules();
    control_Init();

    while(1)
    {
        control_task();
    }
}
