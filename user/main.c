#include "zf_common_headfile.h"
#include "System.h"

void main()
{
    clock_init(SYSTEM_CLOCK_96M);
    System_Init();

    while(1)
    {
        System_Task();
    }
}
