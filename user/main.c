#include "zf_common_headfile.h"
#include "assistant_debug.h"

void main()
{
    clock_init(SYSTEM_CLOCK_96M);
    debug_init();
    assistant_debug_init();

    while(1)
    {
        assistant_debug_poll();
    }
}
