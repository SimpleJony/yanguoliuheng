#ifndef __ASSISTANT_DEBUG_H__
#define __ASSISTANT_DEBUG_H__

#include "zf_common_headfile.h"
#include "Motor.h"

// 逐飞助手在线调试：示波器 + 实时调参
void assistant_debug_init(void);
void assistant_debug_poll(void);

#endif
