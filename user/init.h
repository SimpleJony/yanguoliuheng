#ifndef __INIT_H__
#define __INIT_H__

#include "zf_common_headfile.h"

// 屏幕与spi模块引脚共用 二者只能选其一
#ifdef WIFI_SPI
extern uint8 wifi_spi_test_buffer[];
extern uint8 wifi_spi_recv_buffer[256];
extern uint8 wifi_spi_get_data_buffer[256];
extern uint16 data_length;
#endif

void initAllModules();

#endif
    