#include "imu.h"

int32 alpha = 997;    // 陀螺仪数据权重
int32 angle;          // 内部计算角度
int16 pitch;          // 提供给外部的偏航角数据

void IMU_pitHandler() {
    // 读取IMU数据
    imu660rb_get_acc();
    imu660rb_get_gyro();
    // TODO: 测噪声阈值
    if (func_abs(imu660rb_gyro_x) < 100) imu660rb_gyro_x = 0;   // 角度的死区设置，过滤掉噪声
    // TODO: 修改安装偏差
    angle = (alpha * (angle + imu660rb_gyro_x * 6993 / 20000) + (1000 - alpha) * (atan2_int(-imu660rb_acc_y, func_abs(imu660rb_acc_z)) * 1000 - 4000)) / 1000;  // 放大为千分之一度计算angle
    pitch = angle / 1000;
}

void IMU_Init() {
    while (imu660rb_init());
    system_delay_ms(100);
    pit_ms_init(TIM0_PIT, 5, IMU_pitHandler);
}