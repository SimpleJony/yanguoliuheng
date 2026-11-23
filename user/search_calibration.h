#ifndef SEARCH_CALIBRATION_H_
#define SEARCH_CALIBRATION_H_
// ============================================
// 相机地面标定参数（2026-10-19）
// 原点为近端标线中点，X 向右、Y 向前，单位毫米
// 赛道内宽 450 mm，标线中心间距 200 mm；后轴 (0,-500)，轴距 200 mm
// 车体朝向与坐标轴一致，车辆尺寸详见 run_config.h
// 近端标线下方属于纵向外推区域，使用时保持相机安装姿态一致
// ============================================
#define SEARCH_CALIBRATED 1                     // 是否启用默认标定
#define SEARCH_LANE_WIDTH_MM 450.0000f   // 赛道内侧宽度，毫米
#define SEARCH_SAMPLE_STEP_MM 20.0000f  // 地面路径重采样间距，毫米
#define SEARCH_CALIBRATION_MIN_ROW 38           // 可用图像最远行，含端点
#define SEARCH_CALIBRATION_MAX_ROW 100          // 可用图像最近行，含端点
// 图像到地面的 3×3 齐次矩阵，按行展开
#define SEARCH_IMAGE_TO_GROUND {12.5024f, -0.4515f, -1163.4221f, -0.6989f, -19.8984f, 1212.1532f, 0.0003f, 0.0391f, 1.0000f}
#endif
