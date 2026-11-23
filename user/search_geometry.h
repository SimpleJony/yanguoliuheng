// 相机标定及地面路径接口。SearchPoint 使用毫米坐标：x 向右、y 向前，原点是标定原点。
#ifndef SEARCH_GEOMETRY_H_
#define SEARCH_GEOMETRY_H_
#include "search_line.h"
#define SEARCH_PATH_CAPACITY 100                // 毫米路径及重采样缓冲最多容纳的点数
/* 普通巡线：近端允许空缺20行，随后连续24行实测双边即可省去单边法线投影。 */
#define SEARCH_GROUND_PAIRED_MIN_ROWS 24
#define SEARCH_GROUND_PAIRED_START_GAP 20

// ============================================
// 坐标与边界特征结构
// ============================================
typedef struct {
    int16 x;                                    // 地面横向坐标，毫米，向右为正。
    int16 y;                                    // 地面纵向坐标，毫米，向前为正。
} SearchPoint;
extern uint8 ground_calibrated;                 // 标定矩阵和参数是否通过验证
extern uint8 ground_projection_rejected;        // 几何中线与实测双边中点偏差过大，已清空本帧投影
extern uint8 ground_center_line[SEARCH_IMAGE_H]; // 毫米路径反投影后的逐行中线列坐标
extern uint8 ground_center_valid[SEARCH_IMAGE_H]; // 各行几何中线有效标记
void search_ground_init(void);                  // 清空几何状态并加载默认标定。
void search_ground_update(void);                // 根据当前边界及元素约束，更新地面路径和图像几何中线。
int16 search_ground_near_row(void);             // 返回最近标定行；未标定时返回图像底行。
// 图像列/行转为毫米 x/y；成功1、失败0，不向标定行范围之外外推。
uint8 search_project_point(int16 col, int16 row, int16 *x, int16 *y);
// 验证并设置标定矩阵、赛道宽/采样步长（毫米）和有效行范围；成功1，失败0并禁用几何转换。
uint8 search_set_calibration(const float *matrix, float lane_width_mm,
                            float sample_step_mm, int16 min_row, int16 max_row);
#endif
