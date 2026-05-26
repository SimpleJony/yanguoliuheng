#ifndef SEARCH_LINE_H_
#define SEARCH_LINE_H_
#include "zf_common_headfile.h"

#define SEARCH_IMAGE_W MT9V03X_W
#define SEARCH_IMAGE_H MT9V03X_H
#define BLACKPOINT                  50
#define WHITEMAXMUL 13
#define WHITEMINMUL 7



#define REFERENCEROW 5   //参考点统计行数
#define SEARCHRANGE 10   //搜线半径
#define STOPROW 3       //搜线停止行
#define CONTRASTOFFSET 3    //搜线对比偏移

extern uint8 reference_point;   //参考点
extern uint8 reference_col;     //参考列
extern uint8 white_max_point;   //白点最大
extern uint8 white_min_point;   //白点最小
extern uint8 reference_contrast_ratio;//参考对比度
extern uint8 reference_col_line[SEARCH_IMAGE_H];//参考列数组
extern uint8 remote_diatance[SEARCH_IMAGE_W];//白点远端距离
extern uint8 left_edge_line[SEARCH_IMAGE_H];//左右边界
extern uint8 right_edge_line[SEARCH_IMAGE_H];
extern uint32 if_count;

void get_reference_point(const uint8 *image);

void search_reference_col(const uint8 *image);

void search_line(const uint8 *image);
#endif
