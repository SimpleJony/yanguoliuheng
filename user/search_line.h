#ifndef SEARCH_LINE_H_
#define SEARCH_LINE_H_

#include "zf_common_headfile.h"

#define SEARCH_IMAGE_W              MT9V03X_W
#define SEARCH_IMAGE_H              MT9V03X_H

#define BLACKPOINT                  50
#define WHITEMAXMUL                 13
#define WHITEMINMUL                 7

#define REFERENCEROW                5
#define SEARCHRANGE                 10
#define STOPROW                     3
#define CONTRASTOFFSET              3
#define REFERENCE_VERTICAL_RANGE    20

extern uint8 reference_point;
extern uint8 reference_col;
extern uint8 white_max_point;
extern uint8 white_min_point;
extern uint8 reference_contrast_ratio;
extern uint8 reference_col_line[SEARCH_IMAGE_H];
extern uint8 remote_diatance[SEARCH_IMAGE_W];
extern uint8 left_edge_line[SEARCH_IMAGE_H];
extern uint8 right_edge_line[SEARCH_IMAGE_H];
extern uint8 center_line[SEARCH_IMAGE_H];
extern uint32 if_count;

uint8 find_extreme_value(uint8 *array_value, uint8 num0, uint8 num1, uint8 model);
uint8 find_jump_point(uint8 *array_value, uint8 num0, uint8 num1, uint8 jump_num, uint8 model);
void connect_point(uint8 *array_value, uint8 num0, uint8 num1);

void get_reference_point(const uint8 *image);
void search_reference_col(const uint8 *image);
void search_line(const uint8 *image);
void cross_analysis(void);
void get_center_line(void);
void search_line_process(const uint8 *image);

#endif
