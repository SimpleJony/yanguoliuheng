#ifndef SEARCH_LINE_H_
#define SEARCH_LINE_H_
#include "zf_common_headfile.h"
#include "search_calibration.h"

#define SEARCH_IMAGE_W MT9V03X_W
#define SEARCH_IMAGE_H MT9V03X_H
#define BLACKPOINT 50
#define WHITEMAXMUL 13
#define WHITEMINMUL 7
#define REFERENCEROW 5
#define SEARCHRANGE 10
#define SEARCH_SEED_PREDICT_STEP 20
/* Fill only tiny current-frame breaks with real edges on BOTH ends. */
#define SEARCH_EDGE_MAX_GAP 2
#define EDGE_MEASURED 1
#define EDGE_BRIDGED 2
#define EDGE_CROSS 3
#define SEARCH_EDGE_MIN_RUN 3
/* Detection includes the distant exit of the new cross frames. Metric control
 * remains restricted to SEARCH_CALIBRATION_MIN_ROW..MAX_ROW. */
#define STOPROW 12
#define SEARCH_EDGE_MARGIN 3
#define CONTRASTOFFSET 3
#define REFERENCE_VERTICAL_RANGE 20
#define SEARCH_MIN_DIFF 8
#define SEARCH_MIN_WIDTH 12
#define SEARCH_HISTORY_FRAMES 3
/* Consecutive-wide-row cross detector. Search currently visits every row. */
#define SEARCH_ROW_STEP 1
#define CROSS_WIDE_THRESHOLD 180
#define CROSS_CONSEC_ENTER 5
#define CROSS_CONSEC_EXIT 3
#define CROSS_CONSEC_HALF 20
#define CROSS_JUMP_GAP 6
#define CROSS_JUMP_PIXELS 10
/* Two-sided mode needs a current measured road adjoining the opening. */
#define CROSS_SUPPORT_ROWS 3
#define CROSS_SUPPORT_WINDOW 36
/* One-sided entry needs a continuous measured entry AND same-side exit. */
#define CROSS_SINGLE_SUPPORT_ROWS 7

#define CENTER_NONE 0
#define CENTER_MEASURED 1
#define CENTER_GROUND 2
#define CENTER_CROSS 3
#define CENTER_HISTORY 4
#define CENTER_BRIDGED 5

extern uint8 reference_point, reference_col, white_max_point, white_min_point;
extern uint8 reference_contrast_ratio;
extern uint8 reference_col_line[SEARCH_IMAGE_H], remote_diatance[SEARCH_IMAGE_W];
/* Missing edges remain at 0 / W-1. EDGE_BRIDGED joins tiny current gaps;
 * EDGE_CROSS marks boundaries rewritten by cross connection/extension. */
extern uint8 left_edge_line[SEARCH_IMAGE_H], right_edge_line[SEARCH_IMAGE_H];
extern uint8 left_valid[SEARCH_IMAGE_H], right_valid[SEARCH_IMAGE_H];
extern uint8 center_line[SEARCH_IMAGE_H], center_valid[SEARCH_IMAGE_H];
/* quality: 0 absent, 1 old prediction, 2 geometric/bridge, 3 observed pair. */
extern uint8 center_source[SEARCH_IMAGE_H], center_quality[SEARCH_IMAGE_H];
extern uint8 cross_flag, cross_candidate, path_lost;
/* 0: ordinary/two-sided; 1: trusted left boundary; 2: trusted right boundary. */
extern uint8 cross_single_side;
extern uint8 cross_half_flag;
extern uint8 cross_bridge_extrapolated;
/* Compatibility diagnostic: this algorithm has no far extension (always 0). */
extern uint8 cross_far_extrapolated;
extern int16 cross_near_row, cross_far_row;

void search_init(void);
void search_process(const uint8 *image);
void get_reference_point(const uint8 *image);
void search_reference_col(const uint8 *image);
void search_line(const uint8 *image);
void cross_analysis(void);
void get_center_line(void);
#endif
