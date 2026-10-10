#ifndef SEARCH_GEOMETRY_H_
#define SEARCH_GEOMETRY_H_
#include "search_line.h"
#define SEARCH_PATH_CAPACITY 100
/* Ground coordinates: x right, y forward, millimetres; camera pixels: u col, v row. */
typedef struct { int16 x; int16 y; } SearchPoint;
extern SearchPoint ground_path[SEARCH_PATH_CAPACITY];
extern uint8 ground_path_count, ground_calibrated;
/* Current projection disagrees with observed paired-row centres. */
extern uint8 ground_projection_rejected;
extern uint8 ground_center_line[SEARCH_IMAGE_H], ground_center_valid[SEARCH_IMAGE_H];
void search_ground_init(void);
void search_ground_update(void);
/* Nearest calibrated row; image bottom when geometry is disabled. */
int16 search_ground_near_row(void);
/* Validated image point -> calibration origin, millimetres. No extrapolation
 * beyond the configured image ROI and no fallback to pixel coordinates. */
uint8 search_project_point(int16 col, int16 row, int16 *x, int16 *y);
/* Startup only. A failed calibration disables geometry. No default/fake matrix. */
uint8 search_set_calibration(const float *matrix, float lane_width_mm,
                            float sample_step_mm, int16 min_row, int16 max_row);
#endif
