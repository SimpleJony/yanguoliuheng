#ifndef SEARCH_CALIBRATION_H_
#define SEARCH_CALIBRATION_H_
/* 2026-09-23: origin = near stripe midpoint; X right, Y forward, mm.
 * Lane inner width 450; stripe center spacing 200.
 * Rear axle (0,-500), wheelbase 200, aligned heading; see run_config.h.
 * Below the near stripe is longitudinal extrapolation. Keep camera pose fixed. */
#define SEARCH_CALIBRATED 1
#define SEARCH_LANE_WIDTH_MM 4.500000000e+02f
#define SEARCH_SAMPLE_STEP_MM 2.000000000e+01f
#define SEARCH_CALIBRATION_MIN_ROW 38
#define SEARCH_CALIBRATION_MAX_ROW 100
#define SEARCH_IMAGE_TO_GROUND {1.123589148e+01f, 8.111383628e-01f, -1.169605647e+03f, 3.454604930e-01f, -1.660680364e+01f, 1.008630998e+03f, -8.695492485e-04f, 3.484409477e-02f, 1.000000000e+00f}
#endif
