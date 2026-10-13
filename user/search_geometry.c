/* O7 is reserved for image/geometry algorithms; peripherals default to O3. */
#if defined(__C251__)
#pragma OPTIMIZE(7,SPEED)
#endif

#include "search_geometry.h"
#include "search_calibration.h"

SearchPoint ground_path[SEARCH_PATH_CAPACITY];
uint8 ground_path_count, ground_calibrated;
uint8 ground_projection_rejected;
uint8 ground_center_line[SEARCH_IMAGE_H], ground_center_valid[SEARCH_IMAGE_H];
static SearchPoint raw_points[SEARCH_IMAGE_H], samples[SEARCH_PATH_CAPACITY];
static float forward_map[9], inverse_map[9], half_width, sample_step;
static int16 calibrated_min_row, calibrated_max_row;
static uint8 previous_side;

static float magnitude(float v) { return v < 0.0f ? -v : v; }
static uint8 finite_value(float v) { return v == v && v < 1.0e20f && v > -1.0e20f; }

static uint8 transform(const float *h, float x, float y, float *ox, float *oy)
{
    float divisor;
    divisor = h[6] * x + h[7] * y + h[8];
    if (magnitude(divisor) < 0.000001f) return 0;
    *ox = (h[0] * x + h[1] * y + h[2]) / divisor;
    *oy = (h[3] * x + h[4] * y + h[5]) / divisor;
    return finite_value(*ox) && finite_value(*oy);
}

uint8 search_set_calibration(const float *h, float width, float step,
                            int16 min_row, int16 max_row)
{
    float determinant, x, y, x2, y2;
    int16 i;
    ground_calibrated = ground_path_count = ground_projection_rejected = 0;
    memset(ground_center_valid, 0, sizeof(ground_center_valid));
    if (!h || !finite_value(width) || !finite_value(step) ||
        width < 20.0f || width > 2000.0f || step < 5.0f || step > 200.0f ||
        min_row < STOPROW || max_row >= SEARCH_IMAGE_H || max_row - min_row < 8) return 0;
    for (i = 0; i < 9; ++i) if (!finite_value(h[i])) return 0;
    determinant = h[0]*(h[4]*h[8]-h[5]*h[7]) - h[1]*(h[3]*h[8]-h[5]*h[6]) + h[2]*(h[3]*h[7]-h[4]*h[6]);
    if (!finite_value(determinant) || magnitude(determinant) < 0.000001f) return 0;
    inverse_map[0] = (h[4]*h[8]-h[5]*h[7])/determinant;
    inverse_map[1] = (h[2]*h[7]-h[1]*h[8])/determinant;
    inverse_map[2] = (h[1]*h[5]-h[2]*h[4])/determinant;
    inverse_map[3] = (h[5]*h[6]-h[3]*h[8])/determinant;
    inverse_map[4] = (h[0]*h[8]-h[2]*h[6])/determinant;
    inverse_map[5] = (h[2]*h[3]-h[0]*h[5])/determinant;
    inverse_map[6] = (h[3]*h[7]-h[4]*h[6])/determinant;
    inverse_map[7] = (h[1]*h[6]-h[0]*h[7])/determinant;
    inverse_map[8] = (h[0]*h[4]-h[1]*h[3])/determinant;
    /* Reject reversed axes and a projective horizon inside the configured ROI. */
    for (i = min_row; i <= max_row; ++i) {
        if ((h[6]*0 + h[7]*i + h[8]) *
            (h[6]*(SEARCH_IMAGE_W-1) + h[7]*i + h[8]) <= 0.0f) return 0;
        if (!transform(h, SEARCH_IMAGE_W/2, (float)i, &x, &y) ||
            !transform(h, SEARCH_IMAGE_W/2+1, (float)i, &x2, &y2) || x2 <= x ||
            magnitude(x) > 10000.0f || magnitude(y) > 10000.0f) return 0;
        if (i < max_row) {
            if (!transform(h, SEARCH_IMAGE_W/2, (float)(i+1), &x2, &y2) || y2 >= y) return 0;
        }
    }
    for (i = 0; i < 9; ++i) forward_map[i] = h[i];
    half_width = width * 0.5f;
    sample_step = step;
    calibrated_min_row = min_row;
    calibrated_max_row = max_row;
    previous_side = 0;
    ground_calibrated = 1;
    return 1;
}

void search_ground_init(void)
{
#if SEARCH_CALIBRATED
    static const float calibration[9] = SEARCH_IMAGE_TO_GROUND;
#endif
    ground_calibrated = ground_path_count = ground_projection_rejected = previous_side = 0;
    memset(ground_center_valid, 0, sizeof(ground_center_valid));
#if SEARCH_CALIBRATED
    search_set_calibration(calibration, SEARCH_LANE_WIDTH_MM, SEARCH_SAMPLE_STEP_MM,
                           SEARCH_CALIBRATION_MIN_ROW, SEARCH_CALIBRATION_MAX_ROW);
#endif
}

/* Use a connected near segment. Never bridge a missing boundary in this module. */
int16 search_ground_near_row(void)
{
    return ground_calibrated ? calibrated_max_row : SEARCH_IMAGE_H - 1;
}

uint8 search_project_point(int16 col, int16 row, int16 *x, int16 *y)
{
    float px, py;
    if (!ground_calibrated || col < 0 || col >= SEARCH_IMAGE_W ||
        row < calibrated_min_row || row > calibrated_max_row ||
        !transform(forward_map, (float)col, (float)row, &px, &py) ||
        magnitude(px) > 10000.0f || magnitude(py) > 10000.0f) return 0;
    *x = (int16)(px + (px < 0.0f ? -0.5f : 0.5f));
    *y = (int16)(py + (py < 0.0f ? -0.5f : 0.5f));
    return 1;
}

static int16 segment(uint8 side, int16 *start)
{
    int16 row, count, previous_col, col;
    uint8 valid;
    count = 0;
    previous_col = 0;
    *start = -1;
    for (row = calibrated_max_row; row >= calibrated_min_row; --row) {
        valid = side ? right_valid[row] : left_valid[row];
        col = side ? right_edge_line[row] : left_edge_line[row];
        if (!valid) { if (count) break; else continue; }
        if (*start < 0) {
            if (row < SEARCH_IMAGE_H / 2) break;
            *start = row;
        }
        if (count && magnitude((float)(col - previous_col)) > SEARCHRANGE) break;
        previous_col = col;
        ++count;
    }
    return count;
}

static int16 resample(int16 count)
{
    int16 i, n;
    float dx, dy, length, distance, x, y;
    n = 0;
    distance = 0.0f;
    for (i = 0; i < count - 1 && n < SEARCH_PATH_CAPACITY; ++i) {
        dx = (float)raw_points[i+1].x - raw_points[i].x;
        dy = (float)raw_points[i+1].y - raw_points[i].y;
        length = (float)sqrt(dx*dx + dy*dy);
        if (length < 0.01f) continue;
        while (distance <= length && n < SEARCH_PATH_CAPACITY) {
            x = raw_points[i].x + dx * distance / length;
            y = raw_points[i].y + dy * distance / length;
            samples[n].x = (int16)x;
            samples[n].y = (int16)y;
            ++n;
            distance += sample_step;
        }
        distance -= length;
    }
    return n;
}

static void project_boundary(uint8 side, int16 count, int16 start)
{
    int16 i, row, col, a, b, previous_row, previous_col, j, observed, difference;
    int32 total_difference;
    float x, y, dx, dy, length, u, v, turn;
    ground_path_count = 0;
    ground_projection_rejected = 0;
    memset(ground_center_valid, 0, sizeof(ground_center_valid));
    for (i = 0; i < count; ++i) {
        row = start - i;
        col = side ? right_edge_line[row] : left_edge_line[row];
        if (!transform(forward_map, (float)col, (float)row, &x, &y) ||
            magnitude(x) > 10000.0f || magnitude(y) > 10000.0f) break;
        raw_points[i].x = (int16)x;
        raw_points[i].y = (int16)y;
    }
    count = i;
    /* Mild causal smoothing; use wide arithmetic on a 16-bit target. */
    for (i = 1; i < count; ++i) {
        raw_points[i].x = (int16)(((int32)raw_points[i-1].x + 3L*raw_points[i].x)/4);
        raw_points[i].y = (int16)(((int32)raw_points[i-1].y + 3L*raw_points[i].y)/4);
    }
    count = resample(count);
    if (count < 3) return;
    previous_row = -1;
    previous_col = 0;
    for (i = 0; i < count; ++i) {
        a = i > 1 ? i-2 : 0;
        b = i+2 < count ? i+2 : count-1;
        dx = (float)samples[b].x - samples[a].x;
        dy = (float)samples[b].y - samples[a].y;
        length = (float)sqrt(dx*dx + dy*dy);
        if (length < 1.0f) { previous_row = -1; continue; }
        /* Stop at a sharp corner: offsetting a junction edge is unsafe geometry. */
        if (i > 0 && i < count-1) {
            turn = ((float)samples[i].x-samples[i-1].x)*((float)samples[i+1].x-samples[i].x) +
                   ((float)samples[i].y-samples[i-1].y)*((float)samples[i+1].y-samples[i].y);
            if (turn < sample_step*sample_step*0.5f) break;
        }
        x = samples[i].x + (side ? -1.0f : 1.0f) * half_width * dy/length;
        y = samples[i].y - (side ? -1.0f : 1.0f) * half_width * dx/length;
        if (magnitude(x) > 10000.0f || magnitude(y) > 10000.0f) break;
        ground_path[ground_path_count].x = (int16)x;
        ground_path[ground_path_count].y = (int16)y;
        ++ground_path_count;
        if (!transform(inverse_map, x, y, &u, &v) || u < 0.0f || u > SEARCH_IMAGE_W-1 ||
            v < calibrated_min_row || v > calibrated_max_row) { previous_row = -1; continue; }
        row = (int16)(v+0.5f);
        col = (int16)(u+0.5f);
        /* Row-array adapter only covers locally forward, visible segments. */
        if (previous_row >= 0 && row > previous_row) break;
        if (previous_row > row && previous_row-row <= SEARCH_IMAGE_H/8) {
            for (j = row; j <= previous_row; ++j) {
                ground_center_line[j] = (uint8)(col + (int32)(previous_col-col)*(j-row)/(previous_row-row));
                ground_center_valid[j] = 1;
            }
        }
        ground_center_line[row] = (uint8)col;
        ground_center_valid[row] = 1;
        previous_row = row;
        previous_col = col;
    }
    /* The control output uses paired-row centres where both edges exist.
       Reject a geometric continuation if it would make a large discontinuity
       with that observed path. This is a conservative consistency check,
       not an automatic recalibration or a proof of metric accuracy. */
    observed = 0;
    total_difference = 0;
    for (row = calibrated_min_row; row <= calibrated_max_row; ++row) {
        if (!ground_center_valid[row] || !left_valid[row] || !right_valid[row]) continue;
        if (cross_single_side &&
            (left_valid[row] != EDGE_MEASURED || right_valid[row] != EDGE_MEASURED)) continue;
        difference = (int16)ground_center_line[row] -
            ((int16)left_edge_line[row] + right_edge_line[row]) / 2;
        total_difference += difference < 0 ? -difference : difference;
        ++observed;
    }
    if (observed >= 6 && total_difference > (int32)observed * SEARCHRANGE) {
        ground_projection_rejected = 1;
        ground_path_count = 0;
        memset(ground_center_valid, 0, sizeof(ground_center_valid));
    }
}

/* Count a fresh connected near path, including the observed paired centres.
 * A long edge beyond the island is not a useful near driving path. */
static int16 near_path_score(void)
{
    int16 row, col, previous_col, run, best;
    uint8 valid;
    previous_col = run = best = 0;
    for (row = calibrated_max_row; row >= calibrated_min_row && row >= SEARCH_IMAGE_H / 2; --row) {
        valid = 0;
        col = 0;
        if (left_valid[row] && right_valid[row] &&
            (int16)right_edge_line[row] - left_edge_line[row] >= SEARCH_MIN_WIDTH) {
            col = ((int16)right_edge_line[row] + left_edge_line[row]) / 2;
            valid = 1;
        } else if (ground_center_valid[row]) {
            col = ground_center_line[row];
            valid = 1;
        }
        if (!valid) { run = 0; continue; }
        if (run && magnitude((float)(col - previous_col)) > SEARCHRANGE) run = 0;
        ++run;
        if (run > best) best = run;
        previous_col = col;
    }
    return best;
}

void search_ground_update(void)
{
    int16 left_count, right_count, left_start, right_start, count, start, score;
    uint8 side;
    ground_path_count = ground_projection_rejected = 0;
    memset(ground_center_valid, 0, sizeof(ground_center_valid));
    if (!ground_calibrated) return;
    /* A verified one-sided cross may use only its repaired boundary. Do not
     * switch to the unconfirmed opposite edge or feed it into side history. */
    if (cross_single_side) {
        side = cross_single_side-1;
        count = segment(side, &start);
        if (count >= 6) project_boundary(side, count, start);
        return;
    }
    if (cross_flag || cross_candidate) return;
    left_count = segment(0, &left_start);
    right_count = segment(1, &right_start);
    side = previous_side;
    if (left_count > right_count + 6 || right_count < 6) side = 0;
    if (right_count > left_count + 6 || left_count < 6) side = 1;
    count = side ? right_count : left_count;
    start = side ? right_start : left_start;
    if (count < 6) return;
    project_boundary(side, count, start);
    score = near_path_score();
    /* Only retry when the chosen side fails to provide a usable near path. */
    count = side ? left_count : right_count;
    start = side ? left_start : right_start;
    if (score < SEARCH_IMAGE_H / 10 && count >= 6) {
        project_boundary(!side, count, start);
        if (near_path_score() > score) side = !side;
        else project_boundary(side, side ? right_count : left_count,
            side ? right_start : left_start);
    }
    previous_side = side;
}
