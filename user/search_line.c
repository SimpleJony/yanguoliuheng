/* O7 is reserved for image/geometry algorithms; peripherals default to O3. */
#if defined(__C251__)
#pragma OPTIMIZE(7,SPEED)
#endif

#include "search_line.h"
#include "search_geometry.h"

uint8 reference_point, reference_col, white_max_point, white_min_point;
uint8 reference_contrast_ratio = 20;
uint8 reference_col_line[SEARCH_IMAGE_H], remote_diatance[SEARCH_IMAGE_W];
uint8 left_edge_line[SEARCH_IMAGE_H], right_edge_line[SEARCH_IMAGE_H];
uint8 left_valid[SEARCH_IMAGE_H], right_valid[SEARCH_IMAGE_H];
uint8 center_line[SEARCH_IMAGE_H], center_valid[SEARCH_IMAGE_H];
uint8 center_source[SEARCH_IMAGE_H], center_quality[SEARCH_IMAGE_H];
uint8 cross_flag, cross_candidate, path_lost;
uint8 cross_single_side;
uint8 cross_half_flag;
uint8 cross_bridge_extrapolated;
uint8 cross_far_extrapolated;
int16 cross_near_row = -1, cross_far_row = -1;
static uint8 history_center[SEARCH_IMAGE_H], history_age[SEARCH_IMAGE_H];
static uint8 row_has_white_seed[SEARCH_IMAGE_H];

static int16 limit_col(int16 col)
{
    if (col < CONTRASTOFFSET) return CONTRASTOFFSET;
    if (col > SEARCH_IMAGE_W - 1 - CONTRASTOFFSET)
        return SEARCH_IMAGE_W - 1 - CONTRASTOFFSET;
    return col;
}

static int16 abs16(int16 value) { return value < 0 ? -value : value; }

/* Promote before multiplying: target compiler has 16-bit int. */
static uint8 edge_transition(uint8 inside, uint8 outside)
{
    int16 difference;
    uint16 sum;
    difference = (int16)inside - outside;
    sum = (uint16)inside + outside;
    if (!sum || difference < SEARCH_MIN_DIFF || inside < white_min_point)
        return 0;
    return (int32)difference * 200 > (int32)reference_contrast_ratio * sum;
}

void search_init(void)
{
    int16 row;
    reference_col = SEARCH_IMAGE_W / 2;
    cross_flag = cross_candidate = cross_half_flag = 0;
    cross_single_side = 0;
    cross_bridge_extrapolated = cross_far_extrapolated = 0;
    cross_near_row = cross_far_row = -1;
    path_lost = 1;
    for (row = 0; row < SEARCH_IMAGE_H; ++row) {
        left_edge_line[row] = 0;
        right_edge_line[row] = SEARCH_IMAGE_W - 1;
        left_valid[row] = right_valid[row] = 0;
        row_has_white_seed[row] = 0;
        center_line[row] = history_center[row] = SEARCH_IMAGE_W / 2;
        history_age[row] = SEARCH_HISTORY_FRAMES + 1;
        center_source[row] = center_quality[row] = center_valid[row] = 0;
    }
    search_ground_init();
}

void get_reference_point(const uint8 *image)
{
    uint16 i, count;
    uint32 sum, value;
    const uint8 *pixels;
    count = REFERENCEROW * SEARCH_IMAGE_W;
    pixels = image + (SEARCH_IMAGE_H - REFERENCEROW) * SEARCH_IMAGE_W;
    sum = 0;
    for (i = 0; i < count; ++i) sum += pixels[i];
    reference_point = (uint8)(sum / count);
    value = (uint32)reference_point * WHITEMAXMUL / 10;
    white_max_point = (uint8)(value > 255 ? 255 : (value < BLACKPOINT ? BLACKPOINT : value));
    value = (uint32)reference_point * WHITEMINMUL / 10;
    white_min_point = (uint8)(value > 255 ? 255 : (value < BLACKPOINT ? BLACKPOINT : value));
}

void search_reference_col(const uint8 *image)
{
    int16 col, row, best_row, best_col, old_col;
    uint8 a, b;
    old_col = limit_col(reference_col);
    best_col = old_col;
    best_row = SEARCH_IMAGE_H;
    for (col = 0; col < SEARCH_IMAGE_W; ++col)
        remote_diatance[col] = SEARCH_IMAGE_H - 1;
    /* Continuous bottom-up white reach; ties prefer the preceding reference. */
    for (col = CONTRASTOFFSET; col < SEARCH_IMAGE_W - CONTRASTOFFSET; col += 2) {
        for (row = SEARCH_IMAGE_H - 1; row >= STOPROW + CONTRASTOFFSET; --row) {
            a = image[row * SEARCH_IMAGE_W + col];
            b = image[(row - CONTRASTOFFSET) * SEARCH_IMAGE_W + col];
            if (a < white_min_point || edge_transition(a, b)) break;
        }
        remote_diatance[col] = (uint8)row;
        if (row < best_row || (row == best_row && abs16(col - old_col) < abs16(best_col - old_col))) {
            best_row = row;
            best_col = col;
        }
    }
    reference_col = (uint8)limit_col(best_col);
    for (row = 0; row < SEARCH_IMAGE_H; ++row)
        reference_col_line[row] = reference_col;
}

static int16 white_seed(const uint8 *pixels, int16 prediction)
{
    int16 offset, col, start, end, candidate, best, distance, best_distance;
    prediction = limit_col(prediction);
    if (pixels[prediction] >= white_min_point) return prediction;
    for (offset = 1; offset <= SEARCHRANGE; ++offset) {
        col = prediction - offset;
        if (col >= CONTRASTOFFSET && pixels[col] >= white_min_point) return col;
        col = prediction + offset;
        if (col <= SEARCH_IMAGE_W - 1 - CONTRASTOFFSET && pixels[col] >= white_min_point) return col;
    }
    /* An island/entry may shift the white run beyond the local window.
     * Reacquire the nearest sufficiently wide CURRENT white run. */
    best = -1;
    best_distance = SEARCH_IMAGE_W;
    col = CONTRASTOFFSET;
    while (col < SEARCH_IMAGE_W - CONTRASTOFFSET) {
        if (pixels[col] < white_min_point) { ++col; continue; }
        start = col;
        while (col < SEARCH_IMAGE_W - CONTRASTOFFSET && pixels[col] >= white_min_point) ++col;
        end = col - 1;
        if (end - start + 1 < SEARCH_MIN_WIDTH) continue;
        candidate = prediction < start + 1 ? start + 1 :
            (prediction > end - 1 ? end - 1 : prediction);
        distance = abs16(candidate - prediction);
        if (distance < best_distance) { best_distance = distance; best = candidate; }
    }
    return best;
}

/* Only connect short gaps whose two observed ends and white interior agree.
 * All-black rows, long openings and scene jumps remain missing. */
static void bridge_edge_gaps(const uint8 *image, uint8 *edge, uint8 *valid, uint8 side)
{
    int16 row, near_row, far_row, gap, j, col, inside, delta;
    uint8 accepted;
    near_row = -1;
    for (row = SEARCH_IMAGE_H - 1; row >= STOPROW; --row) {
        if (!valid[row]) continue;
        if (near_row >= 0) {
            far_row = row;
            gap = near_row - far_row - 1;
            delta = (int16)edge[near_row] - edge[far_row];
            if (gap > 0 && gap <= SEARCH_EDGE_MAX_GAP &&
                abs16(delta) <= SEARCHRANGE * (gap + 1)) {
                accepted = 1;
                for (j = far_row + 1; j < near_row; ++j) {
                    col = edge[far_row] + (int32)delta * (j - far_row) / (near_row - far_row);
                    inside = col + (side ? -CONTRASTOFFSET : CONTRASTOFFSET);
                    if (!row_has_white_seed[j] || inside < 0 || inside >= SEARCH_IMAGE_W ||
                        image[j * SEARCH_IMAGE_W + inside] < white_min_point) {
                        accepted = 0;
                        break;
                    }
                }
                if (accepted) {
                    for (j = far_row + 1; j < near_row; ++j) {
                        edge[j] = (uint8)(edge[far_row] +
                            (int32)delta * (j - far_row) / (near_row - far_row));
                        valid[j] = EDGE_BRIDGED;
                    }
                }
            }
        }
        near_row = row;
    }
}

/* Reject isolated high-contrast scenery hits and disconnected 1-2 row tips. */
static void filter_short_edges(uint8 *edge, uint8 *valid, uint8 missing)
{
    int16 row, start, count, previous, j;
    uint8 found;
    start = previous = count = 0;
    for (row = SEARCH_IMAGE_H-1; row >= STOPROW-1; --row) {
        found = row >= STOPROW && valid[row];
        if (count && (!found || abs16((int16)edge[row]-previous) > SEARCHRANGE)) {
            if (count < SEARCH_EDGE_MIN_RUN) {
                for (j = start; j > start-count; --j) {
                    valid[j] = 0;
                    edge[j] = missing;
                }
            }
            count = 0;
        }
        if (found) {
            if (!count) start = row;
            ++count;
            previous = edge[row];
        }
    }
}

void search_line(const uint8 *image)
{
    int16 row, col, seed, prediction, delta;
    const uint8 *pixels;
    prediction = reference_col;
    for (row = 0; row < SEARCH_IMAGE_H; ++row) {
        left_edge_line[row] = 0;
        right_edge_line[row] = SEARCH_IMAGE_W - 1;
        left_valid[row] = right_valid[row] = 0;
        row_has_white_seed[row] = 0;
    }
    /* No permanent stop on a black row: distant exit is searched independently. */
    for (row = SEARCH_IMAGE_H - 1; row >= STOPROW; --row) {
        pixels = image + row * SEARCH_IMAGE_W;
        seed = white_seed(pixels, prediction);
        if (seed < 0) continue;
        row_has_white_seed[row] = 1;
        reference_col_line[row] = (uint8)seed;
        for (col = seed; col >= CONTRASTOFFSET + SEARCH_EDGE_MARGIN; --col) {
            if (edge_transition(pixels[col], pixels[col - CONTRASTOFFSET])) {
                left_edge_line[row] = (uint8)(col - CONTRASTOFFSET);
                left_valid[row] = EDGE_MEASURED;
                break;
            }
            if (pixels[col] < white_min_point) break;
        }
        for (col = seed; col <= SEARCH_IMAGE_W - 1 - CONTRASTOFFSET - SEARCH_EDGE_MARGIN; ++col) {
            if (edge_transition(pixels[col], pixels[col + CONTRASTOFFSET])) {
                right_edge_line[row] = (uint8)(col + CONTRASTOFFSET);
                right_valid[row] = EDGE_MEASURED;
                break;
            }
            if (pixels[col] < white_min_point) break;
        }
        /* Follow a visible edge even after its partner leaves the frame. */
        prediction = seed;
        if (row < SEARCH_IMAGE_H - 1) {
            delta = 0;
            if (left_valid[row] && left_valid[row + 1])
                delta = (int16)left_edge_line[row] - left_edge_line[row + 1];
            else if (right_valid[row] && right_valid[row + 1])
                delta = (int16)right_edge_line[row] - right_edge_line[row + 1];
            prediction += func_limit(delta, SEARCH_SEED_PREDICT_STEP);
        }
        if (left_valid[row] && right_valid[row]) {
            if ((int16)right_edge_line[row] - left_edge_line[row] < SEARCH_MIN_WIDTH) {
                left_valid[row] = right_valid[row] = 0;
            } else prediction = ((int16)left_edge_line[row] + right_edge_line[row]) / 2;
        }
    }
    bridge_edge_gaps(image, left_edge_line, left_valid, 0);
    bridge_edge_gaps(image, right_edge_line, right_valid, 1);
    filter_short_edges(left_edge_line, left_valid, 0);
    filter_short_edges(right_edge_line, right_valid, SEARCH_IMAGE_W-1);
}

static uint8 pair_valid(int16 row)
{
    return left_valid[row] && right_valid[row] &&
        (int16)right_edge_line[row] - left_edge_line[row] >= SEARCH_MIN_WIDTH;
}

static int16 pair_center(int16 row)
{
    return ((int16)left_edge_line[row] + right_edge_line[row]) / 2;
}

/* Compare six rows apart, as in the supplied implementation's original
 * helpers. Return the endpoint closer to the image centre, or -1 if absent.
 * Rows above STOPROW are unsearched in this project and cannot be anchors. */
static int16 find_jump_point(const uint8 *edge, const uint8 *valid, int16 num0, int16 num1,
                             int16 jump_num, uint8 model)
{
    int16 i, a, b, point;
    if (num0 >= SEARCH_IMAGE_H) num0 = SEARCH_IMAGE_H-1;
    if (num1 < STOPROW) num1 = STOPROW;
    if (num0-num1 < CROSS_JUMP_GAP) return -1;
    for (i = 0; i <= num0-num1-CROSS_JUMP_GAP; ++i) {
        a = model ? num0-i : num1+i;
        b = model ? a-CROSS_JUMP_GAP : a+CROSS_JUMP_GAP;
        if (abs16((int16)edge[a]-edge[b]) <= jump_num) continue;
        point = abs16((int16)edge[a]-SEARCH_IMAGE_W/2) <
                abs16((int16)edge[b]-SEARCH_IMAGE_W/2) ? a : b;
        if (valid[point] == EDGE_MEASURED || valid[point] == EDGE_BRIDGED) return point;
    }
    return -1;
}

static void connect_point(uint8 *edge, uint8 *valid, int16 num0, int16 num1)
{
    int16 row, near_col, far_col;
    if (num0 >= SEARCH_IMAGE_H || num1 < STOPROW || num0 <= num1 ||
        !valid[num0] || !valid[num1]) return;
    near_col = edge[num0];
    far_col = edge[num1];
    for (row = num0-1; row > num1; --row) {
        edge[row] = (uint8)(near_col +
            (int32)(far_col-near_col)*(num0-row)/(num0-num1));
        valid[row] = EDGE_CROSS;
    }
}

/* Signed indices prevent start_point-10 wrapping into an array overrun. */
static void extend_line_downward(uint8 *edge, uint8 *valid, int16 num0, int16 num1, int16 extend_to)
{
    int16 row, near_col, far_col, value;
    if (num1 < STOPROW || num0 <= num1 || num0 >= extend_to ||
        extend_to >= SEARCH_IMAGE_H || !valid[num0] || !valid[num1]) return;
    near_col = edge[num0];
    far_col = edge[num1];
    for (row = num0+1; row <= extend_to; ++row) {
        value = (int16)(near_col +
            (int32)(near_col-far_col)*(row-num0)/(num0-num1));
        if (value < 0 || value >= SEARCH_IMAGE_W) break;
        edge[row] = (uint8)value;
        valid[row] = EDGE_CROSS;
        cross_bridge_extrapolated = 1;
    }
}

static uint8 cross_wide_row(int16 row)
{
    /* Both missing edges default to 0/W-1 even on a black frame. Only a
     * current white-road seed makes that full width meaningful. */
    return row_has_white_seed[row] && right_edge_line[row] > left_edge_line[row] &&
        (uint16)(right_edge_line[row]-left_edge_line[row]) > CROSS_WIDE_THRESHOLD;
}

static uint8 cross_measured_pair_run(int16 row, int16 direction, int16 count)
{
    int16 i, previous;
    previous = row;
    for (i = 0; i < count; ++i, row += direction) {
        if (row < STOPROW || row >= SEARCH_IMAGE_H ||
            left_valid[row] != EDGE_MEASURED || right_valid[row] != EDGE_MEASURED ||
            !pair_valid(row) || cross_wide_row(row)) return 0;
        if (i && (abs16((int16)left_edge_line[row]-left_edge_line[previous]) > SEARCHRANGE ||
                  abs16((int16)right_edge_line[row]-right_edge_line[previous]) > SEARCHRANGE)) return 0;
        previous = row;
    }
    return 1;
}

/* Losing both edges in a tight bend also produces full-width rows. A new
 * intersection needs a measured entry below that SAME wide segment. */
static uint8 cross_entry_supported(int16 wide_near)
{
    int16 row;
    for (row = wide_near+1; row < SEARCH_IMAGE_H && row <= wide_near+CROSS_SUPPORT_WINDOW; ++row) {
        if (!row_has_white_seed[row] || cross_wide_row(row)) break;
        if (cross_measured_pair_run(row, 1, CROSS_SUPPORT_ROWS) &&
            (int16)left_edge_line[row]-left_edge_line[wide_near] > CROSS_JUMP_PIXELS &&
            (int16)right_edge_line[wide_near]-right_edge_line[row] > CROSS_JUMP_PIXELS) return 1;
    }
    return 0;
}

/* A single entry is ambiguous with an edge leaving the frame on a bend.
 * Require the SAME boundary to reappear beyond this opening, with compatible
 * entry/exit directions. Never use interpolated or historical anchors. */
static int16 cross_single_anchor(const uint8 *edge, const uint8 *valid,
                                int16 wide_row, int16 direction)
{
    int16 row, j, point;
    for (row = wide_row+direction;
         row >= STOPROW && row < SEARCH_IMAGE_H && abs16(row-wide_row) <= CROSS_SUPPORT_WINDOW;
         row += direction) {
        if (!row_has_white_seed[row] || cross_wide_row(row)) break;
        if (abs16((int16)edge[row]-edge[wide_row]) <= CROSS_JUMP_PIXELS) continue;
        for (j = 0; j < CROSS_SINGLE_SUPPORT_ROWS; ++j) {
            point = row+direction*j;
            if (point < STOPROW || point >= SEARCH_IMAGE_H ||
                !row_has_white_seed[point] || cross_wide_row(point) ||
                valid[point] != EDGE_MEASURED ||
                (j && abs16((int16)edge[point]-edge[point-direction]) > SEARCHRANGE)) break;
        }
        if (j == CROSS_SINGLE_SUPPORT_ROWS) return row;
    }
    return -1;
}

static uint8 cross_connect_single(uint8 *edge, uint8 *valid, uint8 side,
                                  int16 wide_near, int16 wide_far)
{
    int16 near_row, far_row, span, delta, row;
    int32 error;
    near_row = cross_single_anchor(edge, valid, wide_near, 1);
    far_row = cross_single_anchor(edge, valid, wide_far, -1);
    if (near_row < 0 || far_row < 0) return 0;
    span = near_row-far_row;
    delta = (int16)edge[near_row]-edge[far_row];
    error = (int32)delta*(CROSS_SINGLE_SUPPORT_ROWS-1) -
        (int32)((int16)edge[near_row+CROSS_SINGLE_SUPPORT_ROWS-1]-edge[near_row])*span;
    if (error < 0) error = -error;
    if (error > (int32)CROSS_JUMP_PIXELS*(CROSS_SINGLE_SUPPORT_ROWS-1)) return 0;
    error = (int32)delta*(CROSS_SINGLE_SUPPORT_ROWS-1) -
        (int32)((int16)edge[far_row]-edge[far_row-CROSS_SINGLE_SUPPORT_ROWS+1])*span;
    if (error < 0) error = -error;
    if (error > (int32)CROSS_JUMP_PIXELS*(CROSS_SINGLE_SUPPORT_ROWS-1)) return 0;
    for (row = far_row+1; row < near_row; ++row)
        if (!row_has_white_seed[row]) return 0;
    connect_point(edge, valid, near_row, far_row);
    cross_single_side = side;
    cross_candidate = 1;
    return 1;
}

static uint8 cross_single_analysis(void)
{
    int16 row, near_row, count;
    count = 0;
    near_row = -1;
    for (row = SEARCH_IMAGE_H-1; row >= STOPROW-1; --row) {
        if (row >= STOPROW && cross_wide_row(row)) {
            if (!count) near_row = row;
            ++count;
        } else {
            if (count >= (cross_flag ? CROSS_CONSEC_EXIT : CROSS_CONSEC_ENTER) &&
                (cross_connect_single(left_edge_line, left_valid, 1, near_row, row+1) ||
                 cross_connect_single(right_edge_line, right_valid, 2, near_row, row+1))) return 1;
            count = 0;
        }
    }
    return 0;
}

/* Keep the current validity/quality API consistent with rewritten boundaries. */
static void cross_finish_lines(void)
{
    int16 row;
    for (row = STOPROW; row < SEARCH_IMAGE_H; ++row) {
        if (left_valid[row] && right_valid[row] &&
            (int16)right_edge_line[row]-left_edge_line[row] < SEARCH_MIN_WIDTH) {
            if (left_valid[row] == EDGE_CROSS) { left_valid[row] = 0; left_edge_line[row] = 0; }
            if (right_valid[row] == EDGE_CROSS) { right_valid[row] = 0; right_edge_line[row] = SEARCH_IMAGE_W-1; }
        }
        if (left_valid[row] == EDGE_CROSS || right_valid[row] == EDGE_CROSS) {
            if (cross_far_row < 0) cross_far_row = row;
            cross_near_row = row;
        }
    }
    if (cross_near_row < 0) cross_bridge_extrapolated = 0;
}

/* Consecutive wide rows: bottom-half extension first, then entry/connection
 * and the supplied post-repair exit scan. No frame debounce or cooldown. */
void cross_analysis(void)
{
    int16 i, start_i, start_point, end_point, wide_near, left_start, right_start, support;
    uint8 wide_count;
    cross_candidate = cross_half_flag = 0;
    cross_single_side = 0;
    cross_bridge_extrapolated = cross_far_extrapolated = 0;
    cross_near_row = cross_far_row = -1;
    wide_count = 0;
    wide_near = -1;
    start_i = SEARCH_IMAGE_H-1;
    for (i = start_i; i >= STOPROW; i -= SEARCH_ROW_STEP) {
        if (cross_wide_row(i)) {
            if (++wide_count >= CROSS_CONSEC_HALF) { cross_half_flag = 1; break; }
        } else break;
    }
    if (cross_half_flag) {
        left_start = find_jump_point(left_edge_line, left_valid, SEARCH_IMAGE_H-1, 3, CROSS_JUMP_PIXELS, 1);
        right_start = find_jump_point(right_edge_line, right_valid, SEARCH_IMAGE_H-1, 3, CROSS_JUMP_PIXELS, 1);
        support = (left_start < right_start ? left_start : right_start)-4;
        if (left_start >= STOPROW+10 && right_start >= STOPROW+10 &&
            abs16(left_start-right_start) <= CROSS_SUPPORT_WINDOW &&
            cross_measured_pair_run(support, -1, 7) &&
            left_valid[left_start-4] == EDGE_MEASURED && left_valid[left_start-10] == EDGE_MEASURED &&
            right_valid[right_start-4] == EDGE_MEASURED && right_valid[right_start-10] == EDGE_MEASURED) {
            cross_candidate = 1;
            extend_line_downward(left_edge_line, left_valid, left_start-4, left_start-10, SEARCH_IMAGE_H-1);
            extend_line_downward(right_edge_line, right_valid, right_start-4, right_start-10, SEARCH_IMAGE_H-1);
        } else cross_flag = 0;
        cross_half_flag = 0;
        cross_finish_lines();
        return;
    }
    wide_count = 0;
    for (i = start_i; i >= STOPROW; i -= SEARCH_ROW_STEP) {
        if (cross_wide_row(i)) {
            if (!wide_count) wide_near = i;
            if (++wide_count == (cross_flag ? CROSS_CONSEC_EXIT : CROSS_CONSEC_ENTER) &&
                cross_entry_supported(wide_near)) { cross_candidate = 1; break; }
        } else wide_count = 0;
    }
    if (!cross_candidate && !cross_single_analysis()) { cross_flag = 0; return; }
    cross_flag = 1;
    if (cross_flag) {
        /* Keep this frame identifiable as a cross even if its repaired width
         * immediately clears cross_flag in the post-repair exit scan. */
        cross_candidate = 1;
        if (!cross_single_side) {
            start_point = find_jump_point(left_edge_line, left_valid, SEARCH_IMAGE_H-1, 3, CROSS_JUMP_PIXELS, 1);
            end_point = find_jump_point(left_edge_line, left_valid, SEARCH_IMAGE_H-1, 3, CROSS_JUMP_PIXELS, 0);
            if (start_point < SEARCH_IMAGE_H-4 && end_point > 4 && start_point > end_point)
                connect_point(left_edge_line, left_valid, start_point, end_point);
            start_point = find_jump_point(right_edge_line, right_valid, SEARCH_IMAGE_H-1, 3, CROSS_JUMP_PIXELS, 1);
            end_point = find_jump_point(right_edge_line, right_valid, SEARCH_IMAGE_H-1, 3, CROSS_JUMP_PIXELS, 0);
            if (start_point < SEARCH_IMAGE_H-4 && end_point > 4 && start_point > end_point)
                connect_point(right_edge_line, right_valid, start_point, end_point);
        }
        wide_count = 0;
        for (i = start_i; i >= STOPROW; i -= SEARCH_ROW_STEP) {
            if (cross_wide_row(i)) {
                if (++wide_count >= CROSS_CONSEC_EXIT) break;
            } else wide_count = 0;
        }
        if (wide_count < CROSS_CONSEC_EXIT) cross_flag = 0;
    }
    cross_finish_lines();
}

void get_center_line(void)
{
    int16 row, current_rows;
    uint8 source;
    search_ground_update();
    current_rows = 0;
    for (row = 0; row < SEARCH_IMAGE_H; ++row) {
        source = CENTER_NONE;
        center_line[row] = SEARCH_IMAGE_W / 2;
        if (row >= STOPROW) {
            if (pair_valid(row) && (!cross_single_side ||
                (left_valid[row] == EDGE_MEASURED && right_valid[row] == EDGE_MEASURED))) {
                center_line[row] = (uint8)pair_center(row);
                source = left_valid[row] == EDGE_CROSS || right_valid[row] == EDGE_CROSS ?
                    CENTER_CROSS : (left_valid[row] == EDGE_MEASURED && right_valid[row] == EDGE_MEASURED ?
                    CENTER_MEASURED : CENTER_BRIDGED);
            } else if (ground_center_valid[row] &&
                       (cross_single_side || (!cross_flag && !cross_candidate))) {
                center_line[row] = ground_center_line[row];
                source = cross_single_side ? CENTER_CROSS : CENTER_GROUND;
            }
        }
        if (source != CENTER_NONE) {
            history_center[row] = center_line[row];
            history_age[row] = 0;
        } else {
            if (history_age[row] < SEARCH_HISTORY_FRAMES + 1) ++history_age[row];
            if (row >= STOPROW && history_age[row] <= SEARCH_HISTORY_FRAMES) {
                center_line[row] = history_center[row];
                source = CENTER_HISTORY;
            }
        }
        center_source[row] = source;
        center_valid[row] = source != CENTER_NONE;
        center_quality[row] = source == CENTER_MEASURED ? 3 :
            (source == CENTER_HISTORY ? 1 : (source == CENTER_NONE ? 0 : 2));
        /* A single-edge normal offset can move fresh centres upward.
         * Count fresh evidence here; Run separately validates near distance
         * and a connected calibrated segment before commanding steering. */
        if (row >= STOPROW && center_quality[row] >= 2) ++current_rows;
    }
    path_lost = current_rows < SEARCH_IMAGE_H / 10;
}

void search_process(const uint8 *image)
{
    get_reference_point(image);
    search_reference_col(image);
    search_line(image);
    cross_analysis();
    get_center_line();
}
