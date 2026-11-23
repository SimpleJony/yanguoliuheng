// 图像与地面几何转换：使用相机标定把边界投影为毫米点，沿边界法线内移生成参考路径。
// SearchPoint.x 向右为正、y 向前为正，原点为标定原点；不是图像 Point 的行列坐标。
// 环内正常巡线同样使用本模块的单边法线后备路径；仅入/出环强制引导阶段跳过。
#include "search_geometry.h"
#include "search_calibration.h"

// ============================================
// 全局变量与内部接口
// ============================================
static SearchPoint ground_path[SEARCH_PATH_CAPACITY];  // 从近到远的毫米路径，仅前 ground_path_count 个点有效
static uint8 ground_path_count;                        // 本帧毫米路径点数
uint8 ground_calibrated;                        // 标定矩阵和参数是否通过验证
uint8 ground_projection_rejected;               // 几何中线与实测双边中点偏差过大，已清空本帧投影
uint8 ground_center_line[SEARCH_IMAGE_H];       // 毫米路径反投影后的逐行中线列坐标
uint8 ground_center_valid[SEARCH_IMAGE_H];      // 各行几何中线有效标记
static SearchPoint raw_points[SEARCH_IMAGE_H];  // 边界投影和平滑的毫米点工作缓冲
static SearchPoint samples[SEARCH_PATH_CAPACITY]; // 按固定毫米间距重采样的工作缓冲
static float forward_map[9];                    // 图像到地面的 3×3 矩阵，按行排列
static float inverse_map[9];                    // 地面到图像的 3×3 逆矩阵，按行排列
static float half_width;                        // 半赛道宽，毫米
static float sample_step;                       // 地面路径采样间距，毫米
static int16 calibrated_min_row;                // 标定有效区域最远行，含端点
static int16 calibrated_max_row;                // 标定有效区域最近行，含端点
static uint8 previous_side;                     // 普通巡线上一帧选边：0 左、1 右，用于选边滞回
static uint8 projection_cache_col[SEARCH_IMAGE_H]; // 各行最近一次成功投影的列；255 表示没有缓存
static SearchPoint projection_cache[SEARCH_IMAGE_H]; // 同一标定下的整数毫米投影，供瞄点选择、跟踪及速度预瞄复用
/* Keep truncated edge projections separate from rounded control targets.
 * One slot per side/row prevents left -> right -> left fallback thrashing.
 * Keys are calibration + side + row + column, not frame identity. */
static uint8 boundary_cache_col[2][SEARCH_IMAGE_H];
static SearchPoint boundary_cache[2][SEARCH_IMAGE_H];
/* Ordinary fallback may try the other edge and then keep the first one.
 * Save only on that retry. Replaying the first edge would overwrite its own
 * prefix/rows but leave the other edge's unused tail/rows in the public arrays.
 * Keep that exact write footprint, including points/rows later rejected. */
static SearchPoint fallback_path[SEARCH_PATH_CAPACITY];
static uint8 fallback_center_line[SEARCH_IMAGE_H];
static uint8 fallback_center_written[(SEARCH_IMAGE_H + 7) / 8];
static uint8 rejected_center_writes[SEARCH_IMAGE_H];
// ============================================
// 返回浮点数 v 的绝对值，用于矩阵分母和坐标范围检查。
// ============================================
static float magnitude(float v) { return v < 0.0f ? -v : v; }
// ============================================
// 判断 v 是否为可接受的有限数值；排除 NaN、无穷大和过大的数值。
// ============================================
static uint8 finite_value(float v) { return v == v && v < 1.0e20f && v > -1.0e20f; }

// ============================================
// 使用 3×3 矩阵 h 做齐次坐标变换：输入 x/y，输出到 ox/oy；分母 divisor 过小或结果异常返回 0。
// ============================================
static uint8 transform(const float *h, float x, float y, float *ox, float *oy)
{
    float divisor, inv_divisor;                 // 齐次投影分母，过小则拒绝变换
    divisor = h[6] * x + h[7] * y + h[8];
    if (magnitude(divisor) < 0.000001f) return 0;
    inv_divisor = 1.0f / divisor;
    *ox = (h[0] * x + h[1] * y + h[2]) * inv_divisor;
    *oy = (h[3] * x + h[4] * y + h[5]) * inv_divisor;
    return finite_value(*ox) && finite_value(*oy);
}

// ============================================
// 校验并载入图像到地面矩阵 h、赛道宽 width（mm）、采样步长 step（mm）及行范围 min_row..max_row。
// 成功返回 1；失败清除标定有效标志，并停止生成地面路径。
// ============================================
uint8 search_set_calibration(const float *h, float width, float step,
                            int16 min_row, int16 max_row)
{
    float determinant, x, y, x2, y2;            // determinant：行列式；x/y、x2/y2：相邻图像点投影坐标，用于检查方向
    int16 i;                                    // 矩阵元素或标定行索引
    ground_calibrated = ground_path_count = ground_projection_rejected = 0;
    memset(ground_center_valid, 0, sizeof(ground_center_valid));
    // 包括校验失败的情况，都不能继续使用旧标定生成的缓存。
    memset(projection_cache_col, 255, sizeof(projection_cache_col));
    memset(boundary_cache_col, 255, sizeof(boundary_cache_col));
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
    // 拒绝坐标轴反向或投影分母在标定区域内过零的矩阵。
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

// ============================================
// 清空地面路径状态，并根据 SEARCH_CALIBRATED 加载 search_calibration.h 中的 calibration 矩阵。
// ============================================
void search_ground_init(void)
{
#if SEARCH_CALIBRATED
    static const float calibration[9] = SEARCH_IMAGE_TO_GROUND; // 默认图像到地面的 3×3 标定矩阵，按行排列
#endif
    ground_calibrated = ground_path_count = ground_projection_rejected = previous_side = 0;
    memset(ground_center_valid, 0, sizeof(ground_center_valid));
    memset(projection_cache_col, 255, sizeof(projection_cache_col));
    memset(boundary_cache_col, 255, sizeof(boundary_cache_col));
#if SEARCH_CALIBRATED
    search_set_calibration(calibration, SEARCH_LANE_WIDTH_MM, SEARCH_SAMPLE_STEP_MM,
                           SEARCH_CALIBRATION_MIN_ROW, SEARCH_CALIBRATION_MAX_ROW);
#endif
}

// ============================================
// 返回最近的标定有效行；尚无有效标定时返回图像底行，供调用方确定搜索起点。
// ============================================
int16 search_ground_near_row(void)
{
    return ground_calibrated ? calibrated_max_row : SEARCH_IMAGE_H - 1;
}

// ============================================
// 将图像列 col、行 row 转为地面毫米坐标并写入 x/y；px/py 为转换后的浮点坐标，输出四舍五入为 int16。
// 只接受标定区域内的图像点，成功返回 1，失败返回 0。
// 同一行、列在标定不变时直接复用已舍入的结果，避免选瞄点、转向和速度预瞄重复做浮点投影。
// ============================================
uint8 search_project_point(int16 col, int16 row, int16 *x, int16 *y)
{
    float px, py;                               // 投影后的地面毫米坐标，四舍五入后输出
    if (!ground_calibrated || col < 0 || col >= SEARCH_IMAGE_W ||
        row < calibrated_min_row || row > calibrated_max_row) return 0;
    if (projection_cache_col[row] == (uint8)col) {
        *x = projection_cache[row].x;
        *y = projection_cache[row].y;
        return 1;
    }
    if (!transform(forward_map, (float)col, (float)row, &px, &py) ||
        magnitude(px) > 10000.0f || magnitude(py) > 10000.0f) return 0;
    *x = (int16)(px + (px < 0.0f ? -0.5f : 0.5f));
    *y = (int16)(py + (py < 0.0f ? -0.5f : 0.5f));
    projection_cache[row].x = *x;
    projection_cache[row].y = *y;
    projection_cache_col[row] = (uint8)col;
    return 1;
}

// ============================================
// 从近到远取 side（0 左、1 右）的第一段连续边界，返回行数并输出起始行 start，未找到时 start=-1。
// measured_only=1 时仅允许离开图像边缘的实测点；row/col 为当前行列，count 为段长，previous_col 为上一列，valid 为
// 来源标记。
// ============================================
static int16 segment(uint8 side, int16 *start, uint8 measured_only)
{
    int16 row, count, previous_col, col, delta;        // row：扫描行；count：连续行数；previous_col：上一列；col：当前列
    uint8 valid;                                // 当前行边界来源或无效标记
    count = 0;
    previous_col = 0;
    *start = -1;
    for (row = calibrated_max_row; row >= calibrated_min_row; --row) {
        valid = side ? right_valid[row] : left_valid[row];
        col = side ? right_edge_line[row] : left_edge_line[row];
        if (measured_only && (valid != EDGE_MEASURED || col <= RING_EDGE_MARGIN ||
            col >= SEARCH_IMAGE_W - 1 - RING_EDGE_MARGIN)) valid = 0;
        if (!valid) { if (count) break; else continue; }
        if (*start < 0) {
            if (row < SEARCH_IMAGE_H / 2) break;
            *start = row;
        }
        delta = col - previous_col;
        if (count && (delta > SEARCHRANGE || delta < -SEARCHRANGE)) break;
        previous_col = col;
        ++count;
    }
    return count;
}

// ============================================
// 将 raw_points 的 count 个点按 sample_step 毫米间距重采样到 samples，返回实际采样点数。
// ============================================
static int16 resample(int16 count)
{
    int16 i, n;                                 // i：原始线段索引；n：输出采样点数
    // dx/dy：线段向量；length：段长；distance：下一采样点距段起点距离；x/y：插值坐标，单位 mm
    float dx, dy, length, distance, x, y, inv_length, ratio;
    n = 0;
    distance = 0.0f;
    for (i = 0; i < count - 1 && n < SEARCH_PATH_CAPACITY; ++i) {
        dx = (float)raw_points[i+1].x - raw_points[i].x;
        dy = (float)raw_points[i+1].y - raw_points[i].y;
        length = (float)sqrt(dx*dx + dy*dy);
        if (length < 0.01f) continue;
        /* Most short segments emit no sample: do not divide on those. */
        if (distance <= length) {
            inv_length = 1.0f / length;
            while (distance <= length && n < SEARCH_PATH_CAPACITY) {
                ratio = distance * inv_length;
                x = raw_points[i].x + dx * ratio;
                y = raw_points[i].y + dy * ratio;
                samples[n].x = (int16)x;
                samples[n].y = (int16)y;
                ++n;
                distance += sample_step;
            }
        }
        distance -= length;
    }
    return n;
}

// ============================================
// 把 side 侧从 start 起的 count 行边界投影、平滑并重采样；复用 raw_points/samples，返回采样数。
// ============================================
static int16 boundary_samples(uint8 side, int16 count, int16 start)
{
    int16 i, row, col;                          // i：点索引；row/col：图像行列
    float x, y;                                 // 投影后的地面坐标，毫米
    for (i = 0; i < count; ++i) {
        row = start - i;
        col = side ? right_edge_line[row] : left_edge_line[row];
        if (boundary_cache_col[side][row] != (uint8)col) {
            if (!transform(forward_map, (float)col, (float)row, &x, &y) ||
                magnitude(x) > 10000.0f || magnitude(y) > 10000.0f) break;
            boundary_cache[side][row].x = (int16)x;
            boundary_cache[side][row].y = (int16)y;
            boundary_cache_col[side][row] = (uint8)col;
        }
        raw_points[i] = boundary_cache[side][row];
    }
    count = i;
    // 用前一点作轻度平滑；先升为 32 位，避免 C251 的 16 位整数求和溢出。
    for (i = 1; i < count; ++i) {
        raw_points[i].x = (int16)(((int32)raw_points[i-1].x + 3L*raw_points[i].x)/4);
        raw_points[i].y = (int16)(((int32)raw_points[i-1].y + 3L*raw_points[i].y)/4);
    }
    return resample(count);
}

// ============================================
// 将 side 侧 count 行边界生成毫米法线偏移路径，再将可见部分适配回逐行图像中线。
// 返回实际写入的路径前缀长度；即使最后拒绝投影，也保留写入范围供普通选边回退恢复。
// ============================================
static uint8 project_boundary(uint8 side, int16 count, int16 start)
{
    // i：点索引；row/col：图像坐标；a/b：切线邻点；previous_row/previous_col：前一投影点；j：插值行；observed：双
    // 边支持行数；difference：列偏差
    int16 i, row, col, a, b, previous_row, previous_col, j, observed, difference;
    int32 total_difference;                     // 几何中线与双边中点的累计绝对列偏差
    uint8 written_count;
    // x/y：偏移后毫米坐标；dx/dy：切线；length：切线长度；u/v：反投影列行；turn：邻段点积；offset：内移距离 mm
    float x, y, dx, dy, length, u, v, turn, signed_offset, turn_limit, scale;
    ground_path_count = 0;
    ground_projection_rejected = 0;
    memset(ground_center_valid, 0, sizeof(ground_center_valid));
    count = boundary_samples(side, count, start);
    signed_offset = side ? -half_width : half_width;
    turn_limit = sample_step * sample_step * 0.5f;
    if (count < 3) return 0;
    previous_row = -1;
    previous_col = 0;
    for (i = 0; i < count; ++i) {
        a = i > 1 ? i-2 : 0;
        b = i+2 < count ? i+2 : count-1;
        dx = (float)samples[b].x - samples[a].x;
        dy = (float)samples[b].y - samples[a].y;
        length = (float)sqrt(dx*dx + dy*dy);
        if (length < 1.0f) { previous_row = -1; continue; }
        // 遇到急转角即截断，避免对岔路边界继续作法线偏移。
        if (i > 0 && i < count-1) {
            turn = ((float)samples[i].x-samples[i-1].x)*((float)samples[i+1].x-samples[i].x) +
                   ((float)samples[i].y-samples[i-1].y)*((float)samples[i+1].y-samples[i].y);
            if (turn < turn_limit) break;
        }
        scale = signed_offset / length;
        x = samples[i].x + dy * scale;
        y = samples[i].y - dx * scale;
        if (magnitude(x) > 10000.0f || magnitude(y) > 10000.0f) break;
        ground_path[ground_path_count].x = (int16)x;
        ground_path[ground_path_count].y = (int16)y;
        ++ground_path_count;
        if (!transform(inverse_map, x, y, &u, &v) || u < 0.0f || u > SEARCH_IMAGE_W-1 ||
            v < calibrated_min_row || v > calibrated_max_row) { previous_row = -1; continue; }
        row = (int16)(v+0.5f);
        col = (int16)(u+0.5f);
        // 逐行中线仅连接可见且向远处延伸的局部线段。
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
    // 双边可见时，控制优先使用实测中点。
    // 几何路径与这些中点偏差过大则丢弃本帧投影，避免路径跳变；此检查不会重新标定。
    if (ring_state == RING_STATE_APPROACH || ring_follow_outer ||
        ring_state == RING_STATE_EXIT_FOLLOW) return ground_path_count; // 不被对侧环岛的中点拉回
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
    written_count = ground_path_count;
    if (observed >= 6 && total_difference > (int32)observed * SEARCHRANGE) {
        // 被拒绝的投影仍写过公开数组；回退时不能把写入范围误当成空。
        memcpy(rejected_center_writes, ground_center_valid, sizeof(rejected_center_writes));
        ground_projection_rejected = 1;
        ground_path_count = 0;
        memset(ground_center_valid, 0, sizeof(ground_center_valid));
    }
    return written_count;
}

// ============================================
// 仅统计近端连续路径，岛尖另一侧的远端长边不能代替可行驶的近端路径。
// 统计近处连续可用中线的最长行数，为普通巡线选择边界提供评分；同时考虑实测双边与几何中线。
// ============================================
static int16 near_path_score(void)
{
    int16 row, col, previous_col, run, best, delta;    // row/col：当前行列；previous_col：上一列；run/best：当前/最大连续长度
    uint8 valid;                                // 本行是否有可信路径
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
        delta = col - previous_col;
        if (run && (delta > SEARCHRANGE || delta < -SEARCHRANGE)) run = 0;
        ++run;
        if (run > best) best = run;
        previous_col = col;
    }
    return best;
}

// ============================================
// 判断近端首段实测双边中线是否足够长，仅做整数检查，不连接缺口或补线。
// ============================================
static uint8 measured_midline_long_enough(void)
{
    int16 row, count = 0, col, previous_col = 0, delta;
    for (row = calibrated_max_row; row >= calibrated_min_row; --row) {
        if (left_valid[row] != EDGE_MEASURED || right_valid[row] != EDGE_MEASURED ||
            (int16)right_edge_line[row] - left_edge_line[row] < SEARCH_MIN_WIDTH) {
            if (count) break; /* 不把分离的实测段累加，也不跨越补线。 */
            continue;
        }
        if (!count && row < calibrated_max_row - SEARCH_GROUND_PAIRED_START_GAP) break;
        col = ((int16)left_edge_line[row] + right_edge_line[row]) / 2;
        delta = col - previous_col;
        if (count && (delta > SEARCHRANGE || delta < -SEARCHRANGE)) break;
        previous_col = col;
        if (++count >= SEARCH_GROUND_PAIRED_MIN_ROWS) return 1;
    }
    return 0;
}

// 清空本帧几何输出；特殊元素优先，普通巡线实测段不足时才生成单边毫米路径。
void search_ground_update(void)
{
    // left_count/right_count：两侧连续行数；left_start/right_start：起始行；count/start：当前尝试段；score：近端评
    // 分
    int16 left_count, right_count, left_start, right_start, count, start, score, row;
    uint8 side;                                 // 当前选边，0 左、1 右
    uint8 written_count, saved_count, saved_rejected, written;
    ground_path_count = ground_projection_rejected = 0;
    memset(ground_center_valid, 0, sizeof(ground_center_valid));
    // 入环/横向转弯引导使用图像中线；出环直线补边则正常生成毫米法线路径。
    if (ring_state == RING_STATE_STAGE1 || ring_state == RING_STATE_STAGE2 ||
        ring_exit_guiding) return;
    if (!ground_calibrated) return;
    if (ring_state == RING_STATE_APPROACH || ring_follow_outer ||
        ring_state == RING_STATE_EXIT_FOLLOW) {
        // 本帧已确认的前方直线补边标为EDGE_RING，允许其参与投影。
        // 未找到直线时只用本帧实测左边，不使用历史线或右侧环岛。
        count = segment(0, &start, ring_exit_straight ? 0 : 1);
        if (count >= 6) project_boundary(0, count, start);
        return;
    }
    // 单侧十字只使用已确认的补线侧，不改跟未确认对侧，也不更新普通选边历史。
    if (cross_single_side) {
        side = cross_single_side-1;
        count = segment(side, &start, 0);
        if (count >= 6) project_boundary(side, count, start);
        return;
    }
    if (cross_flag || cross_candidate) return;
    // 近端连续实测双边已足够形成中线时，不再计算单边后备路径。
    // 只检查本帧边界，不能读取尚未更新的 center_quality；入口已清除旧几何结果。
    if (measured_midline_long_enough()) return;
    left_count = segment(0, &left_start, 0);
    right_count = segment(1, &right_start, 0);
    side = previous_side;
    if (left_count > right_count + 6 || right_count < 6) side = 0;
    if (right_count > left_count + 6 || left_count < 6) side = 1;
    count = side ? right_count : left_count;
    start = side ? right_start : left_start;
    if (count < 6) return;
    written_count = project_boundary(side, count, start);
    score = near_path_score();
    // 仅当所选侧无法生成足够近端路径时，尝试另一侧。
    count = side ? left_count : right_count;
    start = side ? left_start : right_start;
    if (score < SEARCH_IMAGE_H / 10 && count >= 6) {
        saved_count = ground_path_count;
        saved_rejected = ground_projection_rejected;
        memcpy(fallback_path, ground_path, (uint16)written_count * sizeof(SearchPoint));
        memcpy(fallback_center_line, ground_center_line, sizeof(fallback_center_line));
        memset(fallback_center_written, 0, sizeof(fallback_center_written));
        for (row = 0; row < SEARCH_IMAGE_H; ++row) {
            written = saved_rejected ? rejected_center_writes[row] : ground_center_valid[row];
            if (written) fallback_center_written[row >> 3] |= (uint8)(1U << (row & 7));
        }
        project_boundary(!side, count, start);
        if (near_path_score() > score) side = !side;
        else {
            // 只重放主侧写过的内容，保留另一侧留下的未使用尾部和无效行。
            memcpy(ground_path, fallback_path, (uint16)written_count * sizeof(SearchPoint));
            for (row = 0; row < SEARCH_IMAGE_H; ++row) {
                written = (uint8)((fallback_center_written[row >> 3] >> (row & 7)) & 1U);
                if (written) ground_center_line[row] = fallback_center_line[row];
                ground_center_valid[row] = saved_rejected ? 0 : written;
            }
            ground_path_count = saved_count;
            ground_projection_rejected = saved_rejected;
        }
    }
    previous_side = side;
}
