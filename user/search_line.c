// 当前搜线与圆环主流程：灰度参考 → 白区种子 → 左右边界 → 右圆环识别/补线 → 十字仲裁 → 中线。
// 图像数组以下标表示行，元素值表示列；行号越大越靠近车，列号越大越靠右，单位均为像素。
// 圆环使用本文件的视觉状态机，累计转角不参与阶段判断。
#include "search_line.h"
#include "search_geometry.h"
#include "run_config.h"

// ============================================
// 全局变量与内部接口
// ============================================
static uint8 reference_col;                            // 本帧起搜列
static uint8 white_min_point;                          // 白色判定下阈值，0～255
static uint8 reference_contrast_ratio = 20;            // 边缘对比度阈值：200×亮暗差 > 本值×亮暗和
uint8 left_edge_line[SEARCH_IMAGE_H];           // 左边界列坐标，丢失时为 0，需结合 left_valid 判断
uint8 right_edge_line[SEARCH_IMAGE_H];          // 右边界列坐标，丢失时为 W-1，需结合 right_valid 判断
uint8 left_valid[SEARCH_IMAGE_H];               // 左边界来源，0 无效，其他值见 EDGE_*
uint8 right_valid[SEARCH_IMAGE_H];              // 右边界来源，0 无效，其他值见 EDGE_*
uint8 center_line[SEARCH_IMAGE_H];              // 逐行中线列坐标
uint8 center_valid[SEARCH_IMAGE_H];             // 中线存在标记，包含历史结果，控制还需检查质量
uint8 center_source[SEARCH_IMAGE_H];            // 中线来源，见 CENTER_*
uint8 center_quality[SEARCH_IMAGE_H];           // 中线质量：0 无效、1 历史、2 几何或补线、3 实测双边
uint8 cross_flag;                               // 当前十字状态
uint8 cross_candidate;                          // 本帧十字证据，补线后仍保留用于中线仲裁
uint8 path_lost;                                // 当帧质量至少为 2 的中线不足图像高度的十分之一
uint8 cross_single_side;                        // 单侧十字可信边：0 无指定、1 左、2 右
static uint8 cross_half_flag;                          // 本帧底部连续大宽度标记
uint8 ring_candidate;                           // 本帧入口几何候选，与 IMU 是否就绪无关
uint8 ring_brake_hint;                          // 远端右开口预警，仅用于提前减速
uint8 ring_dir;                                 // 当前方向标志，沿实测外边出环时为 NONE
static uint8 ring_confirm_count;                       // 入口特征连续确认帧数
uint8 ring_state;                               // 实际启用的圆环状态，判断是否在环中应检查它
uint8 ring_follow_outer;                       // 沿左外边或前方直线补线出环，不受右侧环岛影响
uint8 ring_exit_straight;                      // 本帧前方直线拟合补线有效，不作为实测边界
uint8 ring_exit_ready;                         // 当前帧已有可信出口道路；速度由独立的出环延时控制
uint8 ring_exit_guiding;                       // 本帧横向外边界支持的转弯引导
static uint8 history_center[SEARCH_IMAGE_H];    // 各行最近一次中线，历史结果不作为新实测证据
static uint8 history_age[SEARCH_IMAGE_H];       // 各行距上次有效中线更新的帧数
static uint8 row_has_white_seed[SEARCH_IMAGE_H]; // 本行是否找到白区种子，用于区分真实开口和黑图默认边界
/* Pixel-checked gap proposals, not recognition evidence. 255 means absent.
 * Keep only 240 bytes of proposals instead of another full camera frame. */
static uint8 left_gap_col[SEARCH_IMAGE_H], right_gap_col[SEARCH_IMAGE_H];
static uint8 edges_pending;
static uint16 ring_frame_ticks;                // 本帧处理开始时的 5ms 节拍，来自整帧入口
static uint8 ring_entry_recover_count;
static uint16 ring_start_tick, ring_active_last_tick;
static uint16 ring_rearm_tick;
static uint8 ring_rearm_active, ring_wall_visible, ring_align_count;
static uint8 ring_exit_confirm_count, ring_exit_recover_count;
static uint16 ring_exit_start_tick;
static uint8 ring_exit_clear_count;
static int16 ring_exit_line_top, ring_exit_line_bottom, ring_exit_line_mid;
static int16 ring_exit_line_last_col;
static int32 ring_exit_line_mid_q8, ring_exit_line_slope_q8, ring_exit_line_last_slope_q8;
static uint16 ring_approach_tick, ring_approach_last_tick;
static uint8 ring_approach_count;
static uint16 ring_confirm_last_tick;
static uint16 ring_brake_tick;
static uint8 edge_difference_limit[511];      // 按两像素灰度和 0～510 索引的对比度门限，最多存 255
static uint8 edge_cached_ratio;               // 已生成门限表所用的 reference_contrast_ratio
static uint8 edge_limit_ready;                // 门限表是否已生成，首次使用或对比度参数变化时更新
uint8 zebra_candidate, zebra_detected;          // 本帧条纹候选及连续确认结果
static uint8 zebra_confirm_count;                      // 连续命中帧数，最多计到确认门槛
static uint16 zebra_last_tick;                  // 上次候选的 5ms 节拍，用于断帧后重新确认
static void ring_analysis(void);
static void ring_reset(void);
static void ring_connect(uint8 *edge, uint8 *valid, int16 num0, int16 num1);
static uint8 ring_observe_wall(const uint8 *image);

// ============================================
// 限制列坐标到允许比较左右灰度的范围，防止访问 col±CONTRASTOFFSET 时越界；返回裁剪后的列号。
// ============================================
static int16 limit_col(int16 col)
{
    if (col < CONTRASTOFFSET) return CONTRASTOFFSET;
    if (col > SEARCH_IMAGE_W - 1 - CONTRASTOFFSET)
        return SEARCH_IMAGE_W - 1 - CONTRASTOFFSET;
    return col;
}

// ============================================
// 求有符号 16 位图像差值的绝对值，用于行列差和连续性比较。
// ============================================
static int16 abs16(int16 value) { return value < 0 ? -value : value; }

// ============================================
// 更新对比度门限表：200×亮暗差 > ratio×灰度和 等价于 亮暗差 > floor(ratio×灰度和/200)。
// 用商和余数累加生成精确整数门限，只在首次使用或 ratio 改变时计算；逐像素判断不再做 32 位乘法。
// 门限超过 255 时保存 255，因实际亮暗差不超过 255，此时仍然必定拒绝。
// ============================================
static void prepare_edge_difference_limit(void)
{
    uint16 sum, limit, remainder;              // 灰度和索引、当前整数门限、累加余数
    uint8 step, remainder_step;               // ratio/200 的商与余数，用于递推下一项
    if(edge_limit_ready && edge_cached_ratio == reference_contrast_ratio) return;
    edge_cached_ratio = reference_contrast_ratio;
    step = edge_cached_ratio / 200U;
    remainder_step = edge_cached_ratio % 200U;
    limit = remainder = 0;
    for(sum = 0; sum <= 510U; ++sum)
    {
        edge_difference_limit[sum] = (uint8)(limit > 255U ? 255U : limit);
        limit += step;
        remainder += remainder_step;
        if(remainder >= 200U)
        {
            remainder -= 200U;
            ++limit;
        }
    }
    edge_limit_ready = 1;
}

/* Call only after inside >= white_min_point. SEARCH_MIN_DIFF > 0 also
 * rejects sum==0, including test callers using a zero white threshold.
 * Arguments must be local pixel values (no side effects); C251 has no inline. */
#define EDGE_TRANSITION_WHITE(inside, outside) \
    (((int16)(inside) - (int16)(outside) >= SEARCH_MIN_DIFF) && \
     ((int16)(inside) - (int16)(outside) > edge_difference_limit[(uint16)(inside) + (outside)]))

// ============================================
// 重置搜线数组、历史、十字、斑马线和当前右圆环状态，并载入地面标定。
// 只初始化视觉状态；圆环流程不读取或清零 IMU 累计转角。
// ============================================
void search_init(void)
{
    int16 row;                                  // 逐行清空的行索引
    zebra_candidate = zebra_detected = zebra_confirm_count = 0;

    zebra_last_tick = 0;
    ring_reset();
    ring_frame_ticks = 0;
    edges_pending = 0;
    ring_wall_visible = ring_rearm_active = 0;
    ring_rearm_tick = 0;
    ring_approach_count = 0;
    ring_approach_tick = ring_approach_last_tick = ring_confirm_last_tick = 0;
    reference_col = SEARCH_IMAGE_W / 2;
    cross_flag = cross_candidate = cross_half_flag = 0;
    cross_single_side = 0;

    ring_candidate = ring_dir = RING_DIR_NONE;
    ring_confirm_count = 0;

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

// ============================================
// 从 image 底部 REFERENCEROW 行计算平均灰度，更新白色阈值；image 为按行连续存储的灰度图。
// ============================================
static void get_reference_point(const uint8 *image)
{
    uint8 reference_point;
    uint16 i, row, row_sum, count;              // 单行最大 188*255=47940，先用16位求和
    uint32 sum, value;                          // sum：灰度和；value：限幅前白色阈值
    const uint8 *pixels;                        // 底部参考区首地址
    count = REFERENCEROW * SEARCH_IMAGE_W;
    pixels = image + (SEARCH_IMAGE_H - REFERENCEROW) * SEARCH_IMAGE_W;
    sum = 0;
    for (row = 0; row < REFERENCEROW; ++row) {
        row_sum = 0;
        for (i = 0; i < SEARCH_IMAGE_W; ++i) row_sum += *pixels++;
        sum += row_sum;
    }
    reference_point = (uint8)(sum / count);
    value = (uint32)reference_point * WHITEMINMUL / 10;
    white_min_point = (uint8)(value > 255 ? 255 : (value < BLACKPOINT ? BLACKPOINT : value));
}

// ============================================
// 逐列寻找从图像底部延伸最远的白区，确定起搜列；延伸距离相同时优先靠近上一帧起搜列。
// ============================================
static void search_reference_col(const uint8 *image)
{
    int16 col, row, best_row, best_col, old_col, distance, best_distance;
    int16 left_col, right_col;
    uint8 a, b;
    const uint8 *pixel, *outside;
    prepare_edge_difference_limit();
    old_col = limit_col(reference_col);
    best_col = old_col;
    best_row = SEARCH_IMAGE_H;
    best_distance = SEARCH_IMAGE_W;
    /* Visit the original column grid by (distance, column): nearest first,
     * left first for ties. The previous column need not be on this grid.
     * A fully open nearest column then finishes after one vertical scan. */
    left_col = CONTRASTOFFSET + (old_col - CONTRASTOFFSET) / 2 * 2;
    right_col = left_col + 2;
    while (left_col >= CONTRASTOFFSET || right_col < SEARCH_IMAGE_W - CONTRASTOFFSET) {
        if (left_col >= CONTRASTOFFSET &&
            (right_col >= SEARCH_IMAGE_W - CONTRASTOFFSET ||
             old_col - left_col <= right_col - old_col)) {
            col = left_col;
            left_col -= 2;
            distance = old_col - col;
        } else {
            col = right_col;
            right_col += 2;
            distance = col - old_col;
        }
        if (distance >= best_distance) {
            /* Remaining columns cannot improve the sentinel or its tie. */
            if (best_row < STOPROW + CONTRASTOFFSET) break;
            /* A black pixel at best_row permits a tie. Prune only when this
             * column cannot win that tie; never index the initial H sentinel. */
            if (best_row < SEARCH_IMAGE_H && image[best_row * SEARCH_IMAGE_W + col] < white_min_point)
                continue;
        }
        pixel = image + (SEARCH_IMAGE_H - 1) * SEARCH_IMAGE_W + col;
        outside = pixel - CONTRASTOFFSET * SEARCH_IMAGE_W;
        for (row = SEARCH_IMAGE_H - 1; row >= STOPROW + CONTRASTOFFSET;
             --row, pixel -= SEARCH_IMAGE_W, outside -= SEARCH_IMAGE_W) {
            a = *pixel;
            if (a < white_min_point) break;
            b = *outside;
            if (EDGE_TRANSITION_WHITE(a, b)) break;
        }
        if (row < best_row || (row == best_row &&
            (distance < best_distance || (distance == best_distance && col < best_col)))) {
            best_row = row;
            best_col = col;
            best_distance = distance;
        }
    }
    reference_col = (uint8)limit_col(best_col);

}

// ============================================
// 在单行 pixels 中寻找靠近 prediction 的白色起点；先在局部窗口搜索，再寻找足够宽的当前白区，失败返回 -1。
// ============================================
static int16 white_seed(const uint8 *pixels, int16 prediction)
{
    // offset：局部偏移；col：扫描列；start/end：白区范围；candidate/best：当前/最佳种子；distance/best_distance：
    // 距预测列的距离
    int16 offset, col, start, end, candidate, best, distance, best_distance;
    prediction = limit_col(prediction);
    if (pixels[prediction] >= white_min_point) return prediction;
    for (offset = 1; offset <= SEARCHRANGE; ++offset) {
        col = prediction - offset;
        if (col >= CONTRASTOFFSET && pixels[col] >= white_min_point) return col;
        col = prediction + offset;
        if (col <= SEARCH_IMAGE_W - 1 - CONTRASTOFFSET && pixels[col] >= white_min_point) return col;
    }
    // 岛尖或入口会使白区移出局部窗口，改为寻找本行最近且足够宽的白区。
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

// ============================================
// 像素仍锁定时检查短缺口，只保存候选列；不改实测边界，避免补线成为圆环识别证据。
// ============================================
static void prepare_edge_gaps(const uint8 *image, const uint8 *edge,
                              const uint8 *valid, uint8 side, uint8 *proposal)
{
    // row：扫描行；near_row/far_row：缺口两端；gap：缺失行数；j：插值行；col/inside：补边列/道路内侧列；delta：两
    // 端列差
    int16 row, near_row, far_row, gap, j, col, inside, delta;
    uint8 accepted;                             // 缺口内部是否全部满足白区与边界条件
    memset(proposal, 255, SEARCH_IMAGE_H);
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
                        proposal[j] = (uint8)(edge[far_row] +
                            (int32)delta * (j - far_row) / (near_row - far_row));
                    }
                }
            }
        }
        near_row = row;
    }
}

/* Apply only after ring analysis has consumed the raw boundaries. A ring
 * repair may replace an anchor or gap; never bridge across that repair. */
static void apply_edge_gaps(uint8 *edge, uint8 *valid, const uint8 *proposal)
{
    int16 row, near_row, far_row, j;
    uint8 accepted;
    row = SEARCH_IMAGE_H - 1;
    while(row >= STOPROW) {
        if(proposal[row] == 255) { --row; continue; }
        near_row = row + 1;
        accepted = 1;
        do {
            if(valid[row]) accepted = 0;
            --row;
        } while(row >= STOPROW && proposal[row] != 255);
        far_row = row;
        if(!accepted || near_row >= SEARCH_IMAGE_H || far_row < STOPROW ||
            valid[near_row] != EDGE_MEASURED || valid[far_row] != EDGE_MEASURED) continue;
        for(j = far_row + 1; j < near_row; ++j) {
            edge[j] = proposal[j];
            valid[j] = EDGE_BRIDGED;
        }
    }
}

// ============================================
// 删除不足 SEARCH_EDGE_MIN_RUN 行的孤立短边，清除 valid 并写入 missing（左 0、右 W-1）。
// ============================================
static void filter_short_edges(uint8 *edge, uint8 *valid, uint8 missing)
{
    // row：扫描行；start：段起点；count：段长；previous：前一边界列；j：清除行
    int16 row, start, count, previous, j;
    uint8 found;                                // 当前行是否有有效边界
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

// ============================================
// 只做读取像素的工作：实测边界、横墙观察、短缺口白区检查。
// ============================================
static void extract_line_pixels(const uint8 *image)
{
    // row/col：当前行列；seed：白区种子；prediction：下一行预测列；delta：边界移动量
    int16 row, col, seed, prediction, delta;
    uint8 inside, outside;
    const uint8 *pixels;                        // 当前图像行的首像素
    prepare_edge_difference_limit();
    prediction = reference_col;
    for (row = 0; row < SEARCH_IMAGE_H; ++row) {
        left_edge_line[row] = 0;
        right_edge_line[row] = SEARCH_IMAGE_W - 1;
        left_valid[row] = right_valid[row] = 0;
        row_has_white_seed[row] = 0;
    }
    // 黑行只结束本行搜索，后续行仍独立寻找远端出口。
    pixels = image + (SEARCH_IMAGE_H - 1) * SEARCH_IMAGE_W;
    for (row = SEARCH_IMAGE_H - 1; row >= STOPROW; --row, pixels -= SEARCH_IMAGE_W) {
        seed = white_seed(pixels, prediction);
        if (seed < 0) continue;
        row_has_white_seed[row] = 1;

        for (col = seed; col >= CONTRASTOFFSET + SEARCH_EDGE_MARGIN; --col) {
            inside = pixels[col];
            if (inside < white_min_point) break;
            outside = pixels[col - CONTRASTOFFSET];
            if (EDGE_TRANSITION_WHITE(inside, outside)) {
                left_edge_line[row] = (uint8)(col - CONTRASTOFFSET);
                left_valid[row] = EDGE_MEASURED;
                break;
            }
        }
        for (col = seed; col <= SEARCH_IMAGE_W - 1 - CONTRASTOFFSET - SEARCH_EDGE_MARGIN; ++col) {
            inside = pixels[col];
            if (inside < white_min_point) break;
            outside = pixels[col + CONTRASTOFFSET];
            if (EDGE_TRANSITION_WHITE(inside, outside)) {
                right_edge_line[row] = (uint8)(col + CONTRASTOFFSET);
                right_valid[row] = EDGE_MEASURED;
                break;
            }
        }
        // 另一侧离开画面后，继续跟踪仍可见的边界。
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
    // 横墙观察仍使用原始像素；后续圆环识别/投影不再占用相机缓冲区。
    ring_wall_visible = ring_state >= RING_STATE_STAGE1 && ring_state <= RING_STATE_STAGE4 ?
        ring_observe_wall(image) : 0;
    prepare_edge_gaps(image, left_edge_line, left_valid, 0, left_gap_col);
    prepare_edge_gaps(image, right_edge_line, right_valid, 1, right_gap_col);
    edges_pending = 1;
}

static void finish_line_edges(void)
{
    if(!edges_pending) return;
    edges_pending = 0;
    // 圆环先读取未经桥接/过滤的实测边界，再应用本帧像素验证过的短缺口。
    ring_analysis();
    apply_edge_gaps(left_edge_line, left_valid, left_gap_col);
    apply_edge_gaps(right_edge_line, right_valid, right_gap_col);
    filter_short_edges(left_edge_line, left_valid, 0);
    filter_short_edges(right_edge_line, right_valid, SEARCH_IMAGE_W-1);
}

// ============================================
// 右圆环视觉流程：岛边/开口入环，横向外边界引导转弯，寻找前方左直边并补线出环。
// 实车入口：左侧连续直边、右侧独立凸出的岛边；先确认岛边，再等待右侧中近区域开口。
// 状态1/2根据近端左边丢失/恢复释放入环补线；状态3沿实测路径巡线。
// 横向外边界连续确认后进入状态4；找到前方直线或横向边界持续消失后锁定状态5，不退回固定右转。
// 累计角度不参与阶段切换，超时仅静默撤销过期补线。
// ============================================
// 与源工程一致，检测按 2 行步进采样；丢失点数与连续点数以
// “采样点”为单位（源工程搜索本身每隔一行搜索，阈值是为其采样网格调好的）。
// ============================================
// 判断 row 行左边界是否贴近左侧图像边缘；当前判据只检查列坐标，不检查 left_valid 来源。
// ============================================
static uint8 ring_row_left_lost(uint8 row)
{
    return (left_edge_line[row] <= RING_EDGE_MARGIN);
}

// ============================================
// 判断 row 行右边界是否贴近右侧图像边缘；当前判据只检查列坐标，不检查 right_valid 来源。
// ============================================
static uint8 ring_row_right_lost(uint8 row)
{
    return (right_edge_line[row] >= SEARCH_IMAGE_W - RING_EDGE_MARGIN);
}

// ============================================
// 近端左边证据只统计当前白区内的实测点。少量噪点不会阻止入环推进。
static uint8 ring_near_left_points(uint8 *white_points)
{
    int row;
    uint8 measured = 0;
    *white_points = 0;
    for(row = SEARCH_IMAGE_H - 1; row >= RING_BOTTOM_THIRD_TOP; row -= RING_DETECT_ROW_STEP) {
        if(!row_has_white_seed[row]) continue;
        ++*white_points;
        if(left_valid[row] == EDGE_MEASURED && !ring_row_left_lost((uint8)row))
            ++measured;
    }
    return measured;
}

// ============================================
// 检查底部 points 个采样行的实测双直边；在锁定外边出环后确认普通道路恢复。
// row/count：采样行/数量；width：本行左右边界间距；持续恢复帧数由 ring_analysis 单独统计。
// ============================================
static uint8 ring_exit_road_recovered(uint8 points)
{
    int row;                                    // 当前采样行
    uint8 count = 0;                            // 已恢复的底部双边采样点数
    uint8 width;                                // 本行左右边界间距，像素
    int16 top = SEARCH_IMAGE_H - 1 - (points - 1) * RING_DETECT_ROW_STEP;
    int16 span = SEARCH_IMAGE_H - 1 - top;
    int16 residual;

    for(row = SEARCH_IMAGE_H - 1;
        row >= STOPROW && count < points;
        row -= RING_DETECT_ROW_STEP)
    {
        if(!row_has_white_seed[row] || left_valid[row] != EDGE_MEASURED
           || right_valid[row] != EDGE_MEASURED
           || ring_row_left_lost((uint8)row) || ring_row_right_lost((uint8)row)
           || right_edge_line[row] <= left_edge_line[row])
            return 0;

        width = right_edge_line[row] - left_edge_line[row];
        if(width < 20)
            return 0;
        // 双边重新出现还不够；局部必须近似直道，防止环内双边可见就提前复位。
        residual = ((int16)left_edge_line[row] - left_edge_line[top]) * span -
            ((int16)left_edge_line[SEARCH_IMAGE_H-1] - left_edge_line[top]) * (row-top);
        if(abs16(residual) > RING_EXIT_STRAIGHT_TOLERANCE * span) return 0;
        residual = ((int16)right_edge_line[row] - right_edge_line[top]) * span -
            ((int16)right_edge_line[SEARCH_IMAGE_H-1] - right_edge_line[top]) * (row-top);
        if(abs16(residual) > RING_EXIT_STRAIGHT_TOLERANCE * span) return 0;
        count++;
    }

    return (count == points);
}

// ============================================
// 红点处外边界几乎横跨画面；逐列检查当前灰度，不把默认左右边当道路。
// 仅已入环时调用。至少7列支持，限制行高差及相邻斜率，拒绝孤点/杂乱纹理。
static uint8 ring_observe_wall(const uint8 *image)
{
    int16 i, col, row, found, previous = -1, previous_i = -1;
    int16 low = SEARCH_IMAGE_H, high = 0;
    uint8 hits = 0, inside, outside;
    const uint8 *pixel;
    for(i = 0; i < RING_WALL_COLUMNS; ++i) {
        col = RING_WALL_FIRST_COL + i * RING_WALL_COL_STEP;
        found = -1;
        pixel = image + RING_WALL_SCAN_BOTTOM_ROW * SEARCH_IMAGE_W + col;
        for(row = RING_WALL_SCAN_BOTTOM_ROW; row >= STOPROW; --row, pixel -= SEARCH_IMAGE_W) {
            inside = *pixel;
            outside = pixel[-3 * SEARCH_IMAGE_W];
            if(inside >= white_min_point && outside < white_min_point &&
               (int16)inside - outside >= RING_WALL_MIN_DIFF &&
               pixel[SEARCH_IMAGE_W] >= white_min_point &&
               pixel[2 * SEARCH_IMAGE_W] >= white_min_point &&
               pixel[3 * SEARCH_IMAGE_W] >= white_min_point &&
               pixel[-4 * SEARCH_IMAGE_W] < white_min_point &&
               pixel[-5 * SEARCH_IMAGE_W] < white_min_point) {
                found = row;
                break;
            }
        }
        if(found < 0) continue;
        if(previous >= 0 && abs16(found-previous) >
           RING_WALL_MAX_ROW_STEP * (i-previous_i)) return 0;
        previous = found; previous_i = i;
        if(found < low) low = found;
        if(found > high) high = found;
        ++hits;
    }
    return hits >= RING_WALL_MIN_COLUMNS && high-low <= RING_WALL_MAX_ROW_SPAN;
}

// 本帧拟合直线在指定行的列坐标；越出画面时拒绝，不裁剪成假的竖直边。
static int16 ring_exit_line_col(int16 row)
{
    int32 value = ring_exit_line_mid_q8 + ring_exit_line_slope_q8 * (row-ring_exit_line_mid);
    if(value < 0 || value > (int32)(SEARCH_IMAGE_W-1)*256L) return -1;
    return (int16)((value+128L)/256L);
}

/* 从前方实测左边界中寻找连续直段，避免近端仍带圆弧而持续向环内转。
 * 奇数窗口以中行为原点：sum(row-mid)=0，整数最小二乘不依赖单个端点。
 * 仅当前原始边界参与拟合；方向、位置及近端延伸均须通过检查。 */
static uint8 ring_find_exit_straight(void)
{
    int16 start, top, mid, row, col, previous, offset, run, longest;
    int16 nx, ny, fx, fy, dx, dy;
    int32 sum_col, sum_product, sum_square, residual;
    for(start = (int16)func_min(search_ground_near_row(), RING_EXIT_LINE_NEAR_ROW);
        start >= SEARCH_CALIBRATION_MIN_ROW + RING_EXIT_LINE_ROWS-1;
        start -= RING_DETECT_ROW_STEP) {
        top = start-RING_EXIT_LINE_ROWS+1;
        mid = (start+top)/2;
        previous = -1;
        sum_col = sum_product = sum_square = 0;
        for(row = top; row <= start; ++row) {
            col = left_edge_line[row];
            if(!row_has_white_seed[row] || left_valid[row] != EDGE_MEASURED ||
               col <= RING_EDGE_MARGIN || col >= SEARCH_IMAGE_W/2 ||
               (previous >= 0 && abs16(col-previous) > RING_CONTINUE_JUMP_MAX)) break;
            offset = row-mid;
            sum_col += col;
            sum_product += (int32)offset*col;
            sum_square += (int32)offset*offset;
            previous = col;
        }
        if(row <= start || !sum_square) continue;
        ring_exit_line_mid = mid;
        ring_exit_line_mid_q8 = sum_col*256L/RING_EXIT_LINE_ROWS;
        ring_exit_line_slope_q8 = sum_product*256L/sum_square;
        for(row = top; row <= start; ++row) {
            residual = (int32)left_edge_line[row]*256L - ring_exit_line_mid_q8 -
                ring_exit_line_slope_q8*(row-mid);
            if(func_abs(residual) > RING_EXIT_STRAIGHT_TOLERANCE*256L) break;
        }
        if(row <= start) continue;
        if(!search_project_point(ring_exit_line_col(start), start, &nx, &ny) ||
           !search_project_point(ring_exit_line_col(top), top, &fx, &fy)) continue;
        dx = fx-nx; dy = fy-ny;
        if(dy < RING_EXIT_LINE_MIN_DY_MM ||
           (int32)abs16(dx)*1000L > (int32)dy*RING_EXIT_ALIGN_SLOPE_PERMILLE ||
           abs16(nx+(int16)(SEARCH_LANE_WIDTH_MM*0.5f)-RUN_REAR_AXLE_X_MM) >
               RING_EXIT_ALIGN_CENTER_MM) continue;
        ring_exit_line_top = top;
        ring_exit_line_bottom = (int16)func_min(search_ground_near_row(),
            start+RING_EXIT_LINE_EXTEND_ROWS);
        run = longest = 0;
        for(row = ring_exit_line_bottom; row >= top; --row) {
            col = ring_exit_line_col(row);
            if(row_has_white_seed[row] && col > RING_EDGE_MARGIN &&
               col < SEARCH_IMAGE_W-1-RING_EDGE_MARGIN) {
                if(++run > longest) longest = run;
            } else run = 0;
        }
        if(longest >= RING_EXIT_LINE_ROWS) return 1;
    }
    return 0;
}

/* 只在寻直线/出环状态使用本帧拟合，向近处补直左边。
 * 右边环岛不参与；几何模块再按赛道半宽法线偏移生成中线。
 * 用EDGE_RING保留来源区别，原始实测道路恢复必须在调用前判断。 */
static void ring_repair_exit_straight(void)
{
    int16 row, col;
    for(row = search_ground_near_row(); row >= SEARCH_CALIBRATION_MIN_ROW; --row) {
        col = ring_exit_line_col(row);
        if(row >= ring_exit_line_top && row <= ring_exit_line_bottom &&
           row_has_white_seed[row] && col > RING_EDGE_MARGIN &&
           col < SEARCH_IMAGE_W-1-RING_EDGE_MARGIN) {
            left_edge_line[row] = (uint8)col;
            left_valid[row] = EDGE_RING;
        } else {
            left_edge_line[row] = 0;
            left_valid[row] = 0;
        }
    }
    ring_exit_straight = 1;
}

// ============================================
// 入口左侧必须是一条当前实测直边，不能把十字补线、历史或过曝当作支持。
// 只做整数弦线残差检查，图像底部贴边丢失不影响中近区域判断。
static uint8 ring_left_corridor_valid(void)
{
    int16 row, col, top_col, bottom_col, span, residual;
    top_col = left_edge_line[RING_LEFT_SUPPORT_TOP_ROW];
    bottom_col = left_edge_line[RING_OBSERVE_BOTTOM_ROW];
    span = RING_OBSERVE_BOTTOM_ROW - RING_LEFT_SUPPORT_TOP_ROW;
    if(top_col < bottom_col || top_col - bottom_col > RING_LEFT_MAX_SPAN) return 0;
    for(row = RING_LEFT_SUPPORT_TOP_ROW; row <= RING_OBSERVE_BOTTOM_ROW; row += RING_DETECT_ROW_STEP) {
        col = left_edge_line[row];
        if(!row_has_white_seed[row] || left_valid[row] != EDGE_MEASURED ||
           ring_row_left_lost((uint8)row) || col >= SEARCH_IMAGE_W / 2) return 0;
        residual = (col - top_col) * span -
                   (bottom_col - top_col) * (row - RING_LEFT_SUPPORT_TOP_ROW);
        if(abs16(residual) > RING_LEFT_STRAIGHT_TOLERANCE * span) return 0;
    }
    return 1;
}

// 识别独立岛边：近端右侧开口；中部一段连续右边界先向左凸出，
// 再向远端右回折。上下端相对最左端都应有足够横向跨度。
// 不能再要求岛边连接图像顶部，也不能把普通单调斜边当作岛边。
static uint8 ring_detect_feature(void)
{
    int16 row, start = -1, top = -1, minimum = SEARCH_IMAGE_W;
    int16 minimum_row = -1, previous = -1, count = 0, col;
    if(!ring_left_corridor_valid()) return 0;
    for(row = RING_ISLAND_OPEN_TOP_ROW; row <= RING_OBSERVE_BOTTOM_ROW; row += RING_DETECT_ROW_STEP)
        if(!row_has_white_seed[row] || right_valid[row] ||
           !ring_row_right_lost((uint8)row)) return 0;
    for(row = RING_OBSERVE_BOTTOM_ROW; row >= RING_ISLAND_SCAN_TOP_ROW; row -= RING_DETECT_ROW_STEP) {
        col = right_edge_line[row];
        if(right_valid[row] != EDGE_MEASURED || ring_row_right_lost((uint8)row)) {
            if(count) break;
            continue;
        }
        if(col <= SEARCH_IMAGE_W / 2 || left_valid[row] != EDGE_MEASURED ||
           col - left_edge_line[row] < SEARCH_MIN_WIDTH) return 0;
        if(previous >= 0 && abs16(col - previous) > RING_CONTINUE_JUMP_MAX) return 0;
        if(start < 0) start = row;
        top = row;
        previous = col;
        ++count;
        if(col < minimum) { minimum = col; minimum_row = row; }
    }
    if(count < RING_ISLAND_MIN_POINTS || start < RING_ISLAND_BOTTOM_MIN_ROW ||
       minimum_row - top < RING_ISLAND_NOSE_SPAN_ROWS ||
       start - minimum_row < RING_ISLAND_NOSE_SPAN_ROWS) return 0;
    if(right_edge_line[start] - minimum < RING_ISLAND_LOWER_SPAN_PIXELS ||
       right_edge_line[top] - minimum < RING_ISLAND_UPPER_SPAN_PIXELS) return 0;
    return 1;
}

/* Before the full island is visible, a right opening can already interrupt
 * a straight corridor. This is braking evidence only: ordinary junctions
 * may also slow down, but must never acquire a ring state from this hint. */
static uint8 ring_preview_opening(void)
{
    int16 row, col, previous = -1;
    uint8 near_points = 0, open_points = 0;
    if(!ring_left_corridor_valid()) return 0;
    for(row = RING_OBSERVE_BOTTOM_ROW; row >= RING_ISLAND_SCAN_TOP_ROW;
        row -= RING_DETECT_ROW_STEP) {
        col = right_edge_line[row];
        if(!row_has_white_seed[row] || left_valid[row] != EDGE_MEASURED ||
           ring_row_left_lost((uint8)row)) return 0;
        if(!right_valid[row] && ring_row_right_lost((uint8)row)) {
            if(near_points < RING_BRAKE_NEAR_POINTS) return 0;
            if(++open_points >= RING_BRAKE_OPEN_POINTS) return 1;
        } else {
            if(open_points || right_valid[row] != EDGE_MEASURED ||
               col <= SEARCH_IMAGE_W/2 || col-left_edge_line[row] < SEARCH_MIN_WIDTH ||
               (previous >= 0 && abs16(col-previous) > RING_CONTINUE_JUMP_MAX)) return 0;
            previous = col;
            ++near_points;
        }
    }
    return 0;
}

// ============================================
// 两点连线，改写 [num1, num0] 之间的边线并标记 EDGE_RING。
// 对 edge 中 num0（近端）与 num1（远端）之间的行线性插值，valid 写为 EDGE_RING；端点由调用方设置。
// 采用参考代码的 Q8 定点斜率累加，避免逐行除法；远端坐标最后单独写回，保留精确端点。
// ============================================
static void ring_connect(uint8 *edge, uint8 *valid, int16 num0, int16 num1)
{
    int16 row;                                  // 由近到远的插值行
    uint8 end_value;                            // 远端原始列坐标，最后单独写回
    int32 value_q8, step_q8;                    // value_q8：当前列乘 256；step_q8：每行列增量乘 256
    if (num0 >= SEARCH_IMAGE_H || num1 < STOPROW || num0 <= num1) return;
    end_value = edge[num1];
    value_q8 = (int32)edge[num0] * 256;
    step_q8 = (int32)((int16)end_value - edge[num0]) * 256 / (num0 - num1);
    for (row = num0; row > num1; --row) {
        edge[row] = (uint8)(value_q8 >> 8);
        valid[row] = EDGE_RING;
        value_q8 += step_q8;
    }
    edge[num1] = end_value;
    valid[num1] = EDGE_RING;
}

// ============================================
// 将左边界替换为固定斜线：近端（bottom_row,0）连接远端（top_row,W-1），用于入环及出环准备。
// top_row 由阶段传入：入环第 50 行，出环第 13 行；右端下移使相同行的补线更靠右，加强右转引导。
// bottom_row 为图像最底行；斜线端点不跟随入口锚点改变。
// ============================================
static void ring_repair_left_diagonal(uint8 top_row)
{
    uint8 bottom_row = SEARCH_IMAGE_H - 1;      // 固定斜线近端行，当前为 119

    if(bottom_row <= top_row + 2)
        return;

    left_edge_line[bottom_row] = 0;
    left_valid[bottom_row] = EDGE_RING;
    left_edge_line[top_row] = SEARCH_IMAGE_W - 1;
    left_valid[top_row] = EDGE_RING;
    ring_connect(left_edge_line, left_valid, bottom_row, top_row);
}

// 仅在已确认岛边的 APPROACH 状态使用：中近区域岛边已消失，左边仍直且连续。
// 少量孤立噪点允许存在；持续可见的一段右边界仍阻止入环。
static uint8 ring_entry_opening(void)
{
    int16 row;
    uint8 visible = 0, run = 0;
    if(!ring_left_corridor_valid()) return 0;
    for(row = RING_LEFT_SUPPORT_TOP_ROW; row <= RING_OBSERVE_BOTTOM_ROW; row += RING_DETECT_ROW_STEP) {
        if(!row_has_white_seed[row]) return 0;
        if(!right_valid[row] && ring_row_right_lost((uint8)row)) run = 0;
        else {
            ++visible;
            if(++run > RING_ENTRY_MAX_RIGHT_POINTS || visible > RING_ENTRY_MAX_RIGHT_POINTS) return 0;
        }
    }
    return 1;
}

// ============================================
// 清理全部阶段计数、锁存及出环引导标志。
static void ring_reset(void)
{
    ring_state = RING_STATE_NONE;
    ring_dir = RING_DIR_NONE;
    ring_exit_guiding = ring_follow_outer = ring_exit_straight = ring_exit_ready = 0;
    ring_entry_recover_count = ring_approach_count = 0;
    ring_exit_confirm_count = ring_exit_recover_count = ring_align_count = 0;
    ring_exit_clear_count = 0;
    ring_exit_line_top = ring_exit_line_bottom = ring_exit_line_mid = ring_exit_line_last_col = 0;
    ring_exit_line_mid_q8 = ring_exit_line_slope_q8 = ring_exit_line_last_slope_q8 = 0;
    ring_start_tick = ring_active_last_tick = ring_exit_start_tick = 0;
    ring_brake_hint = 0;
    ring_brake_tick = 0;
}

static void ring_finish(void)
{
    ring_reset();
    ring_confirm_count = 0;
    ring_rearm_active = 1;
    ring_rearm_tick = ring_frame_ticks;
}

static void ring_begin(void)
{
    ring_exit_guiding = ring_follow_outer = ring_exit_straight = ring_exit_ready = 0;
    ring_entry_recover_count = ring_exit_confirm_count = ring_exit_recover_count = ring_align_count = 0;
    ring_exit_clear_count = 0;
    ring_start_tick = ring_active_last_tick = ring_frame_ticks;
    ring_exit_start_tick = 0;
    ring_state = RING_STATE_STAGE1;
}

static void ring_begin_exit(void)
{
    ring_state = RING_STATE_STAGE4;
    ring_exit_start_tick = ring_frame_ticks;
    ring_exit_confirm_count = ring_exit_recover_count = ring_align_count = 0;
    ring_exit_clear_count = 0;
}

/* Only stale-data/time bounds remain; no accumulated yaw, fault or motor stop. */
static void ring_check_progress(void)
{
    if((uint16)(ring_frame_ticks-ring_active_last_tick) >= RING_APPROACH_FRAME_GAP_TICKS ||
       (uint16)(ring_frame_ticks-ring_start_tick) >= RING_TIMEOUT_TICKS ||
       ((ring_state == RING_STATE_STAGE4 || ring_state == RING_STATE_EXIT_FOLLOW) &&
        (uint16)(ring_frame_ticks-ring_exit_start_tick) >= RING_EXIT_TIMEOUT_TICKS)) {
        ring_finish();
        return;
    }
    ring_active_last_tick = ring_frame_ticks;
}

/* Visual sequence: island/opening -> entry gap/recovery -> transverse outer
 * boundary -> search for a forward straight -> extend it into an exit path.
 * EXIT_FOLLOW never returns to forced turning after the guide has ended. */
static void ring_analysis(void)
{
    uint8 feature_found, near_left = 0, white_points = 0, aligned;
    int16 line_col;
    ring_exit_guiding = ring_follow_outer = ring_exit_straight = ring_exit_ready = 0;
    if(ring_rearm_active && (uint16)(ring_frame_ticks-ring_rearm_tick) >= RING_REARM_TICKS)
        ring_rearm_active = 0;
    feature_found = !ring_rearm_active &&
        (ring_state == RING_STATE_NONE || ring_state == RING_STATE_APPROACH) &&
        ring_detect_feature();
    ring_candidate = feature_found ? RING_DIR_RIGHT : RING_DIR_NONE;
    if(!ring_rearm_active && (feature_found ||
       (ring_state == RING_STATE_NONE && ring_preview_opening()))) {
        ring_brake_hint = 1;
        ring_brake_tick = ring_frame_ticks;
    } else if((uint16)(ring_frame_ticks-ring_brake_tick) >= RING_BRAKE_HOLD_TICKS) {
        ring_brake_hint = 0;
    }


    if(ring_state >= RING_STATE_STAGE1 && ring_state <= RING_STATE_EXIT_FOLLOW) {
        ring_check_progress();
        near_left = ring_near_left_points(&white_points);
        if(ring_state >= RING_STATE_STAGE1 && ring_state <= RING_STATE_STAGE3) {
            if(ring_wall_visible && white_points >= 10 && near_left <= 4) {
                if(++ring_exit_confirm_count >= RING_EXIT_CONFIRM_FRAMES) ring_begin_exit();
            } else ring_exit_confirm_count = 0;
        }
        if((ring_state == RING_STATE_STAGE1 || ring_state == RING_STATE_STAGE2) &&
           (uint16)(ring_frame_ticks-ring_start_tick) >= RING_ENTRY_GUIDE_MAX_TICKS)
            ring_state = RING_STATE_STAGE3;
    }

    switch(ring_state) {
        case RING_STATE_NONE:
            ring_dir = RING_DIR_NONE;
            if(feature_found) {
                if(ring_confirm_count &&
                   (uint16)(ring_frame_ticks-ring_confirm_last_tick) >= RING_APPROACH_FRAME_GAP_TICKS)
                    ring_confirm_count = 0;
                ring_confirm_last_tick = ring_frame_ticks;
                if(++ring_confirm_count >= RING_CONFIRM_FRAMES) {
                    ring_state = RING_STATE_APPROACH;
                    ring_dir = RING_DIR_RIGHT;
                    ring_confirm_count = ring_approach_count = 0;
                    ring_approach_tick = ring_approach_last_tick = ring_frame_ticks;
                }
            } else ring_confirm_count = 0;
            break;
        case RING_STATE_APPROACH:
            ring_dir = RING_DIR_RIGHT;
            if((uint16)(ring_frame_ticks-ring_approach_tick) >= RING_APPROACH_TIMEOUT_TICKS ||
               (uint16)(ring_frame_ticks-ring_approach_last_tick) >= RING_APPROACH_FRAME_GAP_TICKS) {
                ring_finish();
                break;
            }
            ring_approach_last_tick = ring_frame_ticks;
            if(ring_entry_opening()) {
                if(++ring_approach_count >= RING_APPROACH_CONFIRM_FRAMES) {
                    ring_begin();
                    ring_repair_left_diagonal(RING_ENTRY_REPAIR_TOP_ROW);
                }
            } else ring_approach_count = 0;
            break;
        case RING_STATE_STAGE1:
            ring_dir = RING_DIR_RIGHT;
            if(white_points >= 10 && near_left <= 4) ring_state = RING_STATE_STAGE2;
            ring_repair_left_diagonal(RING_ENTRY_REPAIR_TOP_ROW);
            break;
        case RING_STATE_STAGE2:
            ring_dir = RING_DIR_RIGHT;
            if(near_left >= RING_ENTRY_RECOVER_POINTS) {
                if(++ring_entry_recover_count >= RING_ENTRY_RECOVER_FRAMES)
                    ring_state = RING_STATE_STAGE3;
            } else ring_entry_recover_count = 0;
            if(ring_state == RING_STATE_STAGE2) ring_repair_left_diagonal(RING_ENTRY_REPAIR_TOP_ROW);
            break;
        case RING_STATE_STAGE3:
            ring_dir = RING_DIR_RIGHT;
            break;
        case RING_STATE_STAGE4:
            aligned = ring_find_exit_straight();
            if(aligned) {
                line_col = ring_exit_line_col(RING_EXIT_LINE_NEAR_ROW);
                if(ring_align_count &&
                   (abs16(line_col-ring_exit_line_last_col) > RING_EXIT_LINE_MAX_SHIFT ||
                    func_abs(ring_exit_line_slope_q8-ring_exit_line_last_slope_q8) >
                        RING_EXIT_LINE_MAX_SLOPE_Q8)) ring_align_count = 0;
                ring_exit_line_last_col = line_col;
                ring_exit_line_last_slope_q8 = ring_exit_line_slope_q8;
                if(++ring_align_count >= RING_EXIT_ALIGN_FRAMES) {
                    ring_state = RING_STATE_EXIT_FOLLOW;
                    ring_follow_outer = 1;
                    ring_repair_exit_straight();
                    ring_exit_ready = 1;
                }
            } else ring_align_count = 0;
            if(!ring_wall_visible) {
                ring_follow_outer = 1;
                if(++ring_exit_clear_count >= RING_EXIT_WALL_CLEAR_FRAMES)
                    ring_state = RING_STATE_EXIT_FOLLOW;
            } else ring_exit_clear_count = 0;
            if(ring_state == RING_STATE_STAGE4 && !ring_follow_outer && ring_wall_visible) {
                ring_exit_guiding = 1;
                ring_repair_left_diagonal(RING_EXIT_TURN_TOP_ROW);
            }
            ring_dir = ring_exit_guiding ? RING_DIR_RIGHT : RING_DIR_NONE;
            break;
        case RING_STATE_EXIT_FOLLOW:
            ring_dir = RING_DIR_NONE;
            ring_follow_outer = 1;
            if(ring_exit_road_recovered(RING_EXIT_ROAD_POINTS)) {
                ring_exit_ready = 1;
                if(++ring_exit_recover_count >= RING_EXIT_RECOVER_FRAMES) ring_finish();
            } else ring_exit_recover_count = 0;
            /* Recovery above sees raw boundaries only. Refit each fresh frame;
             * if no straight is visible, use the measured left edge as fallback. */
            if(ring_state == RING_STATE_EXIT_FOLLOW && ring_find_exit_straight()) {
                ring_repair_exit_straight();
                ring_exit_ready = 1;
            }
            break;
        default:
            ring_finish();
            break;
    }
    if(ring_state != RING_STATE_NONE) cross_flag = cross_half_flag = 0;
}

// ============================================
// 检查 row 行双边有效且宽度至少为 SEARCH_MIN_WIDTH；仅用于普通/十字中线分支。
// ============================================
static uint8 pair_valid(int16 row)
{
    return left_valid[row] && right_valid[row] &&
        (int16)right_edge_line[row] - left_edge_line[row] >= SEARCH_MIN_WIDTH;
}

// ============================================
// 返回 row 行左右边界列坐标的整数平均值；调用方负责判断双边是否可信。
// ============================================
static int16 pair_center(int16 row)
{
    return ((int16)left_edge_line[row] + right_edge_line[row]) / 2;
}

// ============================================
// 仅在 STOPROW 及更近区域寻找锚点；更远的行未经搜线，不参与补线。
// 比较相隔 CROSS_JUMP_GAP 行的列差，返回更靠近图像中心的跳变端点行，失败返回 -1。
// num0/num1：搜索近远边界；jump_num：跳变阈值；model=1 从近到远、0 从远到近；i/a/b/point：偏移、比较行和选中行。
// ============================================
static int16 find_jump_point(const uint8 *edge, const uint8 *valid, int16 num0, int16 num1,
                             int16 jump_num, uint8 model)
{
    int16 i, a, b, point;                       // i：扫描偏移；a/b：比较行；point：更靠近图像中心的端点行
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

// ============================================
// 连接两个有效十字端点 num0/num1；近远列坐标为 near_col/far_col，中间插值行标为 EDGE_CROSS。
// ============================================
static void connect_point(uint8 *edge, uint8 *valid, int16 num0, int16 num1)
{
    int16 row, near_col, far_col;               // row：插值行；near_col/far_col：近远端列坐标
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

// ============================================
// 使用有符号行索引，避免 start_point-10 回绕导致数组越界。
// 将十字边界从 num1→num0 沿斜率向近端 extend_to 延长；停止于图像边界。
// row：外推行；near_col/far_col：端点列；value：外推列；有符号行号防止减法下溢。
// ============================================
static void extend_line_downward(uint8 *edge, uint8 *valid, int16 num0, int16 num1, int16 extend_to)
{
    int16 row, near_col, far_col, value;        // row：外推行；near_col/far_col：近远端列；value：有符号外推列
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

    }
}

// ============================================
// 判定 row 行是否为十字大宽度白区；必须同时有本帧白色种子，避免全黑图像的 0/W-1 默认边界误判。
// ============================================
static uint8 cross_wide_row(int16 row)
{
    // 黑图像也有默认 0/W-1 边界；必须存在当帧白区种子才算宽行。
    return row_has_white_seed[row] && right_edge_line[row] > left_edge_line[row] &&
        (uint16)(right_edge_line[row]-left_edge_line[row]) > CROSS_WIDE_THRESHOLD;
}

// ============================================
// 从 row 沿 direction（+1 向近处，-1 向远处）检查 count 行实测双边是否连续、正常宽度且非大宽度白区。
// ============================================
static uint8 cross_measured_pair_run(int16 row, int16 direction, int16 count)
{
    int16 i, previous;                          // i：检查行；previous：上一实测行号
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

// ============================================
// 急弯双边丢失也会形成满宽，需在同一宽段下方找到实测入口。
// 在 wide_near 下方限定窗口内寻找同一白色开口的实测双边入口，区分十字与急弯双边出界。
// ============================================
static uint8 cross_entry_supported(int16 wide_near)
{
    int16 row;                                  // 在同一宽段下方查找入口的扫描行
    for (row = wide_near+1; row < SEARCH_IMAGE_H && row <= wide_near+CROSS_SUPPORT_WINDOW; ++row) {
        if (!row_has_white_seed[row] || cross_wide_row(row)) break;
        if (cross_measured_pair_run(row, 1, CROSS_SUPPORT_ROWS) &&
            (int16)left_edge_line[row]-left_edge_line[wide_near] > CROSS_JUMP_PIXELS &&
            (int16)right_edge_line[wide_near]-right_edge_line[row] > CROSS_JUMP_PIXELS) return 1;
    }
    return 0;
}

// ============================================
// 单个入口与弯道出画面容易混淆，要求同侧出口重新出现且方向相容，只用实测锚点。
// 从 wide_row 沿 direction 寻找同一侧开口的连续实测锚点，成功返回锚点行，失败返回 -1。
// edge/valid 是待查边界及来源，仅使用当前实测点作为锚点。
// ============================================
static int16 cross_single_anchor(const uint8 *edge, const uint8 *valid,
                                int16 wide_row, int16 direction)
{
    int16 row, j, point;                        // row：搜索行；j：实测支持检查行；point：当前候选锚点行
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

// ============================================
// 验证单侧十字开口的近远锚点和两端斜率，成功后连接该侧并记录 side（1 左、2 右），返回 1。
// ============================================
static uint8 cross_connect_single(uint8 *edge, uint8 *valid, uint8 side,
                                  int16 wide_near, int16 wide_far)
{
    // near_row/far_row：锚点行；span：行距；delta：列差；row：方向一致性检查行
    int16 near_row, far_row, span, delta, row;
    int32 error;                                // 锚点连线方向与实测邻段方向的放大误差
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

// ============================================
// 逐段统计连续大宽度行，尝试左侧或右侧的入口/出口连接；成功返回 1。
// ============================================
static uint8 cross_single_analysis(void)
{
    int16 row, near_row, count;                 // row：当前扫描行；near_row：近端锚点行；count：连续宽行数
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

// ============================================
// 整理十字补线结果：撤销造成过窄车道的补线。
// ============================================
static void cross_finish_lines(void)
{
    int16 row;                                  // 整理补线覆盖范围的扫描行
    for (row = STOPROW; row < SEARCH_IMAGE_H; ++row) {
        if (left_valid[row] && right_valid[row] &&
            (int16)right_edge_line[row]-left_edge_line[row] < SEARCH_MIN_WIDTH) {
            if (left_valid[row] == EDGE_CROSS) { left_valid[row] = 0; left_edge_line[row] = 0; }
            if (right_valid[row] == EDGE_CROSS) { right_valid[row] = 0; right_edge_line[row] = SEARCH_IMAGE_W-1; }
        }
    }
}

// ============================================
// 十字检测及补线入口：先处理近端大开口，再验证入口并连接，最后根据剩余大宽度区域更新状态。
// ============================================
static void cross_analysis(void)
{
    // i：扫描行；start_i：宽段扫描起点；start_point/end_point：跳变端点；wide_near：宽段近端；
    // left_start/right_start：左右锚点；support：入口支持行
    int16 i, start_i, start_point, end_point, wide_near, left_start, right_start, support;
    uint8 wide_count;                           // 连续宽行计数
    cross_candidate = cross_half_flag = 0;
    cross_single_side = 0;

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
        // 补线后 cross_flag 可能立即清除，仍保留本帧十字候选供中线仲裁。
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

// ============================================
// 生成逐行中线、来源与质量；入/出环使用有限补线，正常环内与普通路段使用实测/标定中线。
// row：当前行；current_rows：质量至少为 2 的行数；source：本行来源；历史中线只赋质量 1，不能直接用于 Run 转向。
// ============================================
static void get_center_line(void)
{
    int16 row, current_rows;                    // row：当前行；current_rows：质量至少为 2 的中线行数
    uint8 source;                               // 本行中线来源 CENTER_*
    search_ground_update();
    current_rows = 0;
    for (row = 0; row < SEARCH_IMAGE_H; ++row) {
        source = CENTER_NONE;
        center_line[row] = SEARCH_IMAGE_W / 2;
        if (row >= STOPROW) {
            if (ring_state == RING_STATE_APPROACH || ring_follow_outer ||
                ring_state == RING_STATE_EXIT_FOLLOW) {
                // 右侧环岛不能拉偏中线；使用实测左边或本帧前方直线补边的毫米法线。
                if(ground_center_valid[row]) {
                    center_line[row] = ground_center_line[row];
                    source = CENTER_GROUND;
                }
            } else if (ring_state == RING_STATE_STAGE1 || ring_state == RING_STATE_STAGE2 ||
                       ring_exit_guiding) {
                // 入/出环引导仍允许单边开口；黑图、交叉边界不能变成可信中线。
                if(row_has_white_seed[row] && (left_valid[row] || right_valid[row]) &&
                   left_edge_line[row] < right_edge_line[row]) {
                    center_line[row] = (uint8)(((uint16)left_edge_line[row] +
                        right_edge_line[row]) / 2);
                    source = CENTER_RING;
                }
            } else if (pair_valid(row) && (!cross_single_side ||
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
        // 单边法线偏移可能把中线移向远处，这里统计当帧质量。
        // Run 在输出转向前还会独立检查近端距离与标定范围内的连续段。
        if (row >= STOPROW && center_quality[row] >= 2) ++current_rows;
    }
    path_lost = current_rows < SEARCH_IMAGE_H / 10;
}

// ============================================
// 整帧处理兼容入口：识别斑马线、更新灰度和起搜列，推进右圆环/十字，最后生成中线。
// ticks 来自 Motor_GetTicks()，单位5ms；圆环按本帧实测边界推进，此节拍用于断帧/总时长检查。
// ============================================
void search_process_timed(const uint8 *image, uint16 ticks)
{
    search_extract_edges_timed(image, ticks);
    search_finish_frame();
}

// ============================================
// 检查单行是否存在足够长的黑白交替段。阈值取本行灰度范围的 3/4 位置，兼容实拍中反光偏亮的黑条。
// 只累计宽度合格的完整黑条及前置白间隔；首尾截断、孤立像素和大块道路均不算条纹。
// ============================================
static uint8 zebra_row_matches(const uint8 *pixels)
{
    int16 col;                                 // 当前列号，只读取图像中央观察区
    uint8 low = 255, high = 0, threshold;       // 本行灰度极值与二值化阈值
    uint8 dark, next_dark, width = 1;           // 当前段颜色、下一像素颜色及连续宽度
    uint8 white_width = 0, count = 0;          // 前置白间隔宽度和连续有效黑条数
    uint16 span = 0;                           // 当前连续条纹累计跨度，含白间隔
    for(col = ZEBRA_SCAN_LEFT_COL; col <= ZEBRA_SCAN_RIGHT_COL; ++col) {
        if(pixels[col] < low) low = pixels[col];
        if(pixels[col] > high) high = pixels[col];
    }
    if((uint16)high - low < ZEBRA_MIN_CONTRAST) return 0;
    threshold = (uint8)(low + ((uint16)high - low) * 3U / 4U);
    dark = pixels[ZEBRA_SCAN_LEFT_COL] < threshold;
    for(col = ZEBRA_SCAN_LEFT_COL + 1; col <= ZEBRA_SCAN_RIGHT_COL; ++col) {
        next_dark = pixels[col] < threshold;
        if(next_dark == dark) { ++width; continue; }
        if(!dark) {
            white_width = width;
            if(width < ZEBRA_MIN_RUN_WIDTH || width > ZEBRA_MAX_RUN_WIDTH) {
                count = 0; span = 0;
            }
        } else if(white_width >= ZEBRA_MIN_RUN_WIDTH && white_width <= ZEBRA_MAX_RUN_WIDTH &&
                  width >= ZEBRA_MIN_RUN_WIDTH && width <= ZEBRA_MAX_RUN_WIDTH) {
            ++count;
            span += (uint16)white_width + width;
            if(count >= ZEBRA_MIN_BLACK_RUNS && span >= ZEBRA_MIN_PATTERN_SPAN) return 1;
        } else {
            count = 0; span = 0;
        }
        dark = next_dark;
        width = 1;
    }
    return 0;
}

// ============================================
// 在像素帧仍被锁定时识别斑马线：多行条纹形成候选，再按连续帧确认。
// 本函数只发布视觉结果，不开关电机；Run 在确认当帧锁定停车，Image 只显示识别状态。
// ============================================
static void zebra_analysis(const uint8 *image, uint16 ticks)
{
    int16 row;                                 // 从近到远的采样行
    uint8 matched = 0;                         // 当前连续匹配行数
    zebra_candidate = zebra_detected = 0;

    for(row = ZEBRA_SCAN_BOTTOM_ROW; row >= ZEBRA_SCAN_TOP_ROW; row -= ZEBRA_SCAN_ROW_STEP) {
        if(zebra_row_matches(image + row * SEARCH_IMAGE_W)) {
            if(++matched >= ZEBRA_MIN_MATCH_ROWS) { zebra_candidate = 1; break; }
        } else matched = 0;
    }
    if(!zebra_candidate) { zebra_confirm_count = 0; return; }
    if(zebra_confirm_count && (uint16)(ticks - zebra_last_tick) >= ZEBRA_FRAME_GAP_TICKS)
        zebra_confirm_count = 0;
    zebra_last_tick = ticks;
    if(zebra_confirm_count < ZEBRA_CONFIRM_FRAMES) ++zebra_confirm_count;
    zebra_detected = zebra_confirm_count >= ZEBRA_CONFIRM_FRAMES;
}

void search_extract_edges_timed(const uint8 *image, uint16 ticks)
{
    ring_frame_ticks = ticks;                  // 同一帧的圆环处理使用同一时间快照
    // 1. 更新灰度参考和白色阈值
    get_reference_point(image);
    // 斑马线必须读取原始灰度图；extract 返回后相机可能立即覆盖缓冲区。
    zebra_analysis(image, ticks);
    // 2. 确定起搜列
    search_reference_col(image);
    // 3. 只提取像素证据；圆环拟合/补线延后到释放相机帧之后。
    extract_line_pixels(image);
}

void search_finish_frame(void)
{
    finish_line_edges();
    // 4. 圆环激活时跳过十字补线，避免两种元素互相覆盖
    if (ring_state != RING_STATE_NONE) {
        cross_flag = cross_candidate = cross_half_flag = cross_single_side = 0;

    } else cross_analysis();
    // 5. 生成中线、来源和质量，供显示与车辆控制读取
    get_center_line();
}
