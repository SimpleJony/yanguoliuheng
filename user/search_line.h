// 搜线、十字及当前右圆环补线的公共参数与输出。
// 图像坐标均以像素计：数组下标为行，数组元素为列；边界有效标记与坐标必须一起解释。
#ifndef SEARCH_LINE_H_
#define SEARCH_LINE_H_
#include "zf_common_headfile.h"
#include "search_calibration.h"

// ============================================
// 普通搜线参数与边界来源
// ============================================
#define SEARCH_IMAGE_W MT9V03X_W                // 图像宽度，列数
#define SEARCH_IMAGE_H MT9V03X_H                // 图像高度，行数
#define BLACKPOINT 50                           // 白色阈值的最低灰度限制
#define WHITEMINMUL 7                           // 白色下阈值倍率，除以10后使用
#define REFERENCEROW 5                          // 参与灰度参考的底部行数
#define SEARCHRANGE 10                          // 局部搜种子半径及边界连续性列差阈值，像素
#define SEARCH_SEED_PREDICT_STEP 20             // 相邻行种子预测最大移动量，像素

#define SEARCH_EDGE_MAX_GAP 2                   // 允许插值连接的最大缺失行数
#define EDGE_MEASURED 1                         // 当前图像实测边界
#define EDGE_BRIDGED 2                          // 短缺口插值得到的边界
#define EDGE_CROSS 3                            // 十字连接/外推边界

#define EDGE_RING 4                             // 圆环斜线/直线外推边界
#define SEARCH_EDGE_MIN_RUN 3                   // 有效边段最小连续行数

#define STOPROW 12                              // 最远搜索行，行号更小的区域不搜索
#define SEARCH_EDGE_MARGIN 3                    // 搜边时保留的图像边缘余量，像素
#define CONTRASTOFFSET 3                        // 亮暗比较的像素间隔
#define SEARCH_MIN_DIFF 8                       // 确认边缘所需最小亮暗灰度差
#define SEARCH_MIN_WIDTH 12                     // 有效道路/白色连续区的最小列宽
#define SEARCH_HISTORY_FRAMES 3                 // 历史中线最多保留的帧数，质量仅为1

#define SEARCH_ROW_STEP 1                       // 普通搜线/十字逐行扫描步长
// ============================================
// 斑马线：在原始灰度图中寻找横向重复黑白条纹，不依赖可能被条纹截断的左右边界
// ============================================
#define ZEBRA_SCAN_TOP_ROW 44                   // 最远观察行；只在图像中近区域确认停车线
#define ZEBRA_SCAN_BOTTOM_ROW 104               // 最近观察行
#define ZEBRA_SCAN_ROW_STEP 4                   // 每隔 4 行检查，减少逐像素计算
#define ZEBRA_SCAN_LEFT_COL (SEARCH_IMAGE_W / 8) // 去掉两侧容易受到路外背景影响的区域
#define ZEBRA_SCAN_RIGHT_COL (SEARCH_IMAGE_W - 1 - ZEBRA_SCAN_LEFT_COL)
#define ZEBRA_MIN_CONTRAST 40                   // 本行最大与最小灰度至少相差 40，拒绝弱纹理/纯色图
#define ZEBRA_MIN_RUN_WIDTH 3                   // 黑条和白间隔最小像素宽度，排除细碎噪点
#define ZEBRA_MAX_RUN_WIDTH 20                  // 黑条和白间隔最大宽度，排除普通道路大块白区
#define ZEBRA_MIN_BLACK_RUNS 5                  // 每行至少连续出现 5 条有效黑条，且由白间隔分开
#define ZEBRA_MIN_PATTERN_SPAN 50               // 连续条纹横向总跨度至少 50 像素
#define ZEBRA_MIN_MATCH_ROWS 3                  // 连续 3 个采样行满足条纹特征才算本帧候选
#define ZEBRA_CONFIRM_FRAMES 2                  // 连续 2 帧候选才确认；停车锁定由 Run 独立保存
#define ZEBRA_FRAME_GAP_TICKS 50U               // 两次命中间隔达到 250ms 后重新确认，兼容 5ms 时钟回绕
#if ZEBRA_SCAN_TOP_ROW < 0 || ZEBRA_SCAN_BOTTOM_ROW >= SEARCH_IMAGE_H || ZEBRA_SCAN_TOP_ROW > ZEBRA_SCAN_BOTTOM_ROW || ZEBRA_SCAN_ROW_STEP < 1 || ZEBRA_CONFIRM_FRAMES < 1 || ZEBRA_CONFIRM_FRAMES > 255
#error Invalid zebra detection settings.
#endif
// ============================================
// 十字识别参数
// ============================================
#define CROSS_WIDE_THRESHOLD 180                // 十字白区宽度需严格超过此值，像素
#define CROSS_CONSEC_ENTER 5                    // 新十字需要的连续大宽度行数
#define CROSS_CONSEC_EXIT 3                     // 已在十字时维持状态所需连续宽行数
#define CROSS_CONSEC_HALF 20                    // 底部大开口处理分支的连续宽行数
#define CROSS_JUMP_GAP 6                        // 十字跳变比较两点的行距
#define CROSS_JUMP_PIXELS 10                    // 十字跳变/斜率一致性阈值，像素

#define CROSS_SUPPORT_ROWS 3                    // 双侧十字入口的实测支持行数
#define CROSS_SUPPORT_WINDOW 36                 // 寻找同一开口入口/出口的最大行窗口

#define CROSS_SINGLE_SUPPORT_ROWS 7             // 单侧十字锚点需要的连续实测行数

// 当前右圆环入口识别与补线参数，源自 yanguoliuheng/user/search_line.c。
// 搜线逐行进行，但圆环特征按 RING_DETECT_ROW_STEP=2 采样；丢点数量按采样点计。
// ============================================
// 右圆环预识别参数：命中后仅进入 APPROACH，实际转弯由近端岛边事件决定
// ============================================
#define RING_DIR_NONE 0                         // 无方向标志；不能单凭它判断圆环状态已退出
#define RING_DIR_RIGHT 2                        // 当前右圆环方向编码
#define RING_EDGE_MARGIN (CONTRASTOFFSET + 3)   // 圆环贴边丢线判据的边缘范围，像素
#define RING_CONTINUE_JUMP_MAX 10               // 相邻采样点允许的列差
#define RING_CONFIRM_FRAMES 2                   // 独立岛边连续确认帧数；状态切换不依赖累计转角

/* 2026-10-19 实拍：岛边在约22..72行，近端90..100行并无岛边。
 * 左直边与岛边形状用于识别；已识别后，中近区域开口才允许实际入环。 */
#define RING_ISLAND_SCAN_TOP_ROW 20
#define RING_LEFT_SUPPORT_TOP_ROW 30
#define RING_OBSERVE_BOTTOM_ROW 90
#define RING_ISLAND_OPEN_TOP_ROW 80
#define RING_LEFT_STRAIGHT_TOLERANCE 3           // 左边相对两端连线的最大残差，像素
#define RING_LEFT_MAX_SPAN 60                   // 左直边两端最大列差
#define RING_ISLAND_MIN_POINTS 12               // 中部连续实测岛边采样点数
#define RING_ISLAND_BOTTOM_MIN_ROW 50           // 岛边必须已进入中部，而非仅远处小岛
#define RING_ISLAND_NOSE_SPAN_ROWS 6            // 最左端距岛边上下端的最小行差
#define RING_ISLAND_LOWER_SPAN_PIXELS 24         // 近端相对最左端向右展开量
#define RING_ISLAND_UPPER_SPAN_PIXELS 8          // 远端回折量，排除普通单调边线
#define RING_ENTRY_MAX_RIGHT_POINTS 2           // 开口区允许的孤立右边噪点数
#define RING_APPROACH_CONFIRM_FRAMES 2
#define RING_APPROACH_TIMEOUT_TICKS 1200U       // 6秒仍未到入环位置则取消识别
#define RING_APPROACH_FRAME_GAP_TICKS 50U       // 断帧250ms取消，不能沿用旧岛边事件
/* 仅减速的远端开口预警：左直边、近端双边、远端右侧持续开口。
 * 不参与候选确认、补线或入环；短暂丢失证据仍保持300ms限速。 */
#define RING_BRAKE_NEAR_POINTS 6
#define RING_BRAKE_OPEN_POINTS 6
#define RING_BRAKE_HOLD_TICKS 60U
#if RING_CONFIRM_FRAMES < 1 || RING_CONFIRM_FRAMES > 255 || RING_APPROACH_CONFIRM_FRAMES < 1 || RING_APPROACH_CONFIRM_FRAMES > 255
#error Invalid ring confirmation settings.
#endif

// ============================================
// 右圆环补线参数：入环与出环独立配置，行号增大表示端点向画面下方移动
// 入环与红点转弯区均用近端右转引导；随后寻找前方直线，沿直线补左外边出环
// ============================================
#define RING_DETECT_ROW_STEP 2                  // 圆环采样行 119、117……15，独立于普通搜线步长
#if RING_ISLAND_SCAN_TOP_ROW < STOPROW || RING_LEFT_SUPPORT_TOP_ROW < RING_ISLAND_SCAN_TOP_ROW || RING_OBSERVE_BOTTOM_ROW >= SEARCH_IMAGE_H || RING_ISLAND_OPEN_TOP_ROW > RING_OBSERVE_BOTTOM_ROW
#error Ring observation rows must stay inside the searched image.
#endif
#define RING_ENTRY_REPAIR_TOP_ROW 50            // 入环左斜线右端行，原 40，列为 W-1；下移使相同行补线更靠右
#define RING_EXIT_TURN_TOP_ROW 50               // 横向外边界确认后加强右转；仅当前视觉证据有效时使用
#define RING_ENTRY_GUIDE_MAX_TICKS 120U         // 入环补线最多300ms；左边恢复可提前结束，之后交回实测巡线
#define RING_ENTRY_RECOVER_POINTS 8             // 近端20个采样中至少8点为实测左边
#define RING_ENTRY_RECOVER_FRAMES 3             // 先观察近端左边缺失，再以连续实测恢复释放入环补线
#define RING_BOTTOM_THIRD_TOP (SEARCH_IMAGE_H * 2 / 3) // 左边丢失/恢复的近端区域起始行
#define RING_EXIT_CONFIRM_FRAMES 2
#define RING_EXIT_ROAD_POINTS 28                // 跟外边出环后，119..65行完整双直边恢复才结束
#define RING_EXIT_RECOVER_FRAMES 3              // 底部双边连续恢复多少帧才结束圆环
#define RING_EXIT_STRAIGHT_TOLERANCE 2          // 相对端点弦线的最大像素残差
#define RING_WALL_SCAN_BOTTOM_ROW 70            // 按列向上寻找横向外边界；不投影到标定区之外
#define RING_WALL_FIRST_COL 24
#define RING_WALL_COL_STEP 16
#define RING_WALL_COLUMNS 9
#define RING_WALL_MIN_COLUMNS 7                // 至少7列有连续白到暗边，拒绝孤立黑点
#define RING_WALL_MAX_ROW_SPAN 24
#define RING_WALL_MAX_ROW_STEP 5
#define RING_WALL_MIN_DIFF 20
#define RING_EXIT_LINE_NEAR_ROW 86            // 在38..86行寻找前方左直边，不要求近处圆弧先变直
#define RING_EXIT_LINE_ROWS 27                // 拟合窗口必须是奇数，且每行都有当前实测左边
#define RING_EXIT_LINE_EXTEND_ROWS 32         // 向近处最多延伸32行，不越过地面标定范围
#define RING_EXIT_LINE_MIN_DY_MM 60           // 前方直边至少覆盖60mm纵向距离
#define RING_EXIT_LINE_MAX_SHIFT 8            // 连续确认时，同一参考行的线位置变化不超过8像素
#define RING_EXIT_LINE_MAX_SLOPE_Q8 64         // 连续确认时，拟合斜率变化不超过0.25像素/行
#define RING_EXIT_WALL_CLEAR_FRAMES 2         // 横向边界消失2帧后锁定寻直线出环，不再恢复强制右转
#define RING_EXIT_ALIGN_SLOPE_PERMILLE 450       // 地面切线横向/前向位移，不是累计转角
#define RING_EXIT_ALIGN_CENTER_MM 160           // 左边+半车道宽距车体中心的最大横向偏差
#define RING_EXIT_ALIGN_FRAMES 2               // 连续找到方向和位置一致的前方直线后锁定出环
#define RING_REARM_TICKS 80U                    // 结束后400ms内不重认同一环岛
#define RING_TIMEOUT_TICKS 2400U                // 整个入环/环内/出环最多12s
#define RING_EXIT_TIMEOUT_TICKS 600U            // 进入出环后最多3s恢复实测道路；重新接线不重置计时
#if STOPROW < 5 || RING_WALL_SCAN_BOTTOM_ROW + 3 >= SEARCH_IMAGE_H || RING_WALL_FIRST_COL + (RING_WALL_COLUMNS - 1) * RING_WALL_COL_STEP >= SEARCH_IMAGE_W || RING_WALL_MIN_COLUMNS > RING_WALL_COLUMNS
#error Invalid ring transverse-edge observation window.
#endif
#if RING_EXIT_LINE_ROWS < 13 || !(RING_EXIT_LINE_ROWS % 2) || RING_EXIT_LINE_NEAR_ROW > SEARCH_CALIBRATION_MAX_ROW || RING_EXIT_LINE_NEAR_ROW - RING_EXIT_LINE_ROWS + 1 < SEARCH_CALIBRATION_MIN_ROW || RING_EXIT_LINE_EXTEND_ROWS < 0 || RING_EXIT_LINE_EXTEND_ROWS > 40
#error Invalid ring exit straight-line window.
#endif
#if RING_EXIT_ALIGN_FRAMES < 1 || RING_EXIT_ALIGN_FRAMES > 255 || RING_EXIT_WALL_CLEAR_FRAMES < 1 || RING_EXIT_WALL_CLEAR_FRAMES > 255
#error Invalid ring exit straight-line confirmation settings.
#endif

// APPROACH等待入口开口；状态1/2以丢线/恢复引导入环；状态3按真实环道巡线；
// 状态4以横向外边界支持转弯；引导结束后状态5寻找前方直线并补线出环，不回到固定右转。
#define RING_STATE_NONE 0                       // 普通巡线/等待入口确认
#define RING_STATE_STAGE1 1                     // 入环引导，观察近端左边丢失
#define RING_STATE_STAGE2 2                     // 左边恢复后释放入环补线
#define RING_STATE_STAGE3 3                     // 实测路径巡环，观察横向外边界
#define RING_STATE_STAGE4 4                     // 红点转弯区，按当前横向边界引导右转
#define RING_STATE_EXIT_FOLLOW 5                // 寻找前方左直边并补线出环，不再回到强制右转
#define RING_STATE_APPROACH 6                   // 已识别独立岛边，沿实测左边等待右侧开口；不画入环斜线

// ============================================
// 中线来源与质量
// ============================================
#define CENTER_NONE 0                           // 无可用中线
#define CENTER_MEASURED 1                       // 当前实测双边的中点
#define CENTER_GROUND 2                         // 单边毫米法线偏移的反投影中线
#define CENTER_CROSS 3                          // 十字补线参与的中线
#define CENTER_HISTORY 4                        // 历史中线，质量仅为1
#define CENTER_BRIDGED 5                        // 小缺口补边参与的中点

#define CENTER_RING 6                           // 入/出环引导中线，仅当前有白区且边界未交叉时有效

// ============================================
// 全局视觉结果
// ============================================
extern uint8 zebra_candidate;                   // 当前灰度帧是否具有多行连续斑马线条纹
extern uint8 zebra_detected;                    // 当前连续确认是否达到要求；不代表 Run 的停车锁定已解除
extern uint8 left_edge_line[SEARCH_IMAGE_H];    // 左边界列坐标，丢失时为 0，需结合 left_valid 判断
extern uint8 right_edge_line[SEARCH_IMAGE_H];   // 右边界列坐标，丢失时为 W-1，需结合 right_valid 判断
extern uint8 left_valid[SEARCH_IMAGE_H];        // 左边界来源，0 无效，其他值见 EDGE_*
extern uint8 right_valid[SEARCH_IMAGE_H];       // 右边界来源，0 无效，其他值见 EDGE_*
extern uint8 center_line[SEARCH_IMAGE_H];       // 逐行中线列坐标
extern uint8 center_valid[SEARCH_IMAGE_H];      // 中线存在标记，包含历史结果，控制还需检查质量
extern uint8 center_source[SEARCH_IMAGE_H];     // 中线来源，见 CENTER_*
extern uint8 center_quality[SEARCH_IMAGE_H];    // 中线质量：0 无效、1 历史、2 几何或补线、3 实测双边
extern uint8 cross_flag;                        // 当前十字状态
extern uint8 cross_candidate;                   // 本帧十字证据，补线后仍保留用于中线仲裁
extern uint8 path_lost;                         // 当帧质量至少为 2 的中线不足图像高度的十分之一
extern uint8 cross_single_side;                 // 单侧十字可信边：0 无指定、1 左、2 右
// ring_candidate 为完整入口几何候选；ring_confirm_count 为连续特征命中帧数。
// ring_dir 在处理圆环时由状态机赋值，跟外边出环时可为 NONE；是否处于圆环以 ring_state 为准。
// 当前代码只检测右圆环。
extern uint8 ring_candidate;                    // 本帧入口几何候选，与 IMU 是否就绪无关
extern uint8 ring_brake_hint;                   // 仅限速的提前预警，不改变圆环状态或转向路径
extern uint8 ring_dir;                          // 当前方向标志，跟外边出环时可为 NONE
extern uint8 ring_state;                        // 实际启用的圆环状态，判断是否在环中应检查它
extern uint8 ring_exit_guiding;                 // 本帧有横向外边界支持的转弯补线
extern uint8 ring_follow_outer;                 // 出环只使用左外边/其直线补线，禁止右侧环岛拉偏中线
extern uint8 ring_exit_straight;                // 本帧已根据前方实测直线生成EDGE_RING左边补线
extern uint8 ring_exit_ready;                   // 本帧锁定出口直线/恢复双直边；速度由独立的出环延时控制
// ============================================
// 整帧与分步处理接口
// ============================================
void search_init(void);                         // 重置视觉数组、历史、十字、斑马线和右圆环状态，并初始化毫米标定。
// 带时间戳的整帧入口；ticks 是 Motor_GetTicks() 的5ms计数，用于圆环进度/断帧检查，可自然回绕。
void search_process_timed(const uint8 *image, uint16 ticks);
// 分段接口：extract 返回后不再读取 image，可释放相机帧；随后必须完成 finish，才能处理下一帧。
void search_extract_edges_timed(const uint8 *image, uint16 ticks);
void search_finish_frame(void);
#endif
