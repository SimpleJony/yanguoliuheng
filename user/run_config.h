#ifndef RUN_CONFIG_H_
#define RUN_CONFIG_H_

/* Measured vehicle pose in the 2026-10-19 calibration coordinates (mm).
 * Rear axle centered behind the near stripe, heading aligned with +Y.
 * Positive steering follows the original Motor.c convention: left. */
#define RUN_WHEELBASE_MM           200
#define RUN_REAR_AXLE_X_MM           0
#define RUN_REAR_AXLE_Y_MM        (-500)
/* Rear-axle steering target radius, mm. Prefer a farther target while keeping
 * near-path lateral correction. Selection stays on the available calibrated
 * continuous path; a larger radius cannot extend camera visibility.
 * Final servo PWM still follows the smooth trajectory below. */
#define RUN_LOOKAHEAD_MM           730
#define RUN_LOOKAHEAD_MIN_MM       530
#if RUN_LOOKAHEAD_MIN_MM <= 0 || RUN_LOOKAHEAD_MIN_MM > RUN_LOOKAHEAD_MM
#error Lookahead minimum must be positive and no greater than nominal.
#endif
/* Reach the shortest steering radius at 100 mm of near-path lateral error. */
#define RUN_LATERAL_FULL_CORRECTION_MM 100
#if RUN_LATERAL_FULL_CORRECTION_MM < 1
#error Lateral correction range must be positive.
#endif
/* 普通实测路径出现横向回折时，将瞄点留在第一段可达弯曲处。
 * 按前向20mm抽样，至少50mm摆幅、回收40mm且连续3点支持，排除像素抖动。
 * 仍遵守最小530mm瞄点半径；直线、单调弯及元素补线沿用原选点。 */
#define RUN_S_BEND_SAMPLE_MM          20
#define RUN_S_BEND_SWING_MM           50
#define RUN_S_BEND_RETURN_MM          40
#define RUN_S_BEND_CONFIRM_SAMPLES     3
#define RUN_S_BEND_ANGLE_DROP10       20
#if RUN_S_BEND_SAMPLE_MM < 10 || RUN_S_BEND_SWING_MM < 30 || RUN_S_BEND_RETURN_MM < 20 || RUN_S_BEND_CONFIRM_SAMPLES < 3 || RUN_S_BEND_ANGLE_DROP10 < 10
#error Invalid supported S-bend target settings.
#endif
/* 出环补线引导的目标转角上限，单位0.1度，尚未乘Servo Kp。
 * 仅限制红点处的出环引导；跟外边出环后恢复实测路径转角。 */
#define RUN_RING_EXIT_MAX_ANGLE10 220
#if RUN_RING_EXIT_MAX_ANGLE10 < 1 || RUN_RING_EXIT_MAX_ANGLE10 > 320
#error Invalid ring exit guide angle limit.
#endif
/* Ordinary measured bends: move the pursuit target toward the inside.
 * Ground X offset, not pixels or extra servo gain. Set MAX_MM=0 to disable.
 * Require two consistent changes in path slope over 100 mm segments.
 * Preserve at least 120 mm of lateral target-to-inner-edge clearance. */
#ifndef RUN_CURVE_INSIDE_MAX_MM
#define RUN_CURVE_INSIDE_MAX_MM     35
#endif
#define RUN_CURVE_INSIDE_MARGIN_MM 120
#define RUN_CURVE_SAMPLE_MM       100
/* Change in slope (x/y * 1000), normalized to 100 mm of forward travel. */
#define RUN_CURVE_START_SLOPE       60
#define RUN_CURVE_FULL_SLOPE       180
#if RUN_CURVE_INSIDE_MAX_MM < 0 || RUN_CURVE_INSIDE_MAX_MM > 60 || RUN_CURVE_INSIDE_MARGIN_MM < 100
#error Invalid inside-bend offset or edge clearance.
#endif
#if RUN_CURVE_SAMPLE_MM < 80 || RUN_CURVE_SAMPLE_MM > 150 || RUN_CURVE_START_SLOPE < 1 || RUN_CURVE_FULL_SLOPE <= RUN_CURVE_START_SLOPE
#error Invalid bend-shape thresholds.
#endif
#define RUN_MAX_PATH_START_MM      500
/* Keep the existing acceptance range for fresh single-edge ring paths.
 * Shortening the desired steering radius must not reject those paths. */
#define RUN_MAX_GROUND_PATH_START_MM 720
/* Run 平均目标速度，单位为编码器计数/5ms；左右轮仍按转向差速。
 * 直线速度读取菜单 Speed（含 EEPROM 保存值）；CURVE/RING 独立配置。
 * 圆环从开口预警/候选检测起使用 RING，实际入环后从圆环状态结束起再保持2秒，
 * 然后恢复正常直线/弯道分类；延时期间失线仍用圆环档。停车和断帧保护立即生效。 */
/* EEPROM 中没有有效菜单参数时，直线/手动测试速度的默认值。 */
#ifndef RUN_SPEED_STRAIGHT_DEFAULT
#define RUN_SPEED_STRAIGHT_DEFAULT 280
#endif
#ifndef RUN_SPEED_CURVE
#define RUN_SPEED_CURVE            170
#endif
#ifndef RUN_SPEED_RING
#define RUN_SPEED_RING             150
#endif
/* 出环后的圆环档保持时间：400个5ms节拍=2秒，不依赖图像帧率。 */
#define RUN_RING_EXIT_HOLD_TICKS    400U
#if RUN_RING_EXIT_HOLD_TICKS < 1 || RUN_RING_EXIT_HOLD_TICKS > 65535U
#error Invalid ring exit hold duration (5ms ticks).
#endif
#if RUN_SPEED_STRAIGHT_DEFAULT < 0 || RUN_SPEED_STRAIGHT_DEFAULT > 1000 || RUN_SPEED_CURVE < 1 || RUN_SPEED_CURVE > 1000 || RUN_SPEED_RING < 1 || RUN_SPEED_RING > 1000
#error Straight default must be 0..1000; curve/ring speeds must be 1..1000.
#endif
/* 5ms 定时中断线性切换，20拍=100ms；重复图像请求不重启计时。 */
#define RUN_SPEED_TRANSITION_TICKS  20U
/* Independent speed preview, measured from rear axle; calibrated fresh path only. */
#define RUN_SPEED_LOOKAHEAD_MM    1150
#if RUN_SPEED_LOOKAHEAD_MM < RUN_LOOKAHEAD_MM
#error Speed preview must cover the steering lookahead.
#endif
/* 纯追踪预瞄转角（0.1度）：超过3度进入弯道，回落至2度恢复直线。
 * 滞回抑制阈值附近反复切换，不额外等待固定帧数或延时。 */
#define RUN_SPEED_CURVE_ENTER_ANGLE10 30
#define RUN_SPEED_CURVE_EXIT_ANGLE10  20
#if RUN_SPEED_CURVE_EXIT_ANGLE10 < 0 || RUN_SPEED_CURVE_ENTER_ANGLE10 <= RUN_SPEED_CURVE_EXIT_ANGLE10 || RUN_SPEED_CURVE_ENTER_ANGLE10 > 320
#error Invalid straight/curve thresholds.
#endif
/* Extra IMAGE rows beyond the metric calibration: braking evidence only.
 * No metric extrapolation and no changes to the steering target.
 * Three same-direction samples, two rows apart, reject isolated noise.
 * Error is percentage of observed lane width, not an angle in degrees.
 * With calibration starting at row 38, 26 extra rows reach STOPROW=12.
 * 远端连续证据只用于选择弯道档，不生成第四档限速。 */
#define RUN_SPEED_EXTRA_ROWS          26
#define RUN_SPEED_FAR_ENTER_PERCENT    6
#define RUN_SPEED_FAR_EXIT_PERCENT     4
/* 远端最后12行连续实测双边、中线回到车道宽的6%内，且近端舵角不超过6度，
 * 才允许提前恢复直线速度；中段预瞄或远端仍有入弯证据时优先保持弯道档。 */
#define RUN_SPEED_EXIT_STRAIGHT_ROWS  12
#define RUN_SPEED_EXIT_FAR_PERCENT     6
#define RUN_SPEED_EXIT_MAX_ANGLE10    60
#if RUN_SPEED_EXTRA_ROWS < 6 || RUN_SPEED_EXTRA_ROWS > 26 || RUN_SPEED_FAR_EXIT_PERCENT < 0 || RUN_SPEED_FAR_ENTER_PERCENT <= RUN_SPEED_FAR_EXIT_PERCENT
#error Invalid far-image speed preview settings.
#endif
#if RUN_SPEED_EXIT_STRAIGHT_ROWS < 6 || RUN_SPEED_EXIT_STRAIGHT_ROWS > RUN_SPEED_EXTRA_ROWS + 1 || RUN_SPEED_EXIT_FAR_PERCENT < 0 || RUN_SPEED_EXIT_FAR_PERCENT > RUN_SPEED_FAR_ENTER_PERCENT || RUN_SPEED_EXIT_MAX_ANGLE10 < RUN_SPEED_CURVE_ENTER_ANGLE10 || RUN_SPEED_EXIT_MAX_ANGLE10 > 100
#error Invalid early exit speed settings.
#endif
#define RUN_SERVO_GAIN             150
#define RUN_MIN_PATH_POINTS        12
#define RUN_MAX_CENTER_STEP        10
/* Near path acceptance uses RUN_MAX_PATH_START_MM, not a pixel row gap. */
/* 0: blank once on entry; only a latched zebra stop writes a one-time notice.
 * Set to 1 and rebuild to restore image/overlay/status diagnostics. */
#ifndef RUN_DISPLAY_ENABLED
#define RUN_DISPLAY_ENABLED         0
#endif
#define RUN_DISPLAY_PERIOD_TICKS   40U  /* 调试图像和状态最多每 200ms 刷新，控制仍处理每个新帧 */
#define RUN_FRAME_TIMEOUT_TICKS    50U  /* 250 ms without a processed frame */
/* Hold through isolated bad frames; confirmed loss still reduces speed. */
#define RUN_LOST_CONFIRM_FRAMES      3U
#if RUN_LOST_CONFIRM_FRAMES < 1 || RUN_LOST_CONFIRM_FRAMES > 255
#error Invalid lost-frame confirmation count.
#endif
#define RUN_SERVO_MAX_OFFSET       384
/* 舵机最终PWM每5ms更新：从小步开始，逐步加快，按剩余距离提前放缓。
 * MAX是每个节拍的最大步长；ACCEL是步长每节拍允许增加的量。
 * 不滤波或修改视觉目标角度；反向指令立即取消旧方向的步长。
 * 失线保持已下发位置，停车/相机超时清零，不继续执行旧轨迹。 */
#define RUN_SERVO_SLEW_MAX_PWM_PER_TICK    64
#define RUN_SERVO_SLEW_ACCEL_PWM_PER_TICK  12
#if RUN_SERVO_SLEW_ACCEL_PWM_PER_TICK < 1 || RUN_SERVO_SLEW_MAX_PWM_PER_TICK < RUN_SERVO_SLEW_ACCEL_PWM_PER_TICK || RUN_SERVO_SLEW_MAX_PWM_PER_TICK > 96
#error Invalid steering trajectory settings (1 <= ACCEL <= MAX <= 96).
#endif

#endif
