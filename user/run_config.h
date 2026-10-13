#ifndef RUN_CONFIG_H_
#define RUN_CONFIG_H_

/* Measured vehicle pose in the 2026-09-23 calibration coordinates (mm).
 * Rear axle centered behind the near stripe, heading aligned with +Y.
 * Positive steering follows the original Motor.c convention: left. */
#define RUN_WHEELBASE_MM           200
#define RUN_REAR_AXLE_X_MM           0
#define RUN_REAR_AXLE_Y_MM        (-500)
/* Rear-axle target radius, mm. Smaller responds sooner; larger is smoother. */
#define RUN_LOOKAHEAD_MM           800
#define RUN_LOOKAHEAD_MIN_MM       500
#if RUN_LOOKAHEAD_MIN_MM <= 0 || RUN_LOOKAHEAD_MIN_MM > RUN_LOOKAHEAD_MM
#error Lookahead minimum must be positive and no greater than nominal.
#endif
#define RUN_LATERAL_FULL_CORRECTION_MM 100
#define RUN_MAX_PATH_START_MM      500
/* A fresh single-edge normal offset may start farther ahead on a ring.
 * Never accept its first point beyond the nominal lookahead distance. */
#define RUN_MAX_GROUND_PATH_START_MM RUN_LOOKAHEAD_MM
/* 1: keep the Set PID Speed target through curves, weak paths, steering lag
 * and path loss. 0: use preview-angle speed reduction with RUN_MIN_SPEED floor.
 * This fixes the mean wheel speed target; differential steering remains.
 * Manual stop, Speed=0 and camera frame timeout still stop the motors. */
#ifndef RUN_FIXED_SPEED_MODE
#define RUN_FIXED_SPEED_MODE        0
#endif
#if RUN_FIXED_SPEED_MODE != 0 && RUN_FIXED_SPEED_MODE != 1
#error RUN_FIXED_SPEED_MODE must be 0 or 1.
#endif
/* Pure-pursuit preview angle BEFORE Servo Kp, in tenths of a degree.
 * <=5 deg: Set PID Speed; >=20 deg: floor; interpolate between them. */
/* Used when no valid Speed setting is stored in EEPROM. */
#define RUN_DEFAULT_SPEED         230
#define RUN_MIN_SPEED             130
/* Independent speed preview, measured from rear axle; calibrated fresh path only. */
#define RUN_SPEED_LOOKAHEAD_MM    1050
#define RUN_SPEED_RECOVER_HOLD_TICKS 40U /* 200 ms higher-speed requests */
#define RUN_SPEED_ACCEL_TICKS       10U /* increase at most once per 50 ms */
#define RUN_SPEED_ACCEL_STEP         2  /* speed units per interval */
#if RUN_SPEED_LOOKAHEAD_MM < RUN_LOOKAHEAD_MM || RUN_SPEED_RECOVER_HOLD_TICKS < 1 || RUN_SPEED_ACCEL_TICKS < 1 || RUN_SPEED_ACCEL_STEP < 1
#error Invalid independent speed preview/recovery settings.
#endif
#define RUN_SPEED_STRAIGHT_ANGLE10  50
#define RUN_SPEED_FULL_ANGLE10     200
#if RUN_MIN_SPEED < 1 || RUN_SPEED_STRAIGHT_ANGLE10 < 0 || RUN_SPEED_FULL_ANGLE10 <= RUN_SPEED_STRAIGHT_ANGLE10 || RUN_SPEED_FULL_ANGLE10 > 320
#error Invalid preview speed settings.
#endif
#define RUN_LATERAL_SLOW_MM         60
#define RUN_SERVO_GAIN             150
#define RUN_LOOKAHEAD_ROWS         35 /* legacy Motor_Init API only; Run uses mm */
#define RUN_CURVE_COEFFICIENT      100
#define RUN_MIN_PATH_POINTS        12
#define RUN_MAX_CENTER_STEP        10
/* Near path acceptance uses RUN_MAX_PATH_START_MM, not a pixel row gap. */
/* 0: blank once on entry, then no LCD work in the Run processing loop.
 * Set to 1 and rebuild to restore image/overlay/status diagnostics. */
#ifndef RUN_DISPLAY_ENABLED
#define RUN_DISPLAY_ENABLED         0
#endif
#define RUN_DISPLAY_PERIOD_TICKS   10U  /* 50 ms, motor timer ticks */
#define RUN_FRAME_TIMEOUT_TICKS    50U  /* 250 ms without a processed frame */
/* Hold through isolated bad frames; confirmed loss still reduces speed. */
#define RUN_LOST_CONFIRM_FRAMES      3U
#if RUN_LOST_CONFIRM_FRAMES < 1 || RUN_LOST_CONFIRM_FRAMES > 255
#error Invalid lost-frame confirmation count.
#endif
#define RUN_LOST_SPEED_PERCENT     30   /* Percent of speed before path loss */
#if RUN_LOST_SPEED_PERCENT < 1 || RUN_LOST_SPEED_PERCENT > 100
#error RUN_LOST_SPEED_PERCENT must be in 1..100.
#endif
#define RUN_SERVO_MAX_OFFSET       384
/* Limit the final PWM change each 5 ms, after Servo Kp and output clamping.
 * Smaller = smoother/slower, larger = faster. Explicit stop/watchdog bypass
 * the ramp; an invalid image path holds the actual steering output. */
#define RUN_SERVO_SLEW_PWM_PER_TICK  6
#define RUN_SERVO_FAST_SLEW_PWM_PER_TICK 18
#define RUN_SERVO_FAST_ERROR_PWM    72
#if RUN_SERVO_SLEW_PWM_PER_TICK < 1
#error RUN_SERVO_SLEW_PWM_PER_TICK must be positive.
#endif

#endif
