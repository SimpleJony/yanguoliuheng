/* Keep the motor/peripheral interrupt chain at the on-car verified level.
 * Image processing may use O7; Keil IDE and command-line builds both obey this. */
#if defined(__C251__)
#pragma OPTIMIZE(3,SPEED)
#endif

#include "Motor.h"
#include "run_config.h"
#include "search_geometry.h"
#include "imu.h"

volatile int16 encoderL, encoderR, duty, setSpeed, currentSpeed, handle = 0;
static volatile int16 R;
int16 servoP, aimPos, speedLimit, curveCoeff, currentDuty = 0;
volatile int16 setSpeed_L, setSpeed_R, enableDifSpeed = 1;
const int16 tani[33];

volatile PID motorLPid, motorRPid, servoPid;
static int16 motorSpeedKp = 3000;
static int16 motorSpeedKi = 50;
static int16 motorSpeedKd = 50;
static uint8 motorSpeedConfigured = 0;
static volatile uint8 motorEnabled = 0;
static uint8 motorInitialized = 0;
static volatile uint8 motorRunMode = 0;
static volatile uint8 trackingTicks = 0;
static volatile uint16 motorTicks = 0;
static volatile int16 steeringError = 0;
static volatile int16 steeringAngle10 = 0;
static volatile int16 steeringPwm = servoMid;
static volatile int16 steeringOffset = 0;
static volatile uint8 pathRejected = 0;
static volatile uint8 lostPathFrames;
#if !RUN_FIXED_SPEED_MODE
static volatile uint8 previewSpeedInitialized;
static uint8 previewHigherFrames;
static uint16 previewRecoveryTick, previewAccelTick;
#endif

void PID_Init(volatile PID *pid, uint8 which, int16 Kp, int16 Ki, int16 Kd, int16 attitude_coeff,int16 max_out){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->which = which;
    pid->attitude_coeff = attitude_coeff;
    pid->max_out = max_out;
    pid->last_error = 0;
    pid->llast_error = 0;
    pid->base = 0;
}

void Motor_SetSpeedPID(int16 kp, int16 ki, int16 kd) {
    bit irq_state;
    irq_state = EA;
    EA = 0;
    motorSpeedKp = kp;
    motorSpeedKi = ki;
    motorSpeedKd = kd;

    // Keep both wheel speed loops on the same tuning values.
    motorLPid.Kp = kp;
    motorLPid.Ki = ki;
    motorLPid.Kd = kd;
    motorRPid.Kp = kp;
    motorRPid.Ki = ki;
    motorRPid.Kd = kd;
    EA = irq_state;
}

void Motor_SetServoKp(int16 kp) {
    bit irq_state;
    irq_state = EA;
    EA = 0;
    servoP = func_limit_ab(kp, 0, MOTOR_SERVO_KP_MAX);
    servoPid.Kp = servoP;
    servoPid.last_error = 0;
    EA = irq_state;
}

void Motor_SetSpeedLimit(int16 speed) {
    bit irq_state;
    irq_state = EA;
    EA = 0;
    speedLimit = func_limit_ab(speed, 0, 1000);
    setSpeed = motorEnabled ? speedLimit : 0;
    motorSpeedConfigured = 1;
    EA = irq_state;
}

void Motor_SetEnabled(uint8 enabled) {
    bit irq_state;
    irq_state = EA;
    EA = 0;
    if (enabled == 1 && (!motorRunMode || (trackingTicks && !pathRejected))) {
        if (!motorRunMode) setSpeed = speedLimit;
        motorEnabled = 1;
    } else {
        motorEnabled = 0;
#if !RUN_FIXED_SPEED_MODE
        previewSpeedInitialized = 0;
#endif
        setSpeed = 0;
        trackingTicks = 0;
        duty = 0;
        steeringOffset = 0;
        pathRejected = 0;
        lostPathFrames = 0;
        steeringError = steeringAngle10 = 0;
        R = 0;
        motorLPid.base = 0;
        motorLPid.last_error = 0;
        motorLPid.llast_error = 0;
        motorRPid.base = 0;
        motorRPid.last_error = 0;
        motorRPid.llast_error = 0;
        if (motorInitialized) {
            pwm_set_duty(motorPwm_L, 0);
            pwm_set_duty(motorPwm_R, 0);
        }
    }
    EA = irq_state;
}

void Motor_SetRunMode(uint8 enabled) {
    Motor_SetEnabled(0);
    motorRunMode = enabled ? 1 : 0;
}

uint8 Motor_IsEnabled(void) { return motorEnabled; }

void Motor_GetSteering(motor_steering_state_t *state) {
    bit irq_state;
    irq_state = EA;
    EA = 0;
    state->pixel_error = steeringError;
    state->angle10 = steeringAngle10;
    state->pwm = steeringPwm;
    EA = irq_state;
}

void Motor_RejectPath(void) {
    bit irq_state;
    if (!motorRunMode) { Motor_SetEnabled(0); return; }
    irq_state = EA;
    EA = 0;
    /* Loss may continue a live run, but cannot start a car without a path. */
    if (!trackingTicks) {
        Motor_SetEnabled(0);
        EA = irq_state;
        return;
    }
    if(lostPathFrames < RUN_LOST_CONFIRM_FRAMES) {
        ++lostPathFrames;
#if !RUN_FIXED_SPEED_MODE
        if(lostPathFrames == RUN_LOST_CONFIRM_FRAMES) {
            /* Reduce once only after consecutive lost frames. A fresh path
             * clears the count; single-frame glitches do not force the floor. */
            if(setSpeed > 0) {
                setSpeed = (int16)((int32)setSpeed * RUN_LOST_SPEED_PERCENT / 100);
                setSpeed = (int16)func_limit_ab(setSpeed,
                    func_min(speedLimit, RUN_MIN_SPEED), speedLimit);
            }
        }
#endif
    }
#if !RUN_FIXED_SPEED_MODE
    if(lostPathFrames >= RUN_LOST_CONFIRM_FRAMES) {
        previewHigherFrames = 0;
        previewRecoveryTick = previewAccelTick = motorTicks;
    }
#endif
    pathRejected = 1;
#if RUN_FIXED_SPEED_MODE
    setSpeed = speedLimit;
#else
    if (!speedLimit) setSpeed = 0;
#endif
    motorEnabled = setSpeed > 0;
    /* A processed lost frame is alive; missing camera frames still time out. */
    trackingTicks = RUN_FRAME_TIMEOUT_TICKS;
    EA = irq_state;
}

uint16 Motor_GetTicks(void) {
    uint16 ticks;
    bit irq_state;
    irq_state = EA;
    EA = 0;
    ticks = motorTicks;
    EA = irq_state;
    return ticks;
}

int16 steerPIDCalculate(volatile PID *pid, int16 error) {
    int32 p_term, d_term, att_term;
    int32 pid_output;
    p_term = (int32) pid->Kp * error;
    d_term = (int32) pid->Kd * (error - pid->last_error);
    att_term = imu_ready && pid->attitude_coeff && motorRunMode && motorEnabled && trackingTicks && !pathRejected ?
        (int32) pid->attitude_coeff * imu_steering_gyro_z : 0;
    pid->last_error = error;
    pid_output = (p_term + d_term + att_term) / 100;
    return (int16)func_limit(pid_output, pid->max_out);
}

int16 motorPIDCalculate(volatile PID *pid, int16 target, int16 actual) {
    int32 err, delta_err, delta_u;
    err = target - actual;
    delta_err = err - pid->last_error;
    delta_u = (int32) pid->Kp * delta_err +
                 (int32) pid->Ki * err +
                 (int32) pid->Kd * (delta_err - (pid->last_error - pid->llast_error));
    pid->llast_error = pid->last_error;
    pid->last_error = err;
    // Clamp before narrowing to int16 (large starting errors can overflow).
    delta_u = (int32)pid->base + delta_u / 100;
    pid->base = (int16)func_limit(delta_u, pid->max_out);
    gpio_set_level((gpio_pin_enum) pid->which ? motorDir_R : motorDir_L, pid->base >= 0);
    return func_abs(pid->base);
}

static int16 motor_steering_step(int16 delta) {
    int16 step;
    step = func_abs(delta) > RUN_SERVO_FAST_ERROR_PWM ?
        RUN_SERVO_FAST_SLEW_PWM_PER_TICK : RUN_SERVO_SLEW_PWM_PER_TICK;
    if (imu_ready && motorRunMode && motorEnabled && trackingTicks && !pathRejected) {
        step = (int16)(((int32)step * imu_steering_slew_percent + 99) / 100);
        if (step < 1) step = 1;
    }
    return step;
}

static void pitHandler(){
    int16 target_offset, delta, step, drive_speed;
    ++motorTicks;
    if (motorRunMode && trackingTicks) {
        if (--trackingTicks == 0) {
            motorEnabled = 0;
#if !RUN_FIXED_SPEED_MODE
            previewSpeedInitialized = 0;
#endif
            setSpeed = 0;
            duty = 0;
            steeringOffset = 0;
            pathRejected = 0;
        lostPathFrames = 0;
            steeringError = steeringAngle10 = 0;
            R = 0;
        }
    }
    encoderR = (int16)(-2L * encoder_get_count(PWMA_ENCODER));
    encoderL = (int16)(2L * encoder_get_count(PWMC_ENCODER));
    encoder_clear_count(PWMA_ENCODER);
    encoder_clear_count(PWMC_ENCODER);
    /* The camera supplies a held target; approach it every timer tick instead
     * of applying a complete frame-to-frame jump. No float math in the ISR.
     * Explicit stop/watchdog reset the offset; path loss holds it. */
    target_offset = pathRejected ? steeringOffset : steerPIDCalculate(&servoPid, duty);
    delta = target_offset - steeringOffset;
    step = motor_steering_step(delta);
    steeringOffset += func_limit(delta, step);
    steeringPwm = servoMid + steeringOffset;
    pwm_set_duty(servoPwm, steeringPwm);
    currentSpeed = (int16)(((int32)encoderR + encoderL) / 2);
    if (!motorEnabled) {
        setSpeed_L = 0;
        setSpeed_R = 0;
        motorLPid.base = 0;
        motorLPid.last_error = 0;
        motorLPid.llast_error = 0;
        motorRPid.base = 0;
        motorRPid.last_error = 0;
        motorRPid.llast_error = 0;
        pwm_set_duty(motorPwm_L, 0);
        pwm_set_duty(motorPwm_R, 0);
        return;
    }
    drive_speed = setSpeed;
    setSpeed_L = drive_speed - (!R || !enableDifSpeed ? 0 : (int32)drive_speed * 8 / R);
    setSpeed_R = drive_speed + (!R || !enableDifSpeed ? 0 : (int32)drive_speed * 8 / R);
    pwm_set_duty(motorPwm_L, motorPIDCalculate(&motorLPid, setSpeed_L, encoderL));
    pwm_set_duty(motorPwm_R, motorPIDCalculate(&motorRPid, setSpeed_R, encoderR));
}

/* Millimetre pure pursuit with 0.1 degree table interpolation. */
static int16 motor_pursuit_angle10(int16 forward, int16 lateral) {
    int32 denominator, tangent;
    float ratio;
    int16 degree, angle;
    denominator = (int32)forward * forward + (int32)lateral * lateral;
    if(forward <= 0 || !denominator || !lateral) return 0;
    /* Float numerator avoids 32-bit overflow for valid millimetre points. */
    ratio = 2000.0f * RUN_WHEELBASE_MM * func_abs((int32)lateral) / denominator;
    tangent = ratio >= tani[32] ? tani[32] : (int32)(ratio + 0.5f);
    if(tangent >= tani[32]) angle = 320;
    else {
        degree = 0;
        while(degree < 31 && tangent >= tani[degree + 1]) ++degree;
        angle = degree * 10 + (int16)(10L * (tangent - tani[degree]) /
            (tani[degree + 1] - tani[degree]));
    }
    return lateral < 0 ? -angle : angle;
}

#if !RUN_FIXED_SPEED_MODE
/* Preview geometry, independent of Servo Kp and gyro damping. Speed is the
 * mean wheel target; differential steering is applied afterwards. */
static int16 motor_preview_speed(int16 angle10) {
    int16 bend, floor_speed;
    floor_speed = (int16)func_min(speedLimit, RUN_MIN_SPEED);
    bend = (int16)func_abs((int32)angle10);
    if (bend <= RUN_SPEED_STRAIGHT_ANGLE10) return speedLimit;
    if (bend >= RUN_SPEED_FULL_ANGLE10) return floor_speed;
    return speedLimit - (int16)((int32)(speedLimit - floor_speed) *
        (bend - RUN_SPEED_STRAIGHT_ANGLE10) /
        (RUN_SPEED_FULL_ANGLE10 - RUN_SPEED_STRAIGHT_ANGLE10));
}

/* Use three consecutive supported samples to reject isolated far-point spikes.
 * Work on the same continuous fresh path as steering, never history/extrapolation.
 * Decimation limits extra projection work on the MCU. */
static int16 motor_speed_preview(Point roadLine[], int16 count, int16 steering_angle) {
    int16 i, x, y, a, prev1=0, prev2=0, candidate, best;
    uint8 support=0;
    int32 radius2;
    best = (int16)func_abs((int32)steering_angle);
    for(i=0; i<count; i+=4) {
        if(!search_project_point(roadLine[i].y, roadLine[i].x, &x, &y)) {
            support=0; continue;
        }
        x -= RUN_REAR_AXLE_X_MM;
        y -= RUN_REAR_AXLE_Y_MM;
        radius2=(int32)x*x+(int32)y*y;
        if(y < RUN_LOOKAHEAD_MIN_MM || radius2 > (int32)RUN_SPEED_LOOKAHEAD_MM*RUN_SPEED_LOOKAHEAD_MM) {
            support=0; continue;
        }
        a=motor_pursuit_angle10(y,-x);
        if(support>=2 && ((a>=0 && prev1>=0 && prev2>=0) ||
                         (a<=0 && prev1<=0 && prev2<=0))) {
            candidate=(int16)func_min(func_abs((int32)a),
                func_min(func_abs((int32)prev1),func_abs((int32)prev2)));
            if(candidate>best) best=candidate;
        }
        prev2=prev1; prev1=a;
        if(support<2) ++support;
    }
    return best;
}

/* Called with interrupts masked when publishing a new camera target.
 * Immediate lower request; rising requests require sustained higher speed allowance.
 * No accumulated catch-up acceleration after delayed/missing camera frames. */
static int16 motor_recover_speed(int16 request) {
    if(!previewSpeedInitialized || request<=setSpeed) {
        previewSpeedInitialized=1;
        previewHigherFrames=0;
        previewRecoveryTick=previewAccelTick=motorTicks;
        return request;
    }
    if(!previewHigherFrames) previewRecoveryTick=previewAccelTick=motorTicks;
    if(previewHigherFrames<3) ++previewHigherFrames;
    if(previewHigherFrames<3 || (uint16)(motorTicks-previewRecoveryTick)<RUN_SPEED_RECOVER_HOLD_TICKS)
        return setSpeed;
    if((uint16)(motorTicks-previewAccelTick)>=RUN_SPEED_ACCEL_TICKS) {
        previewAccelTick=motorTicks;
        return (int16)func_min((int32)setSpeed+RUN_SPEED_ACCEL_STEP,request);
    }
    return setSpeed;
}
#endif

static void motor_set_steering(int16 angle10, int16 pixel_error, int16 preview_angle) {
    int16 t, r;
#if !RUN_FIXED_SPEED_MODE
    int16 next_speed;
#endif
    bit irq_state;
    angle10 = func_limit(angle10, 320);
    currentDuty = (int16)((int32)angle10 * servoP / 1000);
    currentDuty = func_limit(currentDuty, 32);
    t = tani[func_abs(currentDuty)];
    r = !t ? 0 : (int16)((int32)RUN_WHEELBASE_MM * 100 / t); /* radius, cm */
#if !RUN_FIXED_SPEED_MODE
    next_speed = motor_preview_speed(preview_angle);
#endif
    irq_state = EA;
    EA = 0;
#if RUN_FIXED_SPEED_MODE
    setSpeed = speedLimit;
#else
    if (!handle) setSpeed = motor_recover_speed(next_speed);
#endif
    R = angle10 < 0 ? -r : r;
    steeringError = pixel_error;
    steeringAngle10 = angle10;
    duty = (int16)((int32)angle10 * 12 / 10);
    if (motorRunMode) trackingTicks = RUN_FRAME_TIMEOUT_TICKS;
    pathRejected = 0;
    lostPathFrames = 0;
    EA = irq_state;
}

int16 Motor_SelectTarget(Point roadLine[], int16 count) {
    int16 i, x, y, best, lookahead, lateral;
    int32 radius2, error, best_error;
    best = -1;
    best_error = 2147483647L;
    if (!roadLine) return -1;
    lookahead = RUN_LOOKAHEAD_MM;
    if (count > 0 && search_project_point(roadLine[0].y, roadLine[0].x, &x, &y)) {
        lateral = (int16)func_abs((int32)x - RUN_REAR_AXLE_X_MM);
        if (lateral > RUN_LATERAL_FULL_CORRECTION_MM) lateral = RUN_LATERAL_FULL_CORRECTION_MM;
        lookahead -= (int16)((int32)(RUN_LOOKAHEAD_MM - RUN_LOOKAHEAD_MIN_MM) *
            lateral / RUN_LATERAL_FULL_CORRECTION_MM);
    }
    for (i = 0; i < count; ++i) {
        if (!search_project_point(roadLine[i].y, roadLine[i].x, &x, &y)) continue;
        x -= RUN_REAR_AXLE_X_MM;
        y -= RUN_REAR_AXLE_Y_MM;
        if (y <= 0) continue;
        radius2 = (int32)x * x + (int32)y * y;
        error = radius2 - (int32)lookahead * lookahead;
        if (error < 0) error = -error;
        if (error < best_error) { best_error = error; best = i; }
    }
    return best;
}

void alongLine(Point roadLine[], int16 count, int16 aimPos) {
    int16 x, y, angle, preview;
    if (!roadLine || count < RUN_MIN_PATH_POINTS || aimPos < 0 || aimPos >= count ||
        !search_project_point(roadLine[aimPos].y, roadLine[aimPos].x, &x, &y)) {
        Motor_SetEnabled(0);
        return;
    }
    x -= RUN_REAR_AXLE_X_MM;
    y -= RUN_REAR_AXLE_Y_MM;
    if (y <= 0) { Motor_SetEnabled(0); return; }
    angle = motor_pursuit_angle10(y, -x);
    preview = angle;
#if !RUN_FIXED_SPEED_MODE
    preview = motor_speed_preview(roadLine, count, angle);
#endif
    /* Publish direction/speed/watchdog together with interrupts masked. */
    motor_set_steering(angle, MT9V03X_W / 2 - roadLine[aimPos].y, preview);
}

void setServoDuty(int8 d) {
    d = (int8)func_limit(d, 32);
    motor_set_steering((int16)d * 10, 0, (int16)d * 10);
}

void Motor_Init(int16 _servoP, int16 _aimPos, int16 _curveCoeff, int16 _speedLimit){
    bit irq_state;
    servoP = func_limit_ab(_servoP, 0, MOTOR_SERVO_KP_MAX);
    aimPos = _aimPos;
    curveCoeff = _curveCoeff;
    if (!motorSpeedConfigured) speedLimit = _speedLimit;

    PID_Init(&servoPid, 39, servoP, 0, 0, IMU_STEERING_GYRO_COEFF, RUN_SERVO_MAX_OFFSET);
    PID_Init(&motorLPid, motorL, motorSpeedKp, motorSpeedKi, motorSpeedKd, 0, 4000);
    PID_Init(&motorRPid, motorR, motorSpeedKp, motorSpeedKi, motorSpeedKd, 0, 4000);
    pwm_init(servoPwm, 200, servoMid);
    gpio_init(motorDir_L, GPO, 1, GPO_PUSH_PULL);
    gpio_init(motorDir_R, GPO, 1, GPO_PUSH_PULL);
    pwm_init(motorPwm_L, 17000, 0);
    pwm_init(motorPwm_R, 17000, 0);
    motorInitialized = 1;
    encoder_quad_init(PWMA_ENCODER, PWMA_ENCODER_CH1P_P60, PWMA_ENCODER_CH2P_P62);
    encoder_quad_init(PWMC_ENCODER, PWMC_ENCODER_CH1P_P40, PWMC_ENCODER_CH2P_P42);
    system_delay_ms(200);
    /* Driver enables the timer before assigning its multi-byte callback.
     * Publish the entire callback before an interrupt can dispatch it. */
    irq_state = EA;
    EA = 0;
    pit_ms_init(TIM1_PIT, 5, pitHandler);
    EA = irq_state;
}

int8 getAimPos() {return aimPos;}

// tan查表
const int16 tani[33] = {0,
                        17,  35,  52,  70,  87,  105, 123, 141, 158, 176,
                        194, 213, 231, 249, 268, 287, 306, 325, 344, 364,
                        384, 404, 424, 445, 466, 488, 510, 532, 554, 577, 600, 625};
