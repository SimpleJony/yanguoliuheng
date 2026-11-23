/* 电机/舵机控制：前台根据当前路径发布转向与速度，TIM1 每5ms执行舵机限速、编码器反馈、双轮PID和断帧停车。
 * 当前 Run 使用图像中线 alongLine，由标定坐标计算纯追踪转向。
 * 含 irq_state 的函数用它保存并恢复 EA，保证主循环与中断共享控制状态的一致性。 */
/* Keep the motor/peripheral interrupt chain at the on-car verified level.
 * Image processing may use O7; Keil IDE and command-line builds both obey this. */
#if defined(__C251__)
#pragma OPTIMIZE(3,SPEED)
#endif

#include "Motor.h"
#include "run_config.h"
#include "search_geometry.h"
#include "search_calibration.h"
#include "imu.h"

typedef struct {
    int16 Kp, Ki, Kd; /* 比例、积分、微分参数；不同控制环采用各自的缩放公式。 */
    uint8 which; /* 控制通道标识，轮速PID用于选择左右轮方向引脚。 */
    int16 attitude_coeff; /* 舵机陀螺阻尼，PWM/(10度/秒)；轮速环不使用。 */
    int16 last_error; /* 上一次误差。 */
    int16 llast_error; /* 上上次误差，供增量PID使用。 */
    int16 base; /* 增量PID累计输出基值。 */
    int16 max_out; /* 输出绝对值上限。 */
} PID;

/* setSpeed/currentSpeed：平均目标/反馈速度，单位为编码器计数/5ms。 */
volatile int16 setSpeed, currentSpeed;
static int16 encoderL, encoderR;
static volatile int16 duty;
/* 菜单 Speed 同时用于直线档和 Set PID 页手动测试。 */
static int16 straightSpeed;
static int16 setSpeed_L, setSpeed_R;
/* 0～32度正切查表，数值约为 tan(角度)×1000；用于纯追踪转角和转弯半径计算。 */
static const int16 tani[33];

/* 左轮速度、右轮速度和舵机控制器状态，前台设置参数、定时中断更新历史。 */
static volatile PID motorLPid, motorRPid, servoPid;
/* 双轮速度环比例参数缓存。 */
static int16 motorSpeedKp = 3000;
/* 双轮速度环积分参数缓存。 */
static int16 motorSpeedKi = 50;
/* 双轮速度环微分参数缓存。 */
static int16 motorSpeedKd = 50;
/* 驱动使能状态；停止时定时器向双轮输出0。 */
static volatile uint8 motorEnabled = 0;
/* 电机PWM硬件是否已完成初始化，决定停机时能否立即写PWM。 */
static uint8 motorInitialized = 0;
/* 是否处于摄像头Run模式；该模式必须持续收到处理过的图像来维持驱动。 */
static volatile uint8 motorRunMode = 0;
/* 距离图像命令失效剩余的5ms节拍数；递减到0触发停车。 */
static volatile uint8 trackingTicks = 0;
/* 累计5ms节拍计数，uint16自然回绕；通过 Motor_GetTicks 原子读取。 */
static volatile uint16 motorTicks = 0;
/* 图像中心列减去目标列的像素误差，左偏为正。 */
static volatile int16 steeringError = 0;
/* 纯追踪转角，单位0.1度，正值左转，尚未乘舵机比例增益。 */
static volatile int16 steeringAngle10 = 0;
/* 最近一次定时中断实际写入舵机的PWM值，不是实测舵角。 */
static volatile int16 steeringPwm = servoMid;
/* 经过变化速率限制后的舵机PWM偏移，相对 servoMid。 */
static volatile int16 steeringOffset = 0;
/* 上个5ms节拍的有符号PWM步长；与实际舵角/机械转速无关。 */
static volatile int16 steeringSlewStep = 0;
/* Built once before TIM1 starts; remove repeated stopping-distance divisions
 * from the 5ms servo trajectory while keeping exactly the same step choice. */
static uint16 steeringStopDistance[RUN_SERVO_SLEW_MAX_PWM_PER_TICK + 1];
static volatile int16 steeringGyroRate10, steeringGyroCorrection;
static uint8 steeringImuSequence;
static volatile uint8 steeringImuAge = IMU_STEERING_STALE_TICKS;
/* 本帧路径被拒绝，保持当前舵机偏移并停止使用陀螺仪阻尼更新。 */
static volatile uint8 pathRejected = 0;
/* 连续失线路径帧数，达到阈值后执行一次减速；有效路径清零。 */
static volatile uint8 lostPathFrames;
/* 速度请求与起点由前台在临界区发布，5ms中断推进过渡。 */
static volatile int16 speedTarget, speedStart;
static volatile uint8 speedTransitionTick;
/* 最近一次普通道路分类；圆环结束后重新按当帧路径判断。 */
static volatile uint8 speedCurve;
/* 已实际入环才锁存；圆环状态结束后按5ms时钟保留圆环档。 */
static volatile uint8 ringSpeedWasActive, ringExitHoldActive;
static volatile uint16 ringExitHoldStartTick;

static uint8 motor_ring_active(void)
{
    return ring_state != RING_STATE_NONE || ring_candidate != RING_DIR_NONE || ring_brake_hint;
}

/* 有效/失线路径共用；预警或候选消失不触发出环延时。
 * 临界区保护16位时间戳及停车/断帧中断会清除的锁存状态。 */
static uint8 motor_ring_speed_active(void)
{
    bit irq_state;
    uint8 active;
    irq_state = EA;
    EA = 0;
    if(ring_state >= RING_STATE_STAGE1 && ring_state <= RING_STATE_EXIT_FOLLOW) {
        ringSpeedWasActive = 1;
        ringExitHoldActive = 0;
    } else if(ringSpeedWasActive && ring_state == RING_STATE_NONE) {
        ringSpeedWasActive = 0;
        ringExitHoldStartTick = motorTicks;
        ringExitHoldActive = 1;
    }
    if(ringExitHoldActive &&
       (uint16)(motorTicks - ringExitHoldStartTick) >= RUN_RING_EXIT_HOLD_TICKS)
        ringExitHoldActive = 0;
    active = motor_ring_active() || ringExitHoldActive;
    EA = irq_state;
    return active;
}

/* 调用者已关闭中断；相同请求不重启100ms过渡。 */
static void motor_request_speed(int16 request)
{
    if(request == speedTarget) return;
    speedStart = setSpeed;
    speedTarget = request;
    speedTransitionTick = 0;
}

/* 固定目标最多20拍到达；中途换档从当前已发布速度重新过渡。 */
static void motor_update_speed(void)
{
    if(!motorRunMode || !motorEnabled || speedTransitionTick >= RUN_SPEED_TRANSITION_TICKS) return;
    ++speedTransitionTick;
    setSpeed = speedStart + (int16)((int32)(speedTarget - speedStart) *
        speedTransitionTick / (int32)RUN_SPEED_TRANSITION_TICKS);
}

/* 初始化 pid 的比例/积分/微分参数并清空历史；which 为通道标记，attitude_coeff 为陀螺仪项系数，max_out 为输出限幅。 */
static void PID_Init(volatile PID *pid, uint8 which, int16 Kp, int16 Ki, int16 Kd, int16 attitude_coeff,int16 max_out){
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

/* 在临界区同时更新左右轮速度PID的 kp/ki/kd 及缓存，保持两侧参数一致。 */
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

/* 设置并限幅舵机比例系数 kp，同时清空舵机上一误差；100表示1.00倍量级。 */
void Motor_SetServoKp(int16 kp) {
    bit irq_state;
    irq_state = EA;
    EA = 0;
    servoPid.Kp = func_limit_ab(kp, 0, MOTOR_SERVO_KP_MAX);
    servoPid.last_error = 0;
    EA = irq_state;
}

/* 菜单速度绑定直线档；Run 在下一次有效路径发布时采用新值。
 * 弯道/圆环仍使用各自宏，不以菜单速度作为总上限。 */
void Motor_SetStraightSpeed(int16 speed) {
    bit irq_state;
    irq_state = EA;
    EA = 0;
    straightSpeed = func_limit_ab(speed, 0, 1000);
    if(!motorRunMode) setSpeed = motorEnabled ? straightSpeed : 0;
    EA = irq_state;
}

/* enabled=1请求驱动；Run模式下还需看门狗有余量且路径未拒绝。否则停止双轮并清空控制状态。 */
void Motor_SetEnabled(uint8 enabled) {
    bit irq_state;
    irq_state = EA;
    EA = 0;
    if (enabled == 1 && (!motorRunMode || (trackingTicks && !pathRejected))) {
        if (!motorRunMode) setSpeed = straightSpeed;
        motorEnabled = 1;
    } else {
        motorEnabled = 0;
        setSpeed = speedTarget = speedStart = 0;
        speedTransitionTick = 0;
        speedCurve = 0;
        ringSpeedWasActive = ringExitHoldActive = 0;
        ringExitHoldStartTick = 0;
        trackingTicks = 0;
        duty = 0;
        steeringOffset = 0;
        steeringSlewStep = 0;
        steeringGyroRate10 = steeringGyroCorrection = 0;
        steeringImuAge = IMU_STEERING_STALE_TICKS;
        pathRejected = 0;
        lostPathFrames = 0;
        steeringError = steeringAngle10 = 0;
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

/* 切换摄像头Run模式；先停止电机，enabled 非零表示启用断帧看门狗约束。 */
void Motor_SetRunMode(uint8 enabled) {
    Motor_SetEnabled(0);
    motorRunMode = enabled ? 1 : 0;
}

/* 读取当前驱动使能状态，返回0/1。 */
uint8 Motor_IsEnabled(void) { return motorEnabled; }

/* 原子读取像素误差、纯追踪转角、舵机PWM、滤波角速度和陀螺修正，供显示或调试。 */
void Motor_GetSteering(motor_steering_state_t *state) {
    bit irq_state;
    irq_state = EA;
    EA = 0;
    state->pixel_error = steeringError;
    state->angle10 = steeringAngle10;
    state->pwm = steeringPwm;
    state->gyro_rate10 = steeringGyroRate10;
    state->gyro_pwm = steeringGyroCorrection;
    EA = irq_state;
}

/* 处理本帧无有效路径：已有运行路径时保持舵机输出，并按配置在确认失线后减速；不能凭失线帧启动静止车辆。
 * 处理过的失线帧可刷新运行看门狗；相机不再送帧时仍由定时器停车。 */
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
    if(lostPathFrames < RUN_LOST_CONFIRM_FRAMES) ++lostPathFrames;
    /* 即使当前路径无效，也从首次圆环检测起切换圆环速度，并遵守出环延时。
     * 普通道路连续失线只降到弯道档，不因失线提速；孤立坏帧不改档。 */
    if(motor_ring_speed_active()) {
        speedCurve = 0;
        motor_request_speed(RUN_SPEED_RING);
    } else if(lostPathFrames >= RUN_LOST_CONFIRM_FRAMES) {
        speedCurve = 1;
        motor_request_speed((int16)func_min(speedTarget, RUN_SPEED_CURVE));
    }
    pathRejected = 1;
    steeringSlewStep = 0;
    /* A processed lost frame is alive; missing camera frames still time out. */
    trackingTicks = RUN_FRAME_TIMEOUT_TICKS;
    EA = irq_state;
}

/* 在临界区读取16位电机节拍；返回值单位为5ms，调用方用无符号差值计算经过时间。 */
uint16 Motor_GetTicks(void) {
    uint16 ticks;
    bit irq_state;
    irq_state = EA;
    EA = 0;
    ticks = motorTicks;
    EA = irq_state;
    return ticks;
}

/* 每个电机节拍只读取已发布样本；数据新旧以采样序号判断，恒定角速度不会误判停更。 */
static void motor_sample_steering_imu(void)
{
    imu_steering_sample_t sample;
    IMU_GetSteeringSample(&sample);
    if(!sample.valid) steeringImuAge = IMU_STEERING_STALE_TICKS;
    else if(sample.sequence != steeringImuSequence) {
        steeringImuSequence = sample.sequence;
        steeringImuAge = 0;
    } else if(steeringImuAge < IMU_STEERING_STALE_TICKS) ++steeringImuAge;
    steeringGyroRate10 = steeringImuAge < IMU_STEERING_STALE_TICKS ? sample.rate10 : 0;
}

/* 图像负责路线，快速Z轴负反馈抑制车头摆动；整数软死区、限幅和限速抑制噪声。
 * 仅正常同向大转弯衰减阻尼；车身朝目标反方向摆动时保留完整修正。 */
static int16 motor_steering_damping(volatile PID *pid, int16 error)
{
    int16 rate, bend, percent, target;
    int32 correction;
    if(!imu_ready || steeringImuAge >= IMU_STEERING_STALE_TICKS ||
       pid->attitude_coeff <= 0 || !motorRunMode || !motorEnabled ||
       !trackingTicks || !setSpeed || pathRejected) {
        steeringGyroCorrection = 0;
        return 0;
    }
    rate = steeringGyroRate10;
    if(rate > IMU_STEERING_DEADBAND10) rate -= IMU_STEERING_DEADBAND10;
    else if(rate < -IMU_STEERING_DEADBAND10) rate += IMU_STEERING_DEADBAND10;
    else rate = 0;
    percent = 100;
    if((error > 0 && rate < 0) || (error < 0 && rate > 0)) {
        bend = (int16)func_abs((int32)steeringAngle10);
        if(bend > IMU_STEERING_CURVE_FULL10) bend = IMU_STEERING_CURVE_FULL10;
        if(bend > IMU_STEERING_CURVE_START10)
            percent -= (int16)((int32)(100 - IMU_STEERING_CURVE_PERCENT) *
                (bend - IMU_STEERING_CURVE_START10) /
                (IMU_STEERING_CURVE_FULL10 - IMU_STEERING_CURVE_START10));
    }
    correction = (int32)pid->attitude_coeff * rate / 100L;
    correction = func_limit(correction, IMU_STEERING_MAX_PWM);
    target = (int16)(correction * percent / 100L);
    steeringGyroCorrection += (int16)func_limit(
        (int32)target - steeringGyroCorrection, IMU_STEERING_SLEW_PWM);
    return steeringGyroCorrection;
}

/* 视觉比例/微分项先换算为PWM，再叠加独立限幅的陀螺阻尼，最后统一限幅。 */
static int16 steerPIDCalculate(volatile PID *pid, int16 error) {
    int32 p_term, d_term;
    int32 pid_output;
    p_term = (int32) pid->Kp * error;
    d_term = (int32) pid->Kd * (error - pid->last_error);
    pid->last_error = error;
    pid_output = (p_term + d_term) / 100 + motor_steering_damping(pid, error);
    return (int16)func_limit(pid_output, pid->max_out);
}

/* 增量式轮速PID：target/actual 为目标/反馈编码器速度，更新累积PWM并设置方向引脚，返回PWM绝对值。
 * err/delta_err/delta_u：当前误差、误差变化和本次输出增量。 */
static int16 motorPIDCalculate(volatile PID *pid, int16 target, int16 actual) {
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

/* 固定目标下先加快再放缓；用剩余PWM距离选择能逐步减小到0的步长。
 * stopping_distance = step + (step-ACCEL) + ...，只累加正项。
 * 不跨过当前目标；目标提前回正/反向时立即服从新目标，不沿旧方向继续走。
 * MAX <= 96保证这里的有符号16位中间量不溢出，无浮点或开方。 */
static void motor_slew_steering(int16 target_offset)
{
    int16 delta, distance, step, lower, upper;
    delta = target_offset - steeringOffset;
    if(!delta) {
        steeringSlewStep = 0;
        return;
    }
    if((delta > 0 && steeringSlewStep < 0) ||
       (delta < 0 && steeringSlewStep > 0)) steeringSlewStep = 0;
    distance = (int16)func_abs(delta);
    lower = 1;
    upper = (int16)func_min(distance, func_min(RUN_SERVO_SLEW_MAX_PWM_PER_TICK,
        func_abs(steeringSlewStep) + RUN_SERVO_SLEW_ACCEL_PWM_PER_TICK));
    /* 二分查找最大的可停车步长，MAX<=96时最多7次，限制中断开销。 */
    while(lower < upper) {
        step = (lower + upper + 1) / 2;
        if(steeringStopDistance[step] <= (uint16)distance) lower = step;
        else upper = step - 1;
    }
    steeringSlewStep = delta < 0 ? -lower : lower;
    steeringOffset += steeringSlewStep;
}

/* Match wheel differential to this tick's commanded servo position, including
 * slew limiting and gyro correction. This is not a measured steering angle.
 * Existing mapping: 12 PWM/degree, half wheel track 8cm, wheelbase in mm.
 * Interpolate the tangent table instead of quantizing to whole degrees/cm. */
static int16 motor_steering_differential(int16 drive_speed)
{
    int16 angle10, degree, tangent, differential;
    if(!steeringOffset) return 0;
    angle10 = (int16)func_min(func_abs((int32)steeringOffset) * 10L / 12L, 320L);
    degree = angle10 / 10;
    tangent = tani[degree];
    if(degree < 32)
        tangent += (int16)((int32)(tani[degree + 1] - tangent) * (angle10 % 10) / 10);
    differential = (int16)((int32)drive_speed * 8L * tangent /
        ((int32)RUN_WHEELBASE_MM * 100L));
    return steeringOffset < 0 ? -differential : differential;
}

/* TIM1每5ms执行：更新看门狗、读取编码器、平滑舵机PWM、分配差速并执行双轮速度PID。
 * target_offset：目标舵机偏移；drive_speed：本节拍平均速度目标。 */
static void pitHandler(){
    int16 target_offset, drive_speed, differential;
    ++motorTicks;
    if (motorRunMode && trackingTicks) {
        if (--trackingTicks == 0) {
            motorEnabled = 0;
            setSpeed = speedTarget = speedStart = 0;
            speedTransitionTick = 0;
            speedCurve = 0;
            ringSpeedWasActive = ringExitHoldActive = 0;
            ringExitHoldStartTick = 0;
            duty = 0;
            steeringOffset = 0;
            steeringSlewStep = 0;
            steeringGyroCorrection = 0;
            pathRejected = 0;
            lostPathFrames = 0;
            steeringError = steeringAngle10 = 0;
        }
    }
    /* The camera supplies a held target; approach it every timer tick instead
     * of applying a complete frame-to-frame jump. No float math in the ISR.
     * Explicit stop/watchdog reset the offset; path loss holds it. */
    motor_update_speed();
    motor_sample_steering_imu();
    if(pathRejected) {
        steeringGyroCorrection = 0;
        steeringSlewStep = 0;
        target_offset = steeringOffset;
    } else target_offset = steerPIDCalculate(&servoPid, duty);
    motor_slew_steering(target_offset);
    steeringPwm = servoMid + steeringOffset;
    pwm_set_duty(servoPwm, steeringPwm);
    encoderR = (int16)(-2L * encoder_get_count(PWMA_ENCODER));
    encoderL = (int16)(2L * encoder_get_count(PWMC_ENCODER));
    encoder_clear_count(PWMA_ENCODER);
    encoder_clear_count(PWMC_ENCODER);
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
    differential = motor_steering_differential(drive_speed);
    setSpeed_L = drive_speed - differential;
    setSpeed_R = drive_speed + differential;
    pwm_set_duty(motorPwm_L, motorPIDCalculate(&motorLPid, setSpeed_L, encoderL));
    pwm_set_duty(motorPwm_R, motorPIDCalculate(&motorRPid, setSpeed_R, encoderR));
}

/* Millimetre pure pursuit with 0.1 degree table interpolation. */
/* 按车辆轴距和后轴目标点计算纯追踪转角；forward 向前距离、lateral 向左距离，单位毫米；返回0.1度、限幅±320。
 * denominator：目标距离平方；ratio/tangent：放大后的曲率转角正切；degree/angle：查表区间及插值角度。 */
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

/* Use three consecutive supported samples to reject isolated far-point spikes.
 * Work on the same continuous fresh path as steering, never history/extrapolation.
 * Decimation limits extra projection work on the MCU. */
/* 沿当前图像路径抽样，使用连续三个同向的可信转角决定预瞄弯度，抑制单个远点噪声。
 * i：路径索引；x/y：相对后轴的毫米坐标；a/prev1/prev2：当前及前两次转角；support：连续支持数；candidate/best：本次/最大可信弯度；radius2：距离平方。 */
static int16 motor_speed_preview(Point roadLine[], int16 count, int16 steering_angle) {
    int16 i, x, y, a, prev1=0, prev2=0, candidate, best;
    uint8 support=0;
    int32 radius2;
    best = (int16)func_abs((int32)steering_angle);
    /* 超过入弯阈值后无需继续投影。 */
    if(best > RUN_SPEED_CURVE_ENTER_ANGLE10) return best;
    for(i=0; i<count; i+=2) {
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
            if(best > RUN_SPEED_CURVE_ENTER_ANGLE10) return best;
        }
        prev2=prev1; prev1=a;
        if(support<2) ++support;
    }
    return best;
}

/* A separate image-only braking horizon extends beyond calibrated steering.
 * Follow a continuous measured pair from the steering path's far end; never
 * bridge a gap into an unrelated far segment. All rows are checked, even
 * though only alternate far rows contribute to the three-sample evidence. */
static int16 motor_far_bend(Point roadLine[], int16 count, uint8 *exit_straight)
{
    int16 row, top, col, width, previous_col, previous_width;
    int16 error, prev1=0, prev2=0, evidence=0, candidate;
    uint8 support=0, have_previous=0, straight_rows=0;
    *exit_straight = 0;
    if (cross_flag || cross_candidate || cross_single_side || !roadLine || count < 1)
        return 0;
    row=roadLine[count-1].x;
    top=SEARCH_CALIBRATION_MIN_ROW-RUN_SPEED_EXTRA_ROWS;
    if (top < STOPROW) top=STOPROW;
    if (row < SEARCH_CALIBRATION_MIN_ROW || row >= SEARCH_IMAGE_H) return 0;
    previous_col=previous_width=0;
    for (;row>=top;--row) {
        if (!center_valid[row] || center_quality[row] < 3 || center_source[row] != CENTER_MEASURED ||
            left_valid[row] != EDGE_MEASURED || right_valid[row] != EDGE_MEASURED) break;
        col=center_line[row];
        width=(int16)right_edge_line[row]-left_edge_line[row];
        if (width < SEARCH_MIN_WIDTH || col <= left_edge_line[row] || col >= right_edge_line[row]) break;
        if (have_previous && (func_abs(col-previous_col) > RUN_MAX_CENTER_STEP ||
            func_abs(width-previous_width) > RUN_MAX_CENTER_STEP)) break;
        previous_col=col; previous_width=width; have_previous=1;
        if (row > SEARCH_CALIBRATION_MIN_ROW) continue;
        if((int32)func_abs(SEARCH_IMAGE_W/2-col)*100L <=
           (int32)width*RUN_SPEED_EXIT_FAR_PERCENT) {
            if(straight_rows < RUN_SPEED_EXIT_STRAIGHT_ROWS) ++straight_rows;
        } else straight_rows=0;
        if ((SEARCH_CALIBRATION_MIN_ROW-row)%2) continue;
        error=(int16)((int32)(SEARCH_IMAGE_W/2-col)*100/width);
        if (support >= 2 && ((error >= 0 && prev1 >= 0 && prev2 >= 0) ||
                            (error <= 0 && prev1 <= 0 && prev2 <= 0))) {
            candidate=(int16)func_min(func_abs(error),func_min(func_abs(prev1),func_abs(prev2)));
            if (candidate > evidence) evidence=candidate;
        }
        prev2=prev1; prev1=error;
        if (support < 2) ++support;
    }
    /* 必须从当前路径连续看到最远观察行；缺失、补线、历史点不算出弯。 */
    *exit_straight = row < top && straight_rows >= RUN_SPEED_EXIT_STRAIGHT_ROWS;
    return evidence;
}

/* 只输出直线/弯道/圆环三档。远端证据和预瞄角都只参与道路分类。 */
static int16 motor_path_speed(Point roadLine[], int16 count, int16 angle)
{
    int16 preview, far_bend;
    uint8 exit_straight;
    if(motor_ring_speed_active()) {
        speedCurve = 0;
        /* 出口补线期间仍用圆环档；状态结束后再保持2秒，随后按当前道路分类。 */
        return RUN_SPEED_RING;
    }
    far_bend = motor_far_bend(roadLine, count, &exit_straight);
    preview = motor_speed_preview(roadLine, count, angle);
    /* S弯远端回中不能覆盖中间一折或远端已存在的连续弯道证据。 */
    if(preview > RUN_SPEED_CURVE_ENTER_ANGLE10 || far_bend > RUN_SPEED_FAR_ENTER_PERCENT)
        speedCurve = 1;
    else if(exit_straight && func_abs((int32)angle) <= RUN_SPEED_EXIT_MAX_ANGLE10)
        speedCurve = 0;
    else if(preview <= RUN_SPEED_CURVE_EXIT_ANGLE10 && far_bend <= RUN_SPEED_FAR_EXIT_PERCENT)
        speedCurve = 0;
    return speedCurve ? RUN_SPEED_CURVE : straightSpeed;
}

/* 统一发布 angle10（0.1度转角）、pixel_error（像素误差）及 speed_request（动态目标速度）。
 * 临界区内同时刷新方向、速度请求、路径标记与摄像头看门狗。 */
static void motor_set_steering(int16 angle10, int16 pixel_error, int16 speed_request) {
    bit irq_state;
    angle10 = func_limit(angle10, 320);
    /* Limit only the visual exit guide; aligned outer-edge following keeps
     * its geometric steering. Apply before both servo and wheel differential. */
    if(ring_state == RING_STATE_STAGE4 && ring_exit_guiding)
        angle10 = func_limit(angle10, RUN_RING_EXIT_MAX_ANGLE10);
    irq_state = EA;
    EA = 0;
    motor_request_speed(speed_request);
    steeringError = pixel_error;
    steeringAngle10 = angle10;
    duty = (int16)((int32)angle10 * 12 / 10);
    if (motorRunMode) trackingTicks = RUN_FRAME_TIMEOUT_TICKS;
    pathRejected = 0;
    lostPathFrames = 0;
    EA = irq_state;
}

/* A far target may lie back on the centreline after the first lobe of an S.
 * Require a measured lateral excursion and three spatially separated return
 * samples. Keep the first reachable lobe only when the nominal target loses
 * its steering demand. No extrapolation, history, or cross/ring repairs. */
static int16 motor_first_bend_target(Point roadLine[], int16 nominal)
{
    int16 i, row, x, y, last_x=0, last_y=0, anchor_x=0;
    int16 extreme_x=0, extreme_y=0, extreme_index=0, direction=0;
    int16 candidate, candidate_angle, nominal_angle, nominal_x, nominal_y;
    uint8 have_point=0, returns=0;
    int32 radius2;
    if(nominal < 1 || motor_ring_active() || cross_flag || cross_candidate ||
       cross_single_side || zebra_candidate || zebra_detected) return nominal;
    /* Check the entire segment before considering an earlier target. */
    for(i=0; i<=nominal; ++i) {
        row=roadLine[i].x;
        if(row >= SEARCH_IMAGE_H || !center_valid[row] || center_quality[row] < 3 ||
           center_source[row] != CENTER_MEASURED || left_valid[row] != EDGE_MEASURED ||
           right_valid[row] != EDGE_MEASURED) return nominal;
        if(i && (row != (int16)roadLine[i-1].x-1 ||
           func_abs((int16)roadLine[i].y-roadLine[i-1].y) > RUN_MAX_CENTER_STEP)) return nominal;
    }
    for(i=0; i<=nominal; ++i) {
        if(!search_project_point(roadLine[i].y,roadLine[i].x,&x,&y)) return nominal;
        x-=RUN_REAR_AXLE_X_MM; y-=RUN_REAR_AXLE_Y_MM;
        if(y<=0 || (have_point && y<last_y)) return nominal;
        if(!have_point) {
            anchor_x=extreme_x=last_x=x; extreme_y=last_y=y;
            extreme_index=i; have_point=1; continue;
        }
        if(y-last_y < RUN_S_BEND_SAMPLE_MM) continue;
        if(!direction) {
            if(func_abs(x-anchor_x) >= RUN_S_BEND_SWING_MM) direction=x>anchor_x ? 1 : -1;
            extreme_x=x; extreme_y=y; extreme_index=i;
        } else if((int32)(x-extreme_x)*direction>0) {
            extreme_x=x; extreme_y=y; extreme_index=i; returns=0;
        } else {
            if((int32)(last_x-x)*direction>0) {
                if(returns<RUN_S_BEND_CONFIRM_SAMPLES) ++returns;
            } else returns=0;
            if(returns>=RUN_S_BEND_CONFIRM_SAMPLES &&
               (int32)(extreme_x-anchor_x)*direction>=RUN_S_BEND_SWING_MM &&
               (int32)(extreme_x-x)*direction>=RUN_S_BEND_RETURN_MM &&
               y-extreme_y>=RUN_S_BEND_SAMPLE_MM*(RUN_S_BEND_CONFIRM_SAMPLES-1)) {
                candidate=extreme_index;
                for(;candidate<nominal;++candidate) {
                    int16 cx, cy;
                    if(!search_project_point(roadLine[candidate].y,roadLine[candidate].x,&cx,&cy)) return nominal;
                    cx-=RUN_REAR_AXLE_X_MM; cy-=RUN_REAR_AXLE_Y_MM;
                    radius2=(int32)cx*cx+(int32)cy*cy;
                    if(radius2<(int32)RUN_LOOKAHEAD_MIN_MM*RUN_LOOKAHEAD_MIN_MM) continue;
                    candidate_angle=motor_pursuit_angle10(cy,-cx);
                    if((int32)cx*direction>0 && func_abs(candidate_angle)>RUN_SPEED_CURVE_ENTER_ANGLE10) {
                        if(!search_project_point(roadLine[nominal].y,roadLine[nominal].x,
                           &nominal_x,&nominal_y)) return nominal;
                        nominal_angle=motor_pursuit_angle10(nominal_y-RUN_REAR_AXLE_Y_MM,
                            RUN_REAR_AXLE_X_MM-nominal_x);
                        if((int32)candidate_angle*nominal_angle<=0 ||
                           func_abs(candidate_angle)>=func_abs(nominal_angle)+RUN_S_BEND_ANGLE_DROP10) return candidate;
                    }
                    break;
                }
                /* This lobe was too near or still represented by the target.
                 * Start the opposite excursion at the previous extremum. */
                anchor_x=extreme_x; direction=-direction;
                extreme_x=x; extreme_y=y; extreme_index=i; returns=0;
            }
        }
        last_x=x; last_y=y;
    }
    return nominal;
}

/* 从图像路径 roadLine 的 count 个点中选择瞄点；普通实测回折不能越过可达的第一折，没有可投影前向点返回-1。
 * Point.x是行、Point.y是列；x/y为后轴坐标；lookahead为按横向偏差缩短的半径；radius2/error/best_error为距离平方及误差；best为最优索引。 */
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
    return motor_first_bend_target(roadLine,best);
}

/* Signed ground-X target offset: right positive, left negative.
 * Four fresh measured points support two consecutive bends; a translated or
 * rotated straight must not trigger. No retained bias across frames or S turns.
 * Work outside the ISR; the existing PWM slew limit smooths the final command. */
static int16 motor_curve_inside_offset(Point roadLine[], int16 count,
                                       int16 target, int16 target_x, int16 target_y)
{
    int16 i, row, x, y, px=0, py=0, dx, dy, previous_dy=0;
    int16 inner_x, inner_y, amount, clearance, direction;
    int32 slope, previous_slope=0, bend, first_bend=0, strength;
    uint8 points=0;
    if (!RUN_CURVE_INSIDE_MAX_MM || ring_state != RING_STATE_NONE || ring_candidate ||
        cross_flag || cross_candidate || cross_single_side || zebra_candidate || zebra_detected)
        return 0;
    if (func_abs((int32)target_x) + RUN_CURVE_INSIDE_MAX_MM >= target_y) return 0;
    row = roadLine[target].x;
    if (row >= SEARCH_IMAGE_H || left_valid[row] != EDGE_MEASURED ||
        right_valid[row] != EDGE_MEASURED || center_source[row] != CENTER_MEASURED)
        return 0;
    /* End the observation window at the target: the calibrated image may
     * contain no points beyond it. Anchor samples to absolute forward ranges
     * so row quantization does not accumulate and miss the final sample. */
    for (i=0; i<=target && i<count; ++i) {
        if (!search_project_point(roadLine[i].y, roadLine[i].x, &x, &y)) return 0;
        x -= RUN_REAR_AXLE_X_MM;
        y -= RUN_REAR_AXLE_Y_MM;
        if (!points && y < target_y - 3 * RUN_CURVE_SAMPLE_MM) continue;
        row = roadLine[i].x;
        if (row >= SEARCH_IMAGE_H || center_source[row] != CENTER_MEASURED ||
            left_valid[row] != EDGE_MEASURED || right_valid[row] != EDGE_MEASURED)
            return 0;
        if (!points) {
            if (y > target_y - 2 * RUN_CURVE_SAMPLE_MM - RUN_CURVE_SAMPLE_MM/2) return 0;
            px=x; py=y; points=1; continue;
        }
        dy=y-py;
        if (dy < 0) return 0;
        if (y < target_y - (3-points) * RUN_CURVE_SAMPLE_MM) continue;
        if (dy < RUN_CURVE_SAMPLE_MM/2 || dy > 2 * RUN_CURVE_SAMPLE_MM) return 0;
        dx=x-px;
        slope=1000L*dx/dy;
        if (slope < -2000L || slope > 2000L) return 0;
        if (points >= 2) {
            bend=(slope-previous_slope)*200L/(dy+previous_dy);
            if (points == 2) first_bend=bend;
            else {
                if ((bend > 0 && first_bend > 0) || (bend < 0 && first_bend < 0)) {
                    direction=bend > 0 ? 1 : -1;
                    /* Do not fight a target already asking for the opposite turn. */
                    if ((int32)target_x*direction <= 0) return 0;
                    strength=func_min(func_abs(bend),func_abs(first_bend));
                    if (strength <= RUN_CURVE_START_SLOPE) return 0;
                    strength=func_min(strength,RUN_CURVE_FULL_SLOPE);
                    amount=(int16)((strength-RUN_CURVE_START_SLOPE)*RUN_CURVE_INSIDE_MAX_MM /
                        (RUN_CURVE_FULL_SLOPE-RUN_CURVE_START_SLOPE));
                    row=roadLine[target].x;
                    if (!search_project_point(direction > 0 ? right_edge_line[row] : left_edge_line[row],
                        row,&inner_x,&inner_y)) return 0;
                    clearance=(int16)(((int32)inner_x-RUN_REAR_AXLE_X_MM-target_x)*direction);
                    if (clearance <= RUN_CURVE_INSIDE_MARGIN_MM) return 0;
                    amount=(int16)func_min(amount,clearance-RUN_CURVE_INSIDE_MARGIN_MM);
                    return direction*amount;
                }
                return 0;
            }
        }
        previous_slope=slope; previous_dy=dy; px=x; py=y; ++points;
    }
    return 0;
}

/* 当前Run使用的图像中线跟踪接口：验证count和目标索引aimPos，投影目标并发布纯追踪转向及预瞄速度。
 * x/y为相对后轴毫米坐标，angle为本次转角，preview为综合前瞻后的速度请求；输入无效时停止驱动。 */
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
    x += motor_curve_inside_offset(roadLine, count, aimPos, x, y);
    angle = motor_pursuit_angle10(y, -x);
    preview = motor_path_speed(roadLine, count, angle);
    /* Publish direction/speed/watchdog together with interrupts masked. */
    motor_set_steering(angle, MT9V03X_W / 2 - roadLine[aimPos].y, preview);
}

/* 初始化舵机/电机PWM、编码器、PID及TIM1的5ms回调；servo_kp为舵机增益。
 * irq_state用于保证回调地址完整注册。 */
void Motor_Init(int16 servo_kp){
    bit irq_state;
    uint16 step;
    servo_kp = func_limit_ab(servo_kp, 0, MOTOR_SERVO_KP_MAX);
    PID_Init(&servoPid, 39, servo_kp, 0, 0, IMU_STEERING_GYRO_COEFF, RUN_SERVO_MAX_OFFSET);
    PID_Init(&motorLPid, motorL, motorSpeedKp, motorSpeedKi, motorSpeedKd, 0, 4000);
    PID_Init(&motorRPid, motorR, motorSpeedKp, motorSpeedKi, motorSpeedKd, 0, 4000);
    steeringStopDistance[0] = 0;
    for(step = 1; step <= RUN_SERVO_SLEW_MAX_PWM_PER_TICK; ++step) {
        steeringStopDistance[step] = step;
        if(step > RUN_SERVO_SLEW_ACCEL_PWM_PER_TICK)
            steeringStopDistance[step] +=
                steeringStopDistance[step - RUN_SERVO_SLEW_ACCEL_PWM_PER_TICK];
    }
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

// tan查表
static const int16 tani[33] = {0,
                        17,  35,  52,  70,  87,  105, 123, 141, 158, 176,
                        194, 213, 231, 249, 268, 287, 306, 325, 344, 364,
                        384, 404, 424, 445, 466, 488, 510, 532, 554, 577, 600, 625};
