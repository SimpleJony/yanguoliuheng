#include "Motor.h"

volatile int16 encoderL, encoderR, duty, setSpeed, handle = 0, servoOut, motorOutL, motorOutR;
int16 R, servoP, aimPos, speedLimit, curveCoeff, currentSpeed, setSpeed_L, setSpeed_R, enableDifSpeed = 1, currentDuty = 0;
static uint8 wheelDis = 18; // 真实轴距*图像比例
static uint8 cameraOffset = 18;
const int16 tani[33];

int32 sqrt_int(int32 x) {
    int32 res = 0, Bi = 1L << 20;
    if (x <= 0) return 0;
    while (Bi > x) Bi >>= 2;
    while (Bi) {
        if (x >= res + Bi) {
            x -= res + Bi;
            res = (res >> 1) + Bi;
        } else {
            res >>= 1;
        }
        Bi >>= 2;
    }
    return res;
}

int32 atan2_int(int32 y, int32 x) {
    static const int32 tan_table[31] = {
        0,         1143,      2288,      3434,      4582,      5732,
        6887,      8046,      9212,      10383,     11562,     12749,
        13945,     15151,     16368,     17597,     18840,     20097,
        21369,     22657,     23962,     25286,     26628,     27989,
        29371,     30775,     32203,     33654,     35132,     36637,
        37837
    };
    int32 sign = (y >= 0) ? 1 : -1, ay = (y >= 0) ? y : -y, d = 29, lhs, rhs;
    if (ay == 0) return 0;
    lhs = (int)ay * 65536;
    rhs = (int)x * tan_table[30];
    if (lhs >= rhs) return sign * 30;
    while (d >= 0) {
        if (lhs >= (int)x * tan_table[d]) break;
        d--;
    }
    return sign * d;
}

PID motorLPid, motorRPid, servoPid;

void PID_Init(PID *pid, uint8 which, int16 Kp, int16 Ki, int16 Kd, int16 attitude_coeff,int16 max_out){
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

int16 steerPIDCalculate(PID *pid, int16 error) {
    int32 p_term, d_term, att_term;
    int16 pid_output;
    p_term = (int32) pid->Kp * error;
    d_term = (int32) pid->Kd * (error - pid->last_error);
    att_term = (int32) pid->attitude_coeff * imu660rb_gyro_z;
    pid->last_error = error;
    pid_output = (p_term + d_term + att_term) / 100;
    return func_limit(pid_output, pid->max_out);
}

int16 motorPIDCalculate(PID *pid, int16 target, int16 actual) {
    int32 err, delta_err, delta_u;
    err = target - actual;
    delta_err = err - pid->last_error;
    delta_u = (int32) pid->Kp * delta_err +
                 (int32) pid->Ki * err +
                 (int32) pid->Kd * (delta_err - (pid->last_error - pid->llast_error));
    pid->llast_error = pid->last_error;
    pid->last_error = err;
    pid->base += delta_u / 100;
    pid->base = func_limit(pid->base, pid->max_out);
    gpio_set_level((gpio_pin_enum) pid->which ? motorDir_R : motorDir_L, pid->base >= 0);
    return func_abs(pid->base);
}

static void pitHandler(){
    encoderR = encoder_get_count(PWMA_ENCODER) << 1;
    encoderL = encoder_get_count(PWMC_ENCODER) << 1;
    encoder_clear_count(PWMA_ENCODER);
    encoder_clear_count(PWMC_ENCODER);
    servoOut = steerPIDCalculate(&servoPid, duty);
    pwm_set_duty(servoPwm, servoMid + servoOut);
    currentSpeed = encoderR + encoderL >> 1;
    setSpeed_L = setSpeed - (!R || !enableDifSpeed ? 0 : setSpeed * 8 / R);
    setSpeed_R = setSpeed + (!R || !enableDifSpeed ? 0 : setSpeed * 8 / R);
    motorPIDCalculate(&motorLPid, setSpeed_L, encoderL);
    motorOutL = motorLPid.base;
    pwm_set_duty(motorPwm_L, func_abs(motorOutL));
    motorPIDCalculate(&motorRPid, setSpeed_R, encoderR);
    motorOutR = motorRPid.base;
    pwm_set_duty(motorPwm_R, func_abs(motorOutR));
}

void alongLine(Point roadLine[], int16 count, int16 aimPos) {
    int16 pos, dx, dy;
    int8 d;
    if (count <= 5) return;
    pos = func_limit_ab(aimPos, 5, count - 2);
    dx = MT9V03X_H - roadLine[pos].x + cameraOffset;
    dy = 94 - roadLine[pos].y;
    d = atan2_int(2 * wheelDis * dy, dx * dx + dy * dy);
    setServoDuty(func_limit(d, 32));
}

void setServoDuty(int8 d) {
    int16 data t, r, ts;
    currentDuty = (int16) d * servoP / 100;
    t = tani[func_limit_ab(func_abs(currentDuty), 0, 32)];
    r = !t ? 0 : 20000 / t; // 20000为实际轴距*1000
    ts = speedLimit;
    if (!handle)
        setSpeed = !r ? ts : func_min(sqrt_int(r * (uint32) curveCoeff), ts);
    R = d < 0 ? -r : r;
    duty = (int16) d * 12;
}

void Motor_Init(int16 _servoP, int16 _aimPos, int16 _curveCoeff, int16 _speedLimit){
    servoP = _servoP;
    aimPos = _aimPos;
    curveCoeff = _curveCoeff;
    speedLimit = _speedLimit;

    PID_Init(&servoPid, 39, servoP, 0, 0, 0, 0);
    PID_Init(&motorLPid, motorL, 0, 0, 0, 0, 0);
    PID_Init(&motorRPid, motorR, 0, 0, 0, 0, 0);
    pwm_init(servoPwm, 200, servoMid);
    gpio_init(motorDir_L, GPO, 1, GPO_PUSH_PULL);
    gpio_init(motorDir_R, GPO, 1, GPO_PUSH_PULL);
    pwm_init(motorPwm_L, 17000, 0);
    pwm_init(motorPwm_R, 17000, 0);
    encoder_quad_init(PWMA_ENCODER, PWMA_ENCODER_CH1P_P60, PWMA_ENCODER_CH2P_P62);
    encoder_quad_init(PWMC_ENCODER, PWMC_ENCODER_CH1P_P40, PWMC_ENCODER_CH2P_P42);
    system_delay_ms(200);
    pit_ms_init(TIM1_PIT, 5, pitHandler);
}

int8 getAimPos() {return aimPos;}

// tan查表
const int16 tani[33] = {0,
                        17,  35,  52,  70,  87,  105, 123, 141, 158, 176,
                        194, 213, 231, 249, 268, 287, 306, 325, 344, 364,
                        384, 404, 424, 445, 466, 488, 510, 532, 554, 577, 600, 628};
