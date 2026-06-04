#include "control.h"
#include "servo.h"
#include "search_line.h"

// 逐飞助手的参数通道
#define PARAM_MOTOR1_KP_INDEX       (0)
#define PARAM_MOTOR1_KI_INDEX       (1)
#define PARAM_MOTOR1_KD_INDEX       (2)
#define PARAM_MOTOR1_TARGET_INDEX   (3)
#define PARAM_MOTOR2_KP_INDEX       (4)
#define PARAM_MOTOR2_KI_INDEX       (5)
#define PARAM_MOTOR2_KD_INDEX       (6)
#define PARAM_MOTOR2_TARGET_INDEX   (7)

#define PID_KP_MIN                  (0.0f)              // Kp最小值
#define PID_KP_MAX                  (200.0f)            // Kp最大值
#define PID_KI_MIN                  (0.0f)              // Ki最小值
#define PID_KI_MAX                  (100.0f)            // Ki最大值
#define PID_KD_MIN                  (0.0f)              // Kd最小值
#define PID_KD_MAX                  (200.0f)            // Kd最大值
#define TARGET_SPEED_MIN            (-30000.0f)         // 目标速度最小值
#define TARGET_SPEED_MAX            (30000.0f)          // 目标速度最大值
#define TARGET_RAMP_STEP            (400.0f)            // 目标速度斜坡步长
#define SPEED_LOOP_DUTY_LIMIT       (35.0f)             // 占空比限制
#define SPEED_STOP_DEADBAND         (0.1f)              // 速度停止死区
#define PID_ZERO_DEADBAND           (0.00000001f)       // PID参数零死区
#define SPEED_ENCODER_PPR           (1024.0f)           // 编码器每转脉冲数
#define SPEED_ENCODER_GEAR          (30.0f)             // 编码器齿轮比
#define SPEED_WHEEL_GEAR            (68.0f)             // 轮子齿轮比
#define SPEED_WHEEL_DIAMETER_M      (0.064f)            // 轮子直径(米)
#define SPEED_WHEEL_PI              (3.1415926f)        // 圆周率
#define SPEED_WHEEL_CIRCLE_M        (SPEED_WHEEL_DIAMETER_M * SPEED_WHEEL_PI)                       // 轮子周长
#define SPEED_PULSE_PER_WHEEL       (SPEED_ENCODER_PPR * SPEED_WHEEL_GEAR / SPEED_ENCODER_GEAR)     // 每转轮子对应的编码器脉冲数
#define SPEED_METER_PER_PULSE       (SPEED_WHEEL_CIRCLE_M / SPEED_PULSE_PER_WHEEL)                  // 每个编码器脉冲对应的实际距离
#define ENCODER1_DIR_SIGN           (1)                 // 编码器1方向符号 在逐飞助手里面更好观察才写的
#define ENCODER2_DIR_SIGN           (-1)                // 编码器2方向符号
#define MOTOR1_DIR_SIGN             (-1.0f)             // 电机1方向符号
#define MOTOR2_DIR_SIGN             (-1.0f)             // 电机2方向符号

#define LINE_FOLLOW_BASE_SPEED      (15000.0f)
#define LINE_FOLLOW_MIN_SPEED       (8000.0f)
#define LINE_FOLLOW_STOP_SPEED      (0.0f)
#define LINE_FOLLOW_SERVO_KP        (0.35f)
#define LINE_FOLLOW_TURN_SLOW_KP    (110.0f)
#define LINE_FOLLOW_MAX_ERROR       ((float)(SEARCH_IMAGE_W / 2))
#define LINE_FOLLOW_ROW_FAR         (SEARCH_IMAGE_H * 2 / 3)
#define LINE_FOLLOW_ROW_NEAR        (SEARCH_IMAGE_H - 12)
#define LINE_FOLLOW_ROW_STEP        (8)
#define LINE_FOLLOW_MIN_WIDTH       (20)
#define LINE_FOLLOW_MAX_WIDTH       (SEARCH_IMAGE_W - 6)
#define SCREEN_IMAGE_X              (0)
#define SCREEN_IMAGE_Y              (0)
#define SCREEN_IMAGE_W              (240)
#define SCREEN_IMAGE_H              (153)
#define SCREEN_INFO_Y               (SCREEN_IMAGE_Y + SCREEN_IMAGE_H + 4)
#define SCREEN_LINE_DRAW_STEP       (2)
#define SCREEN_LINE_THICKNESS       (3)

// 各个电机的PID参数和控制变量
static volatile float motor1Kp = 5.0f;
static volatile float motor1Ki = 0.5f;
static volatile float motor1Kd = 0.00f;
static volatile float motor2Kp = 4.0f;
static volatile float motor2Ki = 0.7f;
static volatile float motor2Kd = 0.00f;

static volatile float targetSpeedCommand1 = 0.0f;
static volatile float targetSpeedCommand2 = 0.0f;
static volatile float targetSpeed1 = 0.0f;              // 经过斜坡处理后的目标速度  
static volatile float targetSpeed2 = 0.0f;
static volatile float feedbackSpeed1 = 0.0f;            // 反馈速度
static volatile float feedbackSpeed2 = 0.0f;

static volatile float errorMotor1 = 0.0f;               // 电机1的当前误差
static volatile float errorMotor1Last = 0.0f;           // 电机1的上次误差
static volatile float errorMotor1Prev = 0.0f;           // 电机1的上上次误差
static volatile float deltaOutput1 = 0.0f;              // 电机1的当前增量输出
static volatile float output1 = 0.0f;                   // 电机1的输出

static volatile float errorMotor2 = 0.0f;               // 电机2的当前误差
static volatile float errorMotor2Last = 0.0f;           // 电机2的上次误差
static volatile float errorMotor2Prev = 0.0f;           // 电机2的上上次误差
static volatile float deltaOutput2 = 0.0f;              // 电机2的当前增量输出
static volatile float output2 = 0.0f;

static volatile int16 encoderCount1 = 0;                // 编码器1的计数
static volatile int16 encoderCount2 = 0;                // 编码器2的计数
static volatile float oscSendTick = 0.0f;               // 发送示波器数据的计时器
static volatile float lastParamChannel = 0.0f;          // 上次更新的参数通道 用于示波器显示
static volatile float lastParamValue = 0.0f;            // 上次更新的参数值 用于示波器显示
static volatile float lineFollowError = 0.0f;
static volatile uint8 lineFollowValid = 0;
static volatile uint16 screenFpsElapsedMs = 0;
static uint16 screenFrameCount = 0;
static uint16 screenFps = 0;

// 判断绝对值是否小于某个值的函数
static uint8 control_abs_less(float value, float limit) 
{
    return ((value < limit) && (value > -limit));
}

// 目标速度斜坡处理函数
static float control_ramp_target(float current, float target)
{
    float error = target - current;

    if (error > TARGET_RAMP_STEP)
    {
        current += TARGET_RAMP_STEP;
    }
    else if (error < -TARGET_RAMP_STEP)
    {
        current -= TARGET_RAMP_STEP;
    }
    else
    {
        current = target;
    }

    return current;
}

// 重置PID状态
void control_reset(void)
{
    bit irq_backup = EA;
    EA = 0;

    feedbackSpeed1 = 0.0f;
    feedbackSpeed2 = 0.0f;
    targetSpeedCommand1 = 0.0f;
    targetSpeedCommand2 = 0.0f;
    targetSpeed1 = 0.0f;
    targetSpeed2 = 0.0f;

    errorMotor1 = 0.0f;
    errorMotor1Last = 0.0f;
    errorMotor1Prev = 0.0f;
    deltaOutput1 = 0.0f;
    output1 = 0.0f;

    errorMotor2 = 0.0f;
    errorMotor2Last = 0.0f;
    errorMotor2Prev = 0.0f;
    deltaOutput2 = 0.0f;
    output2 = 0.0f;

    encoderCount1 = 0;
    encoderCount2 = 0;
    lastParamChannel = 0.0f;
    lastParamValue = 0.0f;

    EA = irq_backup;

    encoder_clear_count(SPEED_ENCODER1);
    encoder_clear_count(SPEED_ENCODER2);
    Motor1_SetSpeed(0.0f);
    Motor2_SetSpeed(0.0f);
    Servo_Reset();
}

// 初始化
void control_Init(void)
{
    control_reset();
    pit_ms_init(CONTROL_PIT_CH, CONTROL_PERIOD_MS, control_pit_handler);    // 初始化周期定时器用于PID计算
}

void control_task(void)
{
    seekfree_assistant_data_analysis();     // 处理来自逐飞助手的数据
    control_update();                       // 更新控制参数
    control_line_follow_update();           // 根据中线路径更新舵机和电机目标

    if (oscSendTick >= OSCILLOSCOPE_SEND_PERIOD_MS)
    {
        oscSendTick = 0.0f;
        control_sendOscData();
    }
    else
    {
        oscSendTick += CONTROL_TASK_PERIOD_MS;
    }

    system_delay_ms(CONTROL_TASK_PERIOD_MS);
}

void control_set_target_speed(float motor1Target, float motor2Target)
{
    bit irq_backup = EA;
    EA = 0;

    targetSpeedCommand1 = control_limit(motor1Target, TARGET_SPEED_MIN, TARGET_SPEED_MAX);
    targetSpeedCommand2 = control_limit(motor2Target, TARGET_SPEED_MIN, TARGET_SPEED_MAX);

    if (control_abs_less(targetSpeedCommand1, SPEED_STOP_DEADBAND))
    {
        targetSpeed1 = 0.0f;
        errorMotor1 = 0.0f;
        errorMotor1Last = 0.0f;
        errorMotor1Prev = 0.0f;
        deltaOutput1 = 0.0f;
        output1 = 0.0f;
    }

    if (control_abs_less(targetSpeedCommand2, SPEED_STOP_DEADBAND))
    {
        targetSpeed2 = 0.0f;
        errorMotor2 = 0.0f;
        errorMotor2Last = 0.0f;
        errorMotor2Prev = 0.0f;
        deltaOutput2 = 0.0f;
        output2 = 0.0f;
    }

    EA = irq_backup;
}

static uint8 control_get_line_error(float *error)
{
    int row;
    uint8 weight = 1;
    uint16 weightSum = 0;
    int32 errorSum = 0;
    uint8 validCount = 0;
    uint8 left = 0;
    uint8 right = 0;
    uint8 width = 0;
    int16 rowError = 0;

    for (row = LINE_FOLLOW_ROW_NEAR; row >= LINE_FOLLOW_ROW_FAR; row -= LINE_FOLLOW_ROW_STEP)
    {
        left = left_edge_line[row];
        right = right_edge_line[row];

        if (right <= left)
        {
            continue;
        }

        width = right - left;
        if (width < LINE_FOLLOW_MIN_WIDTH || width > LINE_FOLLOW_MAX_WIDTH)
        {
            continue;
        }

        rowError = (int16)center_line[row] - (int16)(SEARCH_IMAGE_W / 2);
        errorSum += (int32)rowError * weight;
        weightSum += weight;
        validCount++;
        weight++;
    }

    if (!validCount || !weightSum)
    {
        *error = 0.0f;
        return 0;
    }

    *error = (float)errorSum / (float)weightSum;
    *error = control_limit(*error, -LINE_FOLLOW_MAX_ERROR, LINE_FOLLOW_MAX_ERROR);

    return 1;
}

static void control_screen_draw_lines(void)
{
    uint16 row;
    uint16 x;
    uint16 y;
    uint16 dx;
    uint16 dy;

    for(row = 0; row < SEARCH_IMAGE_H; row += SCREEN_LINE_DRAW_STEP)
    {
        y = SCREEN_IMAGE_Y + row * SCREEN_IMAGE_H / SEARCH_IMAGE_H;

        x = SCREEN_IMAGE_X + (uint16)left_edge_line[row] * SCREEN_IMAGE_W / SEARCH_IMAGE_W;
        if(x < (SCREEN_IMAGE_X + SCREEN_IMAGE_W))
        {
            for(dy = 0; dy < SCREEN_LINE_THICKNESS; dy++)
            {
                for(dx = 0; dx < SCREEN_LINE_THICKNESS; dx++)
                {
                    if((x + dx) < (SCREEN_IMAGE_X + SCREEN_IMAGE_W)
                       && (y + dy) < (SCREEN_IMAGE_Y + SCREEN_IMAGE_H))
                    {
                        ips200_draw_point(x + dx, y + dy, RGB565_RED);
                    }
                }
            }
        }

        x = SCREEN_IMAGE_X + (uint16)center_line[row] * SCREEN_IMAGE_W / SEARCH_IMAGE_W;
        if(x < (SCREEN_IMAGE_X + SCREEN_IMAGE_W))
        {
            for(dy = 0; dy < SCREEN_LINE_THICKNESS; dy++)
            {
                for(dx = 0; dx < SCREEN_LINE_THICKNESS; dx++)
                {
                    if((x + dx) < (SCREEN_IMAGE_X + SCREEN_IMAGE_W)
                       && (y + dy) < (SCREEN_IMAGE_Y + SCREEN_IMAGE_H))
                    {
                        ips200_draw_point(x + dx, y + dy, RGB565_GREEN);
                    }
                }
            }
        }

        x = SCREEN_IMAGE_X + (uint16)right_edge_line[row] * SCREEN_IMAGE_W / SEARCH_IMAGE_W;
        if(x < (SCREEN_IMAGE_X + SCREEN_IMAGE_W))
        {
            for(dy = 0; dy < SCREEN_LINE_THICKNESS; dy++)
            {
                for(dx = 0; dx < SCREEN_LINE_THICKNESS; dx++)
                {
                    if((x + dx) < (SCREEN_IMAGE_X + SCREEN_IMAGE_W)
                       && (y + dy) < (SCREEN_IMAGE_Y + SCREEN_IMAGE_H))
                    {
                        ips200_draw_point(x + dx, y + dy, RGB565_BLUE);
                    }
                }
            }
        }
    }
}

static void control_screen_update_fps(void)
{
    bit irq_backup = EA;
    uint16 elapsed;

    screenFrameCount++;

    if(screenFpsElapsedMs < 1000)
    {
        return;
    }

    EA = 0;
    elapsed = screenFpsElapsedMs;
    screenFpsElapsedMs = 0;
    EA = irq_backup;

    if(elapsed > 0)
    {
        screenFps = (uint16)((uint32)screenFrameCount * 1000UL / (uint32)elapsed);
    }
    screenFrameCount = 0;
}

static void control_screen_show_frame(void)
{
    control_screen_update_fps();

    ips200_show_gray_image(SCREEN_IMAGE_X,
                           SCREEN_IMAGE_Y,
                           (const uint8 *)&mt9v03x_image[0][0],
                           SEARCH_IMAGE_W,
                           SEARCH_IMAGE_H,
                           SCREEN_IMAGE_W,
                           SCREEN_IMAGE_H,
                           0);
    control_screen_draw_lines();

    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(0, SCREEN_INFO_Y, "FPS:");
    ips200_show_uint16(32, SCREEN_INFO_Y, screenFps);
    ips200_show_string(72, SCREEN_INFO_Y, "IF:");
    ips200_show_int32(96, SCREEN_INFO_Y, (int32)if_count, 5);
    ips200_show_string(0, SCREEN_INFO_Y + 16, lineFollowValid ? "LINE:OK  " : "LINE:LOST");
}

void control_line_follow_update(void)
{
    float error = 0.0f;
#if (SEEKFREE_ASSISTANT_TUNE_TARGET == ASSISTANT_TUNE_NONE)
    float servoTarget = 0.0f;
    float speedTarget = LINE_FOLLOW_STOP_SPEED;
#endif

    if (!mt9v03x_finish_flag)
    {
        return;
    }

    search_line_process((const uint8 *)&mt9v03x_image[0][0]);

    lineFollowValid = control_get_line_error(&error);
    lineFollowError = error;
    control_screen_show_frame();

#if (SEEKFREE_ASSISTANT_TUNE_TARGET == ASSISTANT_TUNE_NONE)
    if (lineFollowValid)
    {
        servoTarget = error * LINE_FOLLOW_SERVO_KP;
        speedTarget = LINE_FOLLOW_BASE_SPEED - func_abs(error) * LINE_FOLLOW_TURN_SLOW_KP;
        speedTarget = control_limit(speedTarget, LINE_FOLLOW_MIN_SPEED, LINE_FOLLOW_BASE_SPEED);

        Servo_SetTargetPosition(servoTarget);
        control_set_target_speed(speedTarget, speedTarget);
    }
    else
    {
        Servo_SetTargetPosition(0.0f);
        control_set_target_speed(LINE_FOLLOW_STOP_SPEED, LINE_FOLLOW_STOP_SPEED);
    }
#endif

    mt9v03x_finish_flag = 0;
}

void control_pit_handler(void)
{
    float motor1KpTemp = motor1Kp;
    float motor1KiTemp = motor1Ki;
    float motor1KdTemp = motor1Kd;
    float motor2KpTemp = motor2Kp;
    float motor2KiTemp = motor2Ki;
    float motor2KdTemp = motor2Kd;
    float motor1KiUse = 0.0f;
    float motor2KiUse = 0.0f;
    float errorTemp1 = 0.0f;
    float errorTemp2 = 0.0f;
    float deltaOutputTemp1 = 0.0f;
    float deltaOutputTemp2 = 0.0f;
    float outputTemp1 = 0.0f;
    float outputTemp2 = 0.0f;

    if(screenFpsElapsedMs < (60000U - CONTROL_PERIOD_MS))
    {
        screenFpsElapsedMs += CONTROL_PERIOD_MS;
    }

    encoderCount1 = ENCODER1_DIR_SIGN * encoder_get_count(SPEED_ENCODER1);
    encoderCount2 = ENCODER2_DIR_SIGN * encoder_get_count(SPEED_ENCODER2);

    // 清除编码器计数以准备下一次测量
    encoder_clear_count(SPEED_ENCODER1);
    encoder_clear_count(SPEED_ENCODER2);

    // 计算反馈速度
    feedbackSpeed1 = (float)encoderCount1 * (1000.0f / (float)CONTROL_PERIOD_MS);
    feedbackSpeed2 = (float)encoderCount2 * (1000.0f / (float)CONTROL_PERIOD_MS);

    // 斜坡处理目标速度
    targetSpeed1 = control_ramp_target(targetSpeed1, targetSpeedCommand1);
    targetSpeed2 = control_ramp_target(targetSpeed2, targetSpeedCommand2);

    // PID计算
    if (control_abs_less(targetSpeed1, SPEED_STOP_DEADBAND)
        || (control_abs_less(motor1KpTemp, PID_ZERO_DEADBAND)
            && control_abs_less(motor1KiTemp, PID_ZERO_DEADBAND)
            && control_abs_less(motor1KdTemp, PID_ZERO_DEADBAND)))
    {
        errorMotor1 = 0.0f;
        errorMotor1Last = 0.0f;
        errorMotor1Prev = 0.0f;
        deltaOutput1 = 0.0f;
        output1 = 0.0f;
        outputTemp1 = 0.0f;
    }
    else
    {
        errorTemp1 = (targetSpeed1 - feedbackSpeed1) * SPEED_METER_PER_PULSE;   // 将速度误差转换为实际距离误差，
        if (control_abs_less(targetSpeedCommand1 - targetSpeed1, TARGET_RAMP_STEP)) // 只有在目标速度接近斜坡结束时才使用积分
        {
            motor1KiUse = motor1KiTemp;
        }
        deltaOutputTemp1 = motor1KpTemp * (errorTemp1 - errorMotor1Last)
                         + motor1KiUse * errorTemp1
                         + motor1KdTemp * (errorTemp1 - 2.0f * errorMotor1Last + errorMotor1Prev);  // 增量式PID公式
        outputTemp1 = control_limit(output1 + deltaOutputTemp1, -SPEED_LOOP_DUTY_LIMIT, SPEED_LOOP_DUTY_LIMIT); // 将输出限制在占空比范围内

        errorMotor1 = errorTemp1;
        deltaOutput1 = deltaOutputTemp1;
        output1 = outputTemp1;
        errorMotor1Prev = errorMotor1Last;
        errorMotor1Last = errorTemp1;
    }

    if (control_abs_less(targetSpeed2, SPEED_STOP_DEADBAND)
        || (control_abs_less(motor2KpTemp, PID_ZERO_DEADBAND)
            && control_abs_less(motor2KiTemp, PID_ZERO_DEADBAND)
            && control_abs_less(motor2KdTemp, PID_ZERO_DEADBAND)))
    {
        errorMotor2 = 0.0f;
        errorMotor2Last = 0.0f;
        errorMotor2Prev = 0.0f;
        deltaOutput2 = 0.0f;
        output2 = 0.0f;
        outputTemp2 = 0.0f;
    }
    else
    {
        errorTemp2 = (targetSpeed2 - feedbackSpeed2) * SPEED_METER_PER_PULSE;
        if (control_abs_less(targetSpeedCommand2 - targetSpeed2, TARGET_RAMP_STEP))
        {
            motor2KiUse = motor2KiTemp;
        }
        deltaOutputTemp2 = motor2KpTemp * (errorTemp2 - errorMotor2Last)
                         + motor2KiUse * errorTemp2
                         + motor2KdTemp * (errorTemp2 - 2.0f * errorMotor2Last + errorMotor2Prev);
        outputTemp2 = control_limit(output2 + deltaOutputTemp2, -SPEED_LOOP_DUTY_LIMIT, SPEED_LOOP_DUTY_LIMIT);

        errorMotor2 = errorTemp2;
        deltaOutput2 = deltaOutputTemp2;
        output2 = outputTemp2;
        errorMotor2Prev = errorMotor2Last;
        errorMotor2Last = errorTemp2;
    }

#if (SEEKFREE_ASSISTANT_TUNE_TARGET == ASSISTANT_TUNE_SERVO)
    Motor1_SetSpeed(0.0f);
    Motor2_SetSpeed(0.0f);
    Servo_PositionPdUpdate();
#elif (SEEKFREE_ASSISTANT_TUNE_TARGET == ASSISTANT_TUNE_MOTOR)
    Motor1_SetSpeed(outputTemp1);
    Motor2_SetSpeed(outputTemp2);
#elif (SEEKFREE_ASSISTANT_TUNE_TARGET == ASSISTANT_TUNE_NONE)
    Motor1_SetSpeed(outputTemp1);
    Motor2_SetSpeed(outputTemp2);
    Servo_PositionPdUpdate();
#else
#error "SEEKFREE_ASSISTANT_TUNE_TARGET must be ASSISTANT_TUNE_MOTOR, ASSISTANT_TUNE_SERVO or ASSISTANT_TUNE_NONE"
#endif
}

float control_limit(float value, float low, float high)
{
    if (value > high)
    {
        value = high;
    }
    if (value < low)
    {
        value = low;
    }
    return value;
}

void control_update(void)
{
    bit irq_backup = EA;
    EA = 0;

#if (SEEKFREE_ASSISTANT_TUNE_TARGET == ASSISTANT_TUNE_SERVO)
    Servo_AssistantUpdate(&lastParamChannel, &lastParamValue);

#elif (SEEKFREE_ASSISTANT_TUNE_TARGET == ASSISTANT_TUNE_MOTOR)
    if (seekfree_assistant_parameter_update_flag[PARAM_MOTOR1_KP_INDEX])
    {
        motor1Kp = control_limit(seekfree_assistant_parameter[PARAM_MOTOR1_KP_INDEX], PID_KP_MIN, PID_KP_MAX);
        lastParamChannel = (float)(PARAM_MOTOR1_KP_INDEX + 1);
        lastParamValue = motor1Kp;
        seekfree_assistant_parameter_update_flag[PARAM_MOTOR1_KP_INDEX] = 0;
    }
    if (seekfree_assistant_parameter_update_flag[PARAM_MOTOR1_KI_INDEX])
    {
        motor1Ki = control_limit(seekfree_assistant_parameter[PARAM_MOTOR1_KI_INDEX], PID_KI_MIN, PID_KI_MAX);
        lastParamChannel = (float)(PARAM_MOTOR1_KI_INDEX + 1);
        lastParamValue = motor1Ki;
        seekfree_assistant_parameter_update_flag[PARAM_MOTOR1_KI_INDEX] = 0;
    }
    if (seekfree_assistant_parameter_update_flag[PARAM_MOTOR1_KD_INDEX])
    {
        motor1Kd = control_limit(seekfree_assistant_parameter[PARAM_MOTOR1_KD_INDEX], PID_KD_MIN, PID_KD_MAX);
        lastParamChannel = (float)(PARAM_MOTOR1_KD_INDEX + 1);
        lastParamValue = motor1Kd;
        seekfree_assistant_parameter_update_flag[PARAM_MOTOR1_KD_INDEX] = 0;
    }
    if (seekfree_assistant_parameter_update_flag[PARAM_MOTOR1_TARGET_INDEX])
    {
        targetSpeedCommand1 = control_limit(seekfree_assistant_parameter[PARAM_MOTOR1_TARGET_INDEX], TARGET_SPEED_MIN, TARGET_SPEED_MAX);
        if (control_abs_less(targetSpeedCommand1, SPEED_STOP_DEADBAND))
        {
            targetSpeed1 = 0.0f;
            errorMotor1 = 0.0f;
            errorMotor1Last = 0.0f;
            errorMotor1Prev = 0.0f;
            deltaOutput1 = 0.0f;
            output1 = 0.0f;
        }
        lastParamChannel = (float)(PARAM_MOTOR1_TARGET_INDEX + 1);
        lastParamValue = targetSpeedCommand1;
        seekfree_assistant_parameter_update_flag[PARAM_MOTOR1_TARGET_INDEX] = 0;
    }

    if (seekfree_assistant_parameter_update_flag[PARAM_MOTOR2_KP_INDEX])
    {
        motor2Kp = control_limit(seekfree_assistant_parameter[PARAM_MOTOR2_KP_INDEX], PID_KP_MIN, PID_KP_MAX);
        lastParamChannel = (float)(PARAM_MOTOR2_KP_INDEX + 1);
        lastParamValue = motor2Kp;
        seekfree_assistant_parameter_update_flag[PARAM_MOTOR2_KP_INDEX] = 0;
    }
    if (seekfree_assistant_parameter_update_flag[PARAM_MOTOR2_KI_INDEX])
    {
        motor2Ki = control_limit(seekfree_assistant_parameter[PARAM_MOTOR2_KI_INDEX], PID_KI_MIN, PID_KI_MAX);
        lastParamChannel = (float)(PARAM_MOTOR2_KI_INDEX + 1);
        lastParamValue = motor2Ki;
        seekfree_assistant_parameter_update_flag[PARAM_MOTOR2_KI_INDEX] = 0;
    }
    if (seekfree_assistant_parameter_update_flag[PARAM_MOTOR2_KD_INDEX])
    {
        motor2Kd = control_limit(seekfree_assistant_parameter[PARAM_MOTOR2_KD_INDEX], PID_KD_MIN, PID_KD_MAX);
        lastParamChannel = (float)(PARAM_MOTOR2_KD_INDEX + 1);
        lastParamValue = motor2Kd;
        seekfree_assistant_parameter_update_flag[PARAM_MOTOR2_KD_INDEX] = 0;
    }
    if (seekfree_assistant_parameter_update_flag[PARAM_MOTOR2_TARGET_INDEX])
    {
        targetSpeedCommand2 = control_limit(seekfree_assistant_parameter[PARAM_MOTOR2_TARGET_INDEX], TARGET_SPEED_MIN, TARGET_SPEED_MAX);
        if (control_abs_less(targetSpeedCommand2, SPEED_STOP_DEADBAND))
        {
            targetSpeed2 = 0.0f;
            errorMotor2 = 0.0f;
            errorMotor2Last = 0.0f;
            errorMotor2Prev = 0.0f;
            deltaOutput2 = 0.0f;
            output2 = 0.0f;
        }
        lastParamChannel = (float)(PARAM_MOTOR2_TARGET_INDEX + 1);
        lastParamValue = targetSpeedCommand2;
        seekfree_assistant_parameter_update_flag[PARAM_MOTOR2_TARGET_INDEX] = 0;
    }

#elif (SEEKFREE_ASSISTANT_TUNE_TARGET == ASSISTANT_TUNE_NONE)
    lastParamChannel = 0.0f;
    lastParamValue = 0.0f;

#else
#error "SEEKFREE_ASSISTANT_TUNE_TARGET must be ASSISTANT_TUNE_MOTOR, ASSISTANT_TUNE_SERVO or ASSISTANT_TUNE_NONE"
#endif

    EA = irq_backup;
}

void control_sendOscData(void)
{
#if (SEEKFREE_ASSISTANT_TUNE_TARGET == ASSISTANT_TUNE_SERVO)
    float localServoCommand = 0.0f;
    float localServoTarget = 0.0f;
    float localServoFeedback = 0.0f;
    float localServoOutput = 0.0f;
    float localServoPwmDuty = 0.0f;
    float localLastParamChannel = 0.0f;
    float localLastParamValue = 0.0f;
    bit irq_backup = EA;
    EA = 0;

    localServoCommand = Servo_GetTargetCommand();
    localServoTarget = Servo_GetTargetPosition();
    localServoFeedback = Servo_GetFeedbackPosition();
    localServoOutput = Servo_GetOutputDuty();
    localServoPwmDuty = Servo_GetPwmDuty();
    localLastParamChannel = lastParamChannel;
    localLastParamValue = lastParamValue;

    EA = irq_backup;

    seekfree_assistant_oscilloscope_data.dat[0] = localServoCommand;
    seekfree_assistant_oscilloscope_data.dat[1] = localServoTarget;
    seekfree_assistant_oscilloscope_data.dat[2] = localServoFeedback;
    seekfree_assistant_oscilloscope_data.dat[3] = localServoOutput;
    seekfree_assistant_oscilloscope_data.dat[4] = localServoPwmDuty;
    seekfree_assistant_oscilloscope_data.dat[5] = localLastParamChannel;
    seekfree_assistant_oscilloscope_data.dat[6] = localLastParamValue;
    seekfree_assistant_oscilloscope_data.channel_num = 7;

    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
#elif (SEEKFREE_ASSISTANT_TUNE_TARGET == ASSISTANT_TUNE_MOTOR)
    float localTargetSpeed1 = 0.0f;
    float localTargetSpeed2 = 0.0f;
    float localFeedbackSpeed1 = 0.0f;
    float localFeedbackSpeed2 = 0.0f;
    float localOutput1 = 0.0f;
    float localOutput2 = 0.0f;
    float localLastParamChannel = 0.0f;
    float localLastParamValue = 0.0f;
    bit irq_backup = EA;
    EA = 0;

    localTargetSpeed1 = targetSpeed1;
    localTargetSpeed2 = targetSpeed2;
    localFeedbackSpeed1 = feedbackSpeed1;
    localFeedbackSpeed2 = feedbackSpeed2;
    localOutput1 = output1;
    localOutput2 = output2;
    localLastParamChannel = lastParamChannel;
    localLastParamValue = lastParamValue;

    EA = irq_backup;

    seekfree_assistant_oscilloscope_data.dat[0] = localTargetSpeed1;
    seekfree_assistant_oscilloscope_data.dat[1] = localFeedbackSpeed1;
    seekfree_assistant_oscilloscope_data.dat[2] = localOutput1;
    seekfree_assistant_oscilloscope_data.dat[3] = localTargetSpeed2;
    seekfree_assistant_oscilloscope_data.dat[4] = localFeedbackSpeed2;
    seekfree_assistant_oscilloscope_data.dat[5] = localOutput2;
    seekfree_assistant_oscilloscope_data.dat[6] = localLastParamChannel;
    seekfree_assistant_oscilloscope_data.dat[7] = localLastParamValue;
    seekfree_assistant_oscilloscope_data.channel_num = 8;

    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
#elif (SEEKFREE_ASSISTANT_TUNE_TARGET == ASSISTANT_TUNE_NONE)
    float localTargetSpeed1 = 0.0f;
    float localFeedbackSpeed1 = 0.0f;
    float localOutput1 = 0.0f;
    float localTargetSpeed2 = 0.0f;
    float localFeedbackSpeed2 = 0.0f;
    float localOutput2 = 0.0f;
    float localServoTarget = 0.0f;
    float localServoOutput = 0.0f;
    bit irq_backup = EA;
    EA = 0;

    localTargetSpeed1 = targetSpeed1;
    localFeedbackSpeed1 = feedbackSpeed1;
    localOutput1 = output1;
    localTargetSpeed2 = targetSpeed2;
    localFeedbackSpeed2 = feedbackSpeed2;
    localOutput2 = output2;
    localServoTarget = Servo_GetTargetPosition();
    localServoOutput = Servo_GetOutputDuty();

    EA = irq_backup;

    seekfree_assistant_oscilloscope_data.dat[0] = localTargetSpeed1;
    seekfree_assistant_oscilloscope_data.dat[1] = localFeedbackSpeed1;
    seekfree_assistant_oscilloscope_data.dat[2] = localOutput1;
    seekfree_assistant_oscilloscope_data.dat[3] = localTargetSpeed2;
    seekfree_assistant_oscilloscope_data.dat[4] = localFeedbackSpeed2;
    seekfree_assistant_oscilloscope_data.dat[5] = localOutput2;
    seekfree_assistant_oscilloscope_data.dat[6] = localServoTarget;
    seekfree_assistant_oscilloscope_data.dat[7] = localServoOutput;
    seekfree_assistant_oscilloscope_data.channel_num = 8;

    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
#else
#error "SEEKFREE_ASSISTANT_TUNE_TARGET must be ASSISTANT_TUNE_MOTOR, ASSISTANT_TUNE_SERVO or ASSISTANT_TUNE_NONE"
#endif
}

void Motor1_SetSpeed(float dutyPercent)
{
    uint32 pwmDuty = 0;

    dutyPercent *= MOTOR1_DIR_SIGN;

    if (dutyPercent >= 0.0f)
    {
        gpio_set_level(DIR_1, GPIO_HIGH);
    }
    else
    {
        gpio_set_level(DIR_1, GPIO_LOW);
        dutyPercent = -dutyPercent;
    }

    dutyPercent = control_limit(dutyPercent, 0.0f, (float)MAX_DUTY);
    pwmDuty = (uint32)(dutyPercent * ((float)PWM_DUTY_MAX / 100.0f));

    pwm_set_duty(PWM_1, pwmDuty);
}

void Motor2_SetSpeed(float dutyPercent)
{
    uint32 pwmDuty = 0;

    dutyPercent *= MOTOR2_DIR_SIGN;

    if (dutyPercent >= 0.0f)
    {
        gpio_set_level(DIR_2, GPIO_HIGH);
    }
    else
    {
        gpio_set_level(DIR_2, GPIO_LOW);
        dutyPercent = -dutyPercent;
    }

    dutyPercent = control_limit(dutyPercent, 0.0f, (float)MAX_DUTY);
    pwmDuty = (uint32)(dutyPercent * ((float)PWM_DUTY_MAX / 100.0f));

    pwm_set_duty(PWM_2, pwmDuty);
}
