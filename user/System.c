#include "System.h"
#include "Motor.h"
#include "search_line.h"
#include "search_geometry.h"
#include "run_config.h"
#include "imu.h"

#define SYSTEM_KEY_PREV              (IO_P32)
#define SYSTEM_KEY_NEXT              (IO_PB3)
#define SYSTEM_KEY_SELECT            (IO_PB4)
#define SYSTEM_KEY_BACK              (IO_PB2)

#define SYSTEM_KEY_PRESSED           (GPIO_LOW)
#define SYSTEM_KEY_DEBOUNCE_TICKS    (6U) /* 5 ms 节拍：电平稳定 30 ms 后确认按键 */
#define SYSTEM_IMAGE_DISPLAY_PERIOD_TICKS (20U)  /* 图像和叠加线最多每 100ms 刷新，搜线仍处理每个新帧 */
#define SYSTEM_PITCH_DISPLAY_PERIOD_TICKS (100U) /* 俯仰数字最多每 500ms 更新；IMU 状态变化立即显示 */

#define SYSTEM_MAIN_ITEM_COUNT       (3U)
#define SYSTEM_PID_ITEM_COUNT        (6U)
#define SYSTEM_PID_STEP              (5)
#define SYSTEM_PID_MIN               (-30000L)
#define SYSTEM_PID_MAX               (30000L)
#define SYSTEM_MOTOR_SPEED_MAX       (1000)
#define SYSTEM_IMAGE_THRESHOLD       (128U)
#define SYSTEM_IMAGE_X               0U
#define SYSTEM_IMAGE_Y               40U
#define SYSTEM_IMAGE_WIDTH           135U
#define SYSTEM_IMAGE_HEIGHT          86U

// EEPROM page 0 (512 bytes) is reserved for the menu settings.
#define SYSTEM_PID_EEPROM_ADDRESS    (0x0000UL)
#define SYSTEM_PID_STORAGE_SIZE      (14U)
#define SYSTEM_PID_OLD_STORAGE_SIZE  (13U)
#define SYSTEM_PID_MAGIC_0           (0x53U)
#define SYSTEM_PID_MAGIC_1           (0x50U)
#define SYSTEM_PID_STORAGE_VERSION   (4U)

typedef enum
{
    SYSTEM_PAGE_MAIN = 0,
    SYSTEM_PAGE_IMAGE,
    SYSTEM_PAGE_SET_PID,
    SYSTEM_PAGE_RUN
} system_page_t;

typedef struct
{
    gpio_pin_enum pin;
    uint8 stable_level;
    uint8 sampled_level;
    uint16 sampled_tick; /* 当前采样电平开始保持的 5 ms 节拍，不随前台循环次数增加 */
} system_key_t;

// Left/right motor speed PID values, using Motor.c's integer convention.
typedef struct
{
    int16 kp;
    int16 ki;
    int16 kd;
    int16 motor_speed;   // Straight/manual speed in encoder counts per 5 ms, 0..1000
    uint8 motor_enabled; // Runtime only: 0 off, 1 on
    int16 servo_kp;      // Steering gain: 100 = 1.00x, 0..1000
} system_pid_config_t;

static system_pid_config_t system_pid_config = {3000, 50, 50, RUN_SPEED_STRAIGHT_DEFAULT, 0, RUN_SERVO_GAIN};

static system_page_t system_page = SYSTEM_PAGE_MAIN;
static uint8 system_main_item = 0;
static uint8 system_pid_item = 0;
static uint8 system_camera_ready = 0;
static uint8 system_pid_dirty = 0;
/* 当前交给电机控制的连续图像中线段；Point.x为行、Point.y为列，按近到远排列。 */
static Point system_road_line[SEARCH_IMAGE_H];
/* 当前连续可控路径点数，不包含质量为1的历史预测点。 */
static uint8 system_road_count = 0;
/* Motor_SelectTarget选中的system_road_line索引。 */
static uint8 system_target_index = 0;
/* 本次Run是否已经处理过一帧，供等待提示及首次显示使用。 */
static uint8 system_run_has_frame = 0;
static uint8 system_zebra_stopped = 0; /* 本次 Run 已因斑马线停车；仅重新进入 Run 才清除 */
#if RUN_DISPLAY_ENABLED
/* Run画面上次刷新时的5ms节拍，用于限制LCD更新频率。 */
static uint16 system_display_tick = 0;
#endif
static uint16 system_pitch_display_tick;
static uint16 system_image_display_tick; /* 上次图像写屏完成时的 5ms 节拍 */
static uint8 system_image_display_valid; /* 页面首次收到图像时立即显示，不等待刷新周期 */
/* Image 页仅在数值/状态改变时重画文字；进入页面或重试校准后强制刷新。 */
static uint8 system_ring_display_state = 255;
static uint8 system_pitch_display_valid;
static int16 system_pitch_display_value;
static uint8 system_pitch_display_ready, system_imu_display_status, system_imu_display_failure;
/* Image 实测统计使用现有 5 ms 节拍，不额外占用定时器；每秒更新一次。 */
static uint16 system_image_stats_tick, system_image_frame_count;
static uint16 system_image_display_count; /* 本统计窗口实际写屏次数，与处理帧数分开累计 */
static uint32 system_image_vision_ticks, system_image_lcd_ticks;
/* 本帧路径检查状态，SYSTEM_PATH_*表示成功、丢失、无近点、过短或无标定。 */
static uint8 system_path_status = 0;
#define SYSTEM_PATH_OK    0U
#define SYSTEM_PATH_LOST  1U
#define SYSTEM_PATH_NEAR  2U
#define SYSTEM_PATH_SHORT 3U
#define SYSTEM_PATH_CALIB 4U

#define SYSTEM_RUN_IMAGE_X 26U
#define SYSTEM_RUN_IMAGE_Y 32U

static system_key_t system_key_prev;
static system_key_t system_key_next;
static system_key_t system_key_select;
static system_key_t system_key_back;

static uint8 system_pid_crc(const uint8 *bytes, uint8 length)
{
    uint8 crc;
    uint8 i;
    uint8 bit_index;

    crc = 0;
    for(i = 0; i < length; i++)
    {
        crc ^= bytes[i];
        for(bit_index = 0; bit_index < 8; bit_index++)
        {
            if(crc & 0x80)
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static void system_pid_encode_int16(uint8 *destination, int16 value)
{
    uint16 raw_value;

    raw_value = (uint16)value;
    destination[0] = (uint8)(raw_value & 0x00FF);
    destination[1] = (uint8)(raw_value >> 8);
}

static int16 system_pid_decode_int16(const uint8 *source)
{
    uint16 raw_value;

    raw_value = (uint16)source[0] | ((uint16)source[1] << 8);
    return (int16)raw_value;
}

static void system_pid_load(void)
{
    uint8 storage[SYSTEM_PID_STORAGE_SIZE];
    int16 kp;
    int16 ki;
    int16 kd;
    int16 motor_speed;
    int16 servo_kp;

    iap_init();
    iap_read_buff(SYSTEM_PID_EEPROM_ADDRESS, storage, SYSTEM_PID_STORAGE_SIZE);
    iap_idle();

    if((SYSTEM_PID_MAGIC_0 != storage[0]) ||
       (SYSTEM_PID_MAGIC_1 != storage[1]) ||
       ((1U != storage[2]) && (2U != storage[2]) && (3U != storage[2]) &&
        (SYSTEM_PID_STORAGE_VERSION != storage[2])))
    {
        return;
    }

    if(1U == storage[2])
    {
        if(system_pid_crc(storage, 9) != storage[9]) return;
    }
    else if(2U == storage[2])
    {
        if(system_pid_crc(storage, SYSTEM_PID_OLD_STORAGE_SIZE - 1) != storage[12]) return;
    }
    else if(3U == storage[2])
    {
        if(system_pid_crc(storage, 11) != storage[11]) return;
    }
    else if(system_pid_crc(storage, SYSTEM_PID_STORAGE_SIZE - 1) != storage[13])
    {
        return;
    }

    kp = system_pid_decode_int16(&storage[3]);
    ki = system_pid_decode_int16(&storage[5]);
    kd = system_pid_decode_int16(&storage[7]);
    if((kp < SYSTEM_PID_MIN) || (kp > SYSTEM_PID_MAX) ||
       (ki < SYSTEM_PID_MIN) || (ki > SYSTEM_PID_MAX) ||
       (kd < SYSTEM_PID_MIN) || (kd > SYSTEM_PID_MAX))
    {
        return;
    }

    motor_speed = system_pid_config.motor_speed;
    servo_kp = RUN_SERVO_GAIN;
    if(1U != storage[2])
    {
        motor_speed = system_pid_decode_int16(&storage[9]);
        if((motor_speed < 0) || (motor_speed > SYSTEM_MOTOR_SPEED_MAX))
        {
            return;
        }
    }
    if(SYSTEM_PID_STORAGE_VERSION == storage[2])
    {
        servo_kp = system_pid_decode_int16(&storage[11]);
        if(servo_kp < 0 || servo_kp > MOTOR_SERVO_KP_MAX) return;
    }
    system_pid_config.motor_speed = motor_speed;
    system_pid_config.servo_kp = servo_kp;
    system_pid_config.kp = kp;
    system_pid_config.ki = ki;
    system_pid_config.kd = kd;
}

static void system_pid_save(void)
{
    uint8 storage[SYSTEM_PID_STORAGE_SIZE];

    storage[0] = SYSTEM_PID_MAGIC_0;
    storage[1] = SYSTEM_PID_MAGIC_1;
    storage[2] = SYSTEM_PID_STORAGE_VERSION;
    system_pid_encode_int16(&storage[3], system_pid_config.kp);
    system_pid_encode_int16(&storage[5], system_pid_config.ki);
    system_pid_encode_int16(&storage[7], system_pid_config.kd);
    system_pid_encode_int16(&storage[9], system_pid_config.motor_speed);
    system_pid_encode_int16(&storage[11], system_pid_config.servo_kp);
    storage[13] = system_pid_crc(storage, SYSTEM_PID_STORAGE_SIZE - 1);

    iap_init();
    iap_erase_page(SYSTEM_PID_EEPROM_ADDRESS);
    iap_write_buff(SYSTEM_PID_EEPROM_ADDRESS, storage, SYSTEM_PID_STORAGE_SIZE);
    iap_idle();
    system_pid_dirty = 0;
}

static void system_key_init(system_key_t *key, gpio_pin_enum pin)
{
    uint8 level;

    gpio_init(pin, GPI, GPIO_HIGH, GPI_PULL_UP);
    level = gpio_get_level(pin);
    key->pin = pin;
    key->stable_level = level;
    key->sampled_level = level;
    key->sampled_tick = Motor_GetTicks();
}

// 电平持续稳定 30 ms 后仅报告一次按下；真实节拍消抖使 Run 快速循环时也不会误触发。
static uint8 system_key_pressed(system_key_t *key, uint16 now)
{
    uint8 level;

    level = gpio_get_level(key->pin);
    if(level != key->sampled_level)
    {
        key->sampled_level = level;
        key->sampled_tick = now;
    }
    else if(key->stable_level != level &&
        (uint16)(now - key->sampled_tick) >= SYSTEM_KEY_DEBOUNCE_TICKS)
    {
        key->stable_level = level;
        if(SYSTEM_KEY_PRESSED == level)
        {
            return 1;
        }
    }

    return 0;
}

static void system_draw_main(void)
{
    ips200_clear(RGB565_WHITE);
    ips200_set_color(RGB565_BLUE, RGB565_WHITE);
    ips200_show_string(24, 12, "MAIN MENU");

    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(12, 56,  (0 == system_main_item) ? "> Image"   : "  Image");
    ips200_show_string(12, 96,  (1 == system_main_item) ? "> Set PID" : "  Set PID");
    ips200_show_string(12, 136, (2 == system_main_item) ? "> Run"     : "  Run");

    ips200_set_color(RGB565_GREEN, RGB565_WHITE);
    ips200_show_string(4, 200, "UP DOWN SELECT");
}

static void system_draw_pid(void)
{
    ips200_clear(RGB565_WHITE);
    ips200_set_color(RGB565_BLUE, RGB565_WHITE);
    ips200_show_string(24, 8, system_pid_dirty ? "SET PID *" : "SET PID");

    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(8, 40, (0 == system_pid_item) ? "> Kp:" : "  Kp:");
    ips200_show_int16(104, 40, system_pid_config.kp);
    ips200_show_string(8, 68, (1 == system_pid_item) ? "> Ki:" : "  Ki:");
    ips200_show_int16(104, 68, system_pid_config.ki);
    ips200_show_string(8, 96, (2 == system_pid_item) ? "> Kd:" : "  Kd:");
    ips200_show_int16(104, 96, system_pid_config.kd);
    ips200_show_string(8, 124, (3 == system_pid_item) ? "> Speed:" : "  Speed:");
    ips200_show_int16(104, 124, system_pid_config.motor_speed);
    ips200_show_string(8, 152, (4 == system_pid_item) ? "> Enable:" : "  Enable:");
    ips200_show_uint8(104, 152, system_pid_config.motor_enabled);
    ips200_show_string(8, 180, (5 == system_pid_item) ? "> Servo Kp:" : "  Servo Kp:");
    ips200_show_int16(104, 180, system_pid_config.servo_kp);

    ips200_set_color(RGB565_GREEN, RGB565_WHITE);
    ips200_show_string(4, 212, "UP/DOWN: +/-5, 0/1");
    ips200_show_string(4, 232, "SELECT: NEXT");
    ips200_show_string(4, 252, "BACK: SAVE / STOP");
}

/* Use the same source rows as ips200_show_gray_image when scaling. Missing
 * observations remain gaps; history stays visually distinct from fresh data. */
static void system_draw_line_overlay(uint16 x, uint16 y, uint16 width, uint16 height)
{
    uint16 display_row, row;
    for(display_row = 0; display_row < height; ++display_row)
    {
        row = (uint16)((uint32)display_row * SEARCH_IMAGE_H / height);
        if(row < STOPROW) continue;
        if(left_valid[row])
            ips200_draw_point(x + (uint16)((uint32)left_edge_line[row] * width / SEARCH_IMAGE_W),
                y + display_row, RGB565_RED);
        if(right_valid[row])
            ips200_draw_point(x + (uint16)((uint32)right_edge_line[row] * width / SEARCH_IMAGE_W),
                y + display_row, RGB565_BLUE);
        if(center_valid[row])
            ips200_draw_point(x + (uint16)((uint32)center_line[row] * width / SEARCH_IMAGE_W),
                y + display_row, center_source[row] == CENTER_HISTORY ? RGB565_YELLOW : RGB565_GREEN);
    }
}

// ============================================
// 元素提示仅在显示状态改变时刷新，避免相同字符串每帧重复写屏。
// 斑马线提示优先；圆环区分候选和已入环，视觉状态不依赖累计转角。
// ============================================
static void system_draw_ring_status(void)
{
    uint8 state; /* 当前需要显示的状态编码，与上次一致则直接返回。 */
    state = zebra_detected ? 7 : zebra_candidate ? 8 : ring_state == RING_STATE_APPROACH ? 6 :
        (ring_state != RING_STATE_NONE || ring_dir == RING_DIR_RIGHT) ? 2 :
        ring_candidate == RING_DIR_RIGHT ? 5 : 0;
    if(state == system_ring_display_state) return;
    system_ring_display_state = state;
    ips200_show_string(8, 152,
        state == 7 ? "Zebra: CONFIRMED        " :
        state == 8 ? "Zebra: CHECKING         " :
        state == 2 ? "Ring: RIGHT            " :
        state == 5 ? "Ring: RIGHT?           " :
        state == 6 ? "Ring: WAIT ISLAND END  " :
        "Ring: NONE             ");
}

// ============================================
// 每秒显示 Image 页实际处理帧率，以及每次视觉处理/每次图像写屏的平均耗时。
// 图像显示已独立限频，FPS 统计处理帧数；VIS/LCD 单位 ms，精度受 5ms 节拍限制。
// ============================================
static void system_draw_image_stats(void)
{
    uint16 now, elapsed; /* 当前节拍和统计窗口长度，允许 16 位计数回绕。 */
    uint32 value; /* 换算后的帧率或毫秒数，显示前限幅。 */
    now = Motor_GetTicks();
    elapsed = (uint16)(now - system_image_stats_tick);
    if(elapsed < 200U) return;
    value = (uint32)system_image_frame_count * 200UL / elapsed;
    ips200_show_uint8(48, 256, (uint8)(value > 255UL ? 255UL : value));
    value = system_image_frame_count ? system_image_vision_ticks * 5UL / system_image_frame_count : 0;
    ips200_show_int16(40, 280, (int16)(value > 32767UL ? 32767UL : value));
    value = system_image_display_count ? system_image_lcd_ticks * 5UL / system_image_display_count : 0;
    ips200_show_int16(160, 280, (int16)(value > 32767UL ? 32767UL : value));
    system_image_stats_tick = now;
    system_image_frame_count = 0;
    system_image_display_count = 0;
    system_image_vision_ticks = system_image_lcd_ticks = 0;
}

static void system_draw_image_frame(void)
{
    uint16 started, searched, drawn; /* 视觉开始、视觉结束和图像写屏结束的节拍快照。 */
    uint8 draw_image;
    started = Motor_GetTicks();
    draw_image = !system_image_display_valid ||
        (uint16)(started - system_image_display_tick) >= SYSTEM_IMAGE_DISPLAY_PERIOD_TICKS;
    search_extract_edges_timed(&mt9v03x_image[0][0], started);
    if(!draw_image) mt9v03x_finish_flag = 0;
    search_finish_frame();
    searched = Motor_GetTicks();
    if(draw_image)
    {
        // 新帧照常识别，只限制耗时的图像和边线写屏；显示完成前保持相机帧锁。
        ips200_show_gray_image(SYSTEM_IMAGE_X, SYSTEM_IMAGE_Y,
            &mt9v03x_image[0][0], SEARCH_IMAGE_W, SEARCH_IMAGE_H,
            SYSTEM_IMAGE_WIDTH, SYSTEM_IMAGE_HEIGHT, SYSTEM_IMAGE_THRESHOLD);
        system_draw_line_overlay(SYSTEM_IMAGE_X, SYSTEM_IMAGE_Y,
            SYSTEM_IMAGE_WIDTH, SYSTEM_IMAGE_HEIGHT);
        mt9v03x_finish_flag = 0; // 最后一次读取像素后释放；后续统计不能再次清标志
        drawn = Motor_GetTicks();
        system_image_lcd_ticks += (uint16)(drawn - searched);
        ++system_image_display_count;
        system_image_display_tick = drawn;
        system_image_display_valid = 1;
    }
    // 状态提示仅在内容改变时写屏，可立即显示确认/出环结果，不受图像刷新周期限制。
    system_draw_ring_status();
    system_image_vision_ticks += (uint16)(searched - started);
    ++system_image_frame_count;
}

/* Image-only refresh; snapshot the ISR-owned 16-bit value atomically. */
static void system_draw_pitch(void)
{
    int16 value;
    uint8 ready, cal_status, cal_failure; /* IMU 就绪、校准状态及失败原因的同一份快照。 */
    bit irq_state;
    irq_state = EA;
    EA = 0;
    value = pitch;
    ready = imu_ready;
    cal_status = imu_cal_status;
    cal_failure = imu_cal_failure;
    EA = irq_state;
    if(!system_pitch_display_valid || ready != system_pitch_display_ready ||
       (ready && value != system_pitch_display_value))
    {
        ips200_set_color(RGB565_BLACK, RGB565_WHITE);
        /* 数值显示固定六字符；只在有效/无效切换时清除旧占位符。 */
        if(!system_pitch_display_valid || ready != system_pitch_display_ready)
            ips200_show_string(104, 168, "            ");
        if(ready) ips200_show_int16(104, 168, value);
        else ips200_show_string(104, 168, "--");
    }
    if(!system_pitch_display_valid || cal_status != system_imu_display_status ||
       cal_failure != system_imu_display_failure)
    {
        ips200_set_color(RGB565_BLACK, RGB565_WHITE);
        /* 状态改变才清整行，避免较短提示留下旧文字，同时减少重复写屏。 */
        ips200_show_string(8, 184, "                            ");
        if(cal_status == IMU_STATUS_CAL_FAILED)
            ips200_show_string(8, 184,
                cal_failure == (IMU_CAL_FAIL_ABS | IMU_CAL_FAIL_SPAN) ? "IMU FAIL:HIGH+UNSTABLE" :
                (cal_failure & IMU_CAL_FAIL_SPAN) ? "IMU FAIL:UNSTABLE" :
                (cal_failure & IMU_CAL_FAIL_ABS) ? "IMU FAIL:GYRO HIGH" : "IMU CAL FAILED");
        else
            ips200_show_string(8, 184, cal_status == IMU_STATUS_READY ? "IMU CAL OK" :
                (cal_status == IMU_STATUS_CALIBRATING ? "IMU CAL: KEEP STILL" : "IMU MISSING"));
    }
    system_pitch_display_valid = 1;
    system_pitch_display_value = value;
    system_pitch_display_ready = ready;
    system_imu_display_status = cal_status;
    system_imu_display_failure = cal_failure;
    system_pitch_display_tick = Motor_GetTicks();
}

static void system_draw_image_page(void)
{
    system_ring_display_state = 255;
    system_pitch_display_valid = 0;
    system_image_display_valid = 0;
    system_image_display_tick = Motor_GetTicks();
    system_image_frame_count = 0;
    system_image_display_count = 0;
    system_image_vision_ticks = system_image_lcd_ticks = 0;
    ips200_clear(RGB565_WHITE);
    ips200_set_color(RGB565_BLUE, RGB565_WHITE);
    ips200_show_string(16, 8, "BINARY IMAGE");

    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    if(!system_camera_ready)
    {
        ips200_show_string(8, 72, "CAMERA ERROR");
    }
    ips200_show_string(0, 136, "Threshold:");
    ips200_show_uint8(88, 136, SYSTEM_IMAGE_THRESHOLD);
    ips200_show_string(8, 152, "Ring: WAIT FRAME       ");
    ips200_show_string(8, 168, "Pitch(deg):");
    ips200_show_string(8, 200, "SELECT: CAL IMU");
    ips200_show_string(8, 216, "KEEP CAR STILL");
    ips200_show_string(8, 232, "BACK: MENU");
    ips200_show_string(8, 256, "FPS: --");
    ips200_show_string(8, 280, "VIS:");
    ips200_show_string(96, 280, "ms");
    ips200_show_string(128, 280, "LCD:");
    ips200_show_string(216, 280, "ms");
    system_draw_pitch();
    ips200_set_color(RGB565_RED, RGB565_WHITE);
    ips200_show_string(144, 40, "LEFT");
    ips200_set_color(RGB565_BLUE, RGB565_WHITE);
    ips200_show_string(144, 64, "RIGHT");
    ips200_set_color(RGB565_GREEN, RGB565_WHITE);
    ips200_show_string(144, 88, "CENTER");
    ips200_set_color(RGB565_YELLOW, RGB565_WHITE);
    ips200_show_string(144, 112, "HISTORY");
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    system_image_stats_tick = Motor_GetTicks();
}

/* Image 页按 SELECT 后停车并重做静止校准；不在 Run 中自动标定，避免把行驶转速当作零偏。
 * 校准结束清除旧圆环状态和相机帧，重新等待新图像确认入口；失败时仍可操作菜单和再次重试。 */
static void system_recalibrate_imu(void)
{
    system_pid_config.motor_enabled = 0;
    Motor_SetEnabled(0);
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(8, 152, "Ring: CALIBRATING      ");
    ips200_show_string(8, 184, "IMU CAL: KEEP STILL          ");
    IMU_Init();
    search_init();
    mt9v03x_finish_flag = 0;
    system_draw_image_page();
}

static void system_draw_run(void)
{
#if RUN_DISPLAY_ENABLED
    ips200_clear(RGB565_WHITE);
    ips200_set_color(RGB565_BLUE, RGB565_WHITE);
    ips200_show_string(24, 8, "RUN / PURE PURSUIT");
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(8, 160, system_camera_ready ? "WAIT FRAME" : "CAMERA ERROR");
    ips200_set_color(RGB565_RED, RGB565_WHITE);
    ips200_show_string(8, 184, "LEFT");
    ips200_set_color(RGB565_BLUE, RGB565_WHITE);
    ips200_show_string(80, 184, "RIGHT");
    ips200_set_color(RGB565_GREEN, RGB565_WHITE);
    ips200_show_string(160, 184, "CENTER");
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(8, 208, "Points:");
    ips200_show_string(120, 208, "Cross:");
    ips200_show_string(8, 232, "Speed:");
    ips200_show_string(120, 232, "Set:");
    ips200_show_string(184, 160, "G:");
    ips200_show_uint8(200, 160, ground_calibrated);
    ips200_show_string(8, 256, "Err:");
    ips200_show_string(120, 256, "A/10:");
    ips200_show_string(8, 272, "PWM:");
    ips200_show_string(120, 272, "Kp:");
    ips200_show_string(8, 288, "BACK: STOP / MENU");
#else
    /* Run 默认仅进页清屏；斑马线确认停车时允许显示一次停车提示。 */
    ips200_clear(RGB565_BLACK);
#endif
}

/* Point.x is image ROW, Point.y is COLUMN in the existing alongLine API.
 * Build one near-to-far continuous segment; never join gaps or stale history. */
/* 从当前中线构造可控的近到远连续段，排除历史点、过远起点及列坐标突跳，并选择瞄点；成功返回1。
 * row：扫描行；delta：相邻点列差；target：瞄点索引；gx/gy：标定坐标mm；max_start：相对后轴的最大起点前向距离。 */
static uint8 system_build_road(void)
{
    int16 row, delta, target, gx, gy, max_start;
    system_road_count = 0;
    system_target_index = 0;
    system_path_status = SYSTEM_PATH_LOST;
    if(path_lost) return 0;
    if(!ground_calibrated) { system_path_status = SYSTEM_PATH_CALIB; return 0; }
    /* A normal offset from one visible edge moves its centre across rows.
     * Judge the first usable point in vehicle millimetres instead. */
    for(row = search_ground_near_row(); row >= SEARCH_CALIBRATION_MIN_ROW; --row)
    {
        if(!center_valid[row] || center_quality[row] < 2)
        {
            if(system_road_count) break;
            continue;
        }
        if (!system_road_count) {
            max_start = center_source[row] == CENTER_GROUND && !ground_projection_rejected ?
                RUN_MAX_GROUND_PATH_START_MM : RUN_MAX_PATH_START_MM;
            if (!search_project_point(center_line[row], row, &gx, &gy) ||
                gy - RUN_REAR_AXLE_Y_MM > max_start || gy - RUN_REAR_AXLE_Y_MM <= 0) break;
        }
        if(system_road_count)
        {
            delta = (int16)center_line[row] - system_road_line[system_road_count - 1].y;
            if(delta > RUN_MAX_CENTER_STEP || delta < -RUN_MAX_CENTER_STEP) break;
        }
        system_road_line[system_road_count].x = (uint8)row;
        system_road_line[system_road_count].y = center_line[row];
        ++system_road_count;
    }
    if(system_road_count < RUN_MIN_PATH_POINTS) {
        system_path_status = system_road_count ? SYSTEM_PATH_SHORT : SYSTEM_PATH_NEAR;
        return 0;
    }
    target = Motor_SelectTarget(system_road_line, system_road_count);
    if(target < 0) { system_path_status = SYSTEM_PATH_CALIB; return 0; }
    system_target_index = (uint8)target;
    system_path_status = SYSTEM_PATH_OK;
    return 1;
}

#if RUN_DISPLAY_ENABLED
static void system_draw_run_status(void)
{
    motor_steering_state_t steering;
    Motor_GetSteering(&steering);
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    if(system_zebra_stopped)
        ips200_show_string(8, 160, "STOP: ZEBRA       ");
    else if(!system_camera_ready)
        ips200_show_string(8, 160, "CAMERA ERROR       ");
    else if(!system_run_has_frame)
        ips200_show_string(8, 160, "WAIT FRAME         ");
    else if(system_path_status == SYSTEM_PATH_LOST)
        ips200_show_string(8, 160, Motor_IsEnabled() ? "SLOW: LOST        " : "STOP: LOST        ");
    else if(system_path_status == SYSTEM_PATH_NEAR)
        ips200_show_string(8, 160, Motor_IsEnabled() ? "SLOW: NO NEAR     " : "STOP: NO NEAR     ");
    else if(system_path_status == SYSTEM_PATH_SHORT)
        ips200_show_string(8, 160, Motor_IsEnabled() ? "SLOW: SHORT       " : "STOP: SHORT       ");
    else if(system_path_status == SYSTEM_PATH_CALIB)
        ips200_show_string(8, 160, "STOP: CALIBRATION ");
    else if(!Motor_IsEnabled())
        ips200_show_string(8, 160, "STOP: STALE       ");
    else if(ring_state != RING_STATE_NONE)
    {
        ips200_show_string(8, 160, "RING RIGHT:        ");
        ips200_show_uint8(112, 160, ring_state);
    }
    else
        ips200_show_string(8, 160, "TRACKING           ");
    ips200_show_uint8(72, 208, system_road_count);
    ips200_show_uint8(176, 208, cross_flag);
    ips200_show_int16(64, 232, currentSpeed);
    ips200_show_int16(152, 232, setSpeed);
    ips200_show_int16(40, 256, steering.pixel_error);
    ips200_show_int16(168, 256, steering.angle10);
    ips200_show_int16(40, 272, steering.pwm);
    ips200_show_int16(152, 272, system_pid_config.servo_kp);
}

static void system_draw_run_frame(void)
{
    uint16 x, y;
    ips200_show_gray_image(SYSTEM_RUN_IMAGE_X, SYSTEM_RUN_IMAGE_Y,
        &mt9v03x_image[0][0], SEARCH_IMAGE_W, SEARCH_IMAGE_H,
        SEARCH_IMAGE_W, SEARCH_IMAGE_H, 0);
    system_draw_line_overlay(SYSTEM_RUN_IMAGE_X, SYSTEM_RUN_IMAGE_Y,
        SEARCH_IMAGE_W, SEARCH_IMAGE_H);
    if(system_road_count >= RUN_MIN_PATH_POINTS)
    {
        x = SYSTEM_RUN_IMAGE_X + system_road_line[system_target_index].y;
        y = SYSTEM_RUN_IMAGE_Y + system_road_line[system_target_index].x;
        ips200_draw_point(x, y, RGB565_YELLOW);
        ips200_draw_point(x - 1, y, RGB565_YELLOW);
        ips200_draw_point(x + 1, y, RGB565_YELLOW);
        ips200_draw_point(x, y - 1, RGB565_YELLOW);
        ips200_draw_point(x, y + 1, RGB565_YELLOW);
    }
}

#endif

/* Run每轮处理入口：新图像先识别斑马线及右圆环/十字；斑马线确认后锁定停车，否则通过路径发布转向。
 * 路径无标定则停车，其余无效路径交Motor_RejectPath；最后一次读取像素后释放摄像头帧，几何与控制可和采集并行。
 * 显示开启时now为当前5ms节拍，refresh表示本轮达到LCD刷新间隔。 */
static void system_run_task(void)
{
#if RUN_DISPLAY_ENABLED
    uint16 now;
    uint8 refresh;
    now = Motor_GetTicks();
    refresh = (uint16)(now - system_display_tick) >= RUN_DISPLAY_PERIOD_TICKS;
#endif
    if(system_camera_ready && mt9v03x_finish_flag)
    {
        search_extract_edges_timed(&mt9v03x_image[0][0], Motor_GetTicks());
        if(zebra_detected && !system_zebra_stopped) {
            // 确认当帧立即撤销双轮驱动，先于几何计算和任何新的速度/转向发布。
            system_zebra_stopped = 1;
            Motor_SetEnabled(0);
            // 即使 Run 连续显示关闭，也只在停车事件发生时提示一次。
            ips200_set_color(RGB565_BLACK, RGB565_WHITE);
            ips200_show_string(8, 160, "STOP: ZEBRA       ");
        }
#if RUN_DISPLAY_ENABLED
        if(!refresh && system_run_has_frame) mt9v03x_finish_flag = 0;
#else
        mt9v03x_finish_flag = 0;
#endif
        // 从这里起只读边界/中线数组，相机可以同时采集下一帧。
        search_finish_frame();
        if(system_zebra_stopped)
        {
            // 图像仍可继续处理，但普通路面重新出现也不能自动起步。
            Motor_SetEnabled(0);
        }
        else if(system_build_road())
        {
            alongLine(system_road_line, system_road_count, system_target_index);
            Motor_SetEnabled(1);
        }
        else
        {
            if(system_path_status == SYSTEM_PATH_CALIB) Motor_SetEnabled(0);
            else Motor_RejectPath();
        }
#if RUN_DISPLAY_ENABLED
        if(refresh || !system_run_has_frame)
        {
            system_draw_run_frame();
            mt9v03x_finish_flag = 0;
            refresh = 1;
        }
#endif
        system_run_has_frame = 1;
        // 已提前释放的帧不能再清标志，否则会丢掉计算期间刚完成的新帧。
    }
    system_pid_config.motor_enabled = Motor_IsEnabled();
#if RUN_DISPLAY_ENABLED
    if(refresh)
    {
        system_draw_run_status();
        system_display_tick = Motor_GetTicks();
    }
#endif
}

static void system_open_page(system_page_t page)
{
    system_page = page;
    switch(system_page)
    {
        case SYSTEM_PAGE_IMAGE:
            search_init();
            system_draw_image_page();
            break;

        case SYSTEM_PAGE_SET_PID:
            system_draw_pid();
            break;

        case SYSTEM_PAGE_RUN:
            system_draw_run();
            break;

        case SYSTEM_PAGE_MAIN:
        default:
            system_draw_main();
            break;
    }
}

static int16 *system_selected_pid_value(void)
{
    if(0 == system_pid_item)
    {
        return &system_pid_config.kp;
    }
    if(1 == system_pid_item)
    {
        return &system_pid_config.ki;
    }
    if(2 == system_pid_item)
    {
        return &system_pid_config.kd;
    }
    if(5 == system_pid_item)
    {
        return &system_pid_config.servo_kp;
    }
    return &system_pid_config.motor_speed;
}

static void system_adjust_pid(int8 direction)
{
    int16 *value;
    int32 adjusted;

    if(4 == system_pid_item)
    {
        system_pid_config.motor_enabled = (uint8)!system_pid_config.motor_enabled;
        Motor_SetEnabled(system_pid_config.motor_enabled);
        system_draw_pid();
        return;
    }

    value = system_selected_pid_value();
    adjusted = (int32)(*value) + (int32)direction * SYSTEM_PID_STEP;
    if(5 == system_pid_item)
    {
        adjusted = func_limit_ab(adjusted, 0, MOTOR_SERVO_KP_MAX);
    }
    else if((3 == system_pid_item) && (adjusted > SYSTEM_MOTOR_SPEED_MAX))
    {
        adjusted = SYSTEM_MOTOR_SPEED_MAX;
    }
    else if((3 == system_pid_item) && (adjusted < 0))
    {
        adjusted = 0;
    }
    else if(adjusted > SYSTEM_PID_MAX)
    {
        adjusted = SYSTEM_PID_MAX;
    }
    else if(adjusted < SYSTEM_PID_MIN)
    {
        adjusted = SYSTEM_PID_MIN;
    }
    *value = (int16)adjusted;
    if(3 == system_pid_item)
    {
        Motor_SetStraightSpeed(system_pid_config.motor_speed);
    }
    else if(5 == system_pid_item)
    {
        Motor_SetServoKp(system_pid_config.servo_kp);
    }
    else
    {
        Motor_SetSpeedPID(system_pid_config.kp,
            system_pid_config.ki, system_pid_config.kd);
    }
    system_pid_dirty = 1;
    system_draw_pid();
}

/* 初始化屏幕、摄像头、参数和按键，建立电机控制，静止校准IMU并初始化搜线；返回camera_state摄像头初始化状态。 */
uint8 System_Init(void)
{
    uint8 camera_state;

    ips200_init();
    camera_state = mt9v03x_init();
    system_camera_ready = (0 == camera_state);

    system_pid_load();

    system_key_init(&system_key_prev, SYSTEM_KEY_PREV);
    system_key_init(&system_key_next, SYSTEM_KEY_NEXT);
    system_key_init(&system_key_select, SYSTEM_KEY_SELECT);
    system_key_init(&system_key_back, SYSTEM_KEY_BACK);

    Motor_SetSpeedPID(system_pid_config.kp,
        system_pid_config.ki, system_pid_config.kd);
    Motor_SetStraightSpeed(system_pid_config.motor_speed);
    system_pid_config.motor_enabled = 0;
    Motor_SetEnabled(0);
    Motor_Init(system_pid_config.servo_kp);
    ips200_clear(RGB565_WHITE);
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(8, 72, "IMU CAL: KEEP STILL");
    IMU_Init();
    search_init();

    system_open_page(SYSTEM_PAGE_MAIN);
    return camera_state;
}

/* 开始一次Run：启用电机帧看门狗模式、重置视觉状态并清空旧帧标记，随后等待新图像给出可控路径。 */
static void system_enter_run(void)
{
    Motor_SetRunMode(1);
    system_zebra_stopped = 0;
    system_pid_config.motor_enabled = 0;
    search_init();
    system_road_count = 0;
    system_run_has_frame = 0;
#if RUN_DISPLAY_ENABLED
    system_display_tick = Motor_GetTicks();
#endif
    mt9v03x_finish_flag = 0;
    system_open_page(SYSTEM_PAGE_RUN);
}

/* 退出Run并停止电机，返回主菜单；本函数不清空当前视觉数组，再次Run时由search_init重置。 */
static void system_leave_run(void)
{
    Motor_SetRunMode(0);
    system_pid_config.motor_enabled = 0;
    if(SYSTEM_PAGE_RUN == system_page) system_open_page(SYSTEM_PAGE_MAIN);
}

/* 前台周期任务：读取prev/next/select/back四个按键事件，分发菜单、图像、参数和Run页面逻辑；返回键在Run中优先停车。 */
void System_Task(void)
{
    uint16 key_tick; /* 四个按键共用一次原子时钟快照，跨 uint16 回绕仍按经过时间消抖。 */
    uint8 prev_pressed;
    uint8 next_pressed;
    uint8 select_pressed;
    uint8 back_pressed;

    key_tick = Motor_GetTicks();
    prev_pressed = system_key_pressed(&system_key_prev, key_tick);
    next_pressed = system_key_pressed(&system_key_next, key_tick);
    select_pressed = system_key_pressed(&system_key_select, key_tick);
    back_pressed = system_key_pressed(&system_key_back, key_tick);

    if(back_pressed && (SYSTEM_PAGE_MAIN != system_page))
    {
        if(SYSTEM_PAGE_RUN == system_page)
        {
            system_leave_run();
            return;
        }
        if(SYSTEM_PAGE_SET_PID == system_page)
        {
            system_pid_config.motor_enabled = 0;
            Motor_SetEnabled(0);
            if(system_pid_dirty)
            {
                system_pid_save();
            }
        }
        system_open_page(SYSTEM_PAGE_MAIN);
        return;
    }

    switch(system_page)
    {
        case SYSTEM_PAGE_MAIN:
            if(prev_pressed)
            {
                system_main_item = (0 == system_main_item) ?
                    (SYSTEM_MAIN_ITEM_COUNT - 1U) : (system_main_item - 1U);
                system_draw_main();
            }
            else if(next_pressed)
            {
                system_main_item = (system_main_item + 1) % SYSTEM_MAIN_ITEM_COUNT;
                system_draw_main();
            }
            else if(select_pressed)
            {
                if(0 == system_main_item)
                {
                    system_open_page(SYSTEM_PAGE_IMAGE);
                }
                else if(1 == system_main_item)
                {
                    system_open_page(SYSTEM_PAGE_SET_PID);
                }
                else
                {
                    system_enter_run();
                }
            }
            break;

        case SYSTEM_PAGE_IMAGE:
            if(select_pressed)
            {
                system_recalibrate_imu();
                break;
            }
            if(!system_pitch_display_valid || imu_ready != system_pitch_display_ready ||
               imu_cal_status != system_imu_display_status || imu_cal_failure != system_imu_display_failure ||
               (uint16)(key_tick - system_pitch_display_tick) >= SYSTEM_PITCH_DISPLAY_PERIOD_TICKS)
                system_draw_pitch();
            if(system_camera_ready && mt9v03x_finish_flag)
            {
                system_draw_image_frame();
            }
            /* 无新帧时也刷新统计，摄像头停帧会显示 FPS=0，不保留上次帧率。 */
            system_draw_image_stats();
            break;

        case SYSTEM_PAGE_SET_PID:
            if(prev_pressed)
            {
                system_adjust_pid(1);
            }
            else if(next_pressed)
            {
                system_adjust_pid(-1);
            }
            else if(select_pressed)
            {
                system_pid_item = (system_pid_item + 1) % SYSTEM_PID_ITEM_COUNT;
                system_draw_pid();
            }
            break;

        case SYSTEM_PAGE_RUN:
            system_run_task();
            break;

        default:
            break;
    }

    /* 所有页面都直接返回主循环；按键消抖、显示限频使用真实节拍，不再每轮等待 10ms。
     * 电机与 IMU 仍由各自的 5ms 中断驱动，空循环不喂图像看门狗。 */
}
