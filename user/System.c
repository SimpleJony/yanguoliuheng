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
#define SYSTEM_KEY_DEBOUNCE_TICKS    (2U)
#define SYSTEM_TASK_PERIOD_MS        (10U)

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
    uint8 debounce_ticks;
} system_key_t;

system_pid_config_t system_pid_config = {3000, 50, 50, RUN_DEFAULT_SPEED, 0, RUN_SERVO_GAIN};
uint8 system_run_requested = 0;

static system_page_t system_page = SYSTEM_PAGE_MAIN;
static uint8 system_main_item = 0;
static uint8 system_pid_item = 0;
static uint8 system_camera_ready = 0;
static uint8 system_pid_dirty = 0;
static Point system_road_line[SEARCH_IMAGE_H];
static uint8 system_road_count = 0;
static uint8 system_target_index = 0;
static uint8 system_run_has_frame = 0;
#if RUN_DISPLAY_ENABLED
static uint16 system_display_tick = 0;
#endif
static uint16 system_pitch_display_tick;
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
    key->debounce_ticks = 0;
}

// Returns 1 once for every debounced press. Holding a key does not repeat.
static uint8 system_key_pressed(system_key_t *key)
{
    uint8 level;

    level = gpio_get_level(key->pin);
    if(level != key->sampled_level)
    {
        key->sampled_level = level;
        key->debounce_ticks = 0;
    }
    else if(key->debounce_ticks < SYSTEM_KEY_DEBOUNCE_TICKS)
    {
        key->debounce_ticks++;
    }
    else if(key->stable_level != level)
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

static void system_draw_image_frame(void)
{
    search_process(&mt9v03x_image[0][0]);
    ips200_show_gray_image(SYSTEM_IMAGE_X, SYSTEM_IMAGE_Y,
        &mt9v03x_image[0][0], SEARCH_IMAGE_W, SEARCH_IMAGE_H,
        SYSTEM_IMAGE_WIDTH, SYSTEM_IMAGE_HEIGHT, SYSTEM_IMAGE_THRESHOLD);
    system_draw_line_overlay(SYSTEM_IMAGE_X, SYSTEM_IMAGE_Y,
        SYSTEM_IMAGE_WIDTH, SYSTEM_IMAGE_HEIGHT);
}

/* Image-only refresh; snapshot the ISR-owned 16-bit value atomically. */
static void system_draw_pitch(void)
{
    int16 value;
    uint8 ready, cal_status;
    bit irq_state;
    irq_state = EA;
    EA = 0;
    value = pitch;
    ready = imu_ready;
    cal_status = imu_cal_status;
    EA = irq_state;
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(8, 168, "Pitch(deg):");
    ips200_show_string(104, 168, "            ");
    if(ready) ips200_show_int16(104, 168, value);
    else ips200_show_string(104, 168, "--");
    ips200_show_string(8, 184, cal_status == IMU_STATUS_READY ? "IMU CAL OK          " :
        (cal_status == IMU_STATUS_CAL_FAILED ? "IMU CAL FAIL:RESTART" :
        (cal_status == IMU_STATUS_CALIBRATING ? "IMU CAL: KEEP STILL " : "IMU MISSING         ")));
    system_pitch_display_tick = Motor_GetTicks();
}

static void system_draw_image_page(void)
{
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
    ips200_show_string(8, 200, "BACK: MENU");
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
    /* One-time blank before tracking starts; no LCD traffic during Run. */
    ips200_clear(RGB565_BLACK);
#endif
}

/* Point.x is image ROW, Point.y is COLUMN in the existing alongLine API.
 * Build one near-to-far continuous segment; never join gaps or stale history. */
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
    if(!system_camera_ready)
        ips200_show_string(8, 160, "CAMERA ERROR       ");
    else if(!system_run_has_frame)
        ips200_show_string(8, 160, "WAIT FRAME         ");
    else if(system_path_status == SYSTEM_PATH_LOST)
        ips200_show_string(8, 160, Motor_IsEnabled() ? (RUN_FIXED_SPEED_MODE ? "RUN: LOST         " : "SLOW: LOST        ") : "STOP: LOST        ");
    else if(system_path_status == SYSTEM_PATH_NEAR)
        ips200_show_string(8, 160, Motor_IsEnabled() ? (RUN_FIXED_SPEED_MODE ? "RUN: NO NEAR      " : "SLOW: NO NEAR     ") : "STOP: NO NEAR     ");
    else if(system_path_status == SYSTEM_PATH_SHORT)
        ips200_show_string(8, 160, Motor_IsEnabled() ? (RUN_FIXED_SPEED_MODE ? "RUN: SHORT        " : "SLOW: SHORT       ") : "STOP: SHORT       ");
    else if(system_path_status == SYSTEM_PATH_CALIB)
        ips200_show_string(8, 160, "STOP: CALIBRATION ");
    else if(!Motor_IsEnabled())
        ips200_show_string(8, 160, "STOP: STALE       ");
    else if(!system_pid_config.motor_speed)
        ips200_show_string(8, 160, "STOP: SPEED = 0    ");
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
        search_process(&mt9v03x_image[0][0]);
        if(system_build_road())
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
            refresh = 1;
        }
#endif
        system_run_has_frame = 1;
        // Release the camera frame only after processing and optional display.
        mt9v03x_finish_flag = 0;
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
        Motor_SetSpeedLimit(system_pid_config.motor_speed);
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
    Motor_SetSpeedLimit(system_pid_config.motor_speed);
    system_pid_config.motor_enabled = 0;
    Motor_SetEnabled(0);
    Motor_Init(system_pid_config.servo_kp, RUN_LOOKAHEAD_ROWS,
        RUN_CURVE_COEFFICIENT, system_pid_config.motor_speed);
    ips200_clear(RGB565_WHITE);
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(8, 72, "IMU CAL: KEEP STILL");
    IMU_Init();
    search_init();

    system_open_page(SYSTEM_PAGE_MAIN);
    return camera_state;
}

void System_Run_Entry(void)
{
    Motor_SetRunMode(1);
    system_pid_config.motor_enabled = 0;
    search_init();
    system_road_count = 0;
    system_run_has_frame = 0;
#if RUN_DISPLAY_ENABLED
    system_display_tick = Motor_GetTicks();
#endif
    mt9v03x_finish_flag = 0;
    system_run_requested = 1;
    system_open_page(SYSTEM_PAGE_RUN);
}

void System_Clear_Run_Request(void)
{
    Motor_SetRunMode(0);
    system_pid_config.motor_enabled = 0;
    system_run_requested = 0;
    if(SYSTEM_PAGE_RUN == system_page) system_open_page(SYSTEM_PAGE_MAIN);
}

void System_Task(void)
{
    uint8 prev_pressed;
    uint8 next_pressed;
    uint8 select_pressed;
    uint8 back_pressed;

    prev_pressed = system_key_pressed(&system_key_prev);
    next_pressed = system_key_pressed(&system_key_next);
    select_pressed = system_key_pressed(&system_key_select);
    back_pressed = system_key_pressed(&system_key_back);

    if(back_pressed && (SYSTEM_PAGE_MAIN != system_page))
    {
        if(SYSTEM_PAGE_RUN == system_page)
        {
            System_Clear_Run_Request();
            system_delay_ms(SYSTEM_TASK_PERIOD_MS);
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
        system_delay_ms(SYSTEM_TASK_PERIOD_MS);
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
                    System_Run_Entry();
                }
            }
            break;

        case SYSTEM_PAGE_IMAGE:
            if((uint16)(Motor_GetTicks() - system_pitch_display_tick) >= 40U)
                system_draw_pitch();
            if(system_camera_ready && mt9v03x_finish_flag)
            {
                system_draw_image_frame();
                /* Keep this frame held until both image and overlays finish. */
                mt9v03x_finish_flag = 0;
            }
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
            if(system_run_requested) system_run_task();
            break;

        default:
            break;
    }

    system_delay_ms(SYSTEM_TASK_PERIOD_MS);
}
