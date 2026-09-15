#include "System.h"
#include "Motor.h"

#define SYSTEM_KEY_PREV              (IO_P32)
#define SYSTEM_KEY_NEXT              (IO_PB3)
#define SYSTEM_KEY_SELECT            (IO_PB4)
#define SYSTEM_KEY_BACK              (IO_PB2)

#define SYSTEM_KEY_PRESSED           (GPIO_LOW)
#define SYSTEM_KEY_DEBOUNCE_TICKS    (2U)
#define SYSTEM_TASK_PERIOD_MS        (10U)

#define SYSTEM_MAIN_ITEM_COUNT       (3U)
#define SYSTEM_PID_ITEM_COUNT        (3U)
#define SYSTEM_PID_STEP              (5)
#define SYSTEM_PID_MIN               (-30000L)
#define SYSTEM_PID_MAX               (30000L)
#define SYSTEM_IMAGE_THRESHOLD       (128U)

// EEPROM page 0 (512 bytes) is reserved for the menu settings.
#define SYSTEM_PID_EEPROM_ADDRESS    (0x0000UL)
#define SYSTEM_PID_STORAGE_SIZE      (10U)
#define SYSTEM_PID_MAGIC_0           (0x53U)
#define SYSTEM_PID_MAGIC_1           (0x50U)
#define SYSTEM_PID_STORAGE_VERSION   (1U)

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

system_pid_config_t system_pid_config = {0, 0, 0};
uint8 system_run_requested = 0;

static system_page_t system_page = SYSTEM_PAGE_MAIN;
static uint8 system_main_item = 0;
static uint8 system_pid_item = 0;
static uint8 system_camera_ready = 0;
static uint8 system_pid_dirty = 0;

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

    iap_init();
    iap_read_buff(SYSTEM_PID_EEPROM_ADDRESS, storage, SYSTEM_PID_STORAGE_SIZE);
    iap_idle();

    if((SYSTEM_PID_MAGIC_0 != storage[0]) ||
       (SYSTEM_PID_MAGIC_1 != storage[1]) ||
       (SYSTEM_PID_STORAGE_VERSION != storage[2]) ||
       (system_pid_crc(storage, SYSTEM_PID_STORAGE_SIZE - 1) != storage[9]))
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
    storage[9] = system_pid_crc(storage, SYSTEM_PID_STORAGE_SIZE - 1);

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
    ips200_show_string(8, 48, (0 == system_pid_item) ? "> Kp:" : "  Kp:");
    ips200_show_int16(64, 48, system_pid_config.kp);
    ips200_show_string(8, 80, (1 == system_pid_item) ? "> Ki:" : "  Ki:");
    ips200_show_int16(64, 80, system_pid_config.ki);
    ips200_show_string(8, 112, (2 == system_pid_item) ? "> Kd:" : "  Kd:");
    ips200_show_int16(64, 112, system_pid_config.kd);

    ips200_set_color(RGB565_GREEN, RGB565_WHITE);
    ips200_show_string(4, 168, "UP/DOWN: +/-5");
    ips200_show_string(4, 192, "SELECT: NEXT");
    ips200_show_string(4, 216, "BACK: SAVE");
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
}

static void system_draw_run(void)
{
    ips200_clear(RGB565_WHITE);
    ips200_set_color(RGB565_BLUE, RGB565_WHITE);
    ips200_show_string(24, 24, "RUN ENTRY");
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(8, 88, "RUN REQUESTED");
    ips200_show_string(8, 120, "TODO: CAR LOOP");
    ips200_set_color(RGB565_GREEN, RGB565_WHITE);
    ips200_show_string(8, 200, "BACK: MENU");
}

static void system_open_page(system_page_t page)
{
    system_page = page;
    switch(system_page)
    {
        case SYSTEM_PAGE_IMAGE:
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
    return &system_pid_config.kd;
}

static void system_adjust_pid(int8 direction)
{
    int16 *value;
    int32 adjusted;

    value = system_selected_pid_value();
    adjusted = (int32)(*value) + (int32)direction * SYSTEM_PID_STEP;
    if(adjusted > SYSTEM_PID_MAX)
    {
        adjusted = SYSTEM_PID_MAX;
    }
    else if(adjusted < SYSTEM_PID_MIN)
    {
        adjusted = SYSTEM_PID_MIN;
    }
    *value = (int16)adjusted;
    Motor_SetSpeedPID(system_pid_config.kp,
        system_pid_config.ki, system_pid_config.kd);
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

    system_open_page(SYSTEM_PAGE_MAIN);
    return camera_state;
}

void System_Run_Entry(void)
{
    // The real car initialization/main loop will be connected here later.
    system_run_requested = 1;
    system_open_page(SYSTEM_PAGE_RUN);
}

void System_Clear_Run_Request(void)
{
    system_run_requested = 0;
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
        if((SYSTEM_PAGE_SET_PID == system_page) && system_pid_dirty)
        {
            system_pid_save();
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
                    (SYSTEM_MAIN_ITEM_COUNT - 1) : (system_main_item - 1);
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
            if(system_camera_ready && mt9v03x_finish_flag)
            {
                ips200_show_gray_image(0, 40,
                    &mt9v03x_image[0][0], MT9V03X_W, MT9V03X_H,
                    135, 86, SYSTEM_IMAGE_THRESHOLD);
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
        default:
            break;
    }

    system_delay_ms(SYSTEM_TASK_PERIOD_MS);
}
