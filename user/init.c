#include "init.h"
#include "servo.h"
#include "search_line.h"
#include "image_transfer.h"
#include "accelerator.h"

void initAllModules(){
    clock_init(SYSTEM_CLOCK_96M);
    debug_init();
    Motor_Init();
    Servo_Init();
    Encoder_Init();
    if (Accelerator_Init())
    {
        printf("\r\n imu660rb init/calibration failed, yaw feedback disabled. \r\n");
    }
    Camera_Init();
    SeekfreeAssistant_Init();
#if (SEEKFREE_ASSISTANT_TUNE_TARGET == ASSISTANT_TUNE_NONE)
    SeekfreeAssistantCamera_Init();
#endif
    //Screen_Init();
}

void Motor_Init(){
    gpio_init(DIR_1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_1, 17000, 0);

    gpio_init(DIR_2, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_2, 17000, 0);

    gpio_init(DIR_3, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_3, 17000, 0);

    gpio_init(DIR_4, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_4, 17000, 0);
}

void Encoder_Init(){
    encoder_quad_init(SPEED_ENCODER1, SPEED_ENCODER1_PULSE_PIN, SPEED_ENCODER1_DIR_PIN);
    encoder_quad_init(SPEED_ENCODER2, SPEED_ENCODER2_PULSE_PIN, SPEED_ENCODER2_DIR_PIN);
}

void Camera_Init(){
    if(mt9v03x_init())
    {
        printf("\r\n mt9v03x init failed. \r\n");
    }
}

void Servo_Init(){
    pwm_init(SERVO_PWM, SERVO_PWM_FREQ, 0);
    Servo_Reset();
}

void SeekfreeAssistant_Init(){
    seekfree_assistant_init();
#if SEEKFREE_ASSISTANT_USE_WIFI_SPI
    if (0 == WifiSPI_Init())
    {
        seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
        return;
    }

    printf("\r\n wifi spi init failed, fallback to debug uart. \r\n");
#endif
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
}

void SeekfreeAssistantCamera_Init(){
    ImageTransfer_Init();
}

void Screen_Init(){
    ips200_set_dir(IPS200_PORTAIT);
    ips200_init();
}

uint8 WifiSPI_Init(){
    uint8 retry = 0;
    char *wifi_password = NULL;

    if ('\0' != WIFI_PASSWORD[0])
    {
        wifi_password = WIFI_PASSWORD;
    }

    for (retry = 0; retry < WIFI_CONNECT_RETRY_MAX; retry++)
    {
        if (0 == wifi_spi_init(WIFI_SSID, wifi_password))
        {
            break;
        }

        printf("\r\n connect wifi failed, retry %d/%d. \r\n",
               (int)(retry + 1),
               (int)WIFI_CONNECT_RETRY_MAX);
        system_delay_ms(200);
    }

    if (WIFI_CONNECT_RETRY_MAX <= retry)
    {
        return 1;
    }

    if (wifi_spi_socket_connect(WIFI_TRANSPORT_TYPE,
                                WIFI_TARGET_IP,
                                WIFI_TARGET_PORT,
                                WIFI_LOCAL_PORT))
    {
        printf("\r\n connect socket failed. \r\n");
        return 1;
    }

    printf("\r\n wifi spi ready: %s %s:%s local %s. \r\n",
           WIFI_TRANSPORT_TYPE,
           WIFI_TARGET_IP,
           WIFI_TARGET_PORT,
           wifi_spi_ip_addr_port);

    return 0;
}
