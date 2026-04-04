#include "All_Init.h"
#include "PIT_Demo.h"
#include "drone_car_link.h"
#include "flight_control.h"
#include "Kalman Filter/height_manager.h"
#include "icm/icm.h"
#include "menu/parameter_flash.h"
#include "menu/menu.h"
#include "menu/key.h"
#include "pmw/pmw_flow.h"

bool Send_state = 0;

#define KEY_VOLT_PIT               PIT_CH0
#define CONTROL_LOOP_PIT           PIT_CH1
#define MOTOR_OUTPUT_PIT           PIT_CH2
#define TOF_UPDATE_PIT             PIT_CH11
#define PMW3901_UPDATE_PIT         PIT_CH12

static void init_display_and_basic_peripherals(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_init(IPS200_TYPE_SPI);

    adc_init(BATTERY_ADC_CHANNEL, ADC_12BIT);
    adc_init(ADC0_CH00_P06_0, ADC_12BIT);
    adc_init(ADC0_CH01_P06_1, ADC_12BIT);
    dl1b_init();
}

static void init_default_pid_parameters(void)
{
    PID_Init(&gyro_pid_roll, 1.60f, 0.0f, 0.120f, 600.0f, 100.0f, 10.0f, 0.8f);
    PID_Init(&gyro_pid_pitch, 1.60f, 0.0f, 0.120f, 600.0f, 100.0f, 10.0f, 0.8f);
    PID_Init(&gyro_pid_yaw, 1.0f, 0.0f, 0.0f, 400.0f, 10.0f, 0.0f, 0.0f);

    PID_Init(&HeightPID, 5.0f, 0.0f, 0.0f, 200.0f, 50.0f, 10.0f, 0.0f);

    PID_Init(&angle_pid_roll, 3.50f, 0.0f, 0.0f, 200.0f, 30.0f, 0.0f, 0.0f);
    PID_Init(&angle_pid_pitch, 3.50f, 0.0f, 0.0f, 200.0f, 30.0f, 0.0f, 0.0f);
    PID_Init(&angle_pid_yaw, 3.0f, 0.0f, 0.0f, 100.0f, 20.0f, 0.0f, 0.0f);

    PID_Init(&joystick_pid_roll, 1.0f, 0.0f, 0.0f, 20.0f, 10.0f, 0.0f, 0.0f);
    PID_Init(&joystick_pid_pitch, 1.0f, 0.0f, 0.0f, 20.0f, 10.0f, 0.0f, 0.0f);
}

static void init_runtime_modules(void)
{
    my_key_init();
    Parameter_System_Init();
    menu_open_init();

    IMU_calibrate_flag = 1;
    ICM_Calculate_Init();
    HeightManager_Init(100.0f);
    PMW_Flow_Init();
    Flight_Control_Init();
}

static void init_periodic_interrupts(void)
{
    pit_us_init(KEY_VOLT_PIT, 5000);      // 5ms
    pit_us_init(CONTROL_LOOP_PIT, 250);   // 4kHz control
    pit_us_init(MOTOR_OUTPUT_PIT, 500);   // 2kHz PWM update
    pit_us_init(PMW3901_UPDATE_PIT, 20000);  // 20ms PMW3901 update
    pit_ms_init(TOF_UPDATE_PIT, 25);      // 40Hz TOF update
    pit_ms_init(DRONE_CAR_LINK_SERVICE_PIT, 10);
}

void Device_Init(void)
{
    init_display_and_basic_peripherals();
    init_default_pid_parameters();
    init_runtime_modules();
    init_periodic_interrupts();
}
