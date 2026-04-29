#include "common/All_Init.h"
#include "common/project_headfile.h"

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
    dl1b_init();
}

static void init_default_pid_parameters(void)
{
    PID_Init(&gyro_pid_roll, 6.50f, 0.0f, 0.00f, 2000.0f, 100.0f, 10.0f, 0.8f);
    PID_Init(&gyro_pid_pitch, 6.50f, 0.0f, 0.00f, 2000.0f, 100.0f, 10.0f, 0.8f);
    PID_Init(&gyro_pid_yaw, 5.5f, 0.0f, 0.0f, 400.0f, 10.0f, 0.0f, 0.0f);

    /* HeightPID @ 100Hz:
     *   Kp=125, Ki=0, Kd=0, output_limit=300, integral_limit=50, IntegralThreshold=10000, alpha=0
     *
     * 每 tick 直接 `integral += error`，不分区限速。反 windup 只靠 integral_limit=50 截断。
     * IntegralThreshold 设成 10000 作为"永远触发 factor=1 分支"的哨兵值（实际误差最多 200cm
     * 左右，远小于阈值），等效关闭了 pid_controller.c 里的分区增速逻辑。
     *
     * Kp 项：|e|≈2.4cm 就单独顶到 300（125×2.4≈300），更靠近目标高度时即饱和，响应更激进。
     * Ki=0 时积分项不参与，integral_limit 仅作为兜底防止后续打开积分时窜飞。 */
    PID_Init(&HeightPID, 150.0f, 0.0f, 0.0f, 300.0f, 50.0f, 10000.0f, 0.0f);

    PID_Init(&angle_pid_roll,  2.67f, 0.0001f, 0.0f, 300.0f, 60000.0f, 10000.0f, 0.0f);
    PID_Init(&angle_pid_pitch, 2.67f, 0.0001f, 0.0f, 300.0f, 60000.0f, 10000.0f, 0.0f);
    PID_Init(&angle_pid_yaw, 1.0f, 0.0f, 0.0f, 100.0f, 20.0f, 0.0f, 0.0f);

    PID_Init(&joystick_pid_roll, 5.5f, 0.0f, 0.0f, 20.0f, 10.0f, 0.0f, 0.0f);
    PID_Init(&joystick_pid_pitch, 5.5f, 0.0f, 0.0f, 20.0f, 10.0f, 0.0f, 0.0f);
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
    pit_us_init(TOF_UPDATE_PIT, 10000);   // 100Hz TOF poll (sensor 出数率由 dl1b 内部配置决定；提高轮询率只降低消费延迟)
}

void Device_Init(void)
{
    init_display_and_basic_peripherals();
    init_default_pid_parameters();
    init_runtime_modules();
    init_periodic_interrupts();
}
