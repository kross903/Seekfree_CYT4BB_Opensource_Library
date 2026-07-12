#include "common/All_Init.h"
#include "common/project_headfile.h"

bool Send_state = 0;

#define KEY_VOLT_PIT               PIT_CH0
#define CONTROL_LOOP_PIT           PIT_CH1
#define MOTOR_OUTPUT_PIT           PIT_CH2
#define TOF_UPDATE_PIT             PIT_CH11
#define FLOW_UPDATE_PIT            PIT_CH12

/* 实机确认 V 型标志世界朝向和摄像头绕 Z 轴安装偏角后再改为 1。 */
#define VISION_YAW_FUSION_ENABLE           (0U)
#define VISION_MARKER_WORLD_HEADING_DEG    (0.0f)
#define VISION_CAMERA_YAW_OFFSET_DEG       (0.0f)

static void init_optical_flow_sensor(void)
{
#if FLOW_SENSOR_SELECTED == FLOW_SENSOR_UPFLOW302
    upflow302_receive_init();
#endif
}

static void init_display_and_basic_peripherals(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_init(IPS200_TYPE_SPI);

    adc_init(BATTERY_ADC_CHANNEL, ADC_12BIT);
    dl1b_init();
}

static void init_default_pid_parameters(void)
{

    PID_Init(&gyro_pid_roll,
             2.0f, 0.0f, 0.2f,
             2000.0f, 100.0f, 10.0f, 0.8f);
    PID_Init(&gyro_pid_pitch,
             2.0f, 0.0f, 0.4f,
             2000.0f, 100.0f, 10.0f, 0.8f);
    PID_Init(&gyro_pid_yaw,
             3.0f, 0.0f, 0.0f,
             400.0f, 10.0f, 0.0f, 0.0f);
    PID_Init(&HeightPID, 2.0f, 0.0001f, 0.0f, 80.0f, 30.0f, 10000.0f, 0.3f);
    PID_Init(&HeightVelPID, 1.0f, 0.0001f, 0.0f, 80.0f, 20.0f, 10000.0f, 0.3f);
    PID_Init(&angle_pid_roll,  1.2f, 0.0001f, 0.0f, 300.0f, 60000.0f, 10000.0f, 0.30f);
    PID_Init(&angle_pid_pitch, 1.2f, 0.0001f, 0.0f, 300.0f, 60000.0f, 10000.0f, 0.30f);
    PID_Init(&angle_pid_yaw, 2.0f, 0.0f, 0.0f, 100.0f, 20.0f, 0.0f, 0.0f);
    PID_Init(&vel_pid_x, 0.50f, 0.000f, 0.0f, 120.0f, 30.0f, 10000.0f, 0.8f);
    PID_Init(&vel_pid_y, 0.50f, 0.000f, 0.0f, 120.0f, 30.0f, 10000.0f, 0.8f);
    PID_Init(&pos_pid_x, 0.0f, 0.00f, 0.0f, 100.0f, 30.0f, 10000.0f, 1.0f);
    PID_Init(&pos_pid_y, 0.0f, 0.00f, 0.0f, 100.0f, 30.0f, 10000.0f, 1.0f);
}

    /*
    static void init_default_pid_parameters(void)
{

    PID_Init(&gyro_pid_roll,
             6.0f, 0.0f, 0.2f,
             2000.0f, 100.0f, 10.0f, 0.8f);
    PID_Init(&gyro_pid_pitch,
             6.5f, 0.0f, 0.4f,
             2000.0f, 100.0f, 10.0f, 0.8f);
    PID_Init(&gyro_pid_yaw,
             6.5f, 0.0f, 0.0f,
             400.0f, 10.0f, 0.0f, 0.0f);
    PID_Init(&HeightPID, 2.0f, 0.0001f, 0.0f, 80.0f, 30.0f, 10000.0f, 0.3f);
    PID_Init(&HeightVelPID, 1.0f, 0.0001f, 0.0f, 80.0f, 20.0f, 10000.0f, 0.3f);
    PID_Init(&angle_pid_roll,  5.5f, 0.0001f, 0.0f, 300.0f, 60000.0f, 10000.0f, 0.30f);
    PID_Init(&angle_pid_pitch, 4.5f, 0.0001f, 0.0f, 300.0f, 60000.0f, 10000.0f, 0.30f);
    PID_Init(&angle_pid_yaw, 5.0f, 0.0f, 0.0f, 100.0f, 20.0f, 0.0f, 0.0f);
    PID_Init(&vel_pid_x, 0.8f, 0.000f, 0.0f, 120.0f, 30.0f, 10000.0f, 0.8f);
    PID_Init(&vel_pid_y, 0.8f, 0.000f, 0.0f, 120.0f, 30.0f, 10000.0f, 0.8f);
    PID_Init(&pos_pid_x, 1.0f, 0.00f, 0.0f, 100.0f, 30.0f, 10000.0f, 1.0f);
    PID_Init(&pos_pid_y, 1.0f, 0.00f, 0.0f, 100.0f, 30.0f, 10000.0f, 1.0f);
}
     * 光流速度闭环 PID @ 4kHz 控制环。
      * 反馈: 光流 + ICM 融合后的机体水平速度 velocity_x/y_fused_cms (cm/s)。
     * 输出: 水平加速度指令 ax/ay_cmd (cm/s^2)，再换算成姿态修正角。
     * - vel_pid_x: 前向速度 (vx) -> ax_cmd -> pitch 修正
     * - vel_pid_y: 左向速度 (vy) -> ay_cmd -> roll 修正
     */
static void init_runtime_modules(void)
{
    my_key_init();
    menu_open_init();

    IMU_calibrate_flag = 1;
    ICM_Calculate_Init();
    ICM_VisionYaw_Configure(VISION_YAW_FUSION_ENABLE,
                            VISION_MARKER_WORLD_HEADING_DEG,
                            VISION_CAMERA_YAW_OFFSET_DEG);
    HeightManager_Init(100.0f);
    PMW_Flow_Init();
    init_optical_flow_sensor();
    Flight_Control_Init();
}

static void init_periodic_interrupts(void)
{
    pit_us_init(KEY_VOLT_PIT, 5000);      // 5ms
    pit_us_init(CONTROL_LOOP_PIT, 250);   // 4kHz control
    pit_us_init(MOTOR_OUTPUT_PIT, 500);   // 2kHz PWM update
    pit_us_init(FLOW_UPDATE_PIT, FLOW_SENSOR_UPDATE_US);
    pit_us_init(TOF_UPDATE_PIT, 10000);   // 100Hz TOF poll (sensor 出数率由 dl1b 内部配置决定；提高轮询率只降低消费延迟)
}

void Device_Init(void)
{
    init_display_and_basic_peripherals();
    init_default_pid_parameters();
    init_runtime_modules();
    init_periodic_interrupts();
}
