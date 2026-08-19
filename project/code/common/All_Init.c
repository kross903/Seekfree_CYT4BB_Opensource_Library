#include "common/All_Init.h"
#include "common/project_headfile.h"
#include "control/dual_tof_manager.h"

bool Send_state = 0;

#define KEY_VOLT_PIT               PIT_CH0
#define CONTROL_LOOP_PIT           PIT_CH1
#define TOF_UPDATE_PIT             PIT_CH11
#define FLOW_UPDATE_PIT            PIT_CH12

#define BOARD_BUZZER_PIN           P19_4
#define BOARD_BUZZER_PHASE_TICKS   20U   /* 20 x 5ms = 100ms */
#define LEGACY_FAST_PID_DT_S       0.00025f
#define HEIGHT_PID_DT_S            0.010f
#define RATE_PID_RUNTIME_DT_S       0.001f
#define ANGLE_PID_RUNTIME_DT_S      0.002f
#define VELOCITY_PID_RUNTIME_DT_S   0.004f

typedef enum
{
    BOARD_BUZZER_IDLE = 0,
    BOARD_BUZZER_FIRST_ON,
    BOARD_BUZZER_GAP,
    BOARD_BUZZER_SECOND_ON
} BoardBuzzerState_t;

static volatile uint8 s_board_buzzer_state = BOARD_BUZZER_IDLE;
static volatile uint8 s_board_buzzer_ticks = 0U;

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
    gpio_init(BOARD_BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    ips200_set_dir(IPS200_PORTAIT);
    ips200_init(IPS200_TYPE_SPI);

    adc_init(BATTERY_ADC_CHANNEL, ADC_12BIT);
    dl1b_init();
    DualTof_Init();
}

void BoardBuzzer_RequestDoubleBeep(void)
{
    uint32 irq_state = __get_PRIMASK();

    __disable_irq();
    gpio_set_level(BOARD_BUZZER_PIN, GPIO_HIGH);
    s_board_buzzer_state = BOARD_BUZZER_FIRST_ON;
    s_board_buzzer_ticks = BOARD_BUZZER_PHASE_TICKS;
    if(irq_state == 0U)
    {
        __enable_irq();
    }
}

void BoardBuzzer_Task5ms(void)
{
    if(s_board_buzzer_state == BOARD_BUZZER_IDLE)
    {
        return;
    }

    if(s_board_buzzer_ticks > 1U)
    {
        s_board_buzzer_ticks--;
        return;
    }

    switch((BoardBuzzerState_t)s_board_buzzer_state)
    {
        case BOARD_BUZZER_FIRST_ON:
            gpio_set_level(BOARD_BUZZER_PIN, GPIO_LOW);
            s_board_buzzer_state = BOARD_BUZZER_GAP;
            s_board_buzzer_ticks = BOARD_BUZZER_PHASE_TICKS;
            break;

        case BOARD_BUZZER_GAP:
            gpio_set_level(BOARD_BUZZER_PIN, GPIO_HIGH);
            s_board_buzzer_state = BOARD_BUZZER_SECOND_ON;
            s_board_buzzer_ticks = BOARD_BUZZER_PHASE_TICKS;
            break;

        case BOARD_BUZZER_SECOND_ON:
        default:
            gpio_set_level(BOARD_BUZZER_PIN, GPIO_LOW);
            s_board_buzzer_state = BOARD_BUZZER_IDLE;
            s_board_buzzer_ticks = 0U;
            break;
    }
}

static void init_default_pid_parameters(void)
{

    PID_Init(&gyro_pid_roll,
             3.5f, 0.0f, 0.0f,
             250.0f, 10000.0f, 20.0f, 0.8f);  /* Roll角速度内环：1000 Hz，周期1 ms。 */
    PID_Init(&gyro_pid_pitch,
             3.5f, 0.0f, 0.0f,
             250.0f, 10000.0f, 20.0f, 0.8f);  /* Pitch角速度内环：1000 Hz，周期1 ms。 */
    PID_Init(&gyro_pid_yaw,
             3.00f, 0.0f, 0.0f,
             200.0f, 5000.0f, 15.0f, 0.8f);  /* Yaw角速度内环：1000 Hz，周期1 ms。 */
    PID_Init(&HeightPID, 0.5f, 0.005f, 0.0f, 40.0f, 2000.0f, 20.0f, 0.3f);  /* 高度位置外环：100 Hz，周期10 ms。 */
    PID_Init(&HeightVelPID, 2.0f, 0.00f, 4.0f, 150.0f, 2000.0f, 20.0f, 0.3f);  /* 垂直速度内环：100 Hz，周期10 ms。 */
    PID_Init(&angle_pid_roll,  6.50f, 0.01f, 0.0f, 350.0f, 5000.0f, 5.0f, 0.80f);  /* Roll姿态角外环：500 Hz，周期2 ms。 */
    PID_Init(&angle_pid_pitch, 6.50f, 0.01f, 0.0f, 350.0f, 5000.0f, 5.0f, 0.80f);  /* Pitch姿态角外环：500 Hz，周期2 ms。 */
    PID_Init(&angle_pid_yaw,   6.0f, 0.0f, 0.0f, 250.0f, 400.0f, 12.0f, 0.0f);  /* Yaw姿态角外环：500 Hz，周期2 ms。 */
    PID_Init(&vel_pid_x, 2.8f, 0.0f, 0.1f, 458.0f, 2000.0f, 10.0f, 0.0f);  /* 惯性系X速度环：250 Hz，周期4 ms。 */
    PID_Init(&vel_pid_y, 2.8f, 0.0f, 0.1f, 458.0f, 2000.0f, 10.0f, 0.0f);  /* 惯性系Y速度环：250 Hz，周期4 ms。 */
    /* 速度D由velocity_loop.c独立滤波，末尾legacy alpha不参与速度D计算。 */
    /* 水平速度PID输出单位统一为cm/s^2，积分限幅也表示加速度贡献。
    PID_Init(&vel_pid_x, 2.8f, 1.0f, 0.20f, 458.0f, 80.0f, 0.0f, 0.0f);
    PID_Init(&vel_pid_y, 2.8f, 1.0f, 0.20f, 458.0f, 80.0f, 0.0f, 0.0f);
    */
    /* 二维位置参数由position_loop.c中的轨迹配置统一初始化。 */

    PID_SetReferenceDt(&gyro_pid_roll, LEGACY_FAST_PID_DT_S);
    PID_SetReferenceDt(&gyro_pid_pitch, LEGACY_FAST_PID_DT_S);
    PID_SetReferenceDt(&gyro_pid_yaw, LEGACY_FAST_PID_DT_S);
    PID_SetReferenceDt(&angle_pid_roll, LEGACY_FAST_PID_DT_S);
    PID_SetReferenceDt(&angle_pid_pitch, LEGACY_FAST_PID_DT_S);
    PID_SetReferenceDt(&angle_pid_yaw, LEGACY_FAST_PID_DT_S);
    PID_SetReferenceDt(&vel_pid_x, LEGACY_FAST_PID_DT_S);
    PID_SetReferenceDt(&vel_pid_y, LEGACY_FAST_PID_DT_S);
    (void)Velocity_Loop_Set_D_Filter_Hz(VELOCITY_LOOP_AXIS_X,
                                        VELOCITY_LOOP_D_FILTER_DEFAULT_HZ);
    (void)Velocity_Loop_Set_D_Filter_Hz(VELOCITY_LOOP_AXIS_Y,
                                        VELOCITY_LOOP_D_FILTER_DEFAULT_HZ);
    PID_SetReferenceDt(&HeightPID, HEIGHT_PID_DT_S);
    PID_SetReferenceDt(&HeightVelPID, HEIGHT_PID_DT_S);

    PID_PrepareRuntimeDt(&gyro_pid_roll, RATE_PID_RUNTIME_DT_S);
    PID_PrepareRuntimeDt(&gyro_pid_pitch, RATE_PID_RUNTIME_DT_S);
    PID_PrepareRuntimeDt(&gyro_pid_yaw, RATE_PID_RUNTIME_DT_S);
    PID_PrepareRuntimeDt(&angle_pid_roll, ANGLE_PID_RUNTIME_DT_S);
    PID_PrepareRuntimeDt(&angle_pid_pitch, ANGLE_PID_RUNTIME_DT_S);
    PID_PrepareRuntimeDt(&angle_pid_yaw, ANGLE_PID_RUNTIME_DT_S);
    PID_PrepareRuntimeDt(&vel_pid_x, VELOCITY_PID_RUNTIME_DT_S);
    PID_PrepareRuntimeDt(&vel_pid_y, VELOCITY_PID_RUNTIME_DT_S);
    PID_PrepareRuntimeDt(&HeightPID, HEIGHT_PID_DT_S);
    PID_PrepareRuntimeDt(&HeightVelPID, HEIGHT_PID_DT_S);
    Angle_Loop_InitTiming();
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
}
     * 光流速度闭环 PID @ 250Hz 软件分频。
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
    HeightManager_Init();
    PMW_Flow_Init();
    init_optical_flow_sensor();
    Flight_Control_Init();
}

static void init_periodic_interrupts(void)
{
    pit_us_init(KEY_VOLT_PIT, 5000);      // 5ms
    pit_us_init(CONTROL_LOOP_PIT, 250);   // 4kHz control
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
