#include "common/PIT_Demo.h"
#include "common/project_headfile.h"

// PWM motor output: 200Hz ESC PWM, 2000 = stop, 4000 = full output.
// We intentionally cap the commanded range to 60%, so the real max duty is 3200.
#define MOTOR_PWM_FREQ_HZ              200U
#define MOTOR_PWM_CH1                  TCPWM_CH09_P05_0
#define MOTOR_PWM_CH2                  TCPWM_CH10_P05_1
#define MOTOR_PWM_CH3                  TCPWM_CH11_P05_2
#define MOTOR_PWM_CH4                  TCPWM_CH12_P05_3
#define MOTOR_PWM_DUTY_MIN             2000U
#define MOTOR_PWM_DUTY_MAX             3200U

#define THROTTLE_RAMP_RATE             2.0f
#define CONTROL_DT_S                   0.00025f
#define CONTROL_DT_MS                  0.25f
#define VEL_LOOP_GRAVITY_CMS2          981.0f
#define TILT_THRUST_COMP_MAX           1.30f

/*
 * 硬件转速补偿：motor 1 / 3 / 4 实测转速偏弱，在最终 PWM 指令上按各自系数乘以抵消。
 * motor 2 不补偿。
 *
 * 挂点位置：PWM 出口（write_motor_pwm_output）。这样 MANUAL 直通和 ANGLE 混控两条
 * 路径写入的 motor_throttle[] 都会过这一层，开环/闭环都生效。
 * 越界保护：补偿后如超过 MOTOR_OUTPUT_SCALE_MAX 会被截断，不会击穿 ESC 限幅。
 * 电机测试模式会旁路掉，方便单独测量硬件差异。
 */
#define MOTOR_HW_COMP_M1_SCALE         1.00f
#define MOTOR_HW_COMP_M3_SCALE         1.00f
#define MOTOR_HW_COMP_M4_SCALE         1.00f
#define PMW3901_SAMPLE_DT_S            0.020f
#define ADC_INPUT_LPF_ALPHA            0.20f

/*
 * 光流速度闭环参数。
 *
 * 反馈来自 PMW3901 + ToF 解算的机体水平速度 velocity_x/y_cms
 * （单位 cm/s，约定: +x 机体前进，+y 机体右移；见 pmw_flow.c 注释）。
 *
 * 控制器使用独立 PID:
 *   vel_pid_x → 水平前向加速度指令 ax_cmd (cm/s^2)
 *   vel_pid_y → 水平右向加速度指令 ay_cmd (cm/s^2)
 * 再按小角度近似 a≈g*theta 换算成 pitch/roll 修正角后叠加到起飞基准上。
 * Current takeoff base is fixed at 0deg/0deg and is no longer learned from startup attitude.
 *
 * 极性宏用于一键修正机/IMU 安装方向带来的符号问题：
 *   pitch 正 = 低头（机体加速向前），所以 v_body_x>0 应让 pitch 减小。
 *   PID 输出本身已经是 -Kp*v_body_x（误差为 0-vx），符号已正确，
 *   PITCH_SIGN 默认 +1.0 即可；若硬件实际相反，改成 -1.0。
 *
 * Velocity acceleration command magnitude is limited only by vel_pid_x/y.output_limit.
 */
#define VEL_LOOP_PITCH_SIGN            (+1.0f)
#define VEL_LOOP_ROLL_SIGN             (+1.0f)

/*
 * 真位置闭环参数。
 *
 * 物理布置：无人机由长 ROPE_LENGTH_M 的绳子系留在地面遥感单元上，期望
 * 始终飞在遥感正上方。adc_x_nomalize / adc_y_nomalize 由遥感单元给出，
 * 已归一化到 [-1, 1]；满量程对应绳子与地面夹角为 ROPE_GROUND_ANGLE_FULL_DEG。
 *
 * 由此推得无人机相对遥感中心的水平位移：
 *   adc 归一化幅值=1 时位移 = L * cos(50°) = ROPE_LENGTH_M * 0.642788 ≈ 0.8678 m
 * 实现里直接按线性折算 position_error = adc_norm * POSITION_ERROR_MAX_M。
 *
 * ATTITUDE_MAX_DEG 是位置环输出（即偏离起飞基准的姿态修正量）的硬限幅，
 * 等价于无人机姿态控制允许的最大倾角。
 */
#define ROPE_LENGTH_M                  1.35f
#define ROPE_GROUND_ANGLE_FULL_DEG     50.0f
#define POSITION_ERROR_MAX_M           (ROPE_LENGTH_M * 0.642788f) /* L * cos(50°) */
#define ATTITUDE_MAX_DEG               3.0f

/*
 * 双数据源校验 + 加权融合参数。
 *
 * 思路：
 *   r_adc 由 adc_x/adc_y 折算得到水平半径；
 *   r_tof 由 sqrt(L² - h²) 反推（h 来自 tof_height_true_filt，单位 m）；
 *   两条独立观测在残差通过 innovation 门限时按方差倒数加权融合，否则记 fault
 *   并退回只信 adc，避免 ToF 失效（出量程、绳子松弛）把控制器带偏。
 *
 * σ 系数是经验起步值，可在试飞时调；权重在编译期算完，热路径上没有除法。
 */
#define FUSION_SIGMA_ADC_M             0.05f
#define FUSION_SIGMA_TOF_M             0.02f
#define FUSION_GATE_M                  0.16f   /* ~3·sqrt(σ_adc² + σ_tof²) */
#define FUSION_R_ADC_EPS_M             0.01f   /* r_adc 太小时不缩放，避免方向放大 */
#define FUSION_TOF_MIN_M               0.10f
#define FUSION_TOF_MAX_M               ROPE_LENGTH_M
#define FUSION_W_ADC                   (1.0f / (FUSION_SIGMA_ADC_M * FUSION_SIGMA_ADC_M))
#define FUSION_W_TOF                   (1.0f / (FUSION_SIGMA_TOF_M * FUSION_SIGMA_TOF_M))
#define BATTERY_NOMINAL_VOLTAGE        12.4f   /* 3S 充满电压，单位 V。 */

PID_Controller gyro_pid_roll;
PID_Controller gyro_pid_pitch;
PID_Controller gyro_pid_yaw;
PID_Controller angle_pid_roll;
PID_Controller angle_pid_pitch;
PID_Controller angle_pid_yaw;

/* 光流速度闭环 PID。摇杆 PID 已彻底退出控制链路，这里专门给 PMW3901 + ToF 解算的
 * 水平速度做闭环：vel_pid_x 作用于 pitch 修正、vel_pid_y 作用于 roll 修正。 */
PID_Controller vel_pid_x;
PID_Controller vel_pid_y;

volatile uint8 armed = 0;
volatile uint8 control_mode = 0;
volatile uint8 emergency_stop_triggered = 0;
volatile uint8 emergency_stop_reason = EMERGENCY_STOP_REASON_NONE;

volatile float target_throttle = 0.0f;
volatile float current_throttle = 0.0f;
volatile float base_throttle = 0.0f;

volatile float target_angle_roll = 0.0f;
volatile float target_angle_pitch = 0.0f;
volatile float target_angle_yaw = 0.0f;

/*
 * IMU 安装角补偿关闭（0/0/0）：飞行前的自动水平标定流程（Flight_Takeoff_Start）
 * 会在起飞前采样 3s 的 pitch/roll 并作为目标角度，等价于动态的地面标定，
 * 因此不再需要这组静态的安装角偏置。
 */
volatile float angle_roll_compensation = 0.0;
volatile float angle_pitch_compensation = 0.0;
volatile float angle_yaw_compensation = 0.0f;

int low_voltage_protect_flag = 0;
float voltage = BATTERY_NOMINAL_VOLTAGE;
uint16 adc_value = 0;

float current_roll = 0.0f;
float current_pitch = 0.0f;
float current_yaw = 0.0f;
float current_gyro_roll = 0.0f;
float current_gyro_pitch = 0.0f;
float current_gyro_yaw = 0.0f;

float target_roll_rate = 0.0f;
float target_pitch_rate = 0.0f;
float target_yaw_rate = 0.0f;

float roll_output = 0.0f;
float pitch_output = 0.0f;
static float yaw_output = 0.0f;

volatile uint16 motor_throttle[4] = {0, 0, 0, 0};
float motor_output[4] = {0.0f, 0.0f, 0.0f, 0.0f};
uint16 throttle_buffer[4] = {0, 0, 0, 0};

volatile int adc_x = 0;
volatile int adc_y = 0;
volatile int adc_x_filtered = 0.0f;
volatile int adc_y_filtered = 0.0f;
volatile float adc_x_nomalize = 0.0f;
volatile float adc_y_nomalize = 0.0f;
volatile float car_bat_voltage = 0.0f;  /* 小车通过 core_link 发送的电池电压，单位 V。 */

/* 位置闭环 adc/ToF 双数据校验观测量（VOFA / 监视页可读）。 */
volatile float pos_fusion_r_adc    = 0.0f;
volatile float pos_fusion_r_tof    = 0.0f;
volatile float pos_fusion_r_fused  = 0.0f;
volatile float pos_fusion_residual = 0.0f;
volatile uint8 pos_fusion_fault    = 0U;

/* 光流速度闭环：起飞 ramp 完成、首次进入巡航分支时由 PIT_Control_Task_250us
 * 自动 enable (悬停目标 vx=vy=0)。Disarm / Emergency_Stop 会自动 disable,
 * 下次起飞重新走自动 enable 路径。手动 VOFA / 调试也可调用 Velocity_Loop_Set_Enabled。 */
static volatile uint8 g_velocity_loop_enabled       = 0U;
static volatile float g_velocity_loop_target_vx_cms = 0.0f;
static volatile float g_velocity_loop_target_vy_cms = 0.0f;

/* 诊断量：实际叠加到基准角上的修正量与最近一次反馈速度。 */
static volatile float g_velocity_loop_pitch_corr_deg = 0.0f;
static volatile float g_velocity_loop_roll_corr_deg  = 0.0f;
static volatile float g_velocity_loop_fb_vx_cms      = 0.0f;
static volatile float g_velocity_loop_fb_vy_cms      = 0.0f;
static volatile float g_velocity_loop_x_i_out        = 0.0f;
static volatile float g_velocity_loop_y_i_out        = 0.0f;
static volatile uint8 g_velocity_loop_active         = 0U;  /* 上一拍是否真的喂了光流速度 */

/*
 * 光流 ToF 喂数据的容错状态。
 *
 * DL1B (VL53L1X short 模式) 有效量程仅 ~1.3m，飞机超过该高度时 sensor 会持续返回
 * dl1b_finsh_flag=0 / dl1b_distance_mm=8192。原始链路把这种失败直接当成 tof_los_cm=0,
 * 触发 PMW_Flow 内部 invalid 早退，整一帧光流速度被丢，表现为"高度 1m 以上没数据"。
 *
 * 这里在应用层加 3 段降级:
 *   1) dl1b_finsh_flag=1 且距离合理 → 直接用本帧 ToF, 同时缓存 last_valid 与重置 stale 计数
 *   2) 本帧失败但 stale 计数 < 阈值 → 使用 last_valid_los_cm 兜底, 让光流维持运行
 *   3) stale 超阈值 → tof_los_cm = 0, 让 PMW_Flow 内部 invalid (避免高度估计 stale 过久)
 *
 * Source 枚举仅作 VOFA / 监视页诊断:
 *   0 = NONE    (本帧无任何可信 ToF, 光流将 invalid)
 *   1 = LIVE    (来自本帧 dl1b)
 *   2 = HOLD    (兜底用上一帧缓存)
 */
#define PMW_FLOW_TOF_MIN_VALID_MM       (50U)        /* 5cm 与 PMW_FLOW_MIN_HEIGHT_CM 对齐 */
#define PMW_FLOW_TOF_MAX_VALID_MM       (4000U)      /* 与 dl1b_get_distance 内部上限一致 */
#define PMW_FLOW_TOF_STALE_TIMEOUT_TICK (50U)        /* 50 帧 × 20ms = 1s 后承认 ToF 失效 */
#define PMW_FLOW_TOF_SRC_NONE           (0U)
#define PMW_FLOW_TOF_SRC_LIVE           (1U)
#define PMW_FLOW_TOF_SRC_HOLD           (2U)

static volatile float g_pmw3901_flow_height_used_cm = 0.0f;
static volatile uint16 g_pmw3901_tof_stale_ticks = 0U;
static volatile uint8 g_pmw3901_tof_source = PMW_FLOW_TOF_SRC_NONE;
static float g_pmw3901_tof_last_valid_cm = 0.0f;

static uint8 motor_pwm_initialized = 0U;
static volatile uint8 motor_test_active = 0U;
static volatile uint8 motor_test_selected = 0U;
static volatile uint16 motor_test_throttle = 0U;
static volatile uint8 motor_test_output_enabled = 0U;

static void motor_pwm_init_once(void)
{
    if(motor_pwm_initialized)
    {
        return;
    }

#if MOTOR_OUTPUT_PROTOCOL == 0
    pwm_init(MOTOR_PWM_CH1, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_DUTY_MIN);
    pwm_init(MOTOR_PWM_CH2, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_DUTY_MIN);
    pwm_init(MOTOR_PWM_CH3, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_DUTY_MIN);
    pwm_init(MOTOR_PWM_CH4, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_DUTY_MIN);
#else
    dshot_init();
#endif
    motor_pwm_initialized = 1U;
}

static uint16 motor_output_limit_max(void)
{
    if(SAFE_MAX_MOTOR_OUTPUT_LIMIT > MOTOR_OUTPUT_SCALE_MAX)
    {
        return MOTOR_OUTPUT_SCALE_MAX;
    }

    return SAFE_MAX_MOTOR_OUTPUT_LIMIT;
}

static uint16 throttle_to_pwm_duty(uint16 throttle)
{
    if(throttle > MOTOR_OUTPUT_SCALE_MAX)
    {
        throttle = MOTOR_OUTPUT_SCALE_MAX;
    }

    return (uint16)(MOTOR_PWM_DUTY_MIN + throttle);
}

#if MOTOR_OUTPUT_PROTOCOL == 1
/*
 * DShot600 值域映射
 * motor_throttle[0, MOTOR_OUTPUT_SCALE_MAX] → DShot[0 / DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE]
 * throttle == 0 时发送 DISARM(0)，否则线性映射到 300~2047。
 */
static uint16 throttle_to_dshot(uint16 throttle)
{
    if(throttle == 0U)
    {
        return DSHOT_DISARM;
    }
    return (uint16)(DSHOT_MIN_THROTTLE +
                    (uint32)throttle * (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)
                    / MOTOR_OUTPUT_SCALE_MAX);
}
#endif

/*
 * 单电机硬件转速补偿钩子：MANUAL 和 ANGLE 两条路径写完 motor_throttle[] 后，
 * 都会从 write_motor_pwm_output 过这里乘系数。对 motor 1/3/4 生效，motor 2 不补偿。
 * 电机测试模式返回原值，方便单独对比硬件差异。
 */
static uint16 apply_motor_hw_compensation(uint8 motor_idx, uint16 throttle)
{
    float scale;
    float scaled;

    if(motor_test_active)
    {
        return throttle;
    }

    if(motor_idx == 0U)
    {
        scale = MOTOR_HW_COMP_M1_SCALE;
    }
    else if(motor_idx == 2U)
    {
        scale = MOTOR_HW_COMP_M3_SCALE;
    }
    else if(motor_idx == 3U)
    {
        scale = MOTOR_HW_COMP_M4_SCALE;
    }
    else
    {
        return throttle;
    }

    scaled = (float)throttle * scale;
    if(scaled > (float)MOTOR_OUTPUT_SCALE_MAX)
    {
        scaled = (float)MOTOR_OUTPUT_SCALE_MAX;
    }
    return (uint16)scaled;
}

static inline float throttle_safe_limit(float throttle)
{
    if(throttle > SAFE_MAX_THROTTLE)
    {
        throttle = SAFE_MAX_THROTTLE;
    }
    else if(throttle < 0.0f)
    {
        throttle = 0.0f;
    }
    return throttle;
}

static inline void throttle_ramp(float target, volatile float *current, float rate, float dt_ms)
{
    float error = target - (*current);
    float step = rate * dt_ms;

    if(error > step)
    {
        (*current) += step;
    }
    else if(error < -step)
    {
        (*current) -= step;
    }
    else
    {
        (*current) = target;
    }

    *current = throttle_safe_limit(*current);
}

static void clear_motor_throttle_state(void)
{
    motor_throttle[0] = 0U;
    motor_throttle[1] = 0U;
    motor_throttle[2] = 0U;
    motor_throttle[3] = 0U;
}

static void apply_motor_test_output(void)
{
    clear_motor_throttle_state();

    if(!motor_test_output_enabled)
    {
        return;
    }

    if(motor_test_selected < 4U)
    {
        motor_throttle[motor_test_selected] = motor_test_throttle;
    }
}

static void write_motor_pwm_output(void)
{
    if(!armed && !motor_test_output_enabled)
    {
#if MOTOR_OUTPUT_PROTOCOL == 0
        pwm_set_duty(MOTOR_PWM_CH1, MOTOR_PWM_DUTY_MIN);
        pwm_set_duty(MOTOR_PWM_CH2, MOTOR_PWM_DUTY_MIN);
        pwm_set_duty(MOTOR_PWM_CH3, MOTOR_PWM_DUTY_MIN);
        pwm_set_duty(MOTOR_PWM_CH4, MOTOR_PWM_DUTY_MIN);
#else
        dshot_set_all_throttle(0U, 0U, 0U, 0U);
#endif
        return;
    }

#if MOTOR_OUTPUT_PROTOCOL == 0
    pwm_set_duty(MOTOR_PWM_CH1, throttle_to_pwm_duty(apply_motor_hw_compensation(0U, motor_throttle[0])));
    pwm_set_duty(MOTOR_PWM_CH2, throttle_to_pwm_duty(apply_motor_hw_compensation(1U, motor_throttle[1])));
    pwm_set_duty(MOTOR_PWM_CH3, throttle_to_pwm_duty(apply_motor_hw_compensation(2U, motor_throttle[2])));
    pwm_set_duty(MOTOR_PWM_CH4, throttle_to_pwm_duty(apply_motor_hw_compensation(3U, motor_throttle[3])));
#else
    dshot_set_all_throttle(
        throttle_to_dshot(apply_motor_hw_compensation(0U, motor_throttle[0])),
        throttle_to_dshot(apply_motor_hw_compensation(1U, motor_throttle[1])),
        throttle_to_dshot(apply_motor_hw_compensation(2U, motor_throttle[2])),
        throttle_to_dshot(apply_motor_hw_compensation(3U, motor_throttle[3]))
    );
#endif
}

static uint8 check_attitude_safety(void)
{
    if(!armed)
    {
        return 0U;
    }//未解锁时不要因为姿态超限去触发急停

    if(current_roll > SAFE_MAX_ANGLE_ROLL || current_roll < -SAFE_MAX_ANGLE_ROLL)
    {
        Flight_Runtime_Emergency_Stop(EMERGENCY_STOP_REASON_ATTITUDE_ROLL);
        return 1U;
    }
    if(current_pitch > SAFE_MAX_ANGLE_PITCH || current_pitch < -SAFE_MAX_ANGLE_PITCH)
    {
        Flight_Runtime_Emergency_Stop(EMERGENCY_STOP_REASON_ATTITUDE_PITCH);
        return 1U;
    }
    return 0U;
}

static uint16 motor_output_safe_limit(float output)
{
    uint16 output_limit = motor_output_limit_max();

    if(output > output_limit)
    {
        output = output_limit;
    }
    else if(output < 0.0f)
    {
        output = 0.0f;
    }
    return (uint16)output;
}

static uint16 motor_output_flight_limit(float output, float common_throttle)
{
    if((armed != 0U) && (common_throttle >= SAFE_MIN_THROTTLE) && (output < SAFE_MIN_THROTTLE))
    {
        output = SAFE_MIN_THROTTLE;
    }

    return motor_output_safe_limit(output);
}

static void mix_motor_output(void)
{
    uint8 i = 0U;
    float roll_rad;
    float pitch_rad;
    float tilt_cos;
    float thrust_base;
    float thrust_base_comp;

    roll_rad = current_roll * DEG2RAD;
    pitch_rad = current_pitch * DEG2RAD;
    tilt_cos = cosf(roll_rad) * cosf(pitch_rad);
    if(tilt_cos < (1.0f / TILT_THRUST_COMP_MAX))
    {
        tilt_cos = (1.0f / TILT_THRUST_COMP_MAX);
    }

    thrust_base = HeightSpeedPID_output + base_throttle;
    thrust_base_comp = thrust_base / tilt_cos;

    motor_output[3] = thrust_base_comp + roll_output + pitch_output - yaw_output;
    motor_output[2] = thrust_base_comp + roll_output - pitch_output + yaw_output;
    motor_output[1] = thrust_base_comp - roll_output + pitch_output + yaw_output;
    motor_output[0] = thrust_base_comp - roll_output - pitch_output - yaw_output;

    for(i = 0U; i < 4U; i++)
    {
        motor_throttle[i] = motor_output_flight_limit(motor_output[i], thrust_base_comp);
    }
}

void low_voltage_check(void)
{
}

void Flight_Runtime_Emergency_Stop(uint8 reason)
{
    armed = 0U;
    emergency_stop_triggered = 1U;
    emergency_stop_reason = reason;
    motor_test_active = 0U;
    motor_test_output_enabled = 0U;
    motor_test_throttle = 0U;
    target_throttle = 0.0f;
    current_throttle = 0.0f;
    base_throttle = 0.0f;
    Flight_Takeoff_Abort();
    /* 急停立刻断开速度环, 下次起飞 ramp 完成后会再次自动 enable。 */
    Velocity_Loop_Set_Enabled(0U);
    clear_motor_throttle_state();
}

void Flight_Runtime_Clear_Emergency_Stop_Latch(void)
{
    emergency_stop_triggered = 0U;
    emergency_stop_reason = EMERGENCY_STOP_REASON_NONE;
}

void Flight_Runtime_Reset_All_Throttle_States(void)
{
    target_throttle = 0.0f;
    current_throttle = 0.0f;
    base_throttle = 0.0f;
    motor_test_output_enabled = 0U;
    motor_test_throttle = 0U;
    clear_motor_throttle_state();
}

void Flight_Runtime_Set_Throttle_For_Mode(uint8 mode, float throttle)
{
    (void)mode;
    target_throttle = throttle_safe_limit(throttle);
}

float Flight_Runtime_Get_Target_Throttle_For_Mode(uint8 mode)
{
    (void)mode;
    return target_throttle;
}

float Flight_Runtime_Get_Base_Throttle_For_Mode(uint8 mode)
{
    (void)mode;
    return base_throttle;
}

void Flight_Runtime_Sync_Public_Throttle_State(void)
{
}

void Flight_Motor_Test_Start(void)
{
    target_throttle = 0.0f;
    current_throttle = 0.0f;
    base_throttle = 0.0f;

    if(!motor_test_active)
    {
        motor_test_selected = 0U;
        motor_test_throttle = 0U;
        motor_test_output_enabled = 0U;
    }

    motor_test_active = 1U;
    clear_motor_throttle_state();
}

void Flight_Motor_Test_Stop(void)
{
    motor_test_active = 0U;
    motor_test_output_enabled = 0U;
    motor_test_throttle = 0U;
    target_throttle = 0.0f;
    current_throttle = 0.0f;
    base_throttle = 0.0f;
    clear_motor_throttle_state();
}

uint8 Flight_Motor_Test_Is_Active(void)
{
    return motor_test_active;
}

void Flight_Motor_Test_Set_Selected(uint8 motor_index)
{
    if(motor_index >= 4U)
    {
        motor_index = 3U;
    }

    motor_test_selected = motor_index;
}

uint8 Flight_Motor_Test_Get_Selected(void)
{
    return motor_test_selected;
}

void Flight_Motor_Test_Set_Throttle(uint16 throttle)
{
    uint16 output_limit = motor_output_limit_max();
    uint16 test_limit = MOTOR_TEST_MAX_THROTTLE;

    if(test_limit > output_limit)
    {
        test_limit = output_limit;
    }

    if(throttle > test_limit)
    {
        throttle = test_limit;
    }

    motor_test_throttle = throttle;
}

uint16 Flight_Motor_Test_Get_Throttle(void)
{
    return motor_test_throttle;
}

void Flight_Motor_Test_Set_Output_Enable(uint8 enable)
{
    motor_test_output_enabled = enable ? 1U : 0U;
}

uint8 Flight_Motor_Test_Get_Output_Enable(void)
{
    return motor_test_output_enabled;
}

uint16 Flight_Motor_Test_Get_Output_Limit(void)
{
    return motor_output_limit_max();
}

void PIT_KeyVoltage_Task_5ms(void)
{
    key_scanner();
    key_scan();
    adc_value = adc_mean_filter_convert(BATTERY_ADC_CHANNEL, 10);
    voltage = adc_value * BATTERY_ADC_TO_VOLTAGE_SCALE;

    /* 使用小车实测电压做低电保护判断（小车与飞机共用同一块电池）。
     * car_bat_voltage == 0.0f 表示小车未建立通信，此时不做判断。 */
    if(car_bat_voltage > 0.0f)
    {
        if(car_bat_voltage < BATTERY_LOW_WARN_THRESHOLD)
        {
            low_voltage_protect_flag = 1;
        }
        else if(car_bat_voltage >= BATTERY_LOW_RECOVER_THRESHOLD)
        {
            low_voltage_protect_flag = 0;
        }
    }
}

void PIT_Adc_Task_5ms(void)
{
    static uint8 adc_lpf_initialized = 0U;
    static float adc_x_lpf_state = 0.0f;
    static float adc_y_lpf_state = 0.0f;

    if(!adc_lpf_initialized)
    {
        adc_x_lpf_state = (float)adc_x;
        adc_y_lpf_state = (float)adc_y;
        adc_lpf_initialized = 1U;
    }
    else
    {
        adc_x_lpf_state += ((float)adc_x - adc_x_lpf_state) * ADC_INPUT_LPF_ALPHA;
        adc_y_lpf_state += ((float)adc_y - adc_y_lpf_state) * ADC_INPUT_LPF_ALPHA;
    }

    adc_x_filtered = (int)adc_x_lpf_state;
    adc_y_filtered = (int)adc_y_lpf_state;
    adc_x_nomalize = joystick_normalize(adc_x_filtered);
    adc_y_nomalize = joystick_normalize(adc_y_filtered);
}

void PIT_PMW3901_Task_20ms(void)
{
    static uint8 pmw3901_ready = 0;
    float tof_los_cm = 0.0f;
    float flow_height_cm = 0.0f;
    uint8 tof_source = PMW_FLOW_TOF_SRC_NONE;

    if(!pmw3901_ready)
    {
        PMW_Flow_Reset();
        g_pmw3901_flow_height_used_cm = 0.0f;
        g_pmw3901_tof_stale_ticks = 0U;
        g_pmw3901_tof_source = PMW_FLOW_TOF_SRC_NONE;
        g_pmw3901_tof_last_valid_cm = 0.0f;
        pmw3901_ready = (0 == pmw3901_init()) ? 1U : 0U;
        return;
    }

    pmw3901_get_motion();

    /* 三段降级: LIVE → HOLD → NONE。详见上方常量与 g_pmw3901_tof_source 注释。 */
    if(dl1b_finsh_flag &&
       dl1b_distance_mm >= PMW_FLOW_TOF_MIN_VALID_MM &&
       dl1b_distance_mm <= PMW_FLOW_TOF_MAX_VALID_MM)
    {
        tof_los_cm = (float)dl1b_distance_mm * 0.1f;
        g_pmw3901_tof_last_valid_cm = tof_los_cm;
        g_pmw3901_tof_stale_ticks = 0U;
        tof_source = PMW_FLOW_TOF_SRC_LIVE;
    }
    else if(g_pmw3901_tof_last_valid_cm > 0.0f &&
            g_pmw3901_tof_stale_ticks < PMW_FLOW_TOF_STALE_TIMEOUT_TICK)
    {
        /* sensor 短暂失效, 用上一拍缓存的 ToF 维持光流运行 (默认 1s 内). */
        tof_los_cm = g_pmw3901_tof_last_valid_cm;
        g_pmw3901_tof_stale_ticks++;
        tof_source = PMW_FLOW_TOF_SRC_HOLD;
    }
    else
    {
        /* 失效太久或从未取到过 ToF: 喂 0 让 PMW_Flow_UpdateSample 走 invalid 路径. */
        tof_los_cm = 0.0f;
        if(g_pmw3901_tof_stale_ticks < 0xFFFFU)
        {
            g_pmw3901_tof_stale_ticks++;
        }
        tof_source = PMW_FLOW_TOF_SRC_NONE;
    }

    if(tof_los_cm > 0.0f)
    {
        flow_height_cm = HeightManager_GetFlowHeightFromTofCm(tof_los_cm,
                                                              current_roll,
                                                              current_pitch);
    }

    g_pmw3901_flow_height_used_cm = flow_height_cm;
    g_pmw3901_tof_source = tof_source;

    PMW_Flow_UpdateSample(pmw3901_delta_x,
                          pmw3901_delta_y,
                          current_roll,
                          current_pitch,
                          flow_height_cm,
                          PMW3901_SAMPLE_DT_S);
}

void PIT_Motor_Task_500us(void)
{
    write_motor_pwm_output();
}

void PIT_Control_Task_250us(void)
{
    uint8 takeoff_active = 0U;

    motor_pwm_init_once();

    if(icm_calibration_flag)
    {
        motor_test_output_enabled = 0U;
        clear_motor_throttle_state();
        return;
    }

    get_ICM_data();
    Get_Attitude_Angle(&current_roll, &current_pitch, &current_yaw);
    Get_Angular_Rate(&current_gyro_roll, &current_gyro_pitch, &current_gyro_yaw);
    PMW_Flow_AccumulateGyro(icm42688_gyro_x, icm42688_gyro_y, CONTROL_DT_S);// 更新PMW光流传感器的陀螺仪积分值，用于后续的运动补偿计算
    if(check_attitude_safety())
    {
        return;
    }// 急停触发后不继续执行后续的飞控计算，保持电机输出为0，直到飞行器被重置或重新解锁

    /* 驱动自动起飞状态机：采样阶段靠早期 current_roll/pitch 即可累加，
     * ramp 阶段会接管 base_throttle 与 target_angle_roll/pitch。 */
    takeoff_active = Flight_Takeoff_Tick_250us();

    if(motor_test_active)
    {
        if(emergency_stop_triggered)
        {
            motor_test_output_enabled = 0U;
            clear_motor_throttle_state();
            return;
        }

        apply_motor_test_output();
        return;
    }

    if(!armed)
    {
        clear_motor_throttle_state();
        return;
    }

    if(!takeoff_active)
    {
        /* Cruise branch uses a fixed 0deg/0deg takeoff base. */
        throttle_ramp(target_throttle, &current_throttle, THROTTLE_RAMP_RATE, CONTROL_DT_MS);
        base_throttle = current_throttle;
    }
    /* takeoff_active 时 base_throttle / current_throttle / target_throttle 已由 ramp 阶段统一写入。 */

    if(control_mode == CONTROL_MODE_MANUAL)
    {
        uint16 manual_output = motor_output_safe_limit(base_throttle);

        motor_throttle[0] = manual_output;
        motor_throttle[1] = manual_output;
        motor_throttle[2] = manual_output;
        motor_throttle[3] = manual_output;
        return;
    }

    if(!takeoff_active)
    {
        /* 巡航阶段：原 ADC 位置闭环已废弃，目标姿态默认锁在起飞采样得到的
         * 地面基准角上。当 g_velocity_loop_enabled = 1 且光流 valid 时，
         * 用 PMW3901 + ToF 解算的机体水平速度跑速度闭环：
         *   PID(target=0, fb=v_body_x/y)  -> 角度修正量 (deg)
         * 修正量再叠加到 captured 基准角上，限幅只由 vel_pid_x/y.output_limit 决定，
         * 最终写入 target_angle_roll/pitch 交给已有姿态外环。 */
        float captured_roll = 0.0f;
        float captured_pitch = 0.0f;
        uint8 vel_loop_active_now = 0U;

        Flight_Takeoff_Get_Captured_Angles(&captured_roll, &captured_pitch);

        /* 起飞 ramp 完成、控制权回到巡航分支时自动接管速度环 (悬停目标)。
         * Disarm / Emergency_Stop 会把 g_velocity_loop_enabled 清 0,
         * 下次起飞会再次走到这里重新自动 enable。 */
        if(!g_velocity_loop_enabled)
        {
            g_velocity_loop_target_vx_cms = 0.0f;
            g_velocity_loop_target_vy_cms = 0.0f;
            g_velocity_loop_enabled       = 1U;
            PID_Reset(&vel_pid_x);
            PID_Reset(&vel_pid_y);
        }

        if(g_velocity_loop_enabled)
        {
            PMWFlowState_t flow_snap;
            PMW_Flow_GetState(&flow_snap);

            if(flow_snap.valid && (flow_snap.flow_used || flow_snap.acc_used))
            {
                float ax_cmd_cms2;
                float ay_cmd_cms2;
                float pitch_corr;
                float roll_corr;

                /* Velocity PID feedback uses the PMW flow / horizontal-acc fused velocity. */
                ax_cmd_cms2 = PID_Calculate(&vel_pid_x,
                                            g_velocity_loop_target_vx_cms,
                                            flow_snap.velocity_x_fused_cms,
                                            CONTROL_DT_S);
                ay_cmd_cms2 = PID_Calculate(&vel_pid_y,
                                            g_velocity_loop_target_vy_cms,
                                            flow_snap.velocity_y_fused_cms,
                                            CONTROL_DT_S);

                /* Small-angle approximation near hover:
                 *   ax ≈ -g * pitch  -> pitch ≈ -ax / g
                 *   ay ≈  g * roll   -> roll  ≈  ay / g
                 */
                pitch_corr = -(ax_cmd_cms2 / VEL_LOOP_GRAVITY_CMS2) * RAD2DEG;
                roll_corr  =  (ay_cmd_cms2 / VEL_LOOP_GRAVITY_CMS2) * RAD2DEG;

                pitch_corr *= VEL_LOOP_PITCH_SIGN;
                roll_corr  *= VEL_LOOP_ROLL_SIGN;

                target_angle_pitch = captured_pitch + pitch_corr;
                target_angle_roll  = captured_roll  + roll_corr;

                g_velocity_loop_x_i_out        = vel_pid_x.Ki * vel_pid_x.integral;
                g_velocity_loop_y_i_out        = vel_pid_y.Ki * vel_pid_y.integral;
                g_velocity_loop_pitch_corr_deg = pitch_corr;
                g_velocity_loop_roll_corr_deg  = roll_corr;
                g_velocity_loop_fb_vx_cms      = flow_snap.velocity_x_fused_cms;
                g_velocity_loop_fb_vy_cms      = flow_snap.velocity_y_fused_cms;
                vel_loop_active_now = 1U;
            }
            else
            {
                /* 光流暂时无效：退回锁基准角 + reset 速度 PID，下一帧 valid 后从 0 重启。 */
                target_angle_roll  = captured_roll;
                target_angle_pitch = captured_pitch;
                PID_Reset(&vel_pid_x);
                PID_Reset(&vel_pid_y);
                g_velocity_loop_x_i_out        = 0.0f;
                g_velocity_loop_y_i_out        = 0.0f;
                g_velocity_loop_pitch_corr_deg = 0.0f;
                g_velocity_loop_roll_corr_deg  = 0.0f;
            }
        }
        else
        {
            target_angle_roll  = captured_roll;
            target_angle_pitch = captured_pitch;
            PID_Reset(&vel_pid_x);
            PID_Reset(&vel_pid_y);
            g_velocity_loop_x_i_out        = 0.0f;
            g_velocity_loop_y_i_out        = 0.0f;
            g_velocity_loop_pitch_corr_deg = 0.0f;
            g_velocity_loop_roll_corr_deg  = 0.0f;
        }

        g_velocity_loop_active = vel_loop_active_now;

        pos_fusion_r_adc    = 0.0f;
        pos_fusion_r_tof    = 0.0f;
        pos_fusion_r_fused  = 0.0f;
        pos_fusion_residual = 0.0f;
        pos_fusion_fault    = 0U;
    }
    /* takeoff_active 时 target_angle_roll / target_angle_pitch 已由 ramp 阶段锁定为地面均值。 */

    target_roll_rate = PID_Calculate(&angle_pid_roll, target_angle_roll, current_roll, CONTROL_DT_S);
    target_pitch_rate = PID_Calculate(&angle_pid_pitch, target_angle_pitch, current_pitch, CONTROL_DT_S);
    target_yaw_rate = PID_Calculate(&angle_pid_yaw, target_angle_yaw, current_yaw, CONTROL_DT_S);

    roll_output = PID_Calculate(&gyro_pid_roll, target_roll_rate, current_gyro_roll, CONTROL_DT_S);
    pitch_output = PID_Calculate(&gyro_pid_pitch, target_pitch_rate, current_gyro_pitch, CONTROL_DT_S);
    yaw_output = -PID_Calculate(&gyro_pid_yaw, target_yaw_rate, current_gyro_yaw, CONTROL_DT_S);

    mix_motor_output();
}

/*********************************************************************************************************************
 * 光流速度闭环对外接口
 *
 * 默认关闭。打开前请确保:
 *   - 起飞流程已完成（控制权回到 !takeoff_active 分支）
 *   - 光流 valid（PMW3901 + ToF 都有数据）
 *   - vel_pid_x / vel_pid_y 的 Kp/Ki/Kd/output_limit 已按速度环口径调过
 *
 * 关闭时控制环立即回退到"锁基准角"行为，并 PID_Reset。
 *********************************************************************************************************************/
void Velocity_Loop_Set_Enabled(uint8 enabled)
{
    if(enabled)
    {
        g_velocity_loop_enabled = 1U;
    }
    else
    {
        g_velocity_loop_enabled = 0U;
        /* 立即清掉诊断量，避免上位机看到残留值。PID_Reset 留给控制 tick 自己做。 */
        g_velocity_loop_x_i_out        = 0.0f;
        g_velocity_loop_y_i_out        = 0.0f;
        g_velocity_loop_pitch_corr_deg = 0.0f;
        g_velocity_loop_roll_corr_deg  = 0.0f;
        g_velocity_loop_active = 0U;
    }
}

uint8 Velocity_Loop_Is_Enabled(void)
{
    return g_velocity_loop_enabled;
}

void Velocity_Loop_Set_Target_Velocity(float vx_cms, float vy_cms)
{
    g_velocity_loop_target_vx_cms = vx_cms;
    g_velocity_loop_target_vy_cms = vy_cms;
}

void Velocity_Loop_Get_Target_Velocity(float *vx_cms, float *vy_cms)
{
    if(vx_cms != NULL)
    {
        *vx_cms = g_velocity_loop_target_vx_cms;
    }
    if(vy_cms != NULL)
    {
        *vy_cms = g_velocity_loop_target_vy_cms;
    }
}

void Velocity_Loop_Get_Diag(float *pitch_corr_deg,
                            float *roll_corr_deg,
                            float *fb_vx_cms,
                            float *fb_vy_cms,
                            float *x_i_out,
                            float *y_i_out,
                            uint8 *active)
{
    if(pitch_corr_deg != NULL) *pitch_corr_deg = g_velocity_loop_pitch_corr_deg;
    if(roll_corr_deg  != NULL) *roll_corr_deg  = g_velocity_loop_roll_corr_deg;
    if(fb_vx_cms      != NULL) *fb_vx_cms      = g_velocity_loop_fb_vx_cms;
    if(fb_vy_cms      != NULL) *fb_vy_cms      = g_velocity_loop_fb_vy_cms;
    if(x_i_out        != NULL) *x_i_out        = g_velocity_loop_x_i_out;
    if(y_i_out        != NULL) *y_i_out        = g_velocity_loop_y_i_out;
    if(active         != NULL) *active         = g_velocity_loop_active;
}

/*
 * 光流 ToF 容错诊断: 让上位机能直接看到当前 ToF 来自 sensor 还是 hold 缓存,
 * 以及失效连续了多少帧, 用于判断 dl1b 真实量程上限。
 *   tof_los_cm: 本帧实际喂给 PMW_Flow_UpdateSample 的光流安装点竖直高度 (cm), 0 表示失效
 *   stale_ticks: 连续未拿到新 ToF 的帧数 (20ms / 帧)
 *   source: PMW_FLOW_TOF_SRC_NONE / LIVE / HOLD
 */
void PMW_Flow_Get_TOF_Status(float *tof_los_cm, uint16 *stale_ticks, uint8 *source)
{
    if(tof_los_cm   != NULL) *tof_los_cm   = g_pmw3901_flow_height_used_cm;
    if(stale_ticks  != NULL) *stale_ticks  = g_pmw3901_tof_stale_ticks;
    if(source       != NULL) *source       = g_pmw3901_tof_source;
}
