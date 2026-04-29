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
#define JOYSTICK_ROLL_MAX_DEG          30.0f
#define JOYSTICK_PITCH_MAX_DEG         30.0f

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
PID_Controller joystick_pid_roll;
PID_Controller joystick_pid_pitch;

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
volatile float angle_roll_compensation = 0.0f;
volatile float angle_pitch_compensation = 0.0f;
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

    pwm_init(MOTOR_PWM_CH1, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_DUTY_MIN);
    pwm_init(MOTOR_PWM_CH2, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_DUTY_MIN);
    pwm_init(MOTOR_PWM_CH3, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_DUTY_MIN);
    pwm_init(MOTOR_PWM_CH4, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_DUTY_MIN);
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
        pwm_set_duty(MOTOR_PWM_CH1, MOTOR_PWM_DUTY_MIN);
        pwm_set_duty(MOTOR_PWM_CH2, MOTOR_PWM_DUTY_MIN);
        pwm_set_duty(MOTOR_PWM_CH3, MOTOR_PWM_DUTY_MIN);
        pwm_set_duty(MOTOR_PWM_CH4, MOTOR_PWM_DUTY_MIN);
        return;
    }

    pwm_set_duty(MOTOR_PWM_CH1, throttle_to_pwm_duty(apply_motor_hw_compensation(0U, motor_throttle[0])));
    pwm_set_duty(MOTOR_PWM_CH2, throttle_to_pwm_duty(apply_motor_hw_compensation(1U, motor_throttle[1])));
    pwm_set_duty(MOTOR_PWM_CH3, throttle_to_pwm_duty(apply_motor_hw_compensation(2U, motor_throttle[2])));
    pwm_set_duty(MOTOR_PWM_CH4, throttle_to_pwm_duty(apply_motor_hw_compensation(3U, motor_throttle[3])));
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

static void mix_motor_output(void)
{
    uint8 i = 0U;

    motor_output[3] = HeightSpeedPID_output + base_throttle + roll_output + pitch_output - yaw_output;
    motor_output[2] = HeightSpeedPID_output + base_throttle + roll_output - pitch_output + yaw_output;
    motor_output[1] = HeightSpeedPID_output + base_throttle - roll_output + pitch_output + yaw_output;
    motor_output[0] = HeightSpeedPID_output + base_throttle - roll_output - pitch_output - yaw_output;

    for(i = 0U; i < 4U; i++)
    {
        motor_throttle[i] = motor_output_safe_limit(motor_output[i]);
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

    if(!pmw3901_ready)
    {
        PMW_Flow_Reset();
        pmw3901_ready = (0 == pmw3901_init()) ? 1U : 0U;
        return;
    }

    pmw3901_get_motion();

    if(dl1b_distance_mm <= 4000U)
    {
        tof_los_cm = dl1b_distance_mm * 0.1f;
    }

    PMW_Flow_UpdateSample(pmw3901_delta_x,
                          pmw3901_delta_y,
                          current_roll,
                          current_pitch,
                          tof_los_cm,
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
    PMW_Flow_AccumulateGyro(current_gyro_roll, current_gyro_pitch, CONTROL_DT_S);// 更新PMW光流传感器的陀螺仪积分值，用于后续的运动补偿计算
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
        /* 遥感位置闭环已关闭：不再用 adc_x/adc_y + ToF 融合驱动姿态修正，
         * 目标姿态直接锁在起飞采样得到的地面基准角上。joystick PID 状态被
         * 重置以避免再次启用时累计历史发散。观测量清零方便上位机识别"未运行"。 */
        float captured_roll = 0.0f;
        float captured_pitch = 0.0f;

        Flight_Takeoff_Get_Captured_Angles(&captured_roll, &captured_pitch);

        target_angle_roll  = captured_roll;
        target_angle_pitch = captured_pitch;

        PID_Reset(&joystick_pid_roll);
        PID_Reset(&joystick_pid_pitch);

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
