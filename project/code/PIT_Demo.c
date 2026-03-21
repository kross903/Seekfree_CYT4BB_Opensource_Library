#include "PIT_Demo.h"
#include "flight_control.h"
#include "Kalman Filter/height_manager.h"
#include "control/joystick.h"
#include "icm/icm.h"
#include "menu/key.h"
#include "zf_common_headfile.h"

// PWM motor output: 50Hz ESC PWM, 5%-10% duty (1000us-2000us).
#define MOTOR_PWM_FREQ_HZ              50U
#define MOTOR_PWM_CH1                  TCPWM_CH28_P10_0
#define MOTOR_PWM_CH2                  TCPWM_CH29_P10_1
#define MOTOR_PWM_CH3                  TCPWM_CH30_P10_2
#define MOTOR_PWM_CH4                  TCPWM_CH31_P10_3
#define MOTOR_PWM_DUTY_MIN             500U
#define MOTOR_PWM_DUTY_MAX             1000U

#define THROTTLE_RAMP_RATE             2.0f
#define CONTROL_DT_S                   0.00025f
#define CONTROL_DT_MS                  0.25f
#define JOYSTICK_ROLL_MAX_DEG          20.0f
#define JOYSTICK_PITCH_MAX_DEG         20.0f
#define BATTERY_NOMINAL_VOLTAGE        24.0f

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
volatile float target_angle_roll_add = -0.7f;
volatile float target_angle_pitch_add = -1.4f;
volatile float target_angle_yaw_add = 0.0f;

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
volatile float adc_x_filtered = 0.0f;
volatile float adc_y_filtered = 0.0f;
volatile float adc_x_nomalize = 0.0f;
volatile float adc_y_nomalize = 0.0f;

static uint8 motor_pwm_initialized = 0U;

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

static uint16 throttle_to_pwm_duty(uint16 throttle)
{
    uint32 span = (uint32)(MOTOR_PWM_DUTY_MAX - MOTOR_PWM_DUTY_MIN);
    uint32 duty = (uint32)MOTOR_PWM_DUTY_MIN;

    if(throttle > SAFE_MAX_MOTOR_OUTPUT)
    {
        throttle = SAFE_MAX_MOTOR_OUTPUT;
    }

    duty += (span * throttle) / SAFE_MAX_MOTOR_OUTPUT;
    return (uint16)duty;
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

static void write_motor_pwm_output(void)
{
    if(!armed)
    {
        pwm_set_duty(MOTOR_PWM_CH1, MOTOR_PWM_DUTY_MIN);
        pwm_set_duty(MOTOR_PWM_CH2, MOTOR_PWM_DUTY_MIN);
        pwm_set_duty(MOTOR_PWM_CH3, MOTOR_PWM_DUTY_MIN);
        pwm_set_duty(MOTOR_PWM_CH4, MOTOR_PWM_DUTY_MIN);
        return;
    }

    pwm_set_duty(MOTOR_PWM_CH1, throttle_to_pwm_duty(motor_throttle[0]));
    pwm_set_duty(MOTOR_PWM_CH2, throttle_to_pwm_duty(motor_throttle[1]));
    pwm_set_duty(MOTOR_PWM_CH3, throttle_to_pwm_duty(motor_throttle[2]));
    pwm_set_duty(MOTOR_PWM_CH4, throttle_to_pwm_duty(motor_throttle[3]));
}

static uint8 check_attitude_safety(void)
{
    if(!armed)
    {
        return 0U;
    }

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
    if(output > SAFE_MAX_MOTOR_OUTPUT)
    {
        output = SAFE_MAX_MOTOR_OUTPUT;
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

    motor_output[3] = HeightSpeedPID_output + base_throttle + roll_output - pitch_output - yaw_output;
    motor_output[2] = HeightSpeedPID_output + base_throttle - roll_output - pitch_output + yaw_output;
    motor_output[1] = HeightSpeedPID_output + base_throttle + roll_output + pitch_output + yaw_output;
    motor_output[0] = HeightSpeedPID_output + base_throttle - roll_output + pitch_output - yaw_output;

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
    target_throttle = 0.0f;
    current_throttle = 0.0f;
    base_throttle = 0.0f;
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

void PIT_KeyVoltage_Task_5ms(void)
{
    key_scanner();
    key_scan();
    adc_value = adc_mean_filter_convert(ADC1_CH20_P14_0, 50);
    voltage = adc_value * BATTERY_ADC_TO_VOLTAGE_SCALE;
    low_voltage_protect_flag = (voltage < BATTERY_LOW_WARN_THRESHOLD) ? 1 : 0;
}

void PIT_Adc_Task_5ms(void)
{
    adc_x = adc_convert(ADC0_CH00_P06_0);
    adc_y = adc_convert(ADC0_CH01_P06_1);
    adc_x_filtered = (float)adc_x;
    adc_y_filtered = (float)adc_y;
    adc_x_nomalize = joystick_normalize(adc_x);
    adc_y_nomalize = joystick_normalize(adc_y);
}

void PIT_Motor_Task_500us(void)
{
    write_motor_pwm_output();
}

void PIT_Control_Task_250us(void)
{
    motor_pwm_init_once();

    if(icm_calibration_flag)
    {
        clear_motor_throttle_state();
        return;
    }

    get_ICM_data();
    Get_Attitude_Angle(&current_roll, &current_pitch, &current_yaw);
    Get_Angular_Rate(&current_gyro_roll, &current_gyro_pitch, &current_gyro_yaw);

    if(check_attitude_safety())
    {
        return;
    }

    HeightManager_Update();

    if(!armed)
    {
        clear_motor_throttle_state();
        return;
    }

    throttle_ramp(target_throttle, &current_throttle, THROTTLE_RAMP_RATE, CONTROL_DT_MS);
    base_throttle = current_throttle;

    if(control_mode == CONTROL_MODE_MANUAL)
    {
        motor_throttle[0] = (uint16)base_throttle;
        motor_throttle[1] = (uint16)base_throttle;
        motor_throttle[2] = (uint16)base_throttle;
        motor_throttle[3] = (uint16)base_throttle;
        return;
    }

    target_angle_roll = PID_Calculate(&joystick_pid_roll,
                                      adc_x_nomalize * JOYSTICK_ROLL_MAX_DEG,
                                      current_roll,
                                      CONTROL_DT_S);
    target_angle_pitch = PID_Calculate(&joystick_pid_pitch,
                                       adc_y_nomalize * JOYSTICK_PITCH_MAX_DEG,
                                       current_pitch,
                                       CONTROL_DT_S);

    target_roll_rate = PID_Calculate(&angle_pid_roll, target_angle_roll + target_angle_roll_add, current_roll, CONTROL_DT_S);
    target_pitch_rate = PID_Calculate(&angle_pid_pitch, target_angle_pitch + target_angle_pitch_add, current_pitch, CONTROL_DT_S);
    target_yaw_rate = PID_Calculate(&angle_pid_yaw, target_angle_yaw + target_angle_yaw_add, current_yaw, CONTROL_DT_S);

    roll_output = PID_Calculate(&gyro_pid_roll, target_roll_rate, current_gyro_roll, CONTROL_DT_S);
    pitch_output = PID_Calculate(&gyro_pid_pitch, target_pitch_rate, current_gyro_pitch, CONTROL_DT_S);
    yaw_output = -PID_Calculate(&gyro_pid_yaw, target_yaw_rate, current_gyro_yaw, CONTROL_DT_S);

    mix_motor_output();
}
