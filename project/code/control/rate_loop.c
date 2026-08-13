#include "control/rate_loop.h"
#include "control/motor_mixer.h"

#define RATE_LOOP_DT_S                  0.001f
#define RATE_LOOP_AXIS_COUNT            3U
#define RATE_LOOP_AXIS_ROLL             0U
#define RATE_LOOP_AXIS_PITCH            1U
#define RATE_LOOP_AXIS_YAW              2U

PID_Controller gyro_pid_roll;
PID_Controller gyro_pid_pitch;
PID_Controller gyro_pid_yaw;

float roll_output = 0.0f;
float pitch_output = 0.0f;
float yaw_output = 0.0f;

static volatile RateLoopDiag_t s_rate_loop_diag[RATE_LOOP_AXIS_COUNT];

static float rate_loop_update_axis(PID_Controller *pid,
                                   float target_rate,
                                   float actual_rate,
                                   float output_sign,
                                   uint8 axis,
                                   uint8 mixer_limit_flags)
{
    RateLoopDiag_t diag = {0};
    float controller_output;
    float integral_before = pid->integral;
    uint8 pid_limit_flags = mixer_limit_flags;

    /* Yaw控制器输出进入混控前反号，混控方向限幅需映射回PID输出方向。 */
    if(output_sign < 0.0f)
    {
        pid_limit_flags = 0U;
        if(mixer_limit_flags & MOTOR_MIXER_LIMIT_POSITIVE)
        {
            pid_limit_flags |= MOTOR_MIXER_LIMIT_NEGATIVE;
        }
        if(mixer_limit_flags & MOTOR_MIXER_LIMIT_NEGATIVE)
        {
            pid_limit_flags |= MOTOR_MIXER_LIMIT_POSITIVE;
        }
    }

    controller_output = PID_CalculateLimited(
        pid,
        target_rate,
        actual_rate,
        RATE_LOOP_DT_S,
        (pid_limit_flags & MOTOR_MIXER_LIMIT_NEGATIVE) != 0U,
        (pid_limit_flags & MOTOR_MIXER_LIMIT_POSITIVE) != 0U);

    diag.target_rate = target_rate;
    diag.actual_rate = actual_rate;
    diag.error = target_rate - actual_rate;
    diag.p_output = output_sign * pid->Kp * diag.error;
    diag.i_output = output_sign * pid->Ki * pid->integral;
    diag.d_output = output_sign * pid->Kd * pid->derivative;
    diag.raw_output = diag.p_output + diag.i_output + diag.d_output;
    diag.output = output_sign * controller_output;
    diag.limit_flags = mixer_limit_flags;
    diag.integrator_held = (uint8)((pid->integral == integral_before) &&
                                  (((pid_limit_flags & MOTOR_MIXER_LIMIT_POSITIVE) &&
                                    (diag.error > 0.0f)) ||
                                   ((pid_limit_flags & MOTOR_MIXER_LIMIT_NEGATIVE) &&
                                    (diag.error < 0.0f))));
    diag.saturated = ((diag.raw_output > pid->output_limit) ||
                      (diag.raw_output < -pid->output_limit) ||
                      (mixer_limit_flags != 0U)) ? 1U : 0U;

    s_rate_loop_diag[axis] = diag;
    return diag.output;
}

void Rate_Loop_Reset_Output(void)
{
    RateLoopDiag_t empty_diag = {0};

    roll_output = 0.0f;
    pitch_output = 0.0f;
    yaw_output = 0.0f;
    s_rate_loop_diag[RATE_LOOP_AXIS_ROLL] = empty_diag;
    s_rate_loop_diag[RATE_LOOP_AXIS_PITCH] = empty_diag;
    s_rate_loop_diag[RATE_LOOP_AXIS_YAW] = empty_diag;
}

void Rate_Loop_Update(float target_roll_rate,
                      float target_pitch_rate,
                      float target_yaw_rate,
                      float current_gyro_roll,
                      float current_gyro_pitch,
                      float current_gyro_yaw)
{
    MotorMixerLimitDiag_t mixer_diag = {0};

    Motor_Mixer_Get_Limit_Diag(&mixer_diag);
    roll_output = rate_loop_update_axis(&gyro_pid_roll,
                                        target_roll_rate,
                                        current_gyro_roll,
                                        1.0f,
                                        RATE_LOOP_AXIS_ROLL,
                                        mixer_diag.roll_limit_flags);
    pitch_output = rate_loop_update_axis(&gyro_pid_pitch,
                                         target_pitch_rate,
                                         current_gyro_pitch,
                                         1.0f,
                                         RATE_LOOP_AXIS_PITCH,
                                         mixer_diag.pitch_limit_flags);
    yaw_output = rate_loop_update_axis(&gyro_pid_yaw,
                                       target_yaw_rate,
                                       current_gyro_yaw,
                                       -1.0f,
                                       RATE_LOOP_AXIS_YAW,
                                       mixer_diag.yaw_limit_flags);
}

uint8 Rate_Loop_Get_Diag(uint8 axis, RateLoopDiag_t *diag)
{
    uint32 irq_state;

    if((axis >= RATE_LOOP_AXIS_COUNT) || (diag == NULL))
    {
        return 0U;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();
    *diag = s_rate_loop_diag[axis];
    if(irq_state == 0U)
    {
        __enable_irq();
    }

    return 1U;
}
