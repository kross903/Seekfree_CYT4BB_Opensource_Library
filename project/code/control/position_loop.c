#include "control/position_loop.h"
#include "vision/detection/ir_target_localization.h"

#define POSITION_LOOP_TARGET_X_CM      (0.0f)
#define POSITION_LOOP_TARGET_Y_CM      (0.0f)
/* Visual position input now uses X-forward positive, Y-left positive.
 * PID_Calculate() uses error = target - current, so the loop output must flip
 * sign to keep a front/left target commanding positive front/left velocity. */
#define POSITION_LOOP_VX_SIGN          (1.0f)
#define POSITION_LOOP_VY_SIGN          (1.0f)
#define POSITION_LOOP_DEADZONE_CM      (3.0f)
#define POSITION_LOOP_PREDICT_RELEASE_EPS_CM (1.0f)
#define POSITION_LOOP_DT_S             0.00025f

PID_Controller pos_pid_x;
PID_Controller pos_pid_y;

static volatile uint8 g_position_loop_enabled        = 0U;
static volatile uint8 g_position_loop_active         = 0U;
static volatile uint8 g_position_loop_ir_track_valid = 0U;
static volatile uint8 g_position_loop_velocity_ready = 0U;
static volatile float g_position_loop_fb_x_cm        = 0.0f;
static volatile float g_position_loop_fb_y_cm        = 0.0f;
static volatile float g_position_loop_error_x_cm     = 0.0f;
static volatile float g_position_loop_error_y_cm     = 0.0f;
static volatile float g_position_loop_vx_cmd_cms     = 0.0f;
static volatile float g_position_loop_vy_cmd_cms     = 0.0f;
static volatile float g_position_loop_x_i_out        = 0.0f;
static volatile float g_position_loop_y_i_out        = 0.0f;
static uint8 g_position_loop_last_pose_valid = 0U;
static uint8 g_position_loop_predict_hold_active = 0U;
static float g_position_loop_last_x_cm = 0.0f;
static float g_position_loop_last_y_cm = 0.0f;
static float g_position_loop_hold_start_x_cm = 0.0f;
static float g_position_loop_hold_start_y_cm = 0.0f;
static float g_position_loop_hold_distance_cm = 0.0f;
static float g_position_loop_hold_dx_cm = 0.0f;
static float g_position_loop_hold_dy_cm = 0.0f;

static void position_loop_set_output(float vx_cms, float vy_cms, float *target_vx_cms, float *target_vy_cms)
{
    if(target_vx_cms != NULL)
    {
        *target_vx_cms = vx_cms;
    }
    if(target_vy_cms != NULL)
    {
        *target_vy_cms = vy_cms;
    }
}

static void position_loop_reset_runtime(void)
{
    PID_Reset(&pos_pid_x);
    PID_Reset(&pos_pid_y);
    g_position_loop_active = 0U;
    g_position_loop_vx_cmd_cms = 0.0f;
    g_position_loop_vy_cmd_cms = 0.0f;
    g_position_loop_error_x_cm = 0.0f;
    g_position_loop_error_y_cm = 0.0f;
    g_position_loop_x_i_out = 0.0f;
    g_position_loop_y_i_out = 0.0f;
    g_position_loop_last_pose_valid = 0U;
    g_position_loop_predict_hold_active = 0U;
    g_position_loop_last_x_cm = 0.0f;
    g_position_loop_last_y_cm = 0.0f;
    g_position_loop_hold_start_x_cm = 0.0f;
    g_position_loop_hold_start_y_cm = 0.0f;
    g_position_loop_hold_distance_cm = 0.0f;
    g_position_loop_hold_dx_cm = 0.0f;
    g_position_loop_hold_dy_cm = 0.0f;
}

static void position_loop_reset_predict_hold(void)
{
    g_position_loop_predict_hold_active = 0U;
    g_position_loop_hold_start_x_cm = 0.0f;
    g_position_loop_hold_start_y_cm = 0.0f;
    g_position_loop_hold_distance_cm = 0.0f;
    g_position_loop_hold_dx_cm = 0.0f;
    g_position_loop_hold_dy_cm = 0.0f;
}

static float position_loop_apply_deadzone(float error_cm)
{
    if(error_cm >= -POSITION_LOOP_DEADZONE_CM && error_cm <= POSITION_LOOP_DEADZONE_CM)
    {
        return 0.0f;
    }

    return error_cm;
}

static uint8 position_loop_get_raw_ir_track_valid(void)
{
    IR_LossDebug_t ir_loss_debug = {0};

    IR_Localization_GetLossDebug(&ir_loss_debug);

    return (uint8)((ir_loss_debug.raw_detect_valid && ir_pose_estimate.valid && !ir_position_lost_flag) ? 1U : 0U);
}

static uint8 position_loop_start_predict_hold(void)
{
    if(!g_position_loop_last_pose_valid)
    {
        return 0U;
    }

    g_position_loop_hold_start_x_cm = g_position_loop_last_x_cm;
    g_position_loop_hold_start_y_cm = g_position_loop_last_y_cm;
    g_position_loop_hold_distance_cm = sqrtf(g_position_loop_hold_start_x_cm * g_position_loop_hold_start_x_cm +
                                             g_position_loop_hold_start_y_cm * g_position_loop_hold_start_y_cm);

    if(g_position_loop_hold_distance_cm <= (POSITION_LOOP_DEADZONE_CM + POSITION_LOOP_PREDICT_RELEASE_EPS_CM))
    {
        position_loop_reset_predict_hold();
        return 0U;
    }

    g_position_loop_hold_dx_cm = 0.0f;
    g_position_loop_hold_dy_cm = 0.0f;
    g_position_loop_predict_hold_active = 1U;

    return 1U;
}

static uint8 position_loop_update_predict_hold(float measured_vx_cms,
                                               float measured_vy_cms,
                                               float *predict_x_cm,
                                               float *predict_y_cm)
{
    float displacement_distance_cm;
    float predicted_x_cm;
    float predicted_y_cm;
    float predicted_distance_cm;

    if((predict_x_cm == NULL) || (predict_y_cm == NULL))
    {
        return 0U;
    }

    if(!g_position_loop_predict_hold_active)
    {
        if(!position_loop_start_predict_hold())
        {
            return 0U;
        }
    }

    g_position_loop_hold_dx_cm += measured_vx_cms * POSITION_LOOP_DT_S;
    g_position_loop_hold_dy_cm += measured_vy_cms * POSITION_LOOP_DT_S;
    displacement_distance_cm = sqrtf(g_position_loop_hold_dx_cm * g_position_loop_hold_dx_cm +
                                     g_position_loop_hold_dy_cm * g_position_loop_hold_dy_cm);
    predicted_x_cm = g_position_loop_hold_start_x_cm - g_position_loop_hold_dx_cm;
    predicted_y_cm = g_position_loop_hold_start_y_cm - g_position_loop_hold_dy_cm;
    predicted_distance_cm = sqrtf(predicted_x_cm * predicted_x_cm + predicted_y_cm * predicted_y_cm);

    if((displacement_distance_cm >= (g_position_loop_hold_distance_cm - POSITION_LOOP_PREDICT_RELEASE_EPS_CM)) ||
       (predicted_distance_cm <= POSITION_LOOP_DEADZONE_CM))
    {
        g_position_loop_last_pose_valid = 0U;
        position_loop_reset_predict_hold();
        return 0U;
    }

    *predict_x_cm = predicted_x_cm;
    *predict_y_cm = predicted_y_cm;

    return 1U;
}

void Position_Loop_Set_Enabled(uint8 enabled)
{
    if(enabled)
    {
        g_position_loop_enabled = 1U;
        position_loop_reset_runtime();
    }
    else
    {
        g_position_loop_enabled = 0U;
        g_position_loop_ir_track_valid = 0U;
        g_position_loop_velocity_ready = 0U;
        g_position_loop_fb_x_cm = 0.0f;
        g_position_loop_fb_y_cm = 0.0f;
        position_loop_reset_runtime();
    }
}

uint8 Position_Loop_Is_Enabled(void)
{
    return g_position_loop_enabled;
}

void Position_Loop_Update_Velocity_Target(uint8 velocity_ready,
                                          float measured_vx_cms,
                                          float measured_vy_cms,
                                          float *target_vx_cms,
                                          float *target_vy_cms)
{
    uint8 raw_ir_track_valid = position_loop_get_raw_ir_track_valid();
    uint8 position_input_valid = 0U;
    float position_input_x_cm = 0.0f;
    float position_input_y_cm = 0.0f;
    float pos_error_x_cm;
    float pos_error_y_cm;

    if(!g_position_loop_enabled || !velocity_ready)
    {
        g_position_loop_ir_track_valid = 0U;
        g_position_loop_velocity_ready = velocity_ready;
        g_position_loop_fb_x_cm = 0.0f;
        g_position_loop_fb_y_cm = 0.0f;
        g_position_loop_error_x_cm = 0.0f;
        g_position_loop_error_y_cm = 0.0f;
        position_loop_set_output(0.0f, 0.0f, target_vx_cms, target_vy_cms);
        position_loop_reset_runtime();
        return;
    }

    if(raw_ir_track_valid)
    {
        position_input_valid = 1U;
        position_input_x_cm = ir_pose_estimate.x;
        position_input_y_cm = ir_pose_estimate.y;
        g_position_loop_last_pose_valid = 1U;
        g_position_loop_last_x_cm = position_input_x_cm;
        g_position_loop_last_y_cm = position_input_y_cm;
        position_loop_reset_predict_hold();
    }
    else
    {
        position_input_valid = position_loop_update_predict_hold(measured_vx_cms,
                                                                 measured_vy_cms,
                                                                 &position_input_x_cm,
                                                                 &position_input_y_cm);
    }

    pos_error_x_cm = position_loop_apply_deadzone(position_input_x_cm);
    pos_error_y_cm = position_loop_apply_deadzone(position_input_y_cm);

    g_position_loop_ir_track_valid = position_input_valid;
    g_position_loop_velocity_ready = velocity_ready;
    g_position_loop_fb_x_cm = position_input_x_cm;
    g_position_loop_fb_y_cm = position_input_y_cm;
    g_position_loop_error_x_cm = pos_error_x_cm;
    g_position_loop_error_y_cm = pos_error_y_cm;

    if(!position_input_valid)
    {
        position_loop_set_output(0.0f, 0.0f, target_vx_cms, target_vy_cms);
        position_loop_reset_runtime();
        return;
    }

    if(pos_error_x_cm == 0.0f)
    {
        PID_Reset(&pos_pid_x);
    }
    if(pos_error_y_cm == 0.0f)
    {
        PID_Reset(&pos_pid_y);
    }

    g_position_loop_vx_cmd_cms = POSITION_LOOP_VX_SIGN *
                                 PID_Calculate(&pos_pid_x,
                                               pos_error_x_cm,
                                               POSITION_LOOP_TARGET_X_CM,
                                               POSITION_LOOP_DT_S);
    g_position_loop_vy_cmd_cms = POSITION_LOOP_VY_SIGN *
                                 PID_Calculate(&pos_pid_y,
                                               pos_error_y_cm,
                                               POSITION_LOOP_TARGET_Y_CM,
                                               POSITION_LOOP_DT_S);

    position_loop_set_output(g_position_loop_vx_cmd_cms, g_position_loop_vy_cmd_cms, target_vx_cms, target_vy_cms);
    g_position_loop_x_i_out = pos_pid_x.Ki * pos_pid_x.integral;
    g_position_loop_y_i_out = pos_pid_y.Ki * pos_pid_y.integral;
    g_position_loop_active = 1U;
}

void Position_Loop_Get_Diag(PositionLoopDiag_t *diag)
{
    if(diag == NULL)
    {
        return;
    }

    diag->target_x_cm = POSITION_LOOP_TARGET_X_CM;
    diag->target_y_cm = POSITION_LOOP_TARGET_Y_CM;
    diag->fb_x_cm = g_position_loop_fb_x_cm;
    diag->fb_y_cm = g_position_loop_fb_y_cm;
    diag->error_x_cm = g_position_loop_error_x_cm;
    diag->error_y_cm = g_position_loop_error_y_cm;
    diag->vx_cmd_cms = g_position_loop_vx_cmd_cms;
    diag->vy_cmd_cms = g_position_loop_vy_cmd_cms;
    diag->x_i_out = g_position_loop_x_i_out;
    diag->y_i_out = g_position_loop_y_i_out;
    diag->enabled = g_position_loop_enabled;
    diag->active = g_position_loop_active;
    diag->ir_track_valid = g_position_loop_ir_track_valid;
    diag->velocity_ready = g_position_loop_velocity_ready;
}
