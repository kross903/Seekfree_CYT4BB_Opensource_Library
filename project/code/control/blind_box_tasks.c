#include "control/blind_box_tasks.h"

#include "common/PIT_Demo.h"
#include "control/beacon_search.h"
#include "control/flight_control.h"
#include "control/height_manager.h"
#include "control/position_loop.h"
#include "control/throttle_runtime.h"
#include "control/velocity_loop.h"
#include "pmw/pmw_flow.h"
#include "vision/vehicle_marker_localization.h"
#include <math.h>
#include <string.h>

/* 盲盒任务统一入口：起飞后360度预扫 + 缓慢降落。 */
#define BLIND_BOX_TASK_STARTUP_SCAN_ENABLE          (0U)
#define BLIND_BOX_TASK_AUTO_LAND_ENABLE             (0U)

#define BLIND_BOX_TASK_STARTUP_WAIT_MS             (2000U)
#define BLIND_BOX_TASK_STARTUP_SCAN_ANGLE_DEG       (360.0f)
#define BLIND_BOX_TASK_STARTUP_SETTLE_MS           (1000U)
#define BLIND_BOX_TASK_STARTUP_YAW_RATE_DPS         (90.0f)
#define BLIND_BOX_TASK_STARTUP_MAX_YAW_STEP_DEG     (10.0f)

#define AUTO_LAND_CONFIRM_MS                    (5000U)
#define AUTO_LAND_STABLE_MS                      (500U)
#define AUTO_LAND_POSITION_GATE_CM               (30.0f)
#define AUTO_LAND_SPEED_GATE_CMS                 (20.0f)
#define AUTO_LAND_OPEN_LOOP_RATE_PER_S            (80.0f)
#define AUTO_LAND_OPEN_LOOP_STEP                 (AUTO_LAND_OPEN_LOOP_RATE_PER_S * 0.010f)
#define AUTO_LAND_OPEN_LOOP_TIMEOUT_MS         (10000U)

typedef enum
{
    BLIND_BOX_TASK_STARTUP_CONTROL_NONE = 0,
    BLIND_BOX_TASK_STARTUP_CONTROL_SPIN,
    BLIND_BOX_TASK_STARTUP_CONTROL_HOLD
} BlindBoxTasksStartupControl_t;

typedef enum
{
    BLIND_BOX_TASK_STARTUP_WAIT_READY = 0,
    BLIND_BOX_TASK_STARTUP_HOLD_BEFORE,
    BLIND_BOX_TASK_STARTUP_SPIN,
    BLIND_BOX_TASK_STARTUP_HOLD_AFTER,
    BLIND_BOX_TASK_STARTUP_COMPLETE
} BlindBoxTasksStartupPhase_t;

static volatile BlindBoxTasksStartupControl_t s_startup_control =
    BLIND_BOX_TASK_STARTUP_CONTROL_NONE;
static volatile BlindBoxTasksStartupPhase_t s_startup_phase =
    BLIND_BOX_TASK_STARTUP_WAIT_READY;
static volatile uint8 s_startup_turn_complete_event = 0U;
static uint16 s_startup_elapsed_ms = 0U;
static float s_startup_scan_start_yaw_deg = 0.0f;
static float s_startup_command_progress_deg = 0.0f;
static float s_startup_turn_progress_deg = 0.0f;

static volatile BlindBoxTasksState_t s_state = AUTO_LAND_IDLE;
static volatile uint8 s_trigger_reason = BEACON_SEARCH_FAILURE_NONE;
static volatile uint8 s_block_reason = AUTO_LAND_BLOCK_NONE;
static volatile uint8 s_disarm_request = 0U;
static volatile uint16 s_confirm_elapsed_ms = 0U;
static volatile uint16 s_stable_elapsed_ms = 0U;
static volatile uint16 s_open_loop_elapsed_ms = 0U;
static volatile float s_position_error_x_cm = 0.0f;
static volatile float s_position_error_y_cm = 0.0f;
static volatile float s_position_error_cm = 0.0f;
static volatile float s_horizontal_speed_cms = 0.0f;
static volatile float s_open_loop_throttle = 0.0f;
static volatile uint8 s_landing_point_valid = 0U;
static volatile uint8 s_fused_velocity_valid = 0U;
static volatile uint8 s_was_armed = 0U;

static uint16 blind_box_tasks_add_ms(uint16 value, uint16 elapsed_ms)
{
    if(value <= (uint16)(60000U - elapsed_ms))
    {
        return (uint16)(value + elapsed_ms);
    }
    return 60000U;
}

static uint16 blind_box_tasks_startup_add_ms(uint16 value, uint16 elapsed_ms)
{
    if(value <= (uint16)(60000U - elapsed_ms))
    {
        return (uint16)(value + elapsed_ms);
    }
    return 60000U;
}

static float blind_box_tasks_wrap_deg(float angle_deg)
{
    while(angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while(angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static uint8 blind_box_tasks_yaw_at_endpoint(float current_yaw_deg,
                                              float endpoint_yaw_deg)
{
    return (uint8)(fabsf(blind_box_tasks_wrap_deg(endpoint_yaw_deg -
                                                   current_yaw_deg)) <=
                   BLIND_BOX_TASK_STARTUP_MAX_YAW_STEP_DEG);
}

static uint8 blind_box_tasks_startup_is_active(void)
{
    return (uint8)(BLIND_BOX_TASK_STARTUP_SCAN_ENABLE &&
                   (((BLIND_BOX_TASK_STARTUP_WAIT_READY == s_startup_phase) &&
                     Flight_Takeoff_Is_Active()) ||
                    (BLIND_BOX_TASK_STARTUP_HOLD_BEFORE == s_startup_phase) ||
                    (BLIND_BOX_TASK_STARTUP_SPIN == s_startup_phase) ||
                    (BLIND_BOX_TASK_STARTUP_HOLD_AFTER == s_startup_phase)));
}

static void blind_box_tasks_startup_reset_runtime(void)
{
    s_startup_control = BLIND_BOX_TASK_STARTUP_CONTROL_NONE;
    s_startup_phase = BLIND_BOX_TASK_STARTUP_WAIT_READY;
    s_startup_turn_complete_event = 0U;
    s_startup_elapsed_ms = 0U;
    s_startup_scan_start_yaw_deg = 0.0f;
    s_startup_command_progress_deg = 0.0f;
    s_startup_turn_progress_deg = 0.0f;
}

static void blind_box_tasks_startup_tick(uint16 elapsed_ms)
{
#if BLIND_BOX_TASK_STARTUP_SCAN_ENABLE
    switch(s_startup_phase)
    {
    case BLIND_BOX_TASK_STARTUP_WAIT_READY:
        if(Flight_Takeoff_Is_Active())
        {
            s_startup_control = BLIND_BOX_TASK_STARTUP_CONTROL_HOLD;
            s_startup_elapsed_ms = 0U;
            s_startup_phase = BLIND_BOX_TASK_STARTUP_HOLD_BEFORE;
        }
        break;

    case BLIND_BOX_TASK_STARTUP_HOLD_BEFORE:
        s_startup_elapsed_ms = blind_box_tasks_startup_add_ms(
            s_startup_elapsed_ms, elapsed_ms);
        if(s_startup_elapsed_ms >= BLIND_BOX_TASK_STARTUP_WAIT_MS)
        {
            s_startup_elapsed_ms = 0U;
            s_startup_turn_complete_event = 0U;
            s_startup_control = BLIND_BOX_TASK_STARTUP_CONTROL_SPIN;
            s_startup_phase = BLIND_BOX_TASK_STARTUP_SPIN;
        }
        break;

    case BLIND_BOX_TASK_STARTUP_SPIN:
        if(s_startup_turn_complete_event)
        {
            s_startup_turn_complete_event = 0U;
            s_startup_elapsed_ms = 0U;
            s_startup_control = BLIND_BOX_TASK_STARTUP_CONTROL_HOLD;
            s_startup_phase = BLIND_BOX_TASK_STARTUP_HOLD_AFTER;
        }
        break;

    case BLIND_BOX_TASK_STARTUP_HOLD_AFTER:
        s_startup_elapsed_ms = blind_box_tasks_startup_add_ms(
            s_startup_elapsed_ms, elapsed_ms);
        if(s_startup_elapsed_ms >= BLIND_BOX_TASK_STARTUP_SETTLE_MS)
        {
            s_startup_phase = BLIND_BOX_TASK_STARTUP_COMPLETE;
            s_startup_control = BLIND_BOX_TASK_STARTUP_CONTROL_NONE;
            s_startup_elapsed_ms = 0U;
        }
        break;

    case BLIND_BOX_TASK_STARTUP_COMPLETE:
    default:
        break;
    }
#else
    (void)elapsed_ms;
#endif
}

static void blind_box_tasks_reset_runtime(void)
{
    s_state = AUTO_LAND_IDLE;
    s_trigger_reason = BEACON_SEARCH_FAILURE_NONE;
    s_block_reason = AUTO_LAND_BLOCK_NONE;
    s_disarm_request = 0U;
    s_confirm_elapsed_ms = 0U;
    s_stable_elapsed_ms = 0U;
    s_open_loop_elapsed_ms = 0U;
    s_position_error_x_cm = 0.0f;
    s_position_error_y_cm = 0.0f;
    s_position_error_cm = 0.0f;
    s_horizontal_speed_cms = 0.0f;
    s_open_loop_throttle = 0.0f;
    s_landing_point_valid = 0U;
    s_fused_velocity_valid = 0U;
    Position_Loop_End_Frozen_Hold();
    HeightManager_EndAutoLanding();
}

static uint8 blind_box_tasks_flight_eligible(void)
{
    return (uint8)(armed &&
                   (control_mode == CONTROL_MODE_ANGLE) &&
                   !Flight_Takeoff_Is_Active() &&
                   Velocity_Loop_Is_Enabled() &&
                   Position_Loop_Is_Enabled() &&
                   !emergency_stop_triggered);
}

static uint8 blind_box_tasks_update_alignment(uint16 elapsed_ms)
{
    VehicleMarkerLossDebug_t marker_loss = {0};
    PMWFlowState_t flow_state = {0};
    float follow_x_cm = 0.0f;
    float follow_y_cm = 0.0f;
    uint8 follow_valid;

    VehicleMarkerLocalization_GetLossDebug(&marker_loss);
    PMW_Flow_GetState(&flow_state);
    follow_valid = VehicleMarkerLocalization_GetFollowPoint(
        &g_vehicle_marker_pose, &follow_x_cm, &follow_y_cm, NULL, NULL);
    s_landing_point_valid = (uint8)(marker_loss.raw_detect_valid &&
                                     marker_loss.pose_valid &&
                                     !marker_loss.lost_flag &&
                                     follow_valid &&
                                     isfinite(follow_x_cm) &&
                                     isfinite(follow_y_cm));
    s_fused_velocity_valid = flow_state.fused_velocity_valid ? 1U : 0U;

    if(s_landing_point_valid)
    {
        s_position_error_x_cm = follow_x_cm;
        s_position_error_y_cm = follow_y_cm;
        s_position_error_cm = sqrtf(follow_x_cm * follow_x_cm +
                                    follow_y_cm * follow_y_cm);
    }
    else
    {
        s_position_error_x_cm = 0.0f;
        s_position_error_y_cm = 0.0f;
        s_position_error_cm = 0.0f;
    }
    if(s_fused_velocity_valid &&
       isfinite(flow_state.velocity_x_fused_cms) &&
       isfinite(flow_state.velocity_y_fused_cms))
    {
        s_horizontal_speed_cms = sqrtf(
            flow_state.velocity_x_fused_cms * flow_state.velocity_x_fused_cms +
            flow_state.velocity_y_fused_cms * flow_state.velocity_y_fused_cms);
    }
    else
    {
        s_horizontal_speed_cms = 0.0f;
        s_fused_velocity_valid = 0U;
    }

    if(s_landing_point_valid && s_fused_velocity_valid &&
       (s_position_error_cm <= AUTO_LAND_POSITION_GATE_CM) &&
       (s_horizontal_speed_cms <= AUTO_LAND_SPEED_GATE_CMS))
    {
        s_stable_elapsed_ms = blind_box_tasks_add_ms(s_stable_elapsed_ms,
                                                       elapsed_ms);
    }
    else
    {
        s_stable_elapsed_ms = 0U;
    }
    return (s_stable_elapsed_ms >= AUTO_LAND_STABLE_MS) ? 1U : 0U;
}

void BlindBoxTasks_Init(void)
{
    s_was_armed = 0U;
    blind_box_tasks_startup_reset_runtime();
    blind_box_tasks_reset_runtime();
}

void BlindBoxTasks_ControlStep(float dt_s,
                                float current_yaw_deg,
                                volatile float *target_yaw_deg)
{
#if BLIND_BOX_TASK_STARTUP_SCAN_ENABLE
    static BlindBoxTasksStartupControl_t last_control =
        BLIND_BOX_TASK_STARTUP_CONTROL_NONE;
    static float previous_yaw_deg = 0.0f;
    BlindBoxTasksStartupControl_t control = s_startup_control;

    if(NULL == target_yaw_deg)
    {
        return;
    }

    if(BLIND_BOX_TASK_STARTUP_CONTROL_NONE == control)
    {
        last_control = BLIND_BOX_TASK_STARTUP_CONTROL_NONE;
        return;
    }

    if(!blind_box_tasks_startup_is_active())
    {
        last_control = BLIND_BOX_TASK_STARTUP_CONTROL_NONE;
        return;
    }

    if(control != last_control)
    {
        if((BLIND_BOX_TASK_STARTUP_CONTROL_SPIN == control) ||
           (BLIND_BOX_TASK_STARTUP_CONTROL_HOLD == control))
        {
            *target_yaw_deg = current_yaw_deg;
            previous_yaw_deg = current_yaw_deg;
            if(BLIND_BOX_TASK_STARTUP_CONTROL_SPIN == control)
            {
                s_startup_scan_start_yaw_deg = current_yaw_deg;
                s_startup_command_progress_deg = 0.0f;
                s_startup_turn_progress_deg = 0.0f;
                s_startup_turn_complete_event = 0U;
            }
        }
        last_control = control;
    }

    if(BLIND_BOX_TASK_STARTUP_CONTROL_SPIN != control)
    {
        return;
    }

    {
        float yaw_delta_deg = blind_box_tasks_wrap_deg(current_yaw_deg -
                                                        previous_yaw_deg);
        float remaining_deg = BLIND_BOX_TASK_STARTUP_SCAN_ANGLE_DEG -
                              s_startup_command_progress_deg;
        float target_step_deg = BLIND_BOX_TASK_STARTUP_YAW_RATE_DPS * dt_s;

        previous_yaw_deg = current_yaw_deg;
        if(fabsf(yaw_delta_deg) <= BLIND_BOX_TASK_STARTUP_MAX_YAW_STEP_DEG)
        {
            s_startup_turn_progress_deg += yaw_delta_deg;
            if(s_startup_turn_progress_deg < 0.0f)
            {
                s_startup_turn_progress_deg = 0.0f;
            }
        }

        if(isfinite(target_step_deg) && (target_step_deg > 0.0f) &&
           (remaining_deg > 0.0f))
        {
            if(target_step_deg > remaining_deg)
            {
                target_step_deg = remaining_deg;
            }
            *target_yaw_deg = blind_box_tasks_wrap_deg(*target_yaw_deg +
                                                        target_step_deg);
            s_startup_command_progress_deg += target_step_deg;
        }
        else if(remaining_deg <= 0.0f)
        {
            *target_yaw_deg = s_startup_scan_start_yaw_deg;
        }

        if((s_startup_command_progress_deg >=
            BLIND_BOX_TASK_STARTUP_SCAN_ANGLE_DEG) &&
           (s_startup_turn_progress_deg >=
            (BLIND_BOX_TASK_STARTUP_SCAN_ANGLE_DEG -
             BLIND_BOX_TASK_STARTUP_MAX_YAW_STEP_DEG)) &&
           blind_box_tasks_yaw_at_endpoint(current_yaw_deg,
                                             s_startup_scan_start_yaw_deg))
        {
            s_startup_control = BLIND_BOX_TASK_STARTUP_CONTROL_HOLD;
            s_startup_turn_complete_event = 1U;
            *target_yaw_deg = s_startup_scan_start_yaw_deg;
            last_control = BLIND_BOX_TASK_STARTUP_CONTROL_HOLD;
        }
    }
#else
    (void)dt_s;
    (void)current_yaw_deg;
    (void)target_yaw_deg;
#endif
}

float BlindBoxTasks_LimitYawRate(float yaw_rate_dps)
{
    if(!isfinite(yaw_rate_dps))
    {
        return 0.0f;
    }
    if(!blind_box_tasks_startup_is_active())
    {
        return yaw_rate_dps;
    }
    if(yaw_rate_dps > BLIND_BOX_TASK_STARTUP_YAW_RATE_DPS)
    {
        return BLIND_BOX_TASK_STARTUP_YAW_RATE_DPS;
    }
    if(yaw_rate_dps < -BLIND_BOX_TASK_STARTUP_YAW_RATE_DPS)
    {
        return -BLIND_BOX_TASK_STARTUP_YAW_RATE_DPS;
    }
    return yaw_rate_dps;
}

uint8 BlindBoxTasks_IsBeaconSearchPaused(void)
{
    return blind_box_tasks_startup_is_active();
}

uint8 BlindBoxTasks_GetFrameFlags(void)
{
    return blind_box_tasks_startup_is_active() ?
           CORE_LINK_BEACON_FRAME_FLAG_SEARCH_ACTIVE : 0U;
}

void BlindBoxTasks_Tick(uint16 elapsed_ms)
{
#if BLIND_BOX_TASK_AUTO_LAND_ENABLE
    BeaconSearchDiag_t search_diag = {0};
#endif

    blind_box_tasks_startup_tick(elapsed_ms);

    if(!armed)
    {
        s_was_armed = 0U;
        blind_box_tasks_startup_reset_runtime();
#if BLIND_BOX_TASK_AUTO_LAND_ENABLE
        if(s_state != AUTO_LAND_COMPLETE)
        {
            blind_box_tasks_reset_runtime();
        }
        return;
#else
        if(s_state != AUTO_LAND_IDLE)
        {
            blind_box_tasks_reset_runtime();
        }
        return;
#endif
    }
    if(!s_was_armed)
    {
        blind_box_tasks_startup_reset_runtime();
#if BLIND_BOX_TASK_AUTO_LAND_ENABLE
        blind_box_tasks_reset_runtime();
#endif
        s_was_armed = 1U;
    }

#if BLIND_BOX_TASK_AUTO_LAND_ENABLE
    if(!blind_box_tasks_flight_eligible())
    {
        if(s_state < AUTO_LAND_DESCENDING)
        {
            blind_box_tasks_reset_runtime();
        }
        return;
    }

    BeaconSearch_GetDiag(&search_diag);
    if(s_state == AUTO_LAND_IDLE)
    {
        if((search_diag.state == BEACON_SEARCH_FAILED) &&
           (search_diag.failure_reason != BEACON_SEARCH_FAILURE_NONE))
        {
            s_state = AUTO_LAND_CONFIRMING;
            s_trigger_reason = search_diag.failure_reason;
            s_confirm_elapsed_ms = 0U;
            s_block_reason = AUTO_LAND_BLOCK_NONE;
        }
        return;
    }

    if((s_state == AUTO_LAND_CONFIRMING) ||
       (s_state == AUTO_LAND_ALIGNING) ||
       (s_state == AUTO_LAND_BLOCKED))
    {
        if(search_diag.state == BEACON_SEARCH_TRACKING)
        {
            blind_box_tasks_reset_runtime();
            return;
        }
    }

    if(s_state == AUTO_LAND_CONFIRMING)
    {
        s_confirm_elapsed_ms = blind_box_tasks_add_ms(s_confirm_elapsed_ms,
                                                        elapsed_ms);
        if(s_confirm_elapsed_ms >= AUTO_LAND_CONFIRM_MS)
        {
            s_state = AUTO_LAND_ALIGNING;
            s_stable_elapsed_ms = 0U;
        }
        return;
    }

    if((s_state == AUTO_LAND_ALIGNING) ||
       (s_state == AUTO_LAND_BLOCKED))
    {
        if(search_diag.state == BEACON_SEARCH_VERIFYING)
        {
            s_stable_elapsed_ms = 0U;
            return;
        }
        if(!blind_box_tasks_update_alignment(elapsed_ms))
        {
            if(!s_landing_point_valid)
            {
                s_state = AUTO_LAND_BLOCKED;
                s_block_reason = AUTO_LAND_BLOCK_NO_LAND_POINT;
            }
            else
            {
                s_state = AUTO_LAND_ALIGNING;
                s_block_reason = AUTO_LAND_BLOCK_NONE;
            }
            return;
        }
        if(!HeightManager_IsTofValid())
        {
            s_state = AUTO_LAND_BLOCKED;
            s_block_reason = AUTO_LAND_BLOCK_NO_TOF;
            return;
        }
        if(!Position_Loop_Begin_Frozen_Hold())
        {
            s_state = AUTO_LAND_BLOCKED;
            s_block_reason = AUTO_LAND_BLOCK_NO_LAND_POINT;
            return;
        }
        if(!HeightManager_BeginAutoLanding())
        {
            Position_Loop_End_Frozen_Hold();
            s_state = AUTO_LAND_BLOCKED;
            s_block_reason = AUTO_LAND_BLOCK_NO_TOF;
            return;
        }
        s_state = AUTO_LAND_DESCENDING;
        s_block_reason = AUTO_LAND_BLOCK_NONE;
        s_stable_elapsed_ms = AUTO_LAND_STABLE_MS;
    }
#endif
}

void BlindBoxTasks_HeightTick_10ms(void)
{
#if !BLIND_BOX_TASK_AUTO_LAND_ENABLE
    if(s_state != AUTO_LAND_IDLE)
    {
        blind_box_tasks_reset_runtime();
    }
    return;
#else
    if(s_state == AUTO_LAND_DESCENDING)
    {
        if(HeightManager_AutoLandingTouchdownConfirmed())
        {
            s_disarm_request = 1U;
            s_state = AUTO_LAND_COMPLETE;
            return;
        }
        if(!HeightManager_IsTofValid())
        {
            s_open_loop_throttle = base_throttle;
            if(s_open_loop_throttle < SAFE_MIN_THROTTLE)
            {
                s_open_loop_throttle = SAFE_MIN_THROTTLE;
            }
            s_open_loop_elapsed_ms = 0U;
            HeightManager_EndAutoLanding();
            s_state = AUTO_LAND_OPEN_LOOP;
        }
        return;
    }

    if(s_state == AUTO_LAND_OPEN_LOOP)
    {
        s_open_loop_elapsed_ms = blind_box_tasks_add_ms(
            s_open_loop_elapsed_ms, 10U);
        s_open_loop_throttle -= AUTO_LAND_OPEN_LOOP_STEP;
        if(s_open_loop_throttle <= SAFE_MIN_THROTTLE)
        {
            s_open_loop_throttle = SAFE_MIN_THROTTLE;
            s_disarm_request = 1U;
            s_state = AUTO_LAND_COMPLETE;
        }
        else if(s_open_loop_elapsed_ms >= AUTO_LAND_OPEN_LOOP_TIMEOUT_MS)
        {
            s_disarm_request = 1U;
            s_state = AUTO_LAND_COMPLETE;
        }
    }
#endif
}

uint8 BlindBoxTasks_IsActive(void)
{
    return (uint8)((s_state == AUTO_LAND_CONFIRMING) ||
                   (s_state == AUTO_LAND_ALIGNING) ||
                   (s_state == AUTO_LAND_DESCENDING) ||
                   (s_state == AUTO_LAND_OPEN_LOOP) ||
                   (s_state == AUTO_LAND_BLOCKED));
}

uint8 BlindBoxTasks_TakeDisarmRequest(void)
{
    uint8 request;
    uint32 irq_state = __get_PRIMASK();

    __disable_irq();
    request = s_disarm_request;
    s_disarm_request = 0U;
    if(irq_state == 0U)
    {
        __enable_irq();
    }
    return request;
}

uint8 BlindBoxTasks_GetThrottleOverride(float *throttle)
{
    if((throttle == NULL) || (s_state != AUTO_LAND_OPEN_LOOP))
    {
        return 0U;
    }
    *throttle = s_open_loop_throttle;
    return 1U;
}

void BlindBoxTasks_GetDiag(BlindBoxTasksDiag_t *diag)
{
    HeightTofSnapshot_t tof_snapshot;
    uint32 irq_state;

    if(diag == NULL)
    {
        return;
    }
    irq_state = __get_PRIMASK();
    __disable_irq();
    HeightManager_GetTofSnapshot(&tof_snapshot);
    memset(diag, 0, sizeof(*diag));
    diag->state = (uint8)s_state;
    diag->trigger_reason = s_trigger_reason;
    diag->block_reason = s_block_reason;
    diag->landing_point_valid = s_landing_point_valid;
    diag->fused_velocity_valid = s_fused_velocity_valid;
    diag->tof_valid = HeightManager_IsTofValid();
    diag->tof_state = tof_snapshot.state;
    diag->confirm_elapsed_ms = s_confirm_elapsed_ms;
    diag->stable_elapsed_ms = s_stable_elapsed_ms;
    diag->open_loop_elapsed_ms = s_open_loop_elapsed_ms;
    diag->position_error_x_cm = s_position_error_x_cm;
    diag->position_error_y_cm = s_position_error_y_cm;
    diag->position_error_cm = s_position_error_cm;
    diag->horizontal_speed_cms = s_horizontal_speed_cms;
    diag->target_height_cm = HeightManager_GetAutoLandingTarget();
    diag->open_loop_throttle = s_open_loop_throttle;
    if(irq_state == 0U)
    {
        __enable_irq();
    }
}
