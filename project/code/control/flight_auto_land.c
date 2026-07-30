#include "control/flight_auto_land.h"

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

#define AUTO_LAND_CONFIRM_MS                    (5000U)
#define AUTO_LAND_STABLE_MS                      (500U)
#define AUTO_LAND_POSITION_GATE_CM               (30.0f)
#define AUTO_LAND_SPEED_GATE_CMS                 (20.0f)
#define AUTO_LAND_OPEN_LOOP_RATE_PER_S            (80.0f)
#define AUTO_LAND_OPEN_LOOP_STEP                 (AUTO_LAND_OPEN_LOOP_RATE_PER_S * 0.010f)
#define AUTO_LAND_OPEN_LOOP_TIMEOUT_MS         (10000U)

static volatile FlightAutoLandState_t s_state = AUTO_LAND_IDLE;
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

static uint16 flight_auto_land_add_ms(uint16 value, uint16 elapsed_ms)
{
    if(value <= (uint16)(60000U - elapsed_ms))
    {
        return (uint16)(value + elapsed_ms);
    }
    return 60000U;
}

static void flight_auto_land_reset_runtime(void)
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

static uint8 flight_auto_land_flight_eligible(void)
{
    return (uint8)(armed &&
                   (control_mode == CONTROL_MODE_ANGLE) &&
                   !Flight_Takeoff_Is_Active() &&
                   Velocity_Loop_Is_Enabled() &&
                   Position_Loop_Is_Enabled() &&
                   !emergency_stop_triggered);
}

static uint8 flight_auto_land_update_alignment(uint16 elapsed_ms)
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
        s_stable_elapsed_ms = flight_auto_land_add_ms(s_stable_elapsed_ms,
                                                       elapsed_ms);
    }
    else
    {
        s_stable_elapsed_ms = 0U;
    }
    return (s_stable_elapsed_ms >= AUTO_LAND_STABLE_MS) ? 1U : 0U;
}

void FlightAutoLand_Init(void)
{
    s_was_armed = 0U;
    flight_auto_land_reset_runtime();
}

void FlightAutoLand_Tick(uint16 elapsed_ms)
{
    BeaconSearchDiag_t search_diag = {0};

    if(!armed)
    {
        s_was_armed = 0U;
        if(s_state != AUTO_LAND_COMPLETE)
        {
            flight_auto_land_reset_runtime();
        }
        return;
    }
    if(!s_was_armed)
    {
        flight_auto_land_reset_runtime();
        s_was_armed = 1U;
    }

    if(!flight_auto_land_flight_eligible())
    {
        if(s_state < AUTO_LAND_DESCENDING)
        {
            flight_auto_land_reset_runtime();
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
            flight_auto_land_reset_runtime();
            return;
        }
    }

    if(s_state == AUTO_LAND_CONFIRMING)
    {
        s_confirm_elapsed_ms = flight_auto_land_add_ms(s_confirm_elapsed_ms,
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
        if(!flight_auto_land_update_alignment(elapsed_ms))
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
}

void FlightAutoLand_HeightTick_10ms(void)
{
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
        s_open_loop_elapsed_ms = flight_auto_land_add_ms(
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
}

uint8 FlightAutoLand_IsActive(void)
{
    return (uint8)((s_state == AUTO_LAND_CONFIRMING) ||
                   (s_state == AUTO_LAND_ALIGNING) ||
                   (s_state == AUTO_LAND_DESCENDING) ||
                   (s_state == AUTO_LAND_OPEN_LOOP) ||
                   (s_state == AUTO_LAND_BLOCKED));
}

uint8 FlightAutoLand_TakeDisarmRequest(void)
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

uint8 FlightAutoLand_GetThrottleOverride(float *throttle)
{
    if((throttle == NULL) || (s_state != AUTO_LAND_OPEN_LOOP))
    {
        return 0U;
    }
    *throttle = s_open_loop_throttle;
    return 1U;
}

void FlightAutoLand_GetDiag(FlightAutoLandDiag_t *diag)
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
