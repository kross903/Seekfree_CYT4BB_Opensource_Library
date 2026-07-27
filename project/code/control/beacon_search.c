#include "common/project_headfile.h"

#include <math.h>
#include <string.h>

#define BEACON_SEARCH_LOSS_DELAY_MS             (600U)
#define BEACON_SEARCH_VERIFY_TIMEOUT_MS         (300U)
#define BEACON_SEARCH_MAX_DURATION_MS         (18000U)
#define BEACON_SEARCH_CONFIRM_FRAMES              (3U)
#define BEACON_SEARCH_MIN_CONFIDENCE              (96U)
#define BEACON_SEARCH_MIN_SIGMA_CM                 (5U)
#define BEACON_SEARCH_MAX_SIGMA_CM                (15U)
#define BEACON_SEARCH_YAW_RATE_DPS              (30.0f)
#define BEACON_SEARCH_FULL_TURN_DEG             (360.0f)
#define BEACON_SEARCH_MAX_YAW_STEP_DEG           (10.0f)

typedef enum
{
    BEACON_SEARCH_CONTROL_NONE = 0,
    BEACON_SEARCH_CONTROL_SPIN,
    BEACON_SEARCH_CONTROL_HOLD
} BeaconSearchControl_t;

static volatile BeaconSearchState_t s_state = BEACON_SEARCH_DISABLED;
static volatile BeaconSearchControl_t s_control = BEACON_SEARCH_CONTROL_NONE;
static volatile uint8 s_turn_complete_event = 0U;
static volatile uint8 s_search_generation = 0U;
static volatile float s_current_yaw_deg = 0.0f;
static volatile float s_target_yaw_deg = 0.0f;
static volatile float s_turn_progress_deg = 0.0f;

static uint8 s_was_armed = 0U;
static uint8 s_eligible = 0U;
static uint8 s_ever_seen = 0U;
static uint8 s_candidate_track_id = 0U;
static uint8 s_candidate_confirm_count = 0U;
static uint8 s_verify_from_failed = 0U;
static uint8 s_process_count_valid = 0U;
static uint32 s_last_process_count = 0U;
static uint16 s_loss_age_ms = 0U;
static uint16 s_search_elapsed_ms = 0U;
static uint16 s_verify_elapsed_ms = 0U;
static uint16 s_car_link_age_ms = 60000U;
static BeaconSearchFailure_t s_failure_reason = BEACON_SEARCH_FAILURE_NONE;

static uint16 beacon_search_add_ms(uint16 value, uint16 elapsed_ms)
{
    if(value <= (uint16)(60000U - elapsed_ms))
    {
        return (uint16)(value + elapsed_ms);
    }
    return 60000U;
}

static float beacon_search_wrap_deg(float angle_deg)
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

static void beacon_search_clear_candidate(void)
{
    s_candidate_track_id = 0U;
    s_candidate_confirm_count = 0U;
    s_verify_elapsed_ms = 0U;
    s_verify_from_failed = 0U;
}

static void beacon_search_reset_session(void)
{
    s_state = BEACON_SEARCH_DISABLED;
    s_control = BEACON_SEARCH_CONTROL_NONE;
    s_turn_complete_event = 0U;
    s_eligible = 0U;
    s_ever_seen = 0U;
    s_loss_age_ms = 0U;
    s_search_elapsed_ms = 0U;
    s_failure_reason = BEACON_SEARCH_FAILURE_NONE;
    s_process_count_valid = 0U;
    s_last_process_count = 0U;
    beacon_search_clear_candidate();
}

static void beacon_search_begin(void)
{
    s_state = BEACON_SEARCH_SPINNING;
    s_control = BEACON_SEARCH_CONTROL_SPIN;
    s_search_elapsed_ms = 0U;
    s_failure_reason = BEACON_SEARCH_FAILURE_NONE;
    s_turn_complete_event = 0U;
    s_search_generation++;
    beacon_search_clear_candidate();
}

static void beacon_search_fail(BeaconSearchFailure_t reason)
{
    s_state = BEACON_SEARCH_FAILED;
    s_control = BEACON_SEARCH_CONTROL_HOLD;
    s_failure_reason = reason;
    beacon_search_clear_candidate();
}

static uint8 beacon_search_observation_is_trusted(
    const BeaconObservation *observation)
{
    if(NULL == observation)
    {
        return 0U;
    }

    return (uint8)((observation->track_id != 0U) &&
                   (observation->flags & CORE_LINK_BEACON_OBSERVATION_FLAG_LIT) &&
                   (observation->detection_confidence >=
                    BEACON_SEARCH_MIN_CONFIDENCE) &&
                   (observation->position_sigma_cm >=
                    BEACON_SEARCH_MIN_SIGMA_CM) &&
                   (observation->position_sigma_cm <=
                    BEACON_SEARCH_MAX_SIGMA_CM));
}

static uint8 beacon_search_select_trusted_track(
    const core_link_beacon_observation_packet_struct *packet)
{
    uint8 first_track_id = 0U;
    uint8 i;

    if(NULL == packet)
    {
        return 0U;
    }

    for(i = 0U; i < packet->count; ++i)
    {
        if(!beacon_search_observation_is_trusted(&packet->observation[i]))
        {
            continue;
        }
        if(0U == first_track_id)
        {
            first_track_id = packet->observation[i].track_id;
        }
        if(packet->observation[i].track_id == s_candidate_track_id)
        {
            return s_candidate_track_id;
        }
    }

    return first_track_id;
}

static uint8 beacon_search_confirm_candidate(uint8 track_id)
{
    if(0U == track_id)
    {
        s_candidate_track_id = 0U;
        s_candidate_confirm_count = 0U;
        return 0U;
    }

    if(track_id == s_candidate_track_id)
    {
        if(s_candidate_confirm_count < 255U)
        {
            s_candidate_confirm_count++;
        }
    }
    else
    {
        s_candidate_track_id = track_id;
        s_candidate_confirm_count = 1U;
    }

    return (s_candidate_confirm_count >= BEACON_SEARCH_CONFIRM_FRAMES) ? 1U : 0U;
}

void BeaconSearch_Init(void)
{
    s_was_armed = 0U;
    s_car_link_age_ms = 60000U;
    s_current_yaw_deg = 0.0f;
    s_target_yaw_deg = 0.0f;
    s_turn_progress_deg = 0.0f;
    s_search_generation = 0U;
    beacon_search_reset_session();
}

void BeaconSearch_Tick(uint16 elapsed_ms,
                       uint8 armed_state,
                       uint8 angle_mode,
                       uint8 flight_ready,
                       uint8 velocity_loop_enabled,
                       uint8 car_link_valid,
                       uint16 car_link_age_ms)
{
    uint8 eligible;

    s_car_link_age_ms = car_link_age_ms;
    if(!armed_state)
    {
        s_was_armed = 0U;
        beacon_search_reset_session();
        return;
    }

    if(!s_was_armed)
    {
        beacon_search_reset_session();
        s_was_armed = 1U;
    }

    eligible = (uint8)(angle_mode && flight_ready && velocity_loop_enabled);
    s_eligible = eligible;
    if(!eligible)
    {
        s_state = BEACON_SEARCH_DISABLED;
        s_control = BEACON_SEARCH_CONTROL_NONE;
        s_loss_age_ms = 0U;
        s_search_elapsed_ms = 0U;
        s_failure_reason = BEACON_SEARCH_FAILURE_NONE;
        beacon_search_clear_candidate();
        return;
    }

    if(BEACON_SEARCH_DISABLED == s_state)
    {
        s_state = s_ever_seen ? BEACON_SEARCH_TRACKING :
                                BEACON_SEARCH_WAIT_FIRST;
        s_loss_age_ms = 0U;
    }

    if(s_turn_complete_event)
    {
        s_turn_complete_event = 0U;
        beacon_search_fail(BEACON_SEARCH_FAILURE_FULL_TURN);
        return;
    }

    if((BEACON_SEARCH_SPINNING == s_state) ||
       (BEACON_SEARCH_VERIFYING == s_state))
    {
        if(!car_link_valid)
        {
            beacon_search_fail(BEACON_SEARCH_FAILURE_CAR_LINK);
            return;
        }
    }

    if((BEACON_SEARCH_SPINNING == s_state) ||
       ((BEACON_SEARCH_VERIFYING == s_state) && !s_verify_from_failed))
    {
        s_search_elapsed_ms = beacon_search_add_ms(s_search_elapsed_ms,
                                                   elapsed_ms);
        if(s_search_elapsed_ms >= BEACON_SEARCH_MAX_DURATION_MS)
        {
            beacon_search_fail(BEACON_SEARCH_FAILURE_TIMEOUT);
            return;
        }
    }

    if(BEACON_SEARCH_VERIFYING == s_state)
    {
        s_verify_elapsed_ms = beacon_search_add_ms(s_verify_elapsed_ms,
                                                   elapsed_ms);
        if(s_verify_elapsed_ms >= BEACON_SEARCH_VERIFY_TIMEOUT_MS)
        {
            if(s_verify_from_failed)
            {
                beacon_search_fail(s_failure_reason);
            }
            else
            {
                s_state = BEACON_SEARCH_SPINNING;
                s_control = BEACON_SEARCH_CONTROL_SPIN;
                beacon_search_clear_candidate();
            }
        }
        return;
    }

    if((BEACON_SEARCH_TRACKING == s_state) ||
       (BEACON_SEARCH_LOSS_WAIT == s_state))
    {
        s_loss_age_ms = beacon_search_add_ms(s_loss_age_ms, elapsed_ms);
        s_state = BEACON_SEARCH_LOSS_WAIT;
        if(s_loss_age_ms >= BEACON_SEARCH_LOSS_DELAY_MS)
        {
            if(car_link_valid)
            {
                beacon_search_begin();
            }
            else
            {
                beacon_search_fail(BEACON_SEARCH_FAILURE_CAR_LINK);
            }
        }
    }
}

void BeaconSearch_OnObservationFrame(
    const core_link_beacon_observation_packet_struct *packet,
    uint32 vision_process_count)
{
    uint8 trusted_track_id;

    if((NULL == packet) || !s_eligible)
    {
        return;
    }
    if(s_process_count_valid &&
       (vision_process_count == s_last_process_count))
    {
        return;
    }
    s_process_count_valid = 1U;
    s_last_process_count = vision_process_count;
    trusted_track_id = beacon_search_select_trusted_track(packet);

    if(BEACON_SEARCH_WAIT_FIRST == s_state)
    {
        if(beacon_search_confirm_candidate(trusted_track_id))
        {
            s_ever_seen = 1U;
            s_state = BEACON_SEARCH_TRACKING;
            s_loss_age_ms = 0U;
            beacon_search_clear_candidate();
        }
        return;
    }

    if((BEACON_SEARCH_TRACKING == s_state) ||
       (BEACON_SEARCH_LOSS_WAIT == s_state))
    {
        if(0U != trusted_track_id)
        {
            s_state = BEACON_SEARCH_TRACKING;
            s_loss_age_ms = 0U;
            beacon_search_clear_candidate();
        }
        return;
    }

    if(BEACON_SEARCH_SPINNING == s_state)
    {
        if(0U != trusted_track_id)
        {
            s_state = BEACON_SEARCH_VERIFYING;
            s_control = BEACON_SEARCH_CONTROL_HOLD;
            s_verify_from_failed = 0U;
            s_verify_elapsed_ms = 0U;
            (void)beacon_search_confirm_candidate(trusted_track_id);
        }
        return;
    }

    if(BEACON_SEARCH_FAILED == s_state)
    {
        if(0U != trusted_track_id)
        {
            s_state = BEACON_SEARCH_VERIFYING;
            s_control = BEACON_SEARCH_CONTROL_HOLD;
            s_verify_from_failed = 1U;
            s_verify_elapsed_ms = 0U;
            (void)beacon_search_confirm_candidate(trusted_track_id);
        }
        return;
    }

    if(BEACON_SEARCH_VERIFYING == s_state)
    {
        if(beacon_search_confirm_candidate(trusted_track_id))
        {
            s_ever_seen = 1U;
            s_state = BEACON_SEARCH_TRACKING;
            s_control = BEACON_SEARCH_CONTROL_NONE;
            s_loss_age_ms = 0U;
            s_search_elapsed_ms = 0U;
            s_failure_reason = BEACON_SEARCH_FAILURE_NONE;
            beacon_search_clear_candidate();
        }
    }
}

void BeaconSearch_ControlStep(float dt_s,
                              float current_yaw_deg,
                              volatile float *target_yaw_deg)
{
    static BeaconSearchControl_t last_control = BEACON_SEARCH_CONTROL_NONE;
    static uint8 last_search_generation = 0U;
    static float previous_yaw_deg = 0.0f;
    BeaconSearchControl_t control = s_control;

    s_current_yaw_deg = current_yaw_deg;
    if(NULL == target_yaw_deg)
    {
        return;
    }

    if(control != last_control)
    {
        if((BEACON_SEARCH_CONTROL_SPIN == control) ||
           (BEACON_SEARCH_CONTROL_HOLD == control))
        {
            *target_yaw_deg = current_yaw_deg;
            previous_yaw_deg = current_yaw_deg;
        }
        last_control = control;
    }

    if(BEACON_SEARCH_CONTROL_SPIN == control)
    {
        float yaw_delta_deg;

        if(last_search_generation != s_search_generation)
        {
            last_search_generation = s_search_generation;
            s_turn_progress_deg = 0.0f;
            previous_yaw_deg = current_yaw_deg;
            *target_yaw_deg = current_yaw_deg;
        }

        yaw_delta_deg = beacon_search_wrap_deg(current_yaw_deg -
                                                previous_yaw_deg);
        previous_yaw_deg = current_yaw_deg;
        if(fabsf(yaw_delta_deg) <= BEACON_SEARCH_MAX_YAW_STEP_DEG)
        {
            s_turn_progress_deg += yaw_delta_deg;
            if(s_turn_progress_deg < 0.0f)
            {
                s_turn_progress_deg = 0.0f;
            }
        }

        if(s_turn_progress_deg >= BEACON_SEARCH_FULL_TURN_DEG)
        {
            s_control = BEACON_SEARCH_CONTROL_HOLD;
            s_turn_complete_event = 1U;
            *target_yaw_deg = current_yaw_deg;
            last_control = BEACON_SEARCH_CONTROL_HOLD;
        }
        else
        {
            *target_yaw_deg = beacon_search_wrap_deg(
                *target_yaw_deg + BEACON_SEARCH_YAW_RATE_DPS * dt_s);
        }
    }

    s_target_yaw_deg = *target_yaw_deg;
}

uint8 BeaconSearch_GetFrameFlags(void)
{
    if((BEACON_SEARCH_SPINNING == s_state) ||
       (BEACON_SEARCH_VERIFYING == s_state))
    {
        return CORE_LINK_BEACON_FRAME_FLAG_SEARCH_ACTIVE;
    }
    if(BEACON_SEARCH_FAILED == s_state)
    {
        return CORE_LINK_BEACON_FRAME_FLAG_SEARCH_FAILED;
    }
    return 0U;
}

void BeaconSearch_GetDiag(BeaconSearchDiag_t *diag)
{
    uint32 irq_state;

    if(NULL == diag)
    {
        return;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();
    memset(diag, 0, sizeof(*diag));
    diag->state = (uint8)s_state;
    diag->eligible = s_eligible;
    diag->ever_seen = s_ever_seen;
    diag->candidate_track_id = s_candidate_track_id;
    diag->candidate_confirm_count = s_candidate_confirm_count;
    diag->failure_reason = (uint8)s_failure_reason;
    diag->loss_age_ms = s_loss_age_ms;
    diag->search_elapsed_ms = s_search_elapsed_ms;
    diag->car_link_age_ms = s_car_link_age_ms;
    diag->current_yaw_deg = s_current_yaw_deg;
    diag->target_yaw_deg = s_target_yaw_deg;
    diag->turn_progress_deg = s_turn_progress_deg;
    if(0U == irq_state)
    {
        __enable_irq();
    }
}
