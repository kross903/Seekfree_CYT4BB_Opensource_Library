#ifndef BEACON_SEARCH_H
#define BEACON_SEARCH_H

#include "zf_common_headfile.h"
#include "communication/core_link.h"

#define BEACON_SEARCH_SWEEP_ANGLE_DEG            (360.0f)
#define BEACON_SEARCH_POSITION_GATE_CM            (50.0f)
#define BEACON_SEARCH_POSITION_WINDOW_FRAMES      (10U)
#define BEACON_SEARCH_POSITION_REQUIRED_FRAMES    (5U)
#define BEACON_SEARCH_SPEED_GATE_CMS              (25.0f)
#define BEACON_SEARCH_SPEED_STABLE_MS              (300U)
#define BEACON_SEARCH_VISION_FRAME_MAX_AGE_MS      (150U)

typedef enum
{
    BEACON_SEARCH_DISABLED = 0,
    BEACON_SEARCH_WAIT_FIRST,
    BEACON_SEARCH_TRACKING,
    BEACON_SEARCH_LOSS_WAIT,
    BEACON_SEARCH_SPINNING,
    BEACON_SEARCH_VERIFYING,
    BEACON_SEARCH_FAILED
} BeaconSearchState_t;

typedef enum
{
    BEACON_SEARCH_FAILURE_NONE = 0,
    BEACON_SEARCH_FAILURE_SWEEP_COMPLETE,
    BEACON_SEARCH_FAILURE_FULL_TURN = BEACON_SEARCH_FAILURE_SWEEP_COMPLETE,
    BEACON_SEARCH_FAILURE_TIMEOUT,
    BEACON_SEARCH_FAILURE_CAR_LINK
} BeaconSearchFailure_t;

typedef struct
{
    uint32 vision_process_count;
    uint8 position_loop_enabled;
    uint8 position_valid;
    uint8 fused_velocity_valid;
    float position_error_x_cm;
    float position_error_y_cm;
    float fused_velocity_x_cms;
    float fused_velocity_y_cms;
} BeaconSearchStabilityInput_t;

typedef struct
{
    uint8 state;
    uint8 eligible;
    uint8 ever_seen;
    uint8 candidate_track_id;
    uint8 candidate_confirm_count;
    uint8 failure_reason;
    uint16 loss_age_ms;
    uint16 search_elapsed_ms;
    uint16 car_link_age_ms;
    float current_yaw_deg;
    float target_yaw_deg;
    float turn_progress_deg;
    float sweep_angle_deg;
    float position_distance_cm;
    float horizontal_speed_cms;
    uint16 speed_stable_ms;
    uint16 vision_frame_age_ms;
    uint8 position_window_count;
    uint8 position_qualified_count;
    uint8 position_sample_valid;
    uint8 fused_velocity_valid;
    uint8 start_gate_ready;
} BeaconSearchDiag_t;

void BeaconSearch_Init(void);
void BeaconSearch_Tick(uint16 elapsed_ms,
                       uint8 armed_state,
                       uint8 angle_mode,
                       uint8 flight_ready,
                       uint8 velocity_loop_enabled,
                       uint8 car_link_valid,
                       uint16 car_link_age_ms,
                       const BeaconSearchStabilityInput_t *stability_input);
void BeaconSearch_OnObservationFrame(
    const core_link_beacon_observation_packet_struct *packet,
    uint32 vision_process_count);
void BeaconSearch_ControlStep(float dt_s,
                              float current_yaw_deg,
                              volatile float *target_yaw_deg);
uint8 BeaconSearch_GetFrameFlags(void);
void BeaconSearch_GetDiag(BeaconSearchDiag_t *diag);

#endif
