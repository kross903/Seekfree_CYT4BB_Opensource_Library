#ifndef BEACON_SEARCH_H
#define BEACON_SEARCH_H

#include "zf_common_headfile.h"
#include "communication/core_link.h"

#define BEACON_SEARCH_YAW_RATE_DEFAULT_DPS        (90.0f) /* 自搜索Yaw速度默认值，单位deg/s */
#define BEACON_SEARCH_YAW_RATE_MIN_DPS            (30.0f) /* VOFA允许设置的最小自搜索Yaw速度 */
#define BEACON_SEARCH_YAW_RATE_MAX_DPS           (120.0f) /* VOFA允许设置的最大自搜索Yaw速度 */
#define BEACON_SEARCH_SWEEP_DIRECTION_DEFAULT_DEG (360.0f)/* 每轮正、反方向默认各旋转一周 */
#define BEACON_SEARCH_SWEEP_DIRECTION_MIN_DEG       (5.0f)/* VOFA允许设置的最小单向旋转角 */
#define BEACON_SEARCH_SWEEP_DIRECTION_MAX_DEG     (360.0f)/* VOFA允许设置的最大单向旋转角 */
#define BEACON_SEARCH_SWEEP_ANGLE_DEG \
    (2.0f * BEACON_SEARCH_SWEEP_DIRECTION_DEFAULT_DEG)    /* 默认一轮总旋转角：正反方向各360度 */
#define BEACON_SEARCH_POSITION_GATE_CM            (50.0f) /* 启动自搜索前，小车相对位置误差允许范围 */
#define BEACON_SEARCH_POSITION_WINDOW_FRAMES      (10U)   /* 位置稳定判定滑动窗口帧数 */
#define BEACON_SEARCH_POSITION_REQUIRED_FRAMES    (5U)    /* 滑动窗口内至少需要合格的位置帧数 */
#define BEACON_SEARCH_SPEED_GATE_CMS              (25.0f) /* 启动自搜索前，无人机水平速度稳定阈值 */
#define BEACON_SEARCH_SPEED_STABLE_MS              (150U) /* 水平速度连续稳定达到该时长后允许搜索 */
#define BEACON_SEARCH_VISION_FRAME_MAX_AGE_MS      (150U) /* 视觉帧最大有效年龄，超时认为视觉信息过旧 */

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
    BEACON_SEARCH_FAILURE_TIMEOUT,
    BEACON_SEARCH_FAILURE_CAR_LINK
} BeaconSearchFailure_t;

typedef enum
{
    BEACON_SEARCH_PHASE_STARTUP_ALIGN = 0,
    BEACON_SEARCH_PHASE_STARTUP_HOLD,
    BEACON_SEARCH_PHASE_SWEEP_TO_FIRST,
    BEACON_SEARCH_PHASE_SWEEP_HOLD_FIRST,
    BEACON_SEARCH_PHASE_SWEEP_TO_SECOND,
    BEACON_SEARCH_PHASE_SWEEP_HOLD_SECOND,
    BEACON_SEARCH_PHASE_POST_SWEEP_HOLD
} BeaconSearchPhase_t;

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
    float yaw_rate_dps;
    float sweep_direction_angle_deg;
    float position_distance_cm;
    float horizontal_speed_cms;
    uint16 speed_stable_ms;
    uint16 vision_frame_age_ms;
    uint8 position_window_count;
    uint8 position_qualified_count;
    uint8 position_sample_valid;
    uint8 fused_velocity_valid;
    uint8 start_gate_ready;
    uint8 sweep_round_count;
    uint8 terminal_latched;
    uint8 sweep_phase;
    uint8 startup_alignment_complete;
    uint8 startup_sequence_complete;
    uint16 phase_hold_elapsed_ms;
    float search_origin_yaw_deg;
} BeaconSearchDiag_t;

void BeaconSearch_Init(void);
void BeaconSearch_Tick(uint16 elapsed_ms,
                       uint8 armed_state,
                       uint8 angle_mode,
                       uint8 flight_ready,
                       uint8 velocity_loop_enabled,
                       uint8 car_link_valid,
                       uint16 car_link_age_ms,
                       uint8 car_search_requested,
                       float current_yaw_deg,
                       const BeaconSearchStabilityInput_t *stability_input);
void BeaconSearch_OnObservationFrame(
    const core_link_beacon_observation_packet_struct *packet,
    uint32 vision_process_count);
void BeaconSearch_ControlStep(float dt_s,
                              float current_yaw_deg,
                              volatile float *target_yaw_deg);
uint8 BeaconSearch_SetYawRateDps(float yaw_rate_dps);
float BeaconSearch_GetYawRateDps(void);
uint8 BeaconSearch_SetSweepDirectionAngleDeg(float direction_angle_deg);
float BeaconSearch_GetSweepDirectionAngleDeg(void);
float BeaconSearch_GetSweepAngleDeg(void);
float BeaconSearch_LimitYawRate(float yaw_rate_dps);
uint8 BeaconSearch_IsPausedByBlindBoxTask(void);
uint8 BeaconSearch_GetFrameFlags(void);
void BeaconSearch_GetDiag(BeaconSearchDiag_t *diag);

#endif
