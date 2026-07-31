#ifndef BEACON_SEARCH_H
#define BEACON_SEARCH_H

#include "zf_common_headfile.h"
#include "communication/core_link.h"

#define BEACON_SEARCH_YAW_RATE_DEFAULT_DPS        (90.0f) /* 自搜索Yaw速度默认值，单位deg/s */
#define BEACON_SEARCH_YAW_RATE_MIN_DPS            (30.0f) /* VOFA允许设置的最小自搜索Yaw速度 */
#define BEACON_SEARCH_YAW_RATE_MAX_DPS           (120.0f) /* VOFA允许设置的最大自搜索Yaw速度 */
#define BEACON_SEARCH_SWEEP_HALF_ANGLE_DEFAULT_DEG (45.0f)/* 普通自搜索默认半角，范围为-半角到+半角 */
#define BEACON_SEARCH_SWEEP_HALF_ANGLE_MIN_DEG     (5.0f) /* VOFA允许设置的最小自搜索半角 */
#define BEACON_SEARCH_SWEEP_HALF_ANGLE_MAX_DEG    (90.0f) /* VOFA允许设置的最大自搜索半角 */
#define BEACON_SEARCH_SWEEP_ANGLE_DEG \
    (4.0f * BEACON_SEARCH_SWEEP_HALF_ANGLE_DEFAULT_DEG)   /* 默认完整摆扫角度：0→+半角→0→-半角→0 */
#define BEACON_SEARCH_POSITION_GATE_CM            (50.0f) /* 启动自搜索前，小车相对位置误差允许范围 */
#define BEACON_SEARCH_POSITION_WINDOW_FRAMES      (10U)   /* 位置稳定判定滑动窗口帧数 */
#define BEACON_SEARCH_POSITION_REQUIRED_FRAMES    (5U)    /* 滑动窗口内至少需要合格的位置帧数 */
#define BEACON_SEARCH_SPEED_GATE_CMS              (25.0f) /* 启动自搜索前，无人机水平速度稳定阈值 */
#define BEACON_SEARCH_SPEED_STABLE_MS              (300U) /* 水平速度连续稳定达到该时长后允许搜索 */
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
    float yaw_rate_dps;
    float sweep_half_angle_deg;
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
uint8 BeaconSearch_SetYawRateDps(float yaw_rate_dps);
float BeaconSearch_GetYawRateDps(void);
uint8 BeaconSearch_SetSweepHalfAngleDeg(float half_angle_deg);
float BeaconSearch_GetSweepHalfAngleDeg(void);
float BeaconSearch_GetSweepAngleDeg(void);
float BeaconSearch_LimitYawRate(float yaw_rate_dps);
uint8 BeaconSearch_GetFrameFlags(void);
void BeaconSearch_GetDiag(BeaconSearchDiag_t *diag);

#endif
