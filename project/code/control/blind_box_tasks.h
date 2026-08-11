#ifndef BLIND_BOX_TASKS_H
#define BLIND_BOX_TASKS_H

#include "zf_common_typedef.h"

typedef enum
{
    AUTO_LAND_IDLE = 0,
    AUTO_LAND_CONFIRMING,
    AUTO_LAND_ALIGNING,
    AUTO_LAND_DESCENDING,
    AUTO_LAND_OPEN_LOOP,
    AUTO_LAND_COMPLETE,
    AUTO_LAND_BLOCKED
} BlindBoxTasksState_t;

typedef enum
{
    AUTO_LAND_BLOCK_NONE = 0,
    AUTO_LAND_BLOCK_NO_LAND_POINT,
    AUTO_LAND_BLOCK_NO_TOF
} BlindBoxTasksBlockReason_t;

typedef struct
{
    uint8 state;
    uint8 trigger_reason;
    uint8 block_reason;
    uint8 landing_point_valid;
    uint8 fused_velocity_valid;
    uint8 tof_valid;
    uint8 tof_state;
    uint16 confirm_elapsed_ms;
    uint16 stable_elapsed_ms;
    uint16 open_loop_elapsed_ms;
    float position_error_x_cm;
    float position_error_y_cm;
    float position_error_cm;
    float horizontal_speed_cms;
    float target_height_cm;
    float open_loop_throttle;
} BlindBoxTasksDiag_t;

void BlindBoxTasks_Init(void);
void BlindBoxTasks_ControlStep(float dt_s,
                                float current_yaw_deg,
                                volatile float *target_yaw_deg);
float BlindBoxTasks_LimitYawRate(float yaw_rate_dps);
uint8 BlindBoxTasks_IsBeaconSearchPaused(void);
uint8 BlindBoxTasks_GetFrameFlags(void);
void BlindBoxTasks_Tick(uint16 elapsed_ms);
void BlindBoxTasks_HeightTick_10ms(void);
uint8 BlindBoxTasks_IsActive(void);
uint8 BlindBoxTasks_TakeDisarmRequest(void);
uint8 BlindBoxTasks_GetThrottleOverride(float *throttle);
void BlindBoxTasks_GetDiag(BlindBoxTasksDiag_t *diag);

#endif
