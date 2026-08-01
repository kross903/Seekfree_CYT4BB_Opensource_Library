#ifndef FLIGHT_AUTO_LAND_H
#define FLIGHT_AUTO_LAND_H

#include "zf_common_typedef.h"

#define FLIGHT_AUTO_LAND_ENABLE (0U)

typedef enum
{
    AUTO_LAND_IDLE = 0,
    AUTO_LAND_CONFIRMING,
    AUTO_LAND_ALIGNING,
    AUTO_LAND_DESCENDING,
    AUTO_LAND_OPEN_LOOP,
    AUTO_LAND_COMPLETE,
    AUTO_LAND_BLOCKED
} FlightAutoLandState_t;

typedef enum
{
    AUTO_LAND_BLOCK_NONE = 0,
    AUTO_LAND_BLOCK_NO_LAND_POINT,
    AUTO_LAND_BLOCK_NO_TOF
} FlightAutoLandBlockReason_t;

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
} FlightAutoLandDiag_t;

void FlightAutoLand_Init(void);
void FlightAutoLand_Tick(uint16 elapsed_ms);
void FlightAutoLand_HeightTick_10ms(void);
uint8 FlightAutoLand_IsActive(void);
uint8 FlightAutoLand_TakeDisarmRequest(void);
uint8 FlightAutoLand_GetThrottleOverride(float *throttle);
void FlightAutoLand_GetDiag(FlightAutoLandDiag_t *diag);

#endif
