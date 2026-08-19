#ifndef POSITION_LOOP_H_
#define POSITION_LOOP_H_

#include "zf_common_headfile.h"

#define POSITION_LOOP_STATUS_CORRECTION_LIMITED  (0x01U)
#define POSITION_LOOP_STATUS_TARGET_LIMITED      (0x02U)
#define POSITION_LOOP_STATUS_ACCEL_FF_VALID      (0x04U)
#define POSITION_LOOP_STATUS_CAR_FF_VALID        (0x10U)
#define POSITION_LOOP_STATUS_TARGET_READY        (0x20U)
#define POSITION_LOOP_STATUS_TIMING_VALID        (0x40U)
#define POSITION_LOOP_STATUS_BRAKE_LIMITED       (0x80U)

#define POSITION_LOOP_BRAKE_DIAG_CAR_VELOCITY_VALID (0x01U)
#define POSITION_LOOP_BRAKE_DIAG_CLOSING_ACTIVE     (0x02U)
#define POSITION_LOOP_BRAKE_DIAG_STATIC_FALLBACK    (0x04U)

#define POSITION_LOOP_OBSERVATION_INVALID      (0U)
#define POSITION_LOOP_OBSERVATION_LIVE         (1U)
#define POSITION_LOOP_OBSERVATION_HOLD         (2U)
#define POSITION_LOOP_OBSERVATION_REACQUIRING  (3U)

#define POSITION_LOOP_RESET_NONE                (0U)
#define POSITION_LOOP_RESET_FIRST_ENABLE        (1U)
#define POSITION_LOOP_RESET_CONTROL_DISABLED    (2U)
#define POSITION_LOOP_RESET_DISARMED            (3U)
#define POSITION_LOOP_RESET_MANUAL_MODE         (4U)
#define POSITION_LOOP_RESET_TAKEOFF             (5U)
#define POSITION_LOOP_RESET_ESTOP               (6U)
#define POSITION_LOOP_RESET_VELOCITY_INVALID    (7U)
#define POSITION_LOOP_RESET_VELOCITY_RECOVERED  (8U)
#define POSITION_LOOP_RESET_TIMING_INVALID      (9U)
#define POSITION_LOOP_RESET_YAW_RESET           (10U)

typedef struct
{
    uint8 velocity_valid;
    uint8 acceleration_valid;
    float vx_uav_cms;
    float vy_uav_cms;
    float ax_uav_cms2;
    float ay_uav_cms2;
} PositionLoopFeedforwardInput_t;

typedef struct
{
    float position_kp;
    float correction_accel_max_cms2;
    float correction_speed_max_cms;
    float target_speed_max_cms;
} PositionLoopTrajectoryConfig_t;

typedef struct
{
    float effective_accel_cms2;
    float effective_jerk_cms3;
    float control_delay_ms;
    float safety_buffer_cm;
    float boost_margin_cm;
} PositionLoopBrakeConfig_t;

typedef struct
{
    float target_x_cm;
    float target_y_cm;
    float fb_x_cm;
    float fb_y_cm;
    float error_x_cm;
    float error_y_cm;
    float vx_cmd_cms;
    float vy_cmd_cms;
    float correction_vx_cms;
    float correction_vy_cms;
    float car_ff_raw_vx_cms;
    float car_ff_raw_vy_cms;
    float car_ff_filtered_vx_cms;
    float car_ff_filtered_vy_cms;
    float car_ff_raw_ax_cms2;
    float car_ff_raw_ay_cms2;
    float car_ff_applied_ax_cms2;
    float car_ff_applied_ay_cms2;
    float car_ff_gain_x;
    float car_ff_gain_y;
    float desired_vx_cms;
    float desired_vy_cms;
    float target_accel_x_cms2;
    float target_accel_y_cms2;
    float position_error_distance_cm;
    float position_correction_speed_cms;
    float proportional_speed_cms;
    float sqrt_speed_cms;
    float relative_closing_speed_cms;
    float filtered_closing_speed_cms;
    float relative_velocity_x_cms;
    float relative_velocity_y_cms;
    float relative_speed_cms;
    float brake_delay_distance_cm;
    float brake_allowed_speed_cms;
    float trajectory_dt_s;
    float vision_innovation_cm;
    uint32 trajectory_reset_count;
    uint16 vision_hold_age_ms;
    uint8 vision_observation_state;
    uint8 vision_reacquire_count;
    uint8 car_ff_valid;
    uint8 car_accel_ff_valid;
    uint8 limit_flags;
    uint8 enabled;
    uint8 active;
    uint8 ir_track_valid;
    uint8 velocity_ready;
    uint8 timing_valid;
    uint8 trajectory_reset_reason;
    uint8 brake_diag_flags;
    uint8 sqrt_controller_active;
    uint8 frozen_hold_active;
} PositionLoopDiag_t;

void Position_Loop_Set_Enabled(uint8 enabled);
uint8 Position_Loop_Is_Enabled(void);
void Position_Loop_Update_Velocity_Target(uint8 velocity_ready,
                                          float measured_vx_earth_cms,
                                          float measured_vy_earth_cms,
                                          float yaw_deg,
                                          float dt_s,
                                          uint8 timing_valid,
                                          const PositionLoopFeedforwardInput_t *feedforward,
                                          float *target_vx_earth_cms,
                                          float *target_vy_earth_cms,
                                          float *target_ax_earth_cms2,
                                          float *target_ay_earth_cms2);
uint8 Position_Loop_Set_Car_Velocity_Feedforward_Gain(uint8 axis, float gain);
void Position_Loop_Get_Car_Velocity_Feedforward_Gain(float *gain_x, float *gain_y);
void Position_Loop_Reset_Car_Velocity_Feedforward(void);
uint8 Position_Loop_Begin_Frozen_Hold(void);
void Position_Loop_End_Frozen_Hold(void);
uint8 Position_Loop_Is_Frozen_Hold_Active(void);
void Position_Loop_Reset_Trajectory(uint8 reason);
uint8 Position_Loop_Set_Trajectory_Config(const PositionLoopTrajectoryConfig_t *config);
void Position_Loop_Get_Trajectory_Config(PositionLoopTrajectoryConfig_t *config);
uint8 Position_Loop_Set_Brake_Config(const PositionLoopBrakeConfig_t *config);
void Position_Loop_Get_Brake_Config(PositionLoopBrakeConfig_t *config);
void Position_Loop_Get_Diag(PositionLoopDiag_t *diag);

#endif
