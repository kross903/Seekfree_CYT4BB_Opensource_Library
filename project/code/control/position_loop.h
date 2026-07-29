#ifndef POSITION_LOOP_H_
#define POSITION_LOOP_H_

#include "zf_common_headfile.h"

#define POSITION_LOOP_STATUS_CORRECTION_LIMITED  (0x01U)
#define POSITION_LOOP_STATUS_TARGET_LIMITED      (0x02U)
#define POSITION_LOOP_STATUS_ACCEL_LIMITED       (0x04U)
#define POSITION_LOOP_STATUS_JERK_LIMITED        (0x08U)
#define POSITION_LOOP_STATUS_CAR_FF_VALID        (0x10U)
#define POSITION_LOOP_STATUS_TRAJECTORY_READY    (0x20U)
#define POSITION_LOOP_STATUS_TIMING_VALID        (0x40U)
#define POSITION_LOOP_STATUS_STOP_PREDICTION     (0x80U)

#define POSITION_LOOP_STOP_PRED_CAR_VELOCITY_VALID  (0x01U)
#define POSITION_LOOP_STOP_PRED_ACTIVE              (0x02U)
#define POSITION_LOOP_STOP_PRED_OVERSHOOT           (0x04U)
#define POSITION_LOOP_STOP_PRED_STATIC_FALLBACK     (0x08U)

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
    uint8 valid;
    float vx_uav_cms;
    float vy_uav_cms;
} PositionLoopFeedforwardInput_t;

typedef struct
{
    float position_kp;
    float correction_speed_max_cms;
    float target_speed_max_cms;
    float acceleration_max_cms2;
    float jerk_max_cms3;
} PositionLoopTrajectoryConfig_t;

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
    float car_ff_gain_x;
    float car_ff_gain_y;
    float desired_vx_cms;
    float desired_vy_cms;
    float target_accel_x_cms2;
    float target_accel_y_cms2;
    float position_error_distance_cm;
    float position_correction_speed_cms;
    float relative_closing_speed_cms;
    float relative_velocity_x_cms;
    float relative_velocity_y_cms;
    float relative_speed_cms;
    float stopping_distance_cm;
    float stop_offset_x_cm;
    float stop_offset_y_cm;
    float predicted_error_x_cm;
    float predicted_error_y_cm;
    float predicted_error_distance_cm;
    float envelope_speed_cms;
    float trajectory_dt_s;
    float vision_innovation_cm;
    uint32 trajectory_reset_count;
    uint16 vision_hold_age_ms;
    uint8 vision_observation_state;
    uint8 vision_reacquire_count;
    uint8 car_ff_valid;
    uint8 limit_flags;
    uint8 enabled;
    uint8 active;
    uint8 ir_track_valid;
    uint8 velocity_ready;
    uint8 timing_valid;
    uint8 trajectory_reset_reason;
    uint8 stop_prediction_flags;
} PositionLoopDiag_t;

void Position_Loop_Set_Enabled(uint8 enabled);
uint8 Position_Loop_Is_Enabled(void);
void Position_Loop_Update_Velocity_Target(uint8 velocity_ready,
                                          float measured_vx_body_cms,
                                          float measured_vy_body_cms,
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
void Position_Loop_Reset_Trajectory(uint8 reason);
uint8 Position_Loop_Set_Trajectory_Config(const PositionLoopTrajectoryConfig_t *config);
void Position_Loop_Get_Trajectory_Config(PositionLoopTrajectoryConfig_t *config);
void Position_Loop_Get_Diag(PositionLoopDiag_t *diag);

#endif
