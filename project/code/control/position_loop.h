#ifndef POSITION_LOOP_H_
#define POSITION_LOOP_H_

#include "zf_common_headfile.h"
#include "control/pid_controller.h"

#define POSITION_LOOP_STATUS_CORRECTION_LIMITED  (0x01U)
#define POSITION_LOOP_STATUS_TARGET_LIMITED      (0x02U)
#define POSITION_LOOP_STATUS_SLEW_LIMITED        (0x04U)
#define POSITION_LOOP_STATUS_COMMAND_ENVELOPE_LIMITED (0x40U)
#define POSITION_LOOP_STATUS_BRAKE_PHASE_LATCHED (0x80U)
#define POSITION_LOOP_BRAKE_REL_VEL_LPF_HZ       (8.0f)

#define POSITION_LOOP_BRAKE_STATUS_ENABLED              (0x01U)
#define POSITION_LOOP_BRAKE_STATUS_CAR_VELOCITY_FALLBACK (0x02U)
#define POSITION_LOOP_BRAKE_STATUS_DIRECTION_VALID      (0x04U)
#define POSITION_LOOP_BRAKE_STATUS_APPROACHING           (0x08U)
#define POSITION_LOOP_BRAKE_STATUS_INSIDE_STOP_RADIUS    (0x10U)
#define POSITION_LOOP_BRAKE_STATUS_REVERSE_LIMITED       (0x20U)
#define POSITION_LOOP_BRAKE_STATUS_CORRECTION_LIMITED    (0x40U)
#define POSITION_LOOP_BRAKE_STATUS_VELOCITY_FILTER_READY (0x80U)

#define POSITION_LOOP_OBSERVATION_INVALID      (0U)
#define POSITION_LOOP_OBSERVATION_LIVE         (1U)
#define POSITION_LOOP_OBSERVATION_HOLD         (2U)
#define POSITION_LOOP_OBSERVATION_REACQUIRING  (3U)

#define POSITION_LOOP_BRAKE_PHASE_APPROACH  (0U)
#define POSITION_LOOP_BRAKE_PHASE_BRAKING   (1U)
#define POSITION_LOOP_BRAKE_PHASE_SETTLED   (2U)

typedef struct
{
    uint8 valid;
    float vx_uav_cms;
    float vy_uav_cms;
} PositionLoopFeedforwardInput_t;

typedef struct
{
    uint8 enabled;
    float brake_accel_cms2;
    float delay_s;
    float reverse_limit_cms;
    float stop_radius_cm;
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
    float car_ff_gain_x;
    float car_ff_gain_y;
    float x_i_out;
    float y_i_out;
    float brake_position_x_cm;
    float brake_position_y_cm;
    float brake_distance_cm;
    float brake_closing_speed_cms;
    float brake_physical_distance_cm;
    float brake_delay_distance_cm;
    float brake_envelope_speed_cms;
    float brake_overspeed_cms;
    float brake_base_correction_vx_cms;
    float brake_base_correction_vy_cms;
    float brake_candidate_vx_cms;
    float brake_candidate_vy_cms;
    float brake_combined_vx_cms;
    float brake_combined_vy_cms;
    float command_envelope_allowed_speed_cms;
    float command_envelope_raw_radial_cms;
    float command_envelope_limited_radial_cms;
    float command_envelope_vx_cms;
    float command_envelope_vy_cms;
    float brake_profile_reference_speed_cms;
    float brake_profile_trigger_distance_cm;
    float brake_profile_target_speed_cms;
    float vision_innovation_cm;
    uint16 vision_hold_age_ms;
    uint16 brake_profile_settle_ms;
    uint8 vision_observation_state;
    uint8 vision_reacquire_count;
    uint8 command_envelope_active;
    uint8 brake_profile_phase;
    uint8 brake_profile_restart_count;
    uint8 car_ff_valid;
    uint8 limit_flags;
    uint8 brake_active;
    uint8 brake_status_flags;
    uint8 enabled;
    uint8 active;
    uint8 ir_track_valid;
    uint8 velocity_ready;
} PositionLoopDiag_t;

extern PID_Controller pos_pid_x;
extern PID_Controller pos_pid_y;

void Position_Loop_Set_Enabled(uint8 enabled);
uint8 Position_Loop_Is_Enabled(void);
void Position_Loop_Update_Velocity_Target(uint8 velocity_ready,
                                          float measured_vx_cms,
                                          float measured_vy_cms,
                                          const PositionLoopFeedforwardInput_t *feedforward,
                                          float *target_vx_cms,
                                          float *target_vy_cms);
uint8 Position_Loop_Set_Car_Velocity_Feedforward_Gain(uint8 axis, float gain);
void Position_Loop_Get_Car_Velocity_Feedforward_Gain(float *gain_x, float *gain_y);
void Position_Loop_Reset_Car_Velocity_Feedforward(void);
uint8 Position_Loop_Set_Brake_Config(const PositionLoopBrakeConfig_t *config);
void Position_Loop_Get_Brake_Config(PositionLoopBrakeConfig_t *config);
void Position_Loop_Get_Diag(PositionLoopDiag_t *diag);

#endif
