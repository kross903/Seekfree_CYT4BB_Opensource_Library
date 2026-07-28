#ifndef POSITION_LOOP_H_
#define POSITION_LOOP_H_

#include "zf_common_headfile.h"
#include "control/pid_controller.h"

#define POSITION_LOOP_STATUS_CORRECTION_LIMITED  (0x01U)
#define POSITION_LOOP_STATUS_TARGET_LIMITED      (0x02U)
#define POSITION_LOOP_STATUS_SLEW_LIMITED        (0x04U)

typedef struct
{
    uint8 valid;
    float vx_uav_cms;
    float vy_uav_cms;
} PositionLoopFeedforwardInput_t;

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
    uint8 car_ff_valid;
    uint8 limit_flags;
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
void Position_Loop_Get_Diag(PositionLoopDiag_t *diag);

#endif
