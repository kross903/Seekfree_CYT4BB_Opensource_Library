#ifndef POSITION_LOOP_H_
#define POSITION_LOOP_H_

#include "zf_common_headfile.h"
#include "control/pid_controller.h"

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
    float x_i_out;
    float y_i_out;
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
                                          float *target_vx_cms,
                                          float *target_vy_cms);
void Position_Loop_Get_Diag(PositionLoopDiag_t *diag);

#endif
