#ifndef _PMW_FLOW_H_
#define _PMW_FLOW_H_

#include "zf_common_headfile.h"

typedef struct
{
    uint8 valid;
    uint8 reserved0;
    uint8 reserved1;
    uint8 reserved2;
    int16 flow_delta_x_count;
    int16 flow_delta_y_count;
    float dt_s;
    float tof_los_cm;
    float range_cm;
    float roll_deg;
    float pitch_deg;
    float flow_x_rad;
    float flow_y_rad;
    float gyro_flow_x_delta_rad;
    float gyro_flow_y_delta_rad;
    float flow_comp_x_rad;
    float flow_comp_y_rad;
    float velocity_x_raw_cms;
    float velocity_y_raw_cms;
    float velocity_x_cms;
    float velocity_y_cms;
    float speed_cms;
} PMWFlowState_t;

void PMW_Flow_Init(void);
void PMW_Flow_Reset(void);
void PMW_Flow_AccumulateGyro(float gyro_x_rate_dps, float gyro_y_rate_dps, float dt_s);
void PMW_Flow_UpdateSample(int16 flow_delta_x,
                           int16 flow_delta_y,
                           float roll_deg,
                           float pitch_deg,
                           float tof_los_cm,
                           float dt_s);
void PMW_Flow_SyncMainLoopBuffer(void);
void PMW_Flow_GetState(PMWFlowState_t *state);

#endif
