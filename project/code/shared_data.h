/*********************************************************************************************************************
* File               shared_data.h
* Description        Shared data definitions for cross-core communication
********************************************************************************************************************/

#ifndef _SHARED_DATA_H_
#define _SHARED_DATA_H_

#include "zf_common_headfile.h"

// The live control path still uses the globals in PIT_Demo.c.
// Keep this shared-memory layout for the unfinished multi-core split.

extern volatile uint16 shared_motor_throttle[4];

extern volatile uint8 shared_armed;
extern volatile uint8 shared_control_mode;
extern volatile float shared_base_throttle;

extern volatile float shared_gyro_x;
extern volatile float shared_gyro_y;
extern volatile float shared_gyro_z;

extern volatile float shared_target_gyro_roll;
extern volatile float shared_target_gyro_pitch;
extern volatile float shared_target_gyro_yaw;

extern volatile uint8 shared_key_state[4];

extern volatile uint32 shared_loop_count;

#endif
