#ifndef THROTTLE_RUNTIME_H_
#define THROTTLE_RUNTIME_H_

#include "zf_common_headfile.h"

extern volatile float target_throttle;
extern volatile float current_throttle;
extern volatile float base_throttle;

float Throttle_Runtime_Limit(float throttle);
void Throttle_Runtime_Ramp(float target, volatile float *current, float rate, float dt_ms);
void Throttle_Runtime_Reset_Throttle_State(void);

void Flight_Runtime_Set_Throttle_For_Mode(uint8 mode, float throttle);
float Flight_Runtime_Get_Target_Throttle_For_Mode(uint8 mode);
float Flight_Runtime_Get_Base_Throttle_For_Mode(uint8 mode);
void Flight_Runtime_Sync_Public_Throttle_State(void);

#endif
