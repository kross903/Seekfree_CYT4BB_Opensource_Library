#include "control/throttle_runtime.h"

#include "common/PIT_Demo.h"

volatile float target_throttle = 0.0f;
volatile float current_throttle = 0.0f;
volatile float base_throttle = 0.0f;

float Throttle_Runtime_Limit(float throttle)
{
    if(throttle > SAFE_MAX_THROTTLE)
    {
        throttle = SAFE_MAX_THROTTLE;
    }
    else if(throttle < 0.0f)
    {
        throttle = 0.0f;
    }
    return throttle;
}

void Throttle_Runtime_Ramp(float target, volatile float *current, float rate, float dt_ms)
{
    float error = target - (*current);
    float step = rate * dt_ms;

    if(error > step)
    {
        (*current) += step;
    }
    else if(error < -step)
    {
        (*current) -= step;
    }
    else
    {
        (*current) = target;
    }

    *current = Throttle_Runtime_Limit(*current);
}

void Throttle_Runtime_Reset_Throttle_State(void)
{
    target_throttle = 0.0f;
    current_throttle = 0.0f;
    base_throttle = 0.0f;
}

void Flight_Runtime_Set_Throttle_For_Mode(uint8 mode, float throttle)
{
    (void)mode;
    target_throttle = Throttle_Runtime_Limit(throttle);
}

float Flight_Runtime_Get_Target_Throttle_For_Mode(uint8 mode)
{
    (void)mode;
    return target_throttle;
}

float Flight_Runtime_Get_Base_Throttle_For_Mode(uint8 mode)
{
    (void)mode;
    return base_throttle;
}

void Flight_Runtime_Sync_Public_Throttle_State(void)
{
}
