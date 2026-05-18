#include "control/ground_remote_input.h"

#define GROUND_REMOTE_INPUT_LPF_ALPHA     0.20f

volatile int adc_x = 0;
volatile int adc_y = 0;
volatile int adc_x_filtered = 0.0f;
volatile int adc_y_filtered = 0.0f;
volatile float adc_x_nomalize = 0.0f;
volatile float adc_y_nomalize = 0.0f;

static const float DEADZONE_HALF = GROUND_REMOTE_ADC_CENTER * GROUND_REMOTE_DEADZONE_RATIO;
static const float RANGE_POS     = GROUND_REMOTE_ADC_MAX - (GROUND_REMOTE_ADC_CENTER + GROUND_REMOTE_ADC_CENTER * GROUND_REMOTE_DEADZONE_RATIO);
static const float RANGE_NEG     = (GROUND_REMOTE_ADC_CENTER - GROUND_REMOTE_ADC_CENTER * GROUND_REMOTE_DEADZONE_RATIO) - GROUND_REMOTE_ADC_MIN;

float GroundRemoteInput_Normalize(int raw_adc)
{
    float adc = raw_adc;
    float delta = adc - GROUND_REMOTE_ADC_CENTER;

    if(delta > -DEADZONE_HALF && delta < DEADZONE_HALF)
    {
        return 0.0f;
    }

    if(delta >= DEADZONE_HALF)
    {
        float output = (delta - DEADZONE_HALF) / RANGE_POS;
        if(output > 1.0f)
        {
            output = 1.0f;
        }
        return output;
    }

    {
        float output = (delta + DEADZONE_HALF) / RANGE_NEG;
        if(output < -1.0f)
        {
            output = -1.0f;
        }
        return output;
    }
}

void GroundRemoteInput_Task_5ms(void)
{
    static uint8 adc_lpf_initialized = 0U;
    static float adc_x_lpf_state = 0.0f;
    static float adc_y_lpf_state = 0.0f;

    if(!adc_lpf_initialized)
    {
        adc_x_lpf_state = (float)adc_x;
        adc_y_lpf_state = (float)adc_y;
        adc_lpf_initialized = 1U;
    }
    else
    {
        adc_x_lpf_state += ((float)adc_x - adc_x_lpf_state) * GROUND_REMOTE_INPUT_LPF_ALPHA;
        adc_y_lpf_state += ((float)adc_y - adc_y_lpf_state) * GROUND_REMOTE_INPUT_LPF_ALPHA;
    }

    adc_x_filtered = (int)adc_x_lpf_state;
    adc_y_filtered = (int)adc_y_lpf_state;
    adc_x_nomalize = GroundRemoteInput_Normalize(adc_x_filtered);
    adc_y_nomalize = GroundRemoteInput_Normalize(adc_y_filtered);
}
