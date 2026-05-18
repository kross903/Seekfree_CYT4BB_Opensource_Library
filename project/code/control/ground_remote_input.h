#ifndef GROUND_REMOTE_INPUT_H_
#define GROUND_REMOTE_INPUT_H_

#include "zf_common_headfile.h"

#define GROUND_REMOTE_ADC_MIN             0
#define GROUND_REMOTE_ADC_MAX             4000
#define GROUND_REMOTE_ADC_CENTER          2000
#define GROUND_REMOTE_DEADZONE_RATIO      0.1f

extern volatile int adc_x;
extern volatile int adc_y;
extern volatile int adc_x_filtered;
extern volatile int adc_y_filtered;
extern volatile float adc_x_nomalize;
extern volatile float adc_y_nomalize;

float GroundRemoteInput_Normalize(int raw_adc);
void GroundRemoteInput_Task_5ms(void);

#endif
