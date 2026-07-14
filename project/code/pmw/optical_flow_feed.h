#ifndef OPTICAL_FLOW_FEED_H_
#define OPTICAL_FLOW_FEED_H_

#include "zf_common_headfile.h"

void OpticalFlowFeed_Task(void);
void OpticalFlowFeed_GetTofStatus(float *tof_height_cm, uint16 *stale_ticks, uint8 *source);
void PMW_Flow_Get_TOF_Status(float *tof_height_cm, uint16 *stale_ticks, uint8 *source);

#endif
