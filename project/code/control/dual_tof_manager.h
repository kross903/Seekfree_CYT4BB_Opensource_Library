#ifndef DUAL_TOF_MANAGER_H_
#define DUAL_TOF_MANAGER_H_

#include "zf_common_typedef.h"

typedef enum
{
    DUAL_TOF_SOURCE_NONE = 0,
    DUAL_TOF_SOURCE_TOF1 = 1,
    DUAL_TOF_SOURCE_TOF2 = 2,
    DUAL_TOF_SOURCE_EQUAL = 3
} DualTofSource_t;

typedef struct
{
    uint16 tof1_raw_mm;
    uint16 tof2_raw_mm;
    uint16 selected_mm;
    uint16 tof1_age_ms;
    uint16 tof2_age_ms;
    uint16 disagreement_mm;
    uint8 tof1_new_sample;
    uint8 tof2_new_sample;
    uint8 tof1_valid;
    uint8 tof2_valid;
    uint8 selected_new_sample;
    uint8 selected_valid;
    uint8 source;
    uint8 tof1_initialized;
    uint8 tof2_initialized;
    uint32 tof1_invalid_count;
    uint32 tof2_invalid_count;
    uint32 tof2_bus_error_count;
} DualTofSample_t;

uint8 DualTof_Init(void);
void DualTof_Poll(DualTofSample_t *sample);

#endif
