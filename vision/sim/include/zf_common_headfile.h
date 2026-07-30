#ifndef VISION_SIM_ZF_COMMON_HEADFILE_H_
#define VISION_SIM_ZF_COMMON_HEADFILE_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* IAR storage qualifier; host simulation uses normal process memory. */
#ifndef __no_init
#define __no_init
#endif

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;
typedef int16_t int16;
typedef int16_t sint16;
typedef int32_t sint32;

#ifndef CY_CORE_CM7_0
#define CY_CORE_CM7_0 (0)
#endif
#ifndef CY_CORE_CM7_1
#define CY_CORE_CM7_1 (1)
#endif
#define MT9V03X_W (188)
#define MT9V03X_H (120)
#define MT9V03X_IMAGE_SIZE (MT9V03X_W * MT9V03X_H)

#ifndef IMAGE_WIDTH
#define IMAGE_WIDTH MT9V03X_W
#endif
#ifndef IMAGE_HEIGHT
#define IMAGE_HEIGHT MT9V03X_H
#endif

#endif
