#ifndef _PARAMETER_FLASH_H_
#define _PARAMETER_FLASH_H_

#include "zf_common_headfile.h"

#define MAX_PARAMETERS 64
#define PARAMETER_INDEX 1

typedef enum
{
    PARAM_GROUP_GYRO = 0,
    PARAM_GROUP_ANGLE,
    PARAM_GROUP_HEIGHT,
    PARAM_GROUP_VELOCITY,
    PARAM_GROUP_COUNT
} ParameterGroupId;

typedef enum
{
    PARAM_OBJECT_ROLL = 0,
    PARAM_OBJECT_PITCH,
    PARAM_OBJECT_YAW,
    PARAM_OBJECT_HEIGHT,
    PARAM_OBJECT_VEL_X,
    PARAM_OBJECT_VEL_Y,
    PARAM_OBJECT_COUNT
} ParameterObjectId;

typedef enum
{
    PARAM_ITEM_KP = 0,
    PARAM_ITEM_KI,
    PARAM_ITEM_KD,
    PARAM_ITEM_OUTPUT_LIMIT,
    PARAM_ITEM_INTEGRAL_LIMIT,
    PARAM_ITEM_COUNT
} ParameterItemId;

typedef struct
{
    const char *name;
    uint8 index;
    float *target_var;
    float min_value;
    float max_value;
    float step;
    uint8 group;
    uint8 object;
    uint8 item;
} ParameterConfig;

extern float parameter[MAX_PARAMETERS];
extern ParameterConfig param_config[];
extern uint8 parameter_count;

void Parameter_Capture_Runtime_Defaults(void);
void Parameter_Load_Or_Init_Storage(void);
void Parameter_System_Init(void);
void Parameter_flash_load(void);
void Parameter_flash_save(void);
void Parameter_reset_to_default(void);
void parameter_menu_entry(void);
void Parameter_Apply_Config_Value(uint8 config_index, float value);

float truncate_by_step(float value, float step);
void format_float_truncate(char *buffer, float value, uint8 width, uint8 precision);
uint8 check_parameter_index_conflicts(void);

#endif
