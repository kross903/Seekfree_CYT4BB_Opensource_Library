#include "parameter_flash.h"
#include "common/project_headfile.h"
#include <stdio.h>
#include <string.h>

#define PARAMETER_FLASH_MAGIC        0x5041524DU
#define PARAMETER_FLASH_VERSION      1U

#define PARAM_UI_TITLE_Y             10U
#define PARAM_UI_START_Y             40U
#define PARAM_UI_LINE_HEIGHT         18U
#define PARAM_UI_LEFT_X              10U
#define PARAM_UI_VALUE_X             152U
#define PARAM_UI_STATUS_Y            258U
#define PARAM_UI_HINT_Y              284U

typedef enum
{
    PARAM_ROOT_GYRO = 0,
    PARAM_ROOT_ANGLE,
    PARAM_ROOT_HEIGHT,
    PARAM_ROOT_VELOCITY,
    PARAM_ROOT_SAVE,
    PARAM_ROOT_RESET,
    PARAM_ROOT_COUNT
} ParameterRootMenuId;

typedef enum
{
    PARAM_MENU_ROOT = 0,
    PARAM_MENU_OBJECTS,
    PARAM_MENU_ITEMS,
    PARAM_MENU_EDIT,
    PARAM_MENU_CONFIRM_SAVE,
    PARAM_MENU_CONFIRM_RESET,
    PARAM_MENU_CONFIRM_SAVE_DEFAULTS
} ParameterMenuState;

typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 param_count;
    uint32 checksum;
    float values[MAX_PARAMETERS];
} ParameterFlashRecord;

static const char *s_root_menu_items[PARAM_ROOT_COUNT] = {
    "Gyro PID",
    "Angle PID",
    "Height PID",
    "Velocity PID",
    "Save All To Flash",
    "Reset To Defaults"
};

static const char *s_group_titles[PARAM_GROUP_COUNT] = {
    "Gyro PID",
    "Angle PID",
    "Height PID",
    "Velocity PID"
};

static const char *s_object_labels[PARAM_OBJECT_COUNT] = {
    "Roll",
    "Pitch",
    "Yaw",
    "Height",
    "Vel X",
    "Vel Y"
};

static const char *s_item_labels[PARAM_ITEM_COUNT] = {
    "Kp",
    "Ki",
    "Kd",
    "Output Limit",
    "Integral Limit"
};

static const uint8 s_group_object_count[PARAM_GROUP_COUNT] = {
    3U,
    3U,
    1U,
    2U
};

static const uint8 s_group_objects[PARAM_GROUP_COUNT][3] = {
    { PARAM_OBJECT_ROLL, PARAM_OBJECT_PITCH, PARAM_OBJECT_YAW },
    { PARAM_OBJECT_ROLL, PARAM_OBJECT_PITCH, PARAM_OBJECT_YAW },
    { PARAM_OBJECT_HEIGHT, PARAM_OBJECT_HEIGHT, PARAM_OBJECT_HEIGHT },
    { PARAM_OBJECT_VEL_X, PARAM_OBJECT_VEL_Y, PARAM_OBJECT_VEL_Y }
};

float parameter[MAX_PARAMETERS];
static float s_parameter_defaults[MAX_PARAMETERS];

#define PARAM_CFG(name_, idx_, target_, min_, max_, step_, group_, object_, item_) \
    { (name_), (idx_), (target_), (min_), (max_), (step_), (group_), (object_), (item_) }

ParameterConfig param_config[] = {
    PARAM_CFG("Gyro Roll Kp",            0,  &gyro_pid_roll.Kp,                -10.0f,   100.0f,   0.01f,  PARAM_GROUP_GYRO,    PARAM_OBJECT_ROLL,   PARAM_ITEM_KP),
    PARAM_CFG("Gyro Roll Ki",            1,  &gyro_pid_roll.Ki,                -10.0f,   100.0f,   0.01f,  PARAM_GROUP_GYRO,    PARAM_OBJECT_ROLL,   PARAM_ITEM_KI),
    PARAM_CFG("Gyro Roll Kd",            2,  &gyro_pid_roll.Kd,                  0.0f,    10.0f,   0.001f, PARAM_GROUP_GYRO,    PARAM_OBJECT_ROLL,   PARAM_ITEM_KD),
    PARAM_CFG("Gyro Roll Out Limit",     3,  &gyro_pid_roll.output_limit,        0.0f, 10000.0f, 100.0f,    PARAM_GROUP_GYRO,    PARAM_OBJECT_ROLL,   PARAM_ITEM_OUTPUT_LIMIT),
    PARAM_CFG("Gyro Roll Int Limit",     4,  &gyro_pid_roll.integral_limit,      0.0f,   100.0f,  10.0f,    PARAM_GROUP_GYRO,    PARAM_OBJECT_ROLL,   PARAM_ITEM_INTEGRAL_LIMIT),

    PARAM_CFG("Gyro Pitch Kp",           5,  &gyro_pid_pitch.Kp,               -10.0f,   100.0f,   0.01f,  PARAM_GROUP_GYRO,    PARAM_OBJECT_PITCH,  PARAM_ITEM_KP),
    PARAM_CFG("Gyro Pitch Ki",           6,  &gyro_pid_pitch.Ki,               -10.0f,   100.0f,   0.01f,  PARAM_GROUP_GYRO,    PARAM_OBJECT_PITCH,  PARAM_ITEM_KI),
    PARAM_CFG("Gyro Pitch Kd",           7,  &gyro_pid_pitch.Kd,               -10.0f,   100.0f,   0.001f, PARAM_GROUP_GYRO,    PARAM_OBJECT_PITCH,  PARAM_ITEM_KD),
    PARAM_CFG("Gyro Pitch Out Limit",    8,  &gyro_pid_pitch.output_limit,       0.0f,  1000.0f, 100.0f,    PARAM_GROUP_GYRO,    PARAM_OBJECT_PITCH,  PARAM_ITEM_OUTPUT_LIMIT),
    PARAM_CFG("Gyro Pitch Int Limit",    9,  &gyro_pid_pitch.integral_limit,     0.0f,  1000.0f,  10.0f,    PARAM_GROUP_GYRO,    PARAM_OBJECT_PITCH,  PARAM_ITEM_INTEGRAL_LIMIT),

    PARAM_CFG("Gyro Yaw Kp",            10,  &gyro_pid_yaw.Kp,                 -10.0f,   100.0f,   0.01f,  PARAM_GROUP_GYRO,    PARAM_OBJECT_YAW,    PARAM_ITEM_KP),
    PARAM_CFG("Gyro Yaw Ki",            11,  &gyro_pid_yaw.Ki,                 -10.0f,   100.0f,   0.01f,  PARAM_GROUP_GYRO,    PARAM_OBJECT_YAW,    PARAM_ITEM_KI),
    PARAM_CFG("Gyro Yaw Kd",            12,  &gyro_pid_yaw.Kd,                 -10.0f,   100.0f,   0.001f, PARAM_GROUP_GYRO,    PARAM_OBJECT_YAW,    PARAM_ITEM_KD),
    PARAM_CFG("Gyro Yaw Out Limit",     13,  &gyro_pid_yaw.output_limit,         0.0f, 10000.0f, 100.0f,    PARAM_GROUP_GYRO,    PARAM_OBJECT_YAW,    PARAM_ITEM_OUTPUT_LIMIT),
    PARAM_CFG("Gyro Yaw Int Limit",     14,  &gyro_pid_yaw.integral_limit,       0.0f,  1000.0f,  10.0f,    PARAM_GROUP_GYRO,    PARAM_OBJECT_YAW,    PARAM_ITEM_INTEGRAL_LIMIT),

    PARAM_CFG("Angle Roll Kp",          15,  &angle_pid_roll.Kp,              -10.0f,   100.0f,   0.10f,  PARAM_GROUP_ANGLE,   PARAM_OBJECT_ROLL,   PARAM_ITEM_KP),
    PARAM_CFG("Angle Roll Ki",          16,  &angle_pid_roll.Ki,              -10.0f,   100.0f,   0.01f,  PARAM_GROUP_ANGLE,   PARAM_OBJECT_ROLL,   PARAM_ITEM_KI),
    PARAM_CFG("Angle Roll Kd",          17,  &angle_pid_roll.Kd,              -10.0f,   100.0f,   0.01f,  PARAM_GROUP_ANGLE,   PARAM_OBJECT_ROLL,   PARAM_ITEM_KD),
    PARAM_CFG("Angle Roll Out Limit",   18,  &angle_pid_roll.output_limit,      0.0f,  1000.0f, 100.0f,    PARAM_GROUP_ANGLE,   PARAM_OBJECT_ROLL,   PARAM_ITEM_OUTPUT_LIMIT),
    PARAM_CFG("Angle Roll Int Limit",   19,  &angle_pid_roll.integral_limit,    0.0f,  1000.0f,  10.0f,    PARAM_GROUP_ANGLE,   PARAM_OBJECT_ROLL,   PARAM_ITEM_INTEGRAL_LIMIT),

    PARAM_CFG("Angle Pitch Kp",         20,  &angle_pid_pitch.Kp,             -10.0f,   100.0f,   0.10f,  PARAM_GROUP_ANGLE,   PARAM_OBJECT_PITCH,  PARAM_ITEM_KP),
    PARAM_CFG("Angle Pitch Ki",         21,  &angle_pid_pitch.Ki,             -10.0f,   100.0f,   0.01f,  PARAM_GROUP_ANGLE,   PARAM_OBJECT_PITCH,  PARAM_ITEM_KI),
    PARAM_CFG("Angle Pitch Kd",         22,  &angle_pid_pitch.Kd,               0.0f,   100.0f,   0.01f,  PARAM_GROUP_ANGLE,   PARAM_OBJECT_PITCH,  PARAM_ITEM_KD),
    PARAM_CFG("Angle Pitch Out Limit",  23,  &angle_pid_pitch.output_limit,     0.0f, 10000.0f, 100.0f,    PARAM_GROUP_ANGLE,   PARAM_OBJECT_PITCH,  PARAM_ITEM_OUTPUT_LIMIT),
    PARAM_CFG("Angle Pitch Int Limit",  24,  &angle_pid_pitch.integral_limit,   0.0f,  1000.0f,  10.0f,    PARAM_GROUP_ANGLE,   PARAM_OBJECT_PITCH,  PARAM_ITEM_INTEGRAL_LIMIT),

    PARAM_CFG("Angle Yaw Kp",           25,  &angle_pid_yaw.Kp,               -10.0f,   100.0f,   0.10f,  PARAM_GROUP_ANGLE,   PARAM_OBJECT_YAW,    PARAM_ITEM_KP),
    PARAM_CFG("Angle Yaw Ki",           26,  &angle_pid_yaw.Ki,               -10.0f,   100.0f,   0.01f,  PARAM_GROUP_ANGLE,   PARAM_OBJECT_YAW,    PARAM_ITEM_KI),
    PARAM_CFG("Angle Yaw Kd",           27,  &angle_pid_yaw.Kd,               -10.0f,   100.0f,   0.01f,  PARAM_GROUP_ANGLE,   PARAM_OBJECT_YAW,    PARAM_ITEM_KD),
    PARAM_CFG("Angle Yaw Out Limit",    28,  &angle_pid_yaw.output_limit,       0.0f,  1000.0f, 100.0f,    PARAM_GROUP_ANGLE,   PARAM_OBJECT_YAW,    PARAM_ITEM_OUTPUT_LIMIT),
    PARAM_CFG("Angle Yaw Int Limit",    29,  &angle_pid_yaw.integral_limit,     0.0f,  1000.0f,  10.0f,    PARAM_GROUP_ANGLE,   PARAM_OBJECT_YAW,    PARAM_ITEM_INTEGRAL_LIMIT),
    PARAM_CFG("Height Kp",              30,  &HeightPID.Kp,                   -10.0f,   100.0f,   0.10f,  PARAM_GROUP_HEIGHT,  PARAM_OBJECT_HEIGHT, PARAM_ITEM_KP),
    PARAM_CFG("Height Ki",              31,  &HeightPID.Ki,                   -10.0f,   100.0f,   0.01f,  PARAM_GROUP_HEIGHT,  PARAM_OBJECT_HEIGHT, PARAM_ITEM_KI),
    PARAM_CFG("Height Kd",              32,  &HeightPID.Kd,                   -10.0f,   100.0f,   0.01f,  PARAM_GROUP_HEIGHT,  PARAM_OBJECT_HEIGHT, PARAM_ITEM_KD),
    PARAM_CFG("Height Out Limit",       33,  &HeightPID.output_limit,           0.0f,  1000.0f,  10.0f,    PARAM_GROUP_HEIGHT,  PARAM_OBJECT_HEIGHT, PARAM_ITEM_OUTPUT_LIMIT),
    PARAM_CFG("Height Int Limit",       34,  &HeightPID.integral_limit,         0.0f,  1000.0f,   5.0f,    PARAM_GROUP_HEIGHT,  PARAM_OBJECT_HEIGHT, PARAM_ITEM_INTEGRAL_LIMIT),

    /* 速度环 X (vel_pid_x): 前向速度 vx_cms -> pitch 修正
     * 速度环 Y (vel_pid_y): 右向速度 vy_cms -> roll  修正
     * Output Limit / Integral Limit 单位: deg，速度环输出限幅只由 PID 内部 output_limit 决定。 */
    PARAM_CFG("Vel X Kp",               35,  &vel_pid_x.Kp,                    -10.0f,   10.0f,   0.01f,  PARAM_GROUP_VELOCITY, PARAM_OBJECT_VEL_X,  PARAM_ITEM_KP),
    PARAM_CFG("Vel X Ki",               36,  &vel_pid_x.Ki,                    -10.0f,   10.0f,   0.001f, PARAM_GROUP_VELOCITY, PARAM_OBJECT_VEL_X,  PARAM_ITEM_KI),
    PARAM_CFG("Vel X Kd",               37,  &vel_pid_x.Kd,                    -10.0f,   10.0f,   0.001f, PARAM_GROUP_VELOCITY, PARAM_OBJECT_VEL_X,  PARAM_ITEM_KD),
    PARAM_CFG("Vel X Out Limit",        38,  &vel_pid_x.output_limit,            0.0f,   30.0f,   0.5f,   PARAM_GROUP_VELOCITY, PARAM_OBJECT_VEL_X,  PARAM_ITEM_OUTPUT_LIMIT),
    PARAM_CFG("Vel X Int Limit",        39,  &vel_pid_x.integral_limit,          0.0f,   30.0f,   0.5f,   PARAM_GROUP_VELOCITY, PARAM_OBJECT_VEL_X,  PARAM_ITEM_INTEGRAL_LIMIT),

    PARAM_CFG("Vel Y Kp",               40,  &vel_pid_y.Kp,                    -10.0f,   10.0f,   0.01f,  PARAM_GROUP_VELOCITY, PARAM_OBJECT_VEL_Y,  PARAM_ITEM_KP),
    PARAM_CFG("Vel Y Ki",               41,  &vel_pid_y.Ki,                    -10.0f,   10.0f,   0.001f, PARAM_GROUP_VELOCITY, PARAM_OBJECT_VEL_Y,  PARAM_ITEM_KI),
    PARAM_CFG("Vel Y Kd",               42,  &vel_pid_y.Kd,                    -10.0f,   10.0f,   0.001f, PARAM_GROUP_VELOCITY, PARAM_OBJECT_VEL_Y,  PARAM_ITEM_KD),
    PARAM_CFG("Vel Y Out Limit",        43,  &vel_pid_y.output_limit,            0.0f,   30.0f,   0.5f,   PARAM_GROUP_VELOCITY, PARAM_OBJECT_VEL_Y,  PARAM_ITEM_OUTPUT_LIMIT),
    PARAM_CFG("Vel Y Int Limit",        44,  &vel_pid_y.integral_limit,          0.0f,   30.0f,   0.5f,   PARAM_GROUP_VELOCITY, PARAM_OBJECT_VEL_Y,  PARAM_ITEM_INTEGRAL_LIMIT)
};

uint8 parameter_count = (uint8)(sizeof(param_config) / sizeof(param_config[0]));

static ParameterMenuState s_menu_state = PARAM_MENU_ROOT;
static uint8 s_root_selection = 0;
static uint8 s_object_selection = 0;
static uint8 s_item_selection = 0;
static uint8 s_current_group = PARAM_GROUP_GYRO;
static float s_edit_backup_value = 0.0f;
static uint8 s_defaults_captured = 0;
static uint8 s_dirty = 0;

float truncate_by_step(float value, float step)
{
    uint8 is_negative = 0;
    uint32 int_multiplier = 0;
    float offset = 0.0f;
    float multiplier = 0.0f;

    if(step <= 0.0f)
    {
        return value;
    }

    if(value < 0.0f)
    {
        is_negative = 1;
        value = -value;
    }

    offset = step * 0.0001f;
    multiplier = (value + offset) / step;
    int_multiplier = (uint32)multiplier;
    value = (float)int_multiplier * step;

    return is_negative ? -value : value;
}

void format_float_truncate(char *buffer, float value, uint8 width, uint8 precision)
{
    uint8 is_negative = 0;
    uint32 int_part = 0;
    uint32 frac_digits = 0;
    uint32 multiplier = 1;
    char temp[32];
    uint8 i = 0;
    float frac_part = 0.0f;

    if(value < 0.0f)
    {
        is_negative = 1;
        value = -value;
    }

    int_part = (uint32)value;
    frac_part = value - (float)int_part;

    for(i = 0; i < precision; i++)
    {
        multiplier *= 10U;
    }

    if(precision > 0U)
    {
        const float epsilon = 1e-6f;
        float scaled = (frac_part + epsilon) * (float)multiplier;
        if(scaled >= (float)multiplier)
        {
            scaled = (float)multiplier - 1.0f;
        }
        frac_digits = (uint32)scaled;
        snprintf(temp, sizeof(temp), "%s%lu.%0*lu",
                 is_negative ? "-" : "",
                 (unsigned long)int_part,
                 precision,
                 (unsigned long)frac_digits);
    }
    else
    {
        snprintf(temp, sizeof(temp), "%s%lu",
                 is_negative ? "-" : "",
                 (unsigned long)int_part);
    }

    snprintf(buffer, 24, "%*s ", width, temp);
}

static float parameter_sanitize_value(uint8 config_index, float value)
{
    const ParameterConfig *config = &param_config[config_index];

    if(!(value == value) || value > 1e10f || value < -1e10f)
    {
        value = 0.0f;
    }

    if(value > config->max_value)
    {
        value = config->max_value;
    }
    if(value < config->min_value)
    {
        value = config->min_value;
    }

    return truncate_by_step(value, config->step);
}

static uint32 parameter_checksum_update(uint32 hash, const void *data, uint32 length)
{
    const uint8 *bytes = (const uint8 *)data;
    uint32 i = 0;

    for(i = 0; i < length; i++)
    {
        hash ^= bytes[i];
        hash *= 16777619UL;
    }

    return hash;
}

static uint32 parameter_record_checksum(const ParameterFlashRecord *record)
{
    uint32 hash = 2166136261UL;
    uint32 value_bytes = (uint32)record->param_count * (uint32)sizeof(float);

    hash = parameter_checksum_update(hash, &record->magic, (uint32)sizeof(record->magic));
    hash = parameter_checksum_update(hash, &record->version, (uint32)sizeof(record->version));
    hash = parameter_checksum_update(hash, &record->param_count, (uint32)sizeof(record->param_count));
    hash = parameter_checksum_update(hash, record->values, value_bytes);

    return hash;
}

static uint8 parameter_record_is_valid(const ParameterFlashRecord *record)
{
    uint8 i = 0;

    if(record->magic != PARAMETER_FLASH_MAGIC)
    {
        return 0;
    }
    if(record->version != PARAMETER_FLASH_VERSION)
    {
        return 0;
    }
    if(record->param_count != parameter_count || record->param_count > MAX_PARAMETERS)
    {
        return 0;
    }
    if(record->checksum != parameter_record_checksum(record))
    {
        return 0;
    }

    for(i = 0; i < record->param_count; i++)
    {
        if(!(record->values[i] == record->values[i]) ||
           record->values[i] > 1e10f ||
           record->values[i] < -1e10f)
        {
            return 0;
        }
    }

    return 1;
}

static void parameter_build_record(ParameterFlashRecord *record)
{
    uint8 i = 0;

    memset(record, 0, sizeof(*record));
    record->magic = PARAMETER_FLASH_MAGIC;
    record->version = PARAMETER_FLASH_VERSION;
    record->param_count = parameter_count;

    for(i = 0; i < parameter_count; i++)
    {
        record->values[i] = parameter[param_config[i].index];
    }

    record->checksum = parameter_record_checksum(record);
}

static uint8 parameter_read_record(ParameterFlashRecord *record)
{
    flash_read_page_to_buffer(0, PARAMETER_INDEX, (sizeof(*record) + 3U) / 4U);
    memcpy(record, flash_union_buffer, sizeof(*record));
    return parameter_record_is_valid(record);
}

static uint8 parameter_runtime_differs_from_flash(void)
{
    ParameterFlashRecord record;
    uint8 i = 0;

    if(!parameter_read_record(&record))
    {
        return 1;
    }

    for(i = 0; i < parameter_count; i++)
    {
        float flash_value = parameter_sanitize_value(i, record.values[i]);
        float runtime_value = parameter[param_config[i].index];

        if(flash_value != runtime_value)
        {
            return 1;
        }
    }

    return 0;
}

static void apply_gyro_parameter(uint8 axis, uint8 item, float value)
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float out_limit = 0.0f;
    float int_limit = 0.0f;

    Flight_Control_Get_Gyro_PID(axis, &kp, &ki, &kd);
    Flight_Control_Get_Gyro_Limits(axis, &out_limit, &int_limit);

    switch(item)
    {
        case PARAM_ITEM_KP:             kp = value; break;
        case PARAM_ITEM_KI:             ki = value; break;
        case PARAM_ITEM_KD:             kd = value; break;
        case PARAM_ITEM_OUTPUT_LIMIT:   out_limit = value; break;
        case PARAM_ITEM_INTEGRAL_LIMIT: int_limit = value; break;
        default: return;
    }

    Flight_Control_Set_Gyro_PID(axis, kp, ki, kd);
    Flight_Control_Set_Gyro_Limits(axis, out_limit, int_limit);
}

static void apply_angle_parameter(uint8 axis, uint8 item, float value)
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float out_limit = 0.0f;
    float int_limit = 0.0f;

    Flight_Control_Get_Angle_PID(axis, &kp, &ki, &kd);
    Flight_Control_Get_Angle_Limits(axis, &out_limit, &int_limit);

    switch(item)
    {
        case PARAM_ITEM_KP:             kp = value; break;
        case PARAM_ITEM_KI:             ki = value; break;
        case PARAM_ITEM_KD:             kd = value; break;
        case PARAM_ITEM_OUTPUT_LIMIT:   out_limit = value; break;
        case PARAM_ITEM_INTEGRAL_LIMIT: int_limit = value; break;
        default: return;
    }

    Flight_Control_Set_Angle_PID(axis, kp, ki, kd);
    Flight_Control_Set_Angle_Limits(axis, out_limit, int_limit);
}

static void apply_height_parameter(uint8 item, float value)
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float out_limit = 0.0f;
    float int_limit = 0.0f;

    Flight_Control_Get_Height_PID(&kp, &ki, &kd);
    Flight_Control_Get_Height_Limits(&out_limit, &int_limit);

    switch(item)
    {
        case PARAM_ITEM_KP:             kp = value; break;
        case PARAM_ITEM_KI:             ki = value; break;
        case PARAM_ITEM_KD:             kd = value; break;
        case PARAM_ITEM_OUTPUT_LIMIT:   out_limit = value; break;
        case PARAM_ITEM_INTEGRAL_LIMIT: int_limit = value; break;
        default: return;
    }

    Flight_Control_Set_Height_PID(kp, ki, kd);
    Flight_Control_Set_Height_Limits(out_limit, int_limit);
}

static void parameter_sync_from_runtime(void)
{
    uint8 i = 0;

    for(i = 0; i < parameter_count; i++)
    {
        const ParameterConfig *config = &param_config[i];
        float value = 0.0f;

        if(config->target_var != NULL)
        {
            value = *(config->target_var);
        }

        parameter[config->index] = parameter_sanitize_value(i, value);
    }
}

static void parameter_apply_all_current_values(void)
{
    uint8 i = 0;

    for(i = 0; i < parameter_count; i++)
    {
        Parameter_Apply_Config_Value(i, parameter[param_config[i].index]);
    }
}

void Parameter_Capture_Runtime_Defaults(void)
{
    uint8 i = 0;

    parameter_sync_from_runtime();
    memset(s_parameter_defaults, 0, sizeof(s_parameter_defaults));

    for(i = 0; i < parameter_count; i++)
    {
        s_parameter_defaults[param_config[i].index] = parameter[param_config[i].index];
    }

    s_defaults_captured = 1;
}

void Parameter_Apply_Config_Value(uint8 config_index, float value)
{
    uint8 idx = 0;

    if(config_index >= parameter_count)
    {
        return;
    }

    value = parameter_sanitize_value(config_index, value);
    idx = param_config[config_index].index;
    parameter[idx] = value;

    switch(param_config[config_index].group)
    {
        case PARAM_GROUP_GYRO:
            apply_gyro_parameter(param_config[config_index].object, param_config[config_index].item, value);
            break;
        case PARAM_GROUP_ANGLE:
            apply_angle_parameter(param_config[config_index].object, param_config[config_index].item, value);
            break;
        case PARAM_GROUP_HEIGHT:
            apply_height_parameter(param_config[config_index].item, value);
            break;
        case PARAM_GROUP_VELOCITY:
        default:
            /* 速度环 PID 没走 Flight_Control_* 包装层, 直接写 PID 字段即可。 */
            if(param_config[config_index].target_var != NULL)
            {
                *(param_config[config_index].target_var) = value;
            }
            break;
    }
}

void Parameter_reset_to_default(void)
{
    uint8 i = 0;

    if(!s_defaults_captured)
    {
        Parameter_Capture_Runtime_Defaults();
    }

    for(i = 0; i < parameter_count; i++)
    {
        parameter[param_config[i].index] = s_parameter_defaults[param_config[i].index];
    }

    parameter_apply_all_current_values();
}

void Parameter_flash_load(void)
{
    ParameterFlashRecord record;
    uint8 i = 0;

    if(!parameter_read_record(&record))
    {
        Parameter_reset_to_default();
        return;
    }

    for(i = 0; i < parameter_count; i++)
    {
        parameter[param_config[i].index] = parameter_sanitize_value(i, record.values[i]);
    }

    parameter_apply_all_current_values();
}

void Parameter_flash_save(void)
{
    ParameterFlashRecord record;

    parameter_sync_from_runtime();
    parameter_build_record(&record);

    memset(flash_union_buffer, 0xFF, sizeof(flash_union_buffer));
    memcpy(flash_union_buffer, &record, sizeof(record));
    flash_erase_page(0, PARAMETER_INDEX);
    flash_write_page_from_buffer(0, PARAMETER_INDEX, (sizeof(record) + 3U) / 4U);
}

void Parameter_Load_Or_Init_Storage(void)
{
    ParameterFlashRecord record;

    if(!s_defaults_captured)
    {
        Parameter_Capture_Runtime_Defaults();
    }

    if(parameter_count == 0U)
    {
        return;
    }

    if(parameter_read_record(&record))
    {
        Parameter_flash_load();
    }
    else
    {
        Parameter_reset_to_default();
        Parameter_flash_save();
    }
}

void Parameter_System_Init(void)
{
    Parameter_Capture_Runtime_Defaults();
    //Parameter_Load_Or_Init_Storage();//注释掉该函数则不会使用flash中的参数，仍然使用默认值，手动调参依然有效
}

static uint8 parameter_find_config_index(uint8 group, uint8 object, uint8 item)
{
    uint8 i = 0;

    for(i = 0; i < parameter_count; i++)
    {
        if(param_config[i].group == group &&
           param_config[i].object == object &&
           param_config[i].item == item)
        {
            return i;
        }
    }

    return 0xFFU;
}

static uint8 parameter_current_object(void)
{
    return s_group_objects[s_current_group][s_object_selection];
}

static void parameter_draw_root_menu(void)
{
    ui_parameter_draw_root_menu(s_dirty, s_root_menu_items, PARAM_ROOT_COUNT, s_root_selection);
}

static void parameter_draw_object_menu(void)
{
    uint8 count = s_group_object_count[s_current_group];

    ui_parameter_draw_object_menu(s_dirty,
                                  s_group_titles[s_current_group],
                                  s_object_labels,
                                  s_group_objects[s_current_group],
                                  count,
                                  s_object_selection);
}

static void parameter_draw_item_menu(uint8 edit_mode)
{
    uint8 i = 0U;
    char title[32];
    uint8 object_id = parameter_current_object();
    float item_values[PARAM_ITEM_COUNT] = {0.0f};
    uint8 item_available[PARAM_ITEM_COUNT] = {0U};

    snprintf(title, sizeof(title), "%s / %s%s",
             s_group_titles[s_current_group],
             s_object_labels[object_id],
             edit_mode ? " / EDIT" : "");

    for(i = 0; i < PARAM_ITEM_COUNT; i++)
    {
        uint8 config_index = parameter_find_config_index(s_current_group, object_id, i);

        if(config_index == 0xFFU)
        {
            continue;
        }

        item_available[i] = 1U;
        item_values[i] = parameter[param_config[config_index].index];
    }

    ui_parameter_draw_item_menu(s_dirty,
                                title,
                                s_item_labels,
                                item_values,
                                item_available,
                                PARAM_ITEM_COUNT,
                                s_item_selection,
                                edit_mode);
}

static void parameter_draw_confirm(const char *title, const char *message, uint16 color)
{
    ui_parameter_draw_confirm(s_dirty, title, message, color);
}

static void parameter_draw_status(const char *message, uint16 color, uint16 delay_ms)
{
    ui_parameter_draw_status(message, color, delay_ms);
}

static void parameter_render(void)
{
    switch(s_menu_state)
    {
        case PARAM_MENU_ROOT:
            parameter_draw_root_menu();
            break;
        case PARAM_MENU_OBJECTS:
            parameter_draw_object_menu();
            break;
        case PARAM_MENU_ITEMS:
            parameter_draw_item_menu(0U);
            break;
        case PARAM_MENU_EDIT:
            parameter_draw_item_menu(1U);
            break;
        case PARAM_MENU_CONFIRM_SAVE:
            parameter_draw_confirm("Save Parameters", "Save all parameters?", RGB565_CYAN);
            break;
        case PARAM_MENU_CONFIRM_RESET:
            parameter_draw_confirm("Reset Defaults", "Reset to defaults?", RGB565_YELLOW);
            break;
        case PARAM_MENU_CONFIRM_SAVE_DEFAULTS:
            parameter_draw_confirm("Save Defaults", "Save defaults to flash?", RGB565_MAGENTA);
            break;
        default:
            parameter_draw_root_menu();
            break;
    }
}

static void parameter_enter_root(void)
{
    s_menu_state = PARAM_MENU_ROOT;
}

static void parameter_begin_edit(void)
{
    uint8 config_index = parameter_find_config_index(s_current_group, parameter_current_object(), s_item_selection);

    if(config_index == 0xFFU)
    {
        return;
    }

    s_edit_backup_value = parameter[param_config[config_index].index];
    s_menu_state = PARAM_MENU_EDIT;
}

static void parameter_adjust_selected_value(int8 direction)
{
    uint8 config_index = parameter_find_config_index(s_current_group, parameter_current_object(), s_item_selection);
    uint8 idx = 0;
    float new_value = 0.0f;

    if(config_index == 0xFFU)
    {
        return;
    }

    idx = param_config[config_index].index;
    new_value = parameter[idx] + (direction > 0 ? param_config[config_index].step : -param_config[config_index].step);
    new_value = parameter_sanitize_value(config_index, new_value);
    Parameter_Apply_Config_Value(config_index, new_value);
}

void parameter_menu_entry(void)
{
    uint8 exit_flag = 0;
    uint8 redraw = 1;

    parameter_sync_from_runtime();
    s_menu_state = PARAM_MENU_ROOT;
    s_root_selection = 0;
    s_object_selection = 0;
    s_item_selection = 0;
    s_current_group = PARAM_GROUP_GYRO;
    s_edit_backup_value = 0.0f;
    s_dirty = parameter_runtime_differs_from_flash();
    key_flag = 0;

    while(!exit_flag)
    {
        uint8 key = 0;

        core_link_user_poll();

        if(redraw)
        {
            parameter_render();
            redraw = 0;
        }

        if(!key_flag)
        {
            system_delay_ms(20);
            continue;
        }

        key = key_flag;
        key_flag = 0;

        switch(s_menu_state)
        {
            case PARAM_MENU_ROOT:
                if(key == Button_1)
                {
                    s_root_selection = (s_root_selection > 0U) ? (uint8)(s_root_selection - 1U) : (uint8)(PARAM_ROOT_COUNT - 1U);
                    redraw = 1;
                }
                else if(key == Button_2)
                {
                    s_root_selection = (uint8)((s_root_selection + 1U) % PARAM_ROOT_COUNT);
                    redraw = 1;
                }
                else if(key == Button_3)
                {
                    if(s_root_selection <= PARAM_ROOT_VELOCITY)
                    {
                        s_current_group = s_root_selection;
                        s_object_selection = 0;
                        s_menu_state = PARAM_MENU_OBJECTS;
                    }
                    else if(s_root_selection == PARAM_ROOT_SAVE)
                    {
                        s_menu_state = PARAM_MENU_CONFIRM_SAVE;
                    }
                    else
                    {
                        s_menu_state = PARAM_MENU_CONFIRM_RESET;
                    }
                    redraw = 1;
                }
                else if(key == Button_4)
                {
                    exit_flag = 1;
                }
                break;

            case PARAM_MENU_OBJECTS:
                if(key == Button_1)
                {
                    uint8 count = s_group_object_count[s_current_group];
                    s_object_selection = (s_object_selection > 0U) ? (uint8)(s_object_selection - 1U) : (uint8)(count - 1U);
                    redraw = 1;
                }
                else if(key == Button_2)
                {
                    uint8 count = s_group_object_count[s_current_group];
                    s_object_selection = (uint8)((s_object_selection + 1U) % count);
                    redraw = 1;
                }
                else if(key == Button_3)
                {
                    s_item_selection = 0;
                    s_menu_state = PARAM_MENU_ITEMS;
                    redraw = 1;
                }
                else if(key == Button_4)
                {
                    parameter_enter_root();
                    redraw = 1;
                }
                break;

            case PARAM_MENU_ITEMS:
                if(key == Button_1)
                {
                    s_item_selection = (s_item_selection > 0U) ? (uint8)(s_item_selection - 1U) : (uint8)(PARAM_ITEM_COUNT - 1U);
                    redraw = 1;
                }
                else if(key == Button_2)
                {
                    s_item_selection = (uint8)((s_item_selection + 1U) % PARAM_ITEM_COUNT);
                    redraw = 1;
                }
                else if(key == Button_3)
                {
                    parameter_begin_edit();
                    redraw = 1;
                }
                else if(key == Button_4)
                {
                    s_menu_state = PARAM_MENU_OBJECTS;
                    redraw = 1;
                }
                break;

            case PARAM_MENU_EDIT:
                if(key == Button_1)
                {
                    parameter_adjust_selected_value(1);
                    redraw = 1;
                }
                else if(key == Button_2)
                {
                    parameter_adjust_selected_value(-1);
                    redraw = 1;
                }
                else if(key == Button_3)
                {
                    uint8 config_index = parameter_find_config_index(s_current_group, parameter_current_object(), s_item_selection);
                    if(config_index != 0xFFU &&
                       parameter[param_config[config_index].index] != s_edit_backup_value)
                    {
                        s_dirty = 1;
                    }
                    s_menu_state = PARAM_MENU_ITEMS;
                    redraw = 1;
                }
                else if(key == Button_4)
                {
                    uint8 config_index = parameter_find_config_index(s_current_group, parameter_current_object(), s_item_selection);
                    if(config_index != 0xFFU)
                    {
                        Parameter_Apply_Config_Value(config_index, s_edit_backup_value);
                    }
                    s_menu_state = PARAM_MENU_ITEMS;
                    redraw = 1;
                }
                break;

            case PARAM_MENU_CONFIRM_SAVE:
                if(key == Button_3)
                {
                    Parameter_flash_save();
                    s_dirty = 0;
                    parameter_draw_status("Parameters saved", RGB565_GREEN, 600);
                    parameter_enter_root();
                    redraw = 1;
                }
                else if(key == Button_4)
                {
                    parameter_enter_root();
                    redraw = 1;
                }
                break;

            case PARAM_MENU_CONFIRM_RESET:
                if(key == Button_3)
                {
                    Parameter_reset_to_default();
                    s_dirty = 1;
                    s_menu_state = PARAM_MENU_CONFIRM_SAVE_DEFAULTS;
                    redraw = 1;
                }
                else if(key == Button_4)
                {
                    parameter_enter_root();
                    redraw = 1;
                }
                break;

            case PARAM_MENU_CONFIRM_SAVE_DEFAULTS:
                if(key == Button_3)
                {
                    Parameter_flash_save();
                    s_dirty = 0;
                    parameter_draw_status("Defaults saved", RGB565_GREEN, 600);
                    parameter_enter_root();
                    redraw = 1;
                }
                else if(key == Button_4)
                {
                    parameter_draw_status("Defaults loaded only", RGB565_YELLOW, 600);
                    parameter_enter_root();
                    redraw = 1;
                }
                break;

            default:
                parameter_enter_root();
                redraw = 1;
                break;
        }

        system_delay_ms(20);
    }

    key_flag = Button_4;
}

uint8 check_parameter_index_conflicts(void)
{
    uint8 index_used[MAX_PARAMETERS] = {0};
    uint8 conflicts = 0;
    uint8 i = 0;

    for(i = 0; i < parameter_count; i++)
    {
        uint8 idx = param_config[i].index;

        if(idx >= MAX_PARAMETERS)
        {
            conflicts++;
            continue;
        }

        if(index_used[idx])
        {
            conflicts++;
        }
        else
        {
            index_used[idx] = 1;
        }
    }

    return conflicts;
}

