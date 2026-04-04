#include "pmw_flow.h"
#include "common/project_headfile.h"
#include <math.h>

#define PMW_FLOW_DEG2RAD                    (0.01745329251994329577f)
#define PMW_FLOW_RAD_PER_COUNT_X            (1.0f / 385.0f)
#define PMW_FLOW_RAD_PER_COUNT_Y            (1.0f / 385.0f)
#define PMW_FLOW_SENSOR_X_SIGN              (1.0f)
#define PMW_FLOW_SENSOR_Y_SIGN              (1.0f)
/*
 * Flight-test result for rotational compensation:
 * - flow X follows gyro Y
 * - flow Y follows gyro X
 * Keep the cross-axis mapping explicit so later tuning only needs macro changes.
 */
#define PMW_FLOW_GYRO_X_DELTA_SIGN          (1.0f)
#define PMW_FLOW_GYRO_Y_DELTA_SIGN          (1.0f)
#define PMW_FLOW_COMP_X_FROM_GYRO_X_SIGN    (0.0f)
#define PMW_FLOW_COMP_X_FROM_GYRO_Y_SIGN    (-1.0f)
#define PMW_FLOW_COMP_Y_FROM_GYRO_X_SIGN    (-1.0f)
#define PMW_FLOW_COMP_Y_FROM_GYRO_Y_SIGN    (0.0f)
/*
 * PX4-style body velocity mapping from compensated flow integral.
 * Keep the mapping configurable because the final body-frame sign depends on
 * camera installation and the project's body-axis convention.
 */
#define PMW_FLOW_BODY_VX_FROM_FLOW_Y_SIGN   (-1.0f)
#define PMW_FLOW_BODY_VY_FROM_FLOW_X_SIGN   (1.0f)
#define PMW_FLOW_MIN_HEIGHT_CM              (5.0f)
#define PMW_FLOW_MAX_HEIGHT_CM              (400.0f)
#define PMW_FLOW_MIN_DT_S                   (0.001f)
#define PMW_FLOW_MAX_DT_S                   (0.100f)
#define PMW_FLOW_MAX_ABS_DELTA_COUNT        (5000)

static float g_pmw_flow_gyro_flow_x_delta_rad = 0.0f;
static float g_pmw_flow_gyro_flow_y_delta_rad = 0.0f;
static PMWFlowState_t g_pmw_flow_publish_slots[2] = {0};
static volatile uint8 g_pmw_flow_publish_index = 0U;
static volatile uint32 g_pmw_flow_publish_seq = 0U;
static PMWFlowState_t g_pmw_flow_mainloop_state = {0};
static uint32 g_pmw_flow_mainloop_seq = 0U;

static uint8 pmw_flow_is_valid_number(float value)
{
    return (uint8)((value == value) && (value > -1000000.0f) && (value < 1000000.0f));
}

static uint8 pmw_flow_is_valid_delta_count(int16 value)
{
    return (uint8)((value > -PMW_FLOW_MAX_ABS_DELTA_COUNT) && (value < PMW_FLOW_MAX_ABS_DELTA_COUNT));
}

static void pmw_flow_publish_state(const PMWFlowState_t *state)
{
    uint8 next_index;

    if(state == NULL)
    {
        return;
    }

    next_index = (uint8)(g_pmw_flow_publish_index ^ 1U);
    g_pmw_flow_publish_slots[next_index] = *state;
    __DMB();
    g_pmw_flow_publish_index = next_index;
    __DMB();
    g_pmw_flow_publish_seq++;
}

void PMW_Flow_Init(void)
{
    PMW_Flow_Reset();
}

void PMW_Flow_Reset(void)
{
    PMWFlowState_t cleared_state = {0};

    g_pmw_flow_gyro_flow_x_delta_rad = 0.0f;
    g_pmw_flow_gyro_flow_y_delta_rad = 0.0f;
    g_pmw_flow_publish_slots[0] = cleared_state;
    g_pmw_flow_publish_slots[1] = cleared_state;
    g_pmw_flow_publish_index = 0U;
    g_pmw_flow_publish_seq = 0U;
    g_pmw_flow_mainloop_state = cleared_state;
    g_pmw_flow_mainloop_seq = 0U;
}

void PMW_Flow_AccumulateGyro(float gyro_x_rate_dps, float gyro_y_rate_dps, float dt_s)
{
    if(!(dt_s > 0.0f) ||
       !pmw_flow_is_valid_number(gyro_x_rate_dps) ||
       !pmw_flow_is_valid_number(gyro_y_rate_dps))
    {
        return;
    }

    g_pmw_flow_gyro_flow_x_delta_rad += gyro_x_rate_dps * PMW_FLOW_DEG2RAD * dt_s * PMW_FLOW_GYRO_X_DELTA_SIGN;
    g_pmw_flow_gyro_flow_y_delta_rad += gyro_y_rate_dps * PMW_FLOW_DEG2RAD * dt_s * PMW_FLOW_GYRO_Y_DELTA_SIGN;
}

void PMW_Flow_UpdateSample(int16 flow_delta_x,
                           int16 flow_delta_y,
                           float roll_deg,
                           float pitch_deg,
                           float tof_los_cm,
                           float dt_s)
{
    PMWFlowState_t new_state = {0};

    new_state.flow_delta_x_count = flow_delta_x;
    new_state.flow_delta_y_count = flow_delta_y;
    new_state.dt_s = dt_s;
    new_state.tof_los_cm = tof_los_cm;
    new_state.range_cm = tof_los_cm;
    new_state.roll_deg = roll_deg;
    new_state.pitch_deg = pitch_deg;
    new_state.gyro_flow_x_delta_rad = g_pmw_flow_gyro_flow_x_delta_rad;
    new_state.gyro_flow_y_delta_rad = g_pmw_flow_gyro_flow_y_delta_rad;
    g_pmw_flow_gyro_flow_x_delta_rad = 0.0f;
    g_pmw_flow_gyro_flow_y_delta_rad = 0.0f;

    new_state.flow_x_rad = (float)flow_delta_x * PMW_FLOW_RAD_PER_COUNT_X * PMW_FLOW_SENSOR_X_SIGN;
    new_state.flow_y_rad = (float)flow_delta_y * PMW_FLOW_RAD_PER_COUNT_Y * PMW_FLOW_SENSOR_Y_SIGN;

    if(!(dt_s >= PMW_FLOW_MIN_DT_S) ||
       !(dt_s <= PMW_FLOW_MAX_DT_S) ||
       !pmw_flow_is_valid_number(tof_los_cm) ||
       !pmw_flow_is_valid_number(roll_deg) ||
       !pmw_flow_is_valid_number(pitch_deg) ||
       !pmw_flow_is_valid_number(new_state.gyro_flow_x_delta_rad) ||
       !pmw_flow_is_valid_number(new_state.gyro_flow_y_delta_rad) ||
       !pmw_flow_is_valid_delta_count(flow_delta_x) ||
       !pmw_flow_is_valid_delta_count(flow_delta_y) ||
       (tof_los_cm < PMW_FLOW_MIN_HEIGHT_CM) ||
       (tof_los_cm > PMW_FLOW_MAX_HEIGHT_CM))
    {
        pmw_flow_publish_state(&new_state);
        return;
    }

    new_state.flow_comp_x_rad = new_state.flow_x_rad +
                                PMW_FLOW_COMP_X_FROM_GYRO_X_SIGN * new_state.gyro_flow_x_delta_rad +
                                PMW_FLOW_COMP_X_FROM_GYRO_Y_SIGN * new_state.gyro_flow_y_delta_rad;
    new_state.flow_comp_y_rad = new_state.flow_y_rad +
                                PMW_FLOW_COMP_Y_FROM_GYRO_X_SIGN * new_state.gyro_flow_x_delta_rad +
                                PMW_FLOW_COMP_Y_FROM_GYRO_Y_SIGN * new_state.gyro_flow_y_delta_rad;

    new_state.velocity_x_raw_cms = PMW_FLOW_BODY_VX_FROM_FLOW_Y_SIGN * (new_state.range_cm * new_state.flow_comp_y_rad / dt_s);
    new_state.velocity_y_raw_cms = PMW_FLOW_BODY_VY_FROM_FLOW_X_SIGN * (new_state.range_cm * new_state.flow_comp_x_rad / dt_s);

    /* Keep a distinct "final" output slot even though this first version does not add extra filtering yet. */
    new_state.velocity_x_cms = new_state.velocity_x_raw_cms;
    new_state.velocity_y_cms = new_state.velocity_y_raw_cms;
    new_state.speed_cms = sqrtf(new_state.velocity_x_cms * new_state.velocity_x_cms +
                                new_state.velocity_y_cms * new_state.velocity_y_cms);
    new_state.valid = 1U;

    pmw_flow_publish_state(&new_state);
}

void PMW_Flow_SyncMainLoopBuffer(void)
{
    uint32 seq_before;
    uint32 seq_after;
    uint8 published_index;

    if(g_pmw_flow_mainloop_seq == g_pmw_flow_publish_seq)
    {
        return;
    }

    do
    {
        seq_before = g_pmw_flow_publish_seq;
        __DMB();
        published_index = g_pmw_flow_publish_index;
        g_pmw_flow_mainloop_state = g_pmw_flow_publish_slots[published_index];
        __DMB();
        seq_after = g_pmw_flow_publish_seq;
    } while((seq_before != seq_after) || (published_index != g_pmw_flow_publish_index));

    g_pmw_flow_mainloop_seq = seq_after;
}

void PMW_Flow_GetState(PMWFlowState_t *state)
{
    if(state == NULL)
    {
        return;
    }

    PMW_Flow_SyncMainLoopBuffer();
    *state = g_pmw_flow_mainloop_state;
}
