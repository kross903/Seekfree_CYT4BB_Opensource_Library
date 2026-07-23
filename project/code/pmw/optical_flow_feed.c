#include "pmw/optical_flow_feed.h"
#include "common/PIT_Demo.h"
#include "icm/icm.h"
#include "control/height_manager.h"
#include "pmw/pmw_flow.h"

#define PMW_FLOW_TOF_SRC_NONE           PMW_FLOW_TOF_SOURCE_NONE
#define PMW_FLOW_TOF_SRC_LIVE           PMW_FLOW_TOF_SOURCE_LIVE
#define PMW_FLOW_TOF_SRC_HOLD           PMW_FLOW_TOF_SOURCE_HOLD

static volatile float g_pmw3901_flow_height_used_cm = 0.0f;
static volatile uint16 g_pmw3901_tof_stale_ticks = 0U;
static volatile uint8 g_pmw3901_tof_source = PMW_FLOW_TOF_SRC_NONE;

static float flow_sensor_get_sample_dt_s(void)
{
    float sample_dt_s = FLOW_SENSOR_NOMINAL_DT_S;

#if FLOW_SENSOR_SELECTED == FLOW_SENSOR_UPFLOW302
    if(upflow302_receive.upflow302_us > 0)
    {
        float frame_dt_s = (float)upflow302_receive.upflow302_us * 0.000001f;
        if(frame_dt_s >= 0.001f && frame_dt_s <= 0.100f)
        {
            sample_dt_s = frame_dt_s;
        }
    }
#endif

    return sample_dt_s;
}

void OpticalFlowFeed_Task(void)
{
#if FLOW_SENSOR_SELECTED == FLOW_SENSOR_PMW3901
    static uint8 pmw3901_ready = 0;
#endif
    HeightTofSnapshot_t tof_snapshot = {0};
    float flow_height_cm = 0.0f;
    float flow_sample_dt_s = flow_sensor_get_sample_dt_s();
    int16 flow_delta_x = 0;
    int16 flow_delta_y = 0;
    uint8 flow_sample_valid = 0U;
    uint8 tof_source = PMW_FLOW_TOF_SRC_NONE;

#if FLOW_SENSOR_SELECTED == FLOW_SENSOR_PMW3901
    if(!pmw3901_ready)
    {
        PMW_Flow_Reset();
        g_pmw3901_flow_height_used_cm = 0.0f;
        g_pmw3901_tof_stale_ticks = 0U;
        g_pmw3901_tof_source = PMW_FLOW_TOF_SRC_NONE;
        pmw3901_ready = (0 == pmw3901_init()) ? 1U : 0U;
        return;
    }

    pmw3901_get_motion();
    flow_delta_x = pmw3901_delta_x;
    flow_delta_y = pmw3901_delta_y;
    flow_sample_valid = 1U;
#elif FLOW_SENSOR_SELECTED == FLOW_SENSOR_UPFLOW302
    if(upflow302_finsh_flag &&
       upflow302_state_flag &&
       (FLOW_SENSOR_UPFLOW302_VALID_VALUE == upflow302_receive.upflow302_valid))
    {
        flow_delta_x = upflow302_receive.upflow302_x;
        flow_delta_y = upflow302_receive.upflow302_y;
        flow_sample_valid = 1U;
    }
    upflow302_finsh_flag = 0U;
#endif

    HeightManager_GetTofSnapshot(&tof_snapshot);
    g_pmw3901_tof_stale_ticks = (uint16)(tof_snapshot.raw_sample_age_ms / 10U);

    if(tof_snapshot.valid)
    {
        flow_height_cm = tof_snapshot.flow_height_cm;

        if(tof_snapshot.state == (uint8)HEIGHT_TOF_STATE_LIVE)
        {
            tof_source = PMW_FLOW_TOF_SRC_LIVE;
        }
        else if(tof_snapshot.state == (uint8)HEIGHT_TOF_STATE_HELD_REJECTED ||
                tof_snapshot.state == (uint8)HEIGHT_TOF_STATE_REACQUIRING)
        {
            tof_source = PMW_FLOW_TOF_SRC_HOLD;
        }
    }

    if(!flow_sample_valid)
    {
        flow_height_cm = 0.0f;
        tof_source = PMW_FLOW_TOF_SRC_NONE;
    }

    g_pmw3901_flow_height_used_cm = flow_height_cm;
    g_pmw3901_tof_source = tof_source;

    PMW_Flow_UpdateSample(flow_delta_x,
                          flow_delta_y,
                          current_roll,
                          current_pitch,
                          flow_height_cm,
                          tof_source,
                          flow_sample_valid,
                          flow_sample_dt_s,
                          X_V_F,
                          Y_V_F);
}

void OpticalFlowFeed_GetTofStatus(float *tof_height_cm, uint16 *stale_ticks, uint8 *source)
{
    if(tof_height_cm != NULL) *tof_height_cm = g_pmw3901_flow_height_used_cm;
    if(stale_ticks  != NULL) *stale_ticks  = g_pmw3901_tof_stale_ticks;
    if(source       != NULL) *source       = g_pmw3901_tof_source;
}

void PMW_Flow_Get_TOF_Status(float *tof_height_cm, uint16 *stale_ticks, uint8 *source)
{
    OpticalFlowFeed_GetTofStatus(tof_height_cm, stale_ticks, source);
}
