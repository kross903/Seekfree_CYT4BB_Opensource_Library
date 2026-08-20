#include "pmw/optical_flow_feed.h"
#include "common/PIT_Demo.h"
#include "icm/icm.h"
#include "control/height_manager.h"
#include "pmw/pmw_flow.h"
#include <math.h>

#define PMW_FLOW_TOF_SRC_NONE           PMW_FLOW_TOF_SOURCE_NONE
#define PMW_FLOW_TOF_SRC_LIVE           PMW_FLOW_TOF_SOURCE_LIVE
#define PMW_FLOW_TOF_SRC_HOLD           PMW_FLOW_TOF_SOURCE_HOLD

static volatile float g_pmw3901_flow_height_used_cm = 0.0f;
static volatile uint16 g_pmw3901_tof_stale_ticks = 0U;
static volatile uint8 g_pmw3901_tof_source = PMW_FLOW_TOF_SRC_NONE;

static uint8 g_upflow302_timing_initialized = 0U;
static uint16 g_upflow302_sync_error_count = 0U;

static float flow_cycles_to_ms(uint32 cycles)
{
    if(SystemCoreClock == 0U)
    {
        return 0.0f;
    }
    return ((float)cycles * 1000.0f) / (float)SystemCoreClock;
}

static uint8 flow_dt_pair_is_valid(float rx_dt_ms, float sensor_dt_ms)
{
    float tolerance_ms;
    float difference_ms;

    if(!(rx_dt_ms >= 1.0f) || !(rx_dt_ms <= 100.0f) ||
       !(sensor_dt_ms >= 1.0f) || !(sensor_dt_ms <= 100.0f))
    {
        return 0U;
    }

    tolerance_ms = 3.0f;
    if((sensor_dt_ms * 0.25f) > tolerance_ms)
    {
        tolerance_ms = sensor_dt_ms * 0.25f;
    }
    difference_ms = fabsf(rx_dt_ms - sensor_dt_ms);
    return (uint8)(difference_ms <= tolerance_ms);
}

static float flow_select_sample_dt_s(float rx_dt_ms, float sensor_dt_ms)
{
    if(isfinite(rx_dt_ms) && (rx_dt_ms > 0.0f))
    {
        return rx_dt_ms * 0.001f;
    }
    if(isfinite(sensor_dt_ms) && (sensor_dt_ms > 0.0f))
    {
        return sensor_dt_ms * 0.001f;
    }
    return FLOW_SENSOR_NOMINAL_DT_S;
}

static void optical_flow_process_sample(int16 flow_delta_x,
                                        int16 flow_delta_y,
                                        float flow_sample_dt_s,
                                        float gyro_flow_x_delta_rad,
                                        float gyro_flow_y_delta_rad,
                                        float gyro_flow_z_delta_rad,
                                        float flow_yaw_deg,
                                        HeightTofSnapshot_t *tof_snapshot,
                                        uint32 frame_seq,
                                        float rx_dt_ms,
                                        float sensor_dt_ms,
                                        float gyro_window_dt_ms,
                                        float consumer_delay_ms,
                                        uint8 sync_valid,
                                        uint16 dropped_frame_count,
                                        uint16 sync_error_count)
{
    float flow_height_cm = 0.0f;
    uint8 tof_source = PMW_FLOW_TOF_SRC_NONE;

    /* 光流只依赖最终处理高度。即使ToF处于保持、重捕获或失效状态，
     * 只要HeightManager仍提供有限的正高度，就继续使用该高度。 */
    if((tof_snapshot != NULL) &&
       isfinite(tof_snapshot->flow_height_cm) &&
       (tof_snapshot->flow_height_cm > 0.0f))
    {
        flow_height_cm = tof_snapshot->flow_height_cm;

        if(tof_snapshot->state == (uint8)HEIGHT_TOF_STATE_LIVE)
        {
            tof_source = PMW_FLOW_TOF_SRC_LIVE;
        }
        else
        {
            tof_source = PMW_FLOW_TOF_SRC_HOLD;
        }
    }

    g_pmw3901_flow_height_used_cm = flow_height_cm;
    g_pmw3901_tof_source = tof_source;
    PMW_Flow_SetSyncDiagnostics(frame_seq,
                                rx_dt_ms,
                                sensor_dt_ms,
                                gyro_window_dt_ms,
                                consumer_delay_ms,
                                sync_valid,
                                dropped_frame_count,
                                sync_error_count);

    PMW_Flow_UpdateSample(flow_delta_x,
                          flow_delta_y,
                          current_roll,
                          current_pitch,
                          flow_yaw_deg,
                          flow_height_cm,
                          tof_source,
                          flow_sample_dt_s,
                          gyro_flow_x_delta_rad,
                          gyro_flow_y_delta_rad,
                          gyro_flow_z_delta_rad);
}

void OpticalFlowFeed_Task(void)
{
#if FLOW_SENSOR_SELECTED == FLOW_SENSOR_PMW3901
    static uint8 pmw3901_ready = 0;
    HeightTofSnapshot_t tof_snapshot = {0};
    float gyro_x_delta_rad = 0.0f;
    float gyro_y_delta_rad = 0.0f;
    float gyro_z_delta_rad = 0.0f;

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
    HeightManager_GetTofSnapshot(&tof_snapshot);
    g_pmw3901_tof_stale_ticks = (uint16)(tof_snapshot.raw_sample_age_ms / 10U);
    PMW_Flow_GetAccumulatedGyroDelta(&gyro_x_delta_rad,
                                     &gyro_y_delta_rad,
                                     &gyro_z_delta_rad);
    optical_flow_process_sample(pmw3901_delta_x,
                                pmw3901_delta_y,
                                FLOW_SENSOR_NOMINAL_DT_S,
                                gyro_x_delta_rad,
                                gyro_y_delta_rad,
                                gyro_z_delta_rad,
                                current_yaw,
                                &tof_snapshot,
                                0U,
                                FLOW_SENSOR_NOMINAL_DT_S * 1000.0f,
                                FLOW_SENSOR_NOMINAL_DT_S * 1000.0f,
                                FLOW_SENSOR_NOMINAL_DT_S * 1000.0f,
                                0.0f,
                                1U,
                                0U,
                                0U);
#elif FLOW_SENSOR_SELECTED == FLOW_SENSOR_UPFLOW302
    upflow302_frame_t frame = {0};
    HeightTofSnapshot_t tof_snapshot = {0};

    HeightManager_GetTofSnapshot(&tof_snapshot);
    g_pmw3901_tof_stale_ticks = (uint16)(tof_snapshot.raw_sample_age_ms / 10U);

    if(upflow302_pop_latest_frame(&frame))
    {
        float rx_dt_ms = flow_cycles_to_ms(frame.gyro_window_cycles);
        float sensor_dt_ms = (float)frame.frame_dt_us * 0.001f;
        float gyro_window_dt_ms = rx_dt_ms;
        float consumer_delay_ms =
            flow_cycles_to_ms(DWT->CYCCNT - frame.frame_start_cycle);
        uint8 sync_valid = (uint8)(frame.gyro_window_valid &&
                                   frame.yaw_window_valid);
        uint8 dt_pair_valid = flow_dt_pair_is_valid(rx_dt_ms, sensor_dt_ms);
        uint8 baseline_only = (uint8)!g_upflow302_timing_initialized;
        float flow_sample_dt_s = flow_select_sample_dt_s(rx_dt_ms,
                                                         sensor_dt_ms);

        /* 时间差只保留诊断，不再拒绝帧；陀螺和Yaw窗口仍是硬门控。 */
        if(!baseline_only && (!sync_valid || !dt_pair_valid) &&
           (g_upflow302_sync_error_count < 0xFFFFU))
        {
            g_upflow302_sync_error_count++;
        }
        g_upflow302_timing_initialized = 1U;

        HeightManager_GetTofSnapshot(&tof_snapshot);
        g_pmw3901_tof_stale_ticks = (uint16)(tof_snapshot.raw_sample_age_ms / 10U);

        if(!sync_valid)
        {
            PMW_Flow_SetSyncDiagnostics(frame.frame_seq,
                                        rx_dt_ms,
                                        sensor_dt_ms,
                                        gyro_window_dt_ms,
                                        consumer_delay_ms,
                                        0U,
                                        upflow302_get_dropped_frame_count(),
                                        g_upflow302_sync_error_count);
            return;
        }

        optical_flow_process_sample(frame.flow_x,
                                    frame.flow_y,
                                    flow_sample_dt_s,
                                    frame.gyro_x_delta_rad,
                                    frame.gyro_y_delta_rad,
                                    frame.gyro_z_delta_rad,
                                    frame.yaw_mid_deg,
                                    &tof_snapshot,
                                    frame.frame_seq,
                                    rx_dt_ms,
                                    sensor_dt_ms,
                                    gyro_window_dt_ms,
                                    consumer_delay_ms,
                                    1U,
                                    upflow302_get_dropped_frame_count(),
                                    g_upflow302_sync_error_count);
    }
#endif
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
