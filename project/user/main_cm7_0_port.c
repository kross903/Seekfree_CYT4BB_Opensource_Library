#include "zf_common_headfile.h"
#include "../code/All_Init.h"
#include "../code/PIT_Demo.h"
#include "../code/flight_control.h"
#include "../code/Kalman Filter/height_manager.h"
#include "../code/vision/ipm_interface.h"
#include "../code/vision/ir_localization.h"
#include "../code/vision/beacon_detect.h"
#include "../code/menu/menu.h"
#include "../code/menu/screen_pages.h"
#include "../code/cross_core_camera.h"
#include "../code/vofa.h"

static monitor_page_enum g_monitor_page = MONITOR_PAGE_DATA;
static BeaconResult_t g_beacon_results[BEACON_MAX_DETECTIONS];
static int g_beacon_count = 0;
static int g_nearest_idx = -1;

static void build_monitor_runtime_snapshot(MonitorRuntimeSnapshot *snapshot)
{
    if(snapshot == NULL)
    {
        return;
    }

    snapshot->image_buffer = mt9v03x_image[0];
    snapshot->control_mode = control_mode;
    snapshot->armed = (uint8)(armed ? 1U : 0U);
    snapshot->low_voltage = (uint8)(low_voltage_protect_flag ? 1U : 0U);
    snapshot->has_nearest = (uint8)((g_nearest_idx >= 0) ? 1U : 0U);
    snapshot->voltage = voltage;
    snapshot->target_throttle = (int32)Flight_Control_Get_Target_Throttle();
    snapshot->base_throttle = (int32)base_throttle;
    snapshot->roll = current_roll;
    snapshot->pitch = current_pitch;
    snapshot->yaw = current_yaw;
    snapshot->beacon_count = g_beacon_count;
    snapshot->nearest_x = snapshot->has_nearest ? g_beacon_results[g_nearest_idx].world_x : 0.0f;
    snapshot->nearest_y = snapshot->has_nearest ? g_beacon_results[g_nearest_idx].world_y : 0.0f;
    snapshot->tof_raw_cm = dl1b_distance_mm * 0.1f;
    snapshot->tof_height_cm = tof_height_true_filt * 100.0f;
    snapshot->fused_height_cm = g_height_result.height_cm;
    snapshot->ir_height_cm = ir_pose_estimate.z;
    snapshot->fused_velocity_cms = g_height_result.velocity_cms;
}

static void update_monitor_vision_state(void)
{
    if(!mt9v03x_finish_flag)
    {
        return;
    }

    mt9v03x_finish_flag = 0;
    IPM_Interface_UpdateFromIMU(tof_height_true_filt * 100.0f);
    IR_Localization_Process(mt9v03x_image, &ir_pose_estimate);

    g_beacon_count = BeaconDetect_Process(mt9v03x_image, g_beacon_results, BEACON_MAX_DETECTIONS);
    g_nearest_idx = BeaconDetect_GetNearestIndex(g_beacon_results, g_beacon_count);
}

static void service_monitor_telemetry(void)
{
    Send_test();
    USART_PID_Adjust();
    send_date_back();
}

static void update_monitor_view(void)
{
    MonitorRuntimeSnapshot snapshot;

    (void)CrossCoreCamera_ControlCoreFetchFrame();
    build_monitor_runtime_snapshot(&snapshot);

    if(screen_monitor_handle_input(&g_monitor_page, &snapshot, 50.0f))
    {
        return;
    }

    update_monitor_vision_state();
    service_monitor_telemetry();

    build_monitor_runtime_snapshot(&snapshot);
    screen_monitor_refresh_page(g_monitor_page, &snapshot);
}

static void run_monitor_view_loop(void)
{
    MonitorRuntimeSnapshot snapshot;
    build_monitor_runtime_snapshot(&snapshot);
    screen_monitor_enter_page(g_monitor_page, &snapshot);

    while(menu_is_monitor_view())
    {
        update_monitor_view();
    }
}

static void initialize_control_core_modules(void)
{
    Init_uart();
    Device_Init();
    IR_Localization_Init();
    IPM_Interface_Init();
    BeaconDetect_Init();
    HeightManager_Init(100.0f);
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();

    initialize_control_core_modules();

    while(true)
    {
        menu_open();
        run_monitor_view_loop();
    }
}
