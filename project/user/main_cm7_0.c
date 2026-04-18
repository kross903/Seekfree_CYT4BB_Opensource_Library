/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "../code/common/project_headfile.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
static monitor_page_enum g_monitor_page = MONITOR_PAGE_DATA;
static BeaconResult_t g_beacon_results[BEACON_MAX_DETECTIONS];
static int g_beacon_count = 0;
static int g_nearest_idx = -1;

static void build_monitor_runtime_snapshot(MonitorRuntimeSnapshot *snapshot)
{
    PMWFlowState_t pmw_flow_state = {0};
    if(snapshot == NULL)
    {
        return;
    }

    PMW_Flow_GetState(&pmw_flow_state);

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
    snapshot->optical_flow_valid = pmw_flow_state.valid;
    snapshot->optical_flow_dx = pmw_flow_state.flow_delta_x_count;
    snapshot->optical_flow_dy = pmw_flow_state.flow_delta_y_count;
    snapshot->optical_flow_x_integral = pmw3901_delta_x_i;
    snapshot->optical_flow_dt_s = pmw_flow_state.dt_s;
    snapshot->optical_flow_tof_los_cm = pmw_flow_state.tof_los_cm;
    snapshot->optical_flow_range_cm = pmw_flow_state.range_cm;
    snapshot->optical_flow_roll_deg = pmw_flow_state.roll_deg;
    snapshot->optical_flow_pitch_deg = pmw_flow_state.pitch_deg;
    snapshot->optical_flow_x_rad = pmw_flow_state.flow_x_rad;
    snapshot->optical_flow_y_rad = pmw_flow_state.flow_y_rad;
    snapshot->optical_flow_gyro_x_delta_rad = pmw_flow_state.gyro_flow_x_delta_rad;
    snapshot->optical_flow_gyro_y_delta_rad = pmw_flow_state.gyro_flow_y_delta_rad;
    snapshot->optical_flow_comp_x_rad = pmw_flow_state.flow_comp_x_rad;
    snapshot->optical_flow_comp_y_rad = pmw_flow_state.flow_comp_y_rad;
    snapshot->optical_flow_vx_raw_cms = pmw_flow_state.velocity_x_raw_cms;
    snapshot->optical_flow_vy_raw_cms = pmw_flow_state.velocity_y_raw_cms;
    snapshot->optical_flow_vx_cms = pmw_flow_state.velocity_x_cms;
    snapshot->optical_flow_vy_cms = pmw_flow_state.velocity_y_cms;
    snapshot->optical_flow_speed_cms = pmw_flow_state.speed_cms;
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
volatile uint8 Car_Copy_Screen_Flag = 0;
static uint8 drone_car_link_task(void)
{
    uint8 link_tick = 0;

    if(Car_Copy_Screen_Flag)
    {
        Car_Copy_Screen_Flag = 0;
        link_tick = 1;
    }

    core_link_user_poll();
    return link_tick;
}

static void update_monitor_view(void)
{
    static uint8 drone_status_tick_div = 0;
    uint8 link_tick = 0;
    MonitorRuntimeSnapshot snapshot;

    (void)CrossCoreCamera_ControlCoreFetchFrame();
    build_monitor_runtime_snapshot(&snapshot);
    link_tick = drone_car_link_task();

    if(screen_monitor_handle_input(&g_monitor_page, &snapshot, 50.0f))
    {
        return;
    }

    update_monitor_vision_state();
    service_monitor_telemetry();
    build_monitor_runtime_snapshot(&snapshot);

    if(link_tick)
    {
        drone_status_tick_div ++;
        if(drone_status_tick_div >= 10U)
        {
            drone_status_tick_div = 0;
            core_link_user_send_drone_status_from_snapshot(&snapshot);
        }
    }

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
    core_link_init();
    core_link_user_init();
    flash_init();
    Device_Init();
    pmw3901_init();

    IR_Localization_Init();
    IPM_Interface_Init();
    BeaconDetect_Init();
    HeightManager_Init(100.0f);
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等
        
    initialize_control_core_modules();
    

    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码


    menu_open();
    run_monitor_view_loop();
    
      
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
