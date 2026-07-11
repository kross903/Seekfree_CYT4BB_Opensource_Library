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
#define MONITOR_LOCAL_SCREEN_REFRESH_DIV    (5U)

static monitor_page_enum g_monitor_page = MONITOR_PAGE_DATA;

static void build_monitor_runtime_snapshot(MonitorRuntimeSnapshot *snapshot)
{
    BatteryStatus battery_status;
    uint8 ir_track_valid = 0U;
    if(snapshot == NULL)
    {
        return;
    }

    VehicleMarkerLocalization_ServiceLostProtection();
    BatteryMonitor_GetStatus(&battery_status);
    ir_track_valid = (uint8)((g_vehicle_marker_pose.valid && !g_vehicle_marker_position_lost_flag) ? 1U : 0U);

    snapshot->control_mode = control_mode;
    snapshot->armed = (uint8)(armed ? 1U : 0U);
    snapshot->low_voltage = (uint8)(low_voltage_protect_flag ? 1U : 0U);
    snapshot->has_nearest = ir_track_valid;
    snapshot->voltage = battery_status.local_filtered_voltage_v;
    snapshot->target_throttle = (int32)Flight_Control_Get_Target_Throttle();
    snapshot->base_throttle = (int32)base_throttle;
    snapshot->roll = current_roll;
    snapshot->pitch = current_pitch;
    snapshot->yaw = current_yaw;
    snapshot->beacon_count = ir_track_valid ? g_vehicle_marker_pose.detected_point_count : 0;
    snapshot->nearest_x = ir_track_valid ? g_vehicle_marker_pose.x : 0.0f;
    snapshot->nearest_y = ir_track_valid ? g_vehicle_marker_pose.y : 0.0f;
    snapshot->tof_height_cm = tof_height_true_filt * 100.0f;
    snapshot->fused_height_cm = g_height_result.height_cm;
    snapshot->fused_velocity_cms = g_height_result.velocity_cms;
}

static void update_monitor_vision_state(void)
{
    CrossCoreCamera_ControlCoreFetchFrame();
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

static void drone_screen_mirror_begin(uint8 enable)
{
    ips200_mirror_clear_buffer();
    ips200_mirror_set_enable(enable ? 1U : 0U);
}

static void drone_screen_mirror_end(uint8 send)
{
    ips200_mirror_set_enable(0U);

    if(send)
    {
        ips200_mirror_flush();
    }
    else
    {
        ips200_mirror_clear_buffer();
    }
}

static void update_monitor_view(void)
{
    static uint8 drone_status_tick_div = 0;
    static uint8 local_screen_refresh_div = 0U;
    uint8 camera_new_frame = 0U;
    uint8 link_tick = 0;
    uint8 screen_mirror_enabled = 0U;
    uint8 screen_mirror_enabled_changed = 0U;
    uint8 local_screen_refresh_tick = 0U;
    uint8 input_handled = 0U;
    uint8 page_changed = 0U;
    monitor_page_enum page_before;
    MonitorRuntimeSnapshot snapshot;

    if(CommMux_UpdateFromSwitch())
    {
        Vofa_ClearRxState();//清除旧状态，避免对新模式的干扰
    }

    camera_new_frame = CrossCoreCamera_ControlCoreService(current_roll,
                                                          current_pitch,
                                                          tof_height_true_filt * 100.0f);
    if(CommMux_IsAssistantImageMode())
    {
        if(camera_new_frame)
        {
            (void)CrossCoreCamera_ControlCoreSendAssistantFrame();
        }
        return;
    }

    build_monitor_runtime_snapshot(&snapshot);
    link_tick = drone_car_link_task();
    screen_mirror_enabled = core_link_user_is_screen_mirror_enabled();
    screen_mirror_enabled_changed = core_link_user_consume_screen_mirror_enabled_changed();

    page_before = g_monitor_page;
    drone_screen_mirror_begin(screen_mirror_enabled);
    input_handled = screen_monitor_handle_input(&g_monitor_page, &snapshot, 50.0f);
    page_changed = (page_before != g_monitor_page) ? 1U : 0U;
    drone_screen_mirror_end((uint8)(screen_mirror_enabled && page_changed));

    if(input_handled)
    {
        return;
    }

    update_monitor_vision_state();
    service_monitor_telemetry();
    build_monitor_runtime_snapshot(&snapshot);

    if(link_tick)
    {
        core_link_user_tick(20U);
        core_link_user_service_beacon_link();

        local_screen_refresh_div ++;
        if(local_screen_refresh_div >= MONITOR_LOCAL_SCREEN_REFRESH_DIV)
        {
            local_screen_refresh_div = 0U;
            local_screen_refresh_tick = 1U;
        }

        drone_status_tick_div ++;
        if(drone_status_tick_div >= 10U)
        {
            drone_status_tick_div = 0;
            core_link_user_send_drone_status_from_snapshot(&snapshot);
        }
    }

    if(screen_mirror_enabled && screen_mirror_enabled_changed)
    {
        drone_screen_mirror_begin(1U);
        screen_monitor_enter_page(g_monitor_page, &snapshot);
        drone_screen_mirror_end(1U);
    }
    else if(local_screen_refresh_tick)
    {
        drone_screen_mirror_begin(screen_mirror_enabled);
        screen_monitor_refresh_page(g_monitor_page, &snapshot);
        drone_screen_mirror_end(screen_mirror_enabled);
    }
}

static void run_monitor_view_loop(void)
{
    MonitorRuntimeSnapshot snapshot;
    build_monitor_runtime_snapshot(&snapshot);
    drone_screen_mirror_begin(core_link_user_is_screen_mirror_enabled());
    screen_monitor_enter_page(g_monitor_page, &snapshot);
    drone_screen_mirror_end(core_link_user_is_screen_mirror_enabled());

    while(menu_is_monitor_view())
    {
        Vofa_MainLoopProfile_Begin();
        update_monitor_view();
        Vofa_MainLoopProfile_End();
    }
}

static void initialize_control_core_modules(void)
{
    CommMux_Init();
    CommMux_SwitchInit();
    (void)CommMux_UpdateFromSwitch();
    Init_uart();
    CrossCoreCamera_ControlCoreAssistantInit();
    core_link_init();
    core_link_user_init();
    flash_init();
    Device_Init();

    VehicleMarkerLocalization_Init();
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
