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
#include "../code/drone_car_link.h"
#include "../code/vofa.h"
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
    snapshot->optical_flow_dx = pmw3901_delta_x;
    snapshot->optical_flow_dy = pmw3901_delta_y;
    snapshot->optical_flow_x_integral = pmw3901_delta_x_i;
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
    DroneCarLink_MainLoopTask();
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
    flash_init();
    Device_Init();
    pmw3901_init();
    
    DroneCarLink_Init();
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
    
    menu_open();
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
