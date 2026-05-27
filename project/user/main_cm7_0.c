/*********************************************************************************************************************
* CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� CYT4BB ��Դ���һ����
*
* CYT4BB ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          main_cm7_0
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "../code/common/project_headfile.h"

// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������

// **************************** �������� ****************************
static monitor_page_enum g_monitor_page = MONITOR_PAGE_DATA;

static void build_monitor_runtime_snapshot(MonitorRuntimeSnapshot *snapshot)
{
    PMWFlowState_t pmw_flow_state = {0};
    uint8 ir_track_valid = 0U;
    if(snapshot == NULL)
    {
        return;
    }

    PMW_Flow_GetState(&pmw_flow_state);
    IR_Localization_ServiceLostProtection();
    ir_track_valid = (uint8)((ir_pose_estimate.valid && !ir_position_lost_flag) ? 1U : 0U);

    snapshot->image_buffer = NULL;
    snapshot->control_mode = control_mode;
    snapshot->armed = (uint8)(armed ? 1U : 0U);
    snapshot->low_voltage = (uint8)(low_voltage_protect_flag ? 1U : 0U);
    snapshot->has_nearest = ir_track_valid;
    snapshot->voltage = car_bat_voltage;  /* С����ɻ����õ�أ�ֱ����ʾС���ϵ�ʵ���ѹ�� */
    snapshot->target_throttle = (int32)Flight_Control_Get_Target_Throttle();
    snapshot->base_throttle = (int32)base_throttle;
    snapshot->roll = current_roll;
    snapshot->pitch = current_pitch;
    snapshot->yaw = current_yaw;
    snapshot->beacon_count = ir_track_valid ? IR_SQUARE_POINT_COUNT : 0;
    snapshot->nearest_x = ir_track_valid ? ir_pose_estimate.x : 0.0f;
    snapshot->nearest_y = ir_track_valid ? ir_pose_estimate.y : 0.0f;
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
    uint8 camera_new_frame = 0U;
    uint8 link_tick = 0;
    uint8 car_link_enabled = 0U;
    uint8 car_link_enabled_changed = 0U;
    uint8 input_handled = 0U;
    uint8 page_changed = 0U;
    monitor_page_enum page_before;
    MonitorRuntimeSnapshot snapshot;

    if(CommMux_UpdateFromSwitch())
    {
        Vofa_ClearRxState();
    }

    CrossCoreCamera_ControlCorePublishInput(current_roll,
                                            current_pitch,
                                            tof_height_true_filt * 100.0f);
    camera_new_frame = CrossCoreCamera_ControlCoreFetchFrame();
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
    car_link_enabled = core_link_user_is_link_enabled();
    car_link_enabled_changed = core_link_user_consume_link_enabled_changed();

    page_before = g_monitor_page;
    drone_screen_mirror_begin(car_link_enabled);
    input_handled = screen_monitor_handle_input(&g_monitor_page, &snapshot, 50.0f);
    page_changed = (page_before != g_monitor_page) ? 1U : 0U;
    drone_screen_mirror_end((uint8)(car_link_enabled && page_changed));

    if(input_handled)
    {
        return;
    }

    update_monitor_vision_state();
    service_monitor_telemetry();
    build_monitor_runtime_snapshot(&snapshot);

    if(car_link_enabled && link_tick)
    {
        drone_status_tick_div ++;
        if(drone_status_tick_div >= 10U)
        {
            drone_status_tick_div = 0;
            core_link_user_send_drone_status_from_snapshot(&snapshot);
        }
    }
    else if(!car_link_enabled)
    {
        drone_status_tick_div = 0;
    }

    if(car_link_enabled && car_link_enabled_changed)
    {
        drone_screen_mirror_begin(1U);
        screen_monitor_enter_page(g_monitor_page, &snapshot);
        drone_screen_mirror_end(1U);
    }
    else
    {
        drone_screen_mirror_begin((uint8)(car_link_enabled && link_tick));
        screen_monitor_refresh_page(g_monitor_page, &snapshot);
        drone_screen_mirror_end((uint8)(car_link_enabled && link_tick));
    }
}

static void run_monitor_view_loop(void)
{
    MonitorRuntimeSnapshot snapshot;
    build_monitor_runtime_snapshot(&snapshot);
    drone_screen_mirror_begin(core_link_user_is_link_enabled());
    screen_monitor_enter_page(g_monitor_page, &snapshot);
    drone_screen_mirror_end(core_link_user_is_link_enabled());

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

    IR_Localization_Init();
    HeightManager_Init(100.0f);
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_init();                       // ���Դ�����Ϣ��ʼ��
    // �˴���д�û����� ���������ʼ�������
        
    initialize_control_core_modules();
    

    // �˴���д�û����� ���������ʼ�������
    while(true)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���


    menu_open();
    run_monitor_view_loop();
    
      
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

// **************************** �������� ****************************
