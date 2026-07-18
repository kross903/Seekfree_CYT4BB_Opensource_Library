#include "common/project_headfile.h"

/* ????? */
Menu mainMenu;
static Menu testMenu;

/*
 * [2026-03-10]
 * 修改目的:
 * 1. ??????????????????????????????????
 * 2. ???????????????????????
 */
/* ????????? */
static uint8 select_control_mode(void)
{
    uint8 selection = (Flight_Control_Get_Mode() == CONTROL_MODE_ANGLE) ? 1U : 0U;

    ui_menu_draw_control_mode_page(selection);
    key_flag = 0;

    while(1)
    {
        core_link_user_poll();

        if(key_flag != 0)
        {
            switch(key_flag)
            {
                case Button_1:
                case Button_2:
                    selection = selection ? 0U : 1U;
                    ui_menu_draw_control_mode_page(selection);
                    break;

                case Button_3:
                    Flight_Control_Set_Mode(selection);
                    ui_menu_show_control_mode_saved();
                    system_delay_ms(500);
                    return 1U;

                case Button_4:
                    return 0U;

                default:
                    break;
            }
            key_flag = 0;
        }
    }
}
/* ?????????????? */
void exit_menu_system(void)
{
    // ?????????????
    if (!select_control_mode())
        return;  // 用户取消操作
    
    // ????????????????
    ui_menu_play_exit_animation();

    /*
     * [2026-03-10]
     * 修改目的:
     * ????? g_menu_exit_flag ???????????????????
     * ???????????????????????????????
     */
    menu_enter_monitor_view();
    key_flag = 0;
}

void icm_init_menu()
{
    /*
     * [2026-03-10]
     * 修改目的:
     * 1. ??????? ICM ???????????????????????
     * 2. ?????????????????????????????????????????
     */
    key_flag = 0;
    IMU_calibrate_flag = 0;
    ICM_Calculate_Init();

    ips200_clear();
    while(1)
    {
        core_link_user_poll();
        ui_menu_show_icm_init_result_page();
        if(key_flag == Button_4)
        {
            key_flag = Button_4;
            return;
        }

        system_delay_ms(20);
    }
}

#define TEST_PAGE_X             20U
#define TEST_VALUE_X            132U
#define TEST_RIGHT_VALUE_X      188U
#define TEST_ROW_Y(row)         ((uint16)(56U + (uint16)(row) * 20U))
#define VISION_VIEW_SNAPSHOT_WAIT_TICKS 120U
#define VISION_VIEW_SNAPSHOT_WAIT_MS    5U
#define TRACK_LIGHT_RELEASE_TIMEOUT_TICKS 40U

static uint8 s_track_light_page_active = 0U;
static uint8 s_track_light_image_visible = 0U;
static uint8 s_track_light_release_pending = 0U;
static uint8 s_track_light_exit_pending = 0U;
static uint8 s_track_light_release_wait_ticks = 0U;
static uint8 s_track_light_status_tick_div = 0U;

static void test_page_clear_line(uint16 y)
{
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0U, y, "                              ");
}

static void test_page_show_text(uint16 y, const char *label, const char *value, uint16 color)
{
    test_page_clear_line(y);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(TEST_PAGE_X, y, (char *)label);
    ips200_set_color(color, RGB565_BLACK);
    ips200_show_string(TEST_VALUE_X, y, (char *)value);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

static void test_page_show_int(uint16 y, const char *label, int32 value, uint8 digits)
{
    test_page_clear_line(y);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(TEST_PAGE_X, y, (char *)label);
    ips200_show_int(TEST_VALUE_X, y, value, digits);
}

static void test_page_show_float(uint16 y, const char *label, float value, uint8 digits, uint8 point_digits)
{
    test_page_clear_line(y);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(TEST_PAGE_X, y, (char *)label);
    ips200_show_float(TEST_VALUE_X, y, value, digits, point_digits);
}

static void vision_view_send_snapshot(void)
{
    uint8 wait_count = 0U;
    uint8 sent = 0U;

    if(!CommMux_IsAssistantImageMode())
    {
        CrossCoreCamera_ControlCoreSetVisionStatus(CROSS_CORE_CAMERA_VISION_STATUS_ASSISTANT_REQUIRED);
        return;
    }

    CrossCoreCamera_ControlCoreSetVisionStatus(CROSS_CORE_CAMERA_VISION_STATUS_SENDING);
    if(!CrossCoreCamera_ControlCoreRequestAssistantSnapshot())
    {
        CrossCoreCamera_ControlCoreSetVisionStatus(CROSS_CORE_CAMERA_VISION_STATUS_REQUEST_FAILED);
        return;
    }

    while(wait_count < VISION_VIEW_SNAPSHOT_WAIT_TICKS)
    {
        core_link_user_poll();
        (void)CrossCoreCamera_ControlCoreService(current_roll,
                                                 current_pitch,
                                                 tof_height_true_filt * 100.0f);
        if(CrossCoreCamera_ControlCoreSendAssistantFrame())
        {
            sent = 1U;
            break;
        }

        system_delay_ms(VISION_VIEW_SNAPSHOT_WAIT_MS);
        wait_count++;
    }

    CrossCoreCamera_ControlCoreSetVisionStatus(sent ? CROSS_CORE_CAMERA_VISION_STATUS_SENT :
                                                      CROSS_CORE_CAMERA_VISION_STATUS_TIMEOUT);
}

static uint8 test_all_motor_output_zero(void)
{
    return (uint8)((motor_throttle[0] == 0U) &&
                   (motor_throttle[1] == 0U) &&
                   (motor_throttle[2] == 0U) &&
                   (motor_throttle[3] == 0U) &&
                   !Flight_Motor_Test_Get_Output_Enable() &&
                   !Flight_Fly_Test_Get_Output_Enable());
}

static uint8 test_attitude_within_limit(float roll_limit, float pitch_limit)
{
    return (uint8)((current_roll <= roll_limit) &&
                   (current_roll >= -roll_limit) &&
                   (current_pitch <= pitch_limit) &&
                   (current_pitch >= -pitch_limit));
}

static uint8 test_fly_output_safe(void)
{
    return (uint8)((!emergency_stop_triggered) &&
                   test_attitude_within_limit(FLY_TEST_ATTITUDE_SAFE_DEG, FLY_TEST_ATTITUDE_SAFE_DEG));
}

static const char *test_safety_reason(uint8 fly_limit)
{
    if(emergency_stop_triggered)
    {
        return "ESTOP";
    }
    if(low_voltage_protect_flag)
    {
        return "LOWV ";
    }
    if(fly_limit)
    {
        if(!test_attitude_within_limit(FLY_TEST_ATTITUDE_SAFE_DEG, FLY_TEST_ATTITUDE_SAFE_DEG))
        {
            return "ANGLE";
        }
    }
    else if(!test_attitude_within_limit(SAFE_MAX_ANGLE_ROLL, SAFE_MAX_ANGLE_PITCH))
    {
        return "ANGLE";
    }

    return "OK   ";
}

static const char *test_control_mode_name(uint8 mode)
{
    switch(mode)
    {
        case CONTROL_MODE_MANUAL:
            return "MANUAL";
        case CONTROL_MODE_ANGLE:
            return "ANGLE ";
        case CONTROL_MODE_CASCADE:
            return "CASCADE";
        default:
            return "UNK   ";
    }
}

static void preflight_page(void)
{
    BatteryStatus battery_status;
    uint8 imu_ok = (uint8)((Check_IMU_Data_Quality() == 0) && !icm_calibration_flag);
    uint8 angle_ok = test_attitude_within_limit(SAFE_MAX_ANGLE_ROLL, SAFE_MAX_ANGLE_PITCH);
    uint8 motor_zero = test_all_motor_output_zero();

    BatteryMonitor_GetStatus(&battery_status);

    test_page_show_text(TEST_ROW_Y(0U), "IMU:", imu_ok ? "OK   " : (icm_calibration_flag ? "CAL  " : "BAD  "),
                        imu_ok ? RGB565_GREEN : RGB565_RED);
    test_page_show_text(TEST_ROW_Y(1U), "ANGLE:", angle_ok ? "OK   " : "BAD  ",
                        angle_ok ? RGB565_GREEN : RGB565_RED);
    test_page_show_text(TEST_ROW_Y(2U), "LOW VOLT:", low_voltage_protect_flag ? "WARN " : "OK   ",
                        low_voltage_protect_flag ? RGB565_RED : RGB565_GREEN);
    test_page_show_text(TEST_ROW_Y(3U), "ESTOP:", emergency_stop_triggered ? "LATCH" : "OK   ",
                        emergency_stop_triggered ? RGB565_RED : RGB565_GREEN);
    test_page_show_text(TEST_ROW_Y(4U), "MOTOR OUT:", motor_zero ? "ZERO " : "HOT  ",
                        motor_zero ? RGB565_GREEN : RGB565_RED);
    test_page_show_text(TEST_ROW_Y(5U), "MODE:", test_control_mode_name(Flight_Control_Get_Mode()), RGB565_CYAN);
    test_page_show_float(TEST_ROW_Y(6U), "BAT V:", battery_status.local_filtered_voltage_v, 2U, 2U);
    test_page_show_int(TEST_ROW_Y(7U), "TGT THR:", (int32)Flight_Control_Get_Target_Throttle(), 4U);
    test_page_show_int(TEST_ROW_Y(8U), "BASE THR:", (int32)base_throttle, 4U);
    test_page_show_text(TEST_ROW_Y(10U), "K4:", "BACK ", RGB565_WHITE);
}

static void imu_status_page(void)
{
    int imu_quality = Check_IMU_Data_Quality();
    uint8 angle_ok = test_attitude_within_limit(SAFE_MAX_ANGLE_ROLL, SAFE_MAX_ANGLE_PITCH);

    test_page_show_float(TEST_ROW_Y(0U), "ROLL:", current_roll, 3U, 2U);
    test_page_show_float(TEST_ROW_Y(1U), "PITCH:", current_pitch, 3U, 2U);
    test_page_show_float(TEST_ROW_Y(2U), "YAW:", current_yaw, 3U, 2U);
    test_page_show_float(TEST_ROW_Y(3U), "GYRO R:", current_gyro_roll, 4U, 1U);
    test_page_show_float(TEST_ROW_Y(4U), "GYRO P:", current_gyro_pitch, 4U, 1U);
    test_page_show_float(TEST_ROW_Y(5U), "GYRO Y:", current_gyro_yaw, 4U, 1U);
    test_page_show_text(TEST_ROW_Y(6U), "CAL:", icm_calibration_flag ? "RUN  " : "IDLE ",
                        icm_calibration_flag ? RGB565_YELLOW : RGB565_GREEN);
    test_page_show_int(TEST_ROW_Y(7U), "QUALITY:", (int32)imu_quality, 3U);
    test_page_show_text(TEST_ROW_Y(8U), "SAFE:", angle_ok ? "OK   " : "ANGLE",
                        angle_ok ? RGB565_GREEN : RGB565_RED);
    test_page_show_text(TEST_ROW_Y(10U), "K4:", "BACK ", RGB565_WHITE);
}

static void flow_status_page(void)
{
    PMWFlowState_t flow_state = {0};
    float tof_los_cm = 0.0f;
    uint16 tof_stale_ticks = 0U;
    uint8 tof_source = 0U;

    PMW_Flow_GetState(&flow_state);
    PMW_Flow_Get_TOF_Status(&tof_los_cm, &tof_stale_ticks, &tof_source);

    test_page_show_int(TEST_ROW_Y(0U), "SENSOR:", (int32)FLOW_SENSOR_SELECTED, 1U);
    test_page_show_text(TEST_ROW_Y(1U), "VALID:", flow_state.valid ? "YES  " : "NO   ",
                        flow_state.valid ? RGB565_GREEN : RGB565_RED);
    test_page_show_text(TEST_ROW_Y(2U), "FUSED:", flow_state.fused_velocity_valid ? "YES  " : "NO   ",
                        flow_state.fused_velocity_valid ? RGB565_GREEN : RGB565_RED);
    test_page_show_text(TEST_ROW_Y(3U), "FLOW USED:", flow_state.flow_used ? "YES  " : "NO   ",
                        flow_state.flow_used ? RGB565_GREEN : RGB565_YELLOW);
    test_page_show_float(TEST_ROW_Y(4U), "RANGE CM:", flow_state.range_cm, 4U, 1U);
    test_page_show_float(TEST_ROW_Y(5U), "TOF CM:", tof_los_cm, 4U, 1U);
    test_page_show_int(TEST_ROW_Y(6U), "TOF SRC:", (int32)tof_source, 1U);
    test_page_show_float(TEST_ROW_Y(7U), "VX FUSE:", flow_state.velocity_x_fused_cms, 4U, 1U);
    test_page_show_float(TEST_ROW_Y(8U), "VY FUSE:", flow_state.velocity_y_fused_cms, 4U, 1U);
    test_page_show_float(TEST_ROW_Y(9U), "CONF:", flow_state.flow_confidence, 1U, 3U);
    test_page_show_float(TEST_ROW_Y(10U), "RESID:", flow_state.flow_residual_norm_cms, 4U, 1U);
    test_page_show_int(TEST_ROW_Y(11U), "STALE:", (int32)tof_stale_ticks, 4U);
}

static void vofa_status_page(void)
{
    VofaDiag_t diag = {0};
    char command_text[4] = {'-', '-', ' ', '\0'};

    Send_test();
    USART_PID_Adjust();
    Vofa_GetDiag(&diag);

    if(diag.send_back_command != 0U)
    {
        command_text[0] = (char)diag.send_back_command;
        command_text[1] = ' ';
    }

    test_page_show_text(TEST_ROW_Y(0U), "MUX:", diag.comm_mode == COMM_MUX_MODE_VOFA ? "VOFA " : "IMAGE",
                        diag.is_vofa_mode ? RGB565_GREEN : RGB565_YELLOW);
    test_page_show_text(TEST_ROW_Y(1U), "CMD PATH:", diag.is_vofa_mode ? "ON   " : "OFF  ",
                        diag.is_vofa_mode ? RGB565_GREEN : RGB565_RED);
    test_page_show_int(TEST_ROW_Y(2U), "TX FLAG:", (int32)diag.send_back_flag, 3U);
    test_page_show_text(TEST_ROW_Y(3U), "TX CMD:", command_text, RGB565_CYAN);
    test_page_show_int(TEST_ROW_Y(4U), "PKT READY:", (int32)diag.packet_ready, 1U);
    test_page_show_int(TEST_ROW_Y(5U), "PKT QUEUE:", (int32)diag.packet_queue_count, 2U);
    test_page_show_float(TEST_ROW_Y(6U), "LOOP US:", diag.loop_last_us, 5U, 1U);
    test_page_show_float(TEST_ROW_Y(7U), "AVG US:", diag.loop_avg_us, 5U, 1U);
    test_page_show_float(TEST_ROW_Y(8U), "MAX US:", diag.loop_max_us, 5U, 1U);
    test_page_show_float(TEST_ROW_Y(9U), "FREQ:", diag.loop_freq_hz, 4U, 1U);
    test_page_show_int(TEST_ROW_Y(10U), "OVERRUN:", (int32)diag.loop_overrun_count, 5U);
    test_page_show_text(TEST_ROW_Y(11U), "STREAM:", "VIEW ", RGB565_WHITE);
}

static void vision_view_page(void)
{
    static uint8 request_sent = 0U;
    static uint8 display_mode = CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY;
    uint8 wait_count = 0U;

    if(!request_sent)
    {
        display_mode = CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY;
        ips200_clear();
        ips200_show_string(20U, 72U, "CM7_1 Vision View");
        ips200_show_string(20U, 96U, "Waiting image...");
        ips200_show_string(20U, 120U, "K1 IMG K3 SEND");
        ips200_show_string(20U, 144U, "K4 BACK DIP ASSIST");
        CrossCoreCamera_ControlCoreSetVisionDisplayMode(display_mode);
        CrossCoreCamera_ControlCoreSetVisionStatus(CROSS_CORE_CAMERA_VISION_STATUS_IDLE);
        CrossCoreCamera_ControlCoreRequestVisionScreen(1U);
        request_sent = 1U;
        key_flag = 0;
    }

    core_link_user_poll();

    if(key_flag == Button_1)
    {
        if(display_mode == CROSS_CORE_CAMERA_VISION_DISPLAY_RAW)
        {
            display_mode = CROSS_CORE_CAMERA_VISION_DISPLAY_UNDISTORT;
        }
        else if(display_mode == CROSS_CORE_CAMERA_VISION_DISPLAY_UNDISTORT)
        {
            display_mode = CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY;
        }
        else
        {
            display_mode = CROSS_CORE_CAMERA_VISION_DISPLAY_RAW;
        }
        CrossCoreCamera_ControlCoreSetVisionDisplayMode(display_mode);
        key_flag = 0;
        return;
    }

    if(key_flag == Button_3)
    {
        key_flag = 0;
        vision_view_send_snapshot();
        return;
    }

    if(key_flag == Button_4)
    {
        CrossCoreCamera_ControlCoreRequestVisionScreen(0U);
        while(CrossCoreCamera_ControlCoreIsVisionScreenActive() && wait_count < 20U)
        {
            core_link_user_poll();
            system_delay_ms(10);
            wait_count++;
        }

        ips200_set_dir(IPS200_PORTAIT);
        ips200_init(IPS200_TYPE_SPI);
        ips200_set_font(IPS200_8X16_FONT);
        request_sent = 0U;
        key_flag = Button_4;
        return;
    }
}

static uint8 track_light_service_downlink(void)
{
    if(!Car_Copy_Screen_Flag)
    {
        return 0U;
    }

    Car_Copy_Screen_Flag = 0U;
    core_link_user_tick((uint16)(FLOW_SENSOR_UPDATE_US / 1000U));
    core_link_user_service_beacon_link();

    s_track_light_status_tick_div++;
    if(s_track_light_status_tick_div >= 10U)
    {
        MonitorRuntimeSnapshot snapshot = {0};
        BatteryStatus battery_status;
        ICM_Health_t imu_health = {0};
        uint8 ir_track_valid;

        s_track_light_status_tick_div = 0U;
        VehicleMarkerLocalization_ServiceLostProtection();
        BatteryMonitor_GetStatus(&battery_status);
        ICM_GetHealth(&imu_health);
        ir_track_valid = (uint8)((g_vehicle_marker_pose.valid
                                  && !g_vehicle_marker_position_lost_flag) ? 1U : 0U);

        snapshot.control_mode = control_mode;
        snapshot.armed = (uint8)(armed ? 1U : 0U);
        snapshot.low_voltage = (uint8)(low_voltage_protect_flag ? 1U : 0U);
        snapshot.has_nearest = ir_track_valid;
        snapshot.tof_ready = HeightManager_IsReadyForTakeoff();
        snapshot.rearm_inhibit_active =
            (uint8)(Flight_Safety_GetRearmInhibitRemainingMs() > 0U);
        snapshot.takeoff_state = (uint8)Flight_Takeoff_Get_State();
        snapshot.voltage = battery_status.local_filtered_voltage_v;
        snapshot.target_throttle = (int32)Flight_Control_Get_Target_Throttle();
        snapshot.base_throttle = (int32)base_throttle;
        snapshot.roll = current_roll;
        snapshot.pitch = current_pitch;
        snapshot.yaw = current_yaw;
        snapshot.beacon_count = ir_track_valid ? g_vehicle_marker_pose.detected_point_count : 0;
        snapshot.nearest_x = ir_track_valid ? g_vehicle_marker_pose.x : 0.0f;
        snapshot.nearest_y = ir_track_valid ? g_vehicle_marker_pose.y : 0.0f;
        snapshot.tof_height_cm = tof_height_true_filt * 100.0f;
        snapshot.fused_height_cm = g_height_result.height_cm;
        snapshot.fused_velocity_cms = g_height_result.velocity_cms;
        snapshot.imu_fault_mask = imu_health.fault_mask;
        core_link_user_send_drone_status_from_snapshot(&snapshot);
    }

    return 1U;
}

static void track_light_reset_page_state(void)
{
    s_track_light_page_active = 0U;
    s_track_light_image_visible = 0U;
    s_track_light_release_pending = 0U;
    s_track_light_exit_pending = 0U;
    s_track_light_release_wait_ticks = 0U;
    s_track_light_status_tick_div = 0U;
}

static void track_light_restore_local_screen(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_clear();
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(TEST_PAGE_X, 20U, "Track Light Test");
}

static void track_light_draw_beacon_slot(uint8 row,
                                         uint8 slot,
                                         const VisionRuntimeBeaconSlot_t *beacon)
{
    uint16 y = TEST_ROW_Y(row);

    test_page_clear_line(y);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_char(TEST_PAGE_X, y, 'B');
    ips200_show_uint((uint16)(TEST_PAGE_X + 8U), y, (uint32)(slot + 1U), 1U);
    ips200_set_color(beacon->valid ? RGB565_GREEN : RGB565_RED, RGB565_BLACK);
    ips200_show_string((uint16)(TEST_PAGE_X + 24U), y, beacon->valid ? "ON " : "OFF");
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string((uint16)(TEST_PAGE_X + 56U), y, "X:");
    ips200_show_float((uint16)(TEST_PAGE_X + 72U), y,
                      beacon->valid ? beacon->x_cm : 0.0f, 3U, 1U);
    ips200_show_string((uint16)(TEST_PAGE_X + 136U), y, "Y:");
    ips200_show_float((uint16)(TEST_PAGE_X + 152U), y,
                      beacon->valid ? beacon->y_cm : 0.0f, 3U, 1U);
}

static void track_light_draw_data(void)
{
    VisionRuntimeDiag_t diag = {0};
    VisionRuntimeBeaconSlot_t slots[VISION_RUNTIME_BEACON_SLOT_COUNT] = {0};
    uint8 car_valid;
    uint8 i;

    VisionRuntime_GetDiag(&diag);
    VisionRuntime_GetBeaconSlots(slots, VISION_RUNTIME_BEACON_SLOT_COUNT);
    car_valid = (uint8)(g_vehicle_marker_pose.valid
                        && !g_vehicle_marker_position_lost_flag
                        && (g_vehicle_marker_pose.detected_point_count >= 3U));

    ips200_set_font(IPS200_8X16_FONT);
    test_page_show_text(TEST_ROW_Y(0U), "DOWNLINK:", "RUN  ", RGB565_GREEN);
    test_page_show_text(TEST_ROW_Y(1U), "IMAGE:", "OFF  ", RGB565_WHITE);
    test_page_show_text(TEST_ROW_Y(2U), "CAR:", car_valid ? "VALID" : "LOST ",
                        car_valid ? RGB565_GREEN : RGB565_RED);
    test_page_show_float(TEST_ROW_Y(3U), "CAR X CM:", car_valid ? g_vehicle_marker_pose.x : 0.0f,
                         3U, 1U);
    test_page_show_float(TEST_ROW_Y(4U), "CAR Y CM:", car_valid ? g_vehicle_marker_pose.y : 0.0f,
                         3U, 1U);
    test_page_show_int(TEST_ROW_Y(5U), "BEACONS:", (int32)diag.beacon_count, 2U);

    for(i = 0U; i < VISION_RUNTIME_BEACON_SLOT_COUNT; ++i)
    {
        track_light_draw_beacon_slot((uint8)(6U + i), i, &slots[i]);
    }

    test_page_show_text(TEST_ROW_Y(11U), "K3:", "IMAGE K4 BACK", RGB565_CYAN);
}

static uint8 track_light_wait_for_screen_release(void)
{
    if(!CrossCoreCamera_ControlCoreIsVisionScreenActive())
    {
        return 1U;
    }

    if(s_track_light_release_wait_ticks < TRACK_LIGHT_RELEASE_TIMEOUT_TICKS)
    {
        s_track_light_release_wait_ticks++;
        return 0U;
    }

    return 1U;
}

static void track_light_test_page(void)
{
    uint8 refresh_due = track_light_service_downlink();

    if(!s_track_light_page_active)
    {
        track_light_reset_page_state();
        s_track_light_page_active = 1U;
        track_light_restore_local_screen();
        track_light_draw_data();
        key_flag = 0U;
    }

    if(key_flag == Button_4 && !s_track_light_exit_pending)
    {
        if(s_track_light_image_visible || s_track_light_release_pending
           || CrossCoreCamera_ControlCoreIsVisionScreenActive())
        {
            CrossCoreCamera_ControlCoreRequestVisionScreen(0U);
            s_track_light_image_visible = 0U;
            s_track_light_release_pending = 1U;
            s_track_light_exit_pending = 1U;
            s_track_light_release_wait_ticks = 0U;
            key_flag = 0U;
        }
        else
        {
            track_light_reset_page_state();
            return;
        }
    }

    if(s_track_light_exit_pending)
    {
        key_flag = 0U;
        if(!track_light_wait_for_screen_release())
        {
            return;
        }

        track_light_restore_local_screen();
        track_light_reset_page_state();
        key_flag = Button_4;
        return;
    }

    if(s_track_light_release_pending)
    {
        key_flag = 0U;
        if(!track_light_wait_for_screen_release())
        {
            return;
        }

        track_light_restore_local_screen();
        s_track_light_release_pending = 0U;
        s_track_light_release_wait_ticks = 0U;
        track_light_draw_data();
        return;
    }

    if(key_flag == Button_3)
    {
        key_flag = 0U;
        if(s_track_light_image_visible)
        {
            CrossCoreCamera_ControlCoreRequestVisionScreen(0U);
            s_track_light_image_visible = 0U;
            s_track_light_release_pending = 1U;
            s_track_light_release_wait_ticks = 0U;
        }
        else
        {
            CrossCoreCamera_ControlCoreSetVisionDisplayMode(CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY);
            CrossCoreCamera_ControlCoreSetVisionStatus(CROSS_CORE_CAMERA_VISION_STATUS_IDLE);
            CrossCoreCamera_ControlCoreRequestVisionScreen(1U);
            s_track_light_image_visible = 1U;
        }
        return;
    }

    if(s_track_light_image_visible)
    {
        return;
    }

    if(refresh_due)
    {
        track_light_draw_data();
    }
}

static void fly_test_page(void)
{
    uint16 throttle = 0U;
    uint8 output_enable = 0U;
    uint8 safety_latched = 0U;
    uint8 confirm_state = FLY_TEST_CONFIRM_IDLE;
    const char *safe_text = NULL;
    float target_roll = 0.0f;
    float target_pitch = 0.0f;
    float target_yaw = 0.0f;

    if(!Flight_Fly_Test_Is_Active())
    {
        Flight_Fly_Test_Start();
    }

    output_enable = Flight_Fly_Test_Get_Output_Enable();
    confirm_state = Flight_Fly_Test_Get_Confirm_State();
    if(!test_fly_output_safe() && (output_enable || confirm_state == FLY_TEST_CONFIRM_WAIT))
    {
        Flight_Fly_Test_Set_Output_Enable(0U);
        Flight_Fly_Test_Set_Confirm_State(FLY_TEST_CONFIRM_IDLE);
        output_enable = 0U;
        confirm_state = FLY_TEST_CONFIRM_IDLE;
    }

    if(key_flag == Button_1)
    {
        if(confirm_state == FLY_TEST_CONFIRM_WAIT)
        {
            Flight_Fly_Test_Set_Confirm_State(FLY_TEST_CONFIRM_IDLE);
        }
        else if(!output_enable)
        {
            throttle = Flight_Fly_Test_Get_Throttle();
            throttle = (throttle + FLY_TEST_THROTTLE_STEP > FLY_TEST_THROTTLE_MAX) ?
                       FLY_TEST_THROTTLE_MAX : (throttle + FLY_TEST_THROTTLE_STEP);
            Flight_Fly_Test_Set_Throttle(throttle);
        }
        key_flag = 0;
    }
    else if(key_flag == Button_2)
    {
        if(confirm_state == FLY_TEST_CONFIRM_WAIT)
        {
            Flight_Fly_Test_Set_Output_Enable(1U);
        }
        else if(!output_enable)
        {
            throttle = Flight_Fly_Test_Get_Throttle();
            throttle = (throttle > FLY_TEST_THROTTLE_MIN + FLY_TEST_THROTTLE_STEP) ?
                       (throttle - FLY_TEST_THROTTLE_STEP) : FLY_TEST_THROTTLE_MIN;
            Flight_Fly_Test_Set_Throttle(throttle);
        }
        key_flag = 0;
    }
    else if(key_flag == Button_3)
    {
        output_enable = Flight_Fly_Test_Get_Output_Enable();
        if(output_enable)
        {
            Flight_Fly_Test_Set_Output_Enable(0U);
            Flight_Fly_Test_Set_Confirm_State(FLY_TEST_CONFIRM_IDLE);
        }
        else if(test_fly_output_safe())
        {
            Flight_Fly_Test_Set_Confirm_State(FLY_TEST_CONFIRM_WAIT);
        }
        key_flag = 0;
    }
    else if(key_flag == Button_4)
    {
        Flight_Fly_Test_Stop();
        return;
    }

    throttle = Flight_Fly_Test_Get_Throttle();
    output_enable = Flight_Fly_Test_Get_Output_Enable();
    safety_latched = Flight_Fly_Test_Get_Safety_Latched();
    confirm_state = Flight_Fly_Test_Get_Confirm_State();
    Flight_Fly_Test_Get_Target(&target_roll, &target_pitch, &target_yaw);
    safe_text = safety_latched ? "LATCH" : test_safety_reason(1U);

    test_page_show_text(TEST_ROW_Y(0U), "KEY:", confirm_state == FLY_TEST_CONFIRM_WAIT ? "K2 OK K1 NO" : "K1+ K2- K3",
                        confirm_state == FLY_TEST_CONFIRM_WAIT ? RGB565_YELLOW : RGB565_WHITE);
    test_page_show_int(TEST_ROW_Y(1U), "THR:", (int32)throttle, 4U);
    test_page_show_text(TEST_ROW_Y(2U), "OUT:", output_enable ? "ON   " : "OFF  ",
                        output_enable ? RGB565_RED : RGB565_GREEN);
    test_page_show_text(TEST_ROW_Y(3U), "CONF:",
                        (confirm_state == FLY_TEST_CONFIRM_WAIT) ? "WAIT " :
                        (confirm_state == FLY_TEST_CONFIRM_ON) ? "ON   " : "IDLE ",
                        confirm_state == FLY_TEST_CONFIRM_WAIT ? RGB565_YELLOW :
                        (confirm_state == FLY_TEST_CONFIRM_ON ? RGB565_RED : RGB565_WHITE));
    test_page_show_text(TEST_ROW_Y(4U), "SAFE:", safe_text,
                        (safe_text[0] == 'O') ? RGB565_GREEN : RGB565_RED);
    test_page_show_float(TEST_ROW_Y(5U), "ROLL:", current_roll, 3U, 1U);
    test_page_show_float(TEST_ROW_Y(6U), "PITCH:", current_pitch, 3U, 1U);
    test_page_show_float(TEST_ROW_Y(7U), "T ROLL:", target_roll, 3U, 1U);
    test_page_show_float(TEST_ROW_Y(8U), "T PITCH:", target_pitch, 3U, 1U);
    test_page_show_int(TEST_ROW_Y(9U), "M1 M2:", (int32)motor_throttle[0], 4U);
    ips200_show_int(TEST_RIGHT_VALUE_X, TEST_ROW_Y(9U), (int32)motor_throttle[1], 4U);
    test_page_show_int(TEST_ROW_Y(10U), "M3 M4:", (int32)motor_throttle[2], 4U);
    ips200_show_int(TEST_RIGHT_VALUE_X, TEST_ROW_Y(10U), (int32)motor_throttle[3], 4U);
    test_page_show_text(TEST_ROW_Y(11U), "LIMIT:", "FIXED FRAME", RGB565_YELLOW);
    (void)target_yaw;
}

static void motor_test_page(void)
{
    static uint8 edit_field = 0U;
    uint8 selected_motor = 0U;
    uint8 output_enable = 0U;
    uint16 throttle = 0U;
    uint16 output_limit = 0U;
    uint16 test_limit = 0U;

    if(!Flight_Motor_Test_Is_Active())
    {
        Flight_Motor_Test_Start();
        edit_field = 0U;
    }

    if(key_flag == Button_4)
    {
        Flight_Motor_Test_Stop();
        return;
    }

    selected_motor = Flight_Motor_Test_Get_Selected();
    throttle = Flight_Motor_Test_Get_Throttle();
    output_limit = Flight_Motor_Test_Get_Output_Limit();
    test_limit = (MOTOR_TEST_MAX_THROTTLE < output_limit) ? MOTOR_TEST_MAX_THROTTLE : output_limit;

    if(key_flag == Button_1)
    {
        switch(edit_field)
        {
            case 0U:
                selected_motor = (selected_motor == 0U) ? 3U : (selected_motor - 1U);
                Flight_Motor_Test_Set_Selected(selected_motor);
                break;

            case 1U:
                throttle = (throttle > MOTOR_TEST_STEP) ? (throttle - MOTOR_TEST_STEP) : 0U;
                Flight_Motor_Test_Set_Throttle(throttle);
                break;

            case 2U:
                Flight_Motor_Test_Set_Output_Enable(0U);
                break;

            default:
                break;
        }
        key_flag = 0;
    }
    else if(key_flag == Button_2)
    {
        switch(edit_field)
        {
            case 0U:
                selected_motor = (selected_motor + 1U) & 0x03U;
                Flight_Motor_Test_Set_Selected(selected_motor);
                break;

            case 1U:
                throttle = (throttle + MOTOR_TEST_STEP > test_limit) ? test_limit : (throttle + MOTOR_TEST_STEP);
                Flight_Motor_Test_Set_Throttle(throttle);
                break;

            case 2U:
                Flight_Motor_Test_Set_Output_Enable(1U);
                break;

            default:
                break;
        }
        key_flag = 0;
    }
    else if(key_flag == Button_3)
    {
        edit_field = (edit_field + 1U) % 3U;
        key_flag = 0;
    }

    selected_motor = Flight_Motor_Test_Get_Selected();
    throttle = Flight_Motor_Test_Get_Throttle();
    output_enable = Flight_Motor_Test_Get_Output_Enable();

    test_page_show_text(TEST_ROW_Y(0U), "KEY:",
                        (edit_field == 2U) ? "K1 OFF K2 ON" : "K1- K2+ K3",
                        RGB565_WHITE);
    test_page_show_text(TEST_ROW_Y(1U), (edit_field == 0U) ? "> MOTOR:" : "  MOTOR:", "M", RGB565_WHITE);
    ips200_show_int((uint16)(TEST_VALUE_X + 16U), TEST_ROW_Y(1U), (int32)(selected_motor + 1U), 1U);
    test_page_show_int(TEST_ROW_Y(2U), (edit_field == 1U) ? "> THR:" : "  THR:", (int32)throttle, 4U);
    test_page_show_text(TEST_ROW_Y(3U), (edit_field == 2U) ? "> OUT:" : "  OUT:",
                        output_enable ? "ON   " : "OFF  ",
                        output_enable ? RGB565_RED : RGB565_GREEN);
    test_page_show_text(TEST_ROW_Y(5U), "PROP:", "REMOVE", RGB565_YELLOW);
}

/*
 * ????????
 * ???????????????????????????????
 */
void init_test_menu(void)
{
    // 初始化主菜单对象
    strncpy(mainMenu.title, "Main Menu", MAX_MENU_NAME_LEN-1);
    mainMenu.title[MAX_MENU_NAME_LEN - 1] = '\0';
    mainMenu.itemCount = 0;
    mainMenu.currentSelection = 0;
    mainMenu.parentMenu = NULL;

    strncpy(testMenu.title, "Test Menu", MAX_MENU_NAME_LEN - 1);
    testMenu.title[MAX_MENU_NAME_LEN - 1] = '\0';
    testMenu.itemCount = 0;
    testMenu.currentSelection = 0;
    testMenu.parentMenu = NULL;
    
    // 注册主菜单项
    menu_create_item(&mainMenu, "Test", NULL, &testMenu);
    menu_create_item(&mainMenu, "ICM Init", icm_init_menu, NULL);
    menu_create_item(&mainMenu, "Flight View", exit_menu_system, NULL);

    menu_create_item(&testMenu, "Preflight", preflight_page, NULL);
    menu_create_item(&testMenu, "Motor Test", motor_test_page, NULL);
    menu_create_item(&testMenu, "Fly Test", fly_test_page, NULL);
    menu_create_item(&testMenu, "IMU Status", imu_status_page, NULL);
    menu_create_item(&testMenu, "Flow Status", flow_status_page, NULL);
    menu_create_item(&testMenu, "Vision View", vision_view_page, NULL);
    menu_create_item(&testMenu, "Track Light", track_light_test_page, NULL);
    menu_create_item(&testMenu, "VOFA Status", vofa_status_page, NULL);
    
}

/*
 * 菜单启动前初始化:
 * ?????????????????????????????
 */
void menu_open_init()
{
    /*
     * [2026-03-10]
     * 修改目的:
     * ?????????????????????????????
     */
    menu_reset_exit_flag();
    menu_enter_main_menu_view();
}

/*
 * 菜单运行入口:
 * ????????????????????????????????????
 */
void menu_open(void)
{
    if (menu_is_monitor_view())
    {
        return;
    }

    menu_init();
    menu_reset_exit_flag();
    menu_enter_main_menu_view();
    init_test_menu();
    menu_set_current(&mainMenu);
    
    // 菜单处理循环
    while (!g_menu_exit_flag && !menu_is_monitor_view())
    {
        // 处理菜单逻辑
        g_currentMenu = menu_process(g_currentMenu);
#if CY_CORE_CM7_0
        (void)CrossCoreCamera_ControlCoreService(current_roll,
                                                 current_pitch,
                                                 tof_height_true_filt * 100.0f);
#endif
        Send_test();
        USART_PID_Adjust();
        if (g_currentMenu == NULL)
        {
            // ?????????
            break;
        }
        
        // ????????????????
        system_delay_ms(10);
    }
    
    if (!menu_is_monitor_view())
    {
        // 退出菜单后清屏，避免残留旧页面内容
        ips200_clear();
    }
}
