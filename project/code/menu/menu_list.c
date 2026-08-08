#include "common/project_headfile.h"
#include "motor_hardware/small_driver_uart_control.h"

#include <stdio.h>

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
#define SENSOR_LEFT_LABEL_X     4U
#define SENSOR_LEFT_VALUE_X     70U
#define SENSOR_RIGHT_LABEL_X    124U
#define SENSOR_RIGHT_VALUE_X    196U
#define SENSOR_TOF_WARN_AGE_MS  80U
#define SENSOR_TOF_STALE_AGE_MS 100U
#define SENSOR_ESC_WARN_AGE_MS  400U
#define SENSOR_STATUS_LOOP_PERIOD_MS 5U
#define SENSOR_STATUS_REFRESH_MS 50U
#define SENSOR_STATUS_FRAME_WINDOW_MS 1000U
#define VISION_VIEW_SNAPSHOT_WAIT_TICKS 120U
#define VISION_VIEW_SNAPSHOT_WAIT_MS    5U
#define TRACK_LIGHT_RELEASE_TIMEOUT_TICKS 40U
#define REAR_BOARD_RX_REFRESH_TICKS       20U
#define MT9V03X_ADJUST_INFO_Y           128U
#define MT9V03X_ADJUST_STEP_Y           148U
#define MT9V03X_ADJUST_STATUS_Y         168U
#define MT9V03X_ADJUST_HINT_Y           188U
#define MT9V03X_ADJUST_EXPOSURE_MIN     1U
#define MT9V03X_ADJUST_EXPOSURE_MAX     65535U
#define BINARY_ADJUST_THRESHOLD_Y       128U
#define BINARY_ADJUST_STEP_Y            148U
#define BINARY_ADJUST_MODE_Y            168U
#define BINARY_ADJUST_HINT_Y            188U
#define BINARY_ADJUST_THRESHOLD_MIN     1U
#define BINARY_ADJUST_THRESHOLD_MAX     255U

static uint8 s_track_light_page_active = 0U;
static uint8 s_track_light_image_visible = 0U;
static uint8 s_track_light_release_pending = 0U;
static uint8 s_track_light_exit_pending = 0U;
static uint8 s_track_light_release_wait_ticks = 0U;
static uint8 s_track_light_status_tick_div = 0U;
static uint8 s_sensor_status_page_active = 0U;
static uint8 s_sensor_status_view = 0U;
static uint16 s_sensor_status_refresh_ms = 0U;
static uint16 s_sensor_status_frame_window_ms = 0U;
static uint32 s_sensor_status_frame_previous_count = 0U;
static uint32 s_sensor_status_frame_accumulated = 0U;
static float s_sensor_status_image_fps = 0.0f;
static uint8 s_sensor_status_frame_rate_valid = 0U;
static uint8 s_rear_board_rx_page_active = 0U;
static uint8 s_rear_board_rx_view = 0U;
static uint8 s_rear_board_rx_beacon_index = 0U;
static uint8 s_rear_board_rx_refresh_ticks = 0U;

static void test_page_clear_line(uint16 y);

static void mt9v03x_adjust_page(void)
{
    static uint16 exposure_time = MT9V03X_EXP_TIME_DEF;
    static uint16 adjust_step = 1U;
    static uint8 set_result = 0U;
    static uint8 page_active = 0U;
    uint16 next_exposure = exposure_time;
    uint16 y;

    if(!page_active)
    {
        page_active = 1U;
        key_flag = 0U;
    }

    if(key_flag == Button_4)
    {
        page_active = 0U;
        return;
    }

    if(key_flag == Button_1)
    {
        next_exposure = (exposure_time > adjust_step) ?
                        (uint16)(exposure_time - adjust_step) :
                        (uint16)MT9V03X_ADJUST_EXPOSURE_MIN;
        key_flag = 0U;
    }
    else if(key_flag == Button_2)
    {
        next_exposure = (exposure_time > (MT9V03X_ADJUST_EXPOSURE_MAX - adjust_step)) ?
                        (uint16)MT9V03X_ADJUST_EXPOSURE_MAX :
                        (uint16)(exposure_time + adjust_step);
        key_flag = 0U;
    }
    else if(key_flag == Button_3)
    {
        adjust_step = (adjust_step == 1U) ? 10U : 1U;
        key_flag = 0U;
    }

    if(next_exposure != exposure_time)
    {
        set_result = mt9v03x_set_exposure_time(next_exposure);
        if(set_result == 0U)
        {
            exposure_time = next_exposure;
        }
        key_flag = 0U;
    }

    ips200_show_gray_image(0U, 0U, mt9v03x_image[0],
                           MT9V03X_W, MT9V03X_H,
                           MT9V03X_W, MT9V03X_H, 0U);

    y = MT9V03X_ADJUST_INFO_Y;
    test_page_clear_line(y);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(TEST_PAGE_X, y, "EXPOSURE:");
    ips200_show_uint(TEST_VALUE_X, y, exposure_time, 5U);

    y = MT9V03X_ADJUST_STEP_Y;
    test_page_clear_line(y);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(TEST_PAGE_X, y, "STEP:");
    ips200_show_uint(TEST_VALUE_X, y, adjust_step, 2U);

    y = MT9V03X_ADJUST_STATUS_Y;
    test_page_clear_line(y);
    ips200_set_color(set_result == 0U ? RGB565_GREEN : RGB565_RED, RGB565_BLACK);
    ips200_show_string(TEST_PAGE_X, y, set_result == 0U ? "SET: OK" : "SET: FAIL");

    y = MT9V03X_ADJUST_HINT_Y;
    test_page_clear_line(y);
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    ips200_show_string(TEST_PAGE_X, y, "K1- K2+ K3 STEP K4 BACK");
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

static void binary_threshold_adjust_page(void)
{
    static uint8 threshold = (uint8)VISION_BINARY_THRESHOLD_INNER;
    static uint8 zone = VISION_BINARY_ZONE_INNER;
    static uint8 page_active = 0U;
    uint16 next_threshold = threshold;
    uint16 y;
    const char *zone_text;

    if(!page_active)
    {
        page_active = 1U;
        zone = VISION_BINARY_ZONE_INNER;
        threshold = CrossCoreCamera_ControlCoreGetBinaryThreshold(zone);
        key_flag = 0U;
    }
    next_threshold = threshold;

    if(key_flag == Button_4)
    {
        page_active = 0U;
        return;
    }

    if(key_flag == Button_1)
    {
        next_threshold = (threshold > BINARY_ADJUST_THRESHOLD_MIN) ?
                         (uint16)(threshold - 1U) :
                         (uint16)BINARY_ADJUST_THRESHOLD_MIN;
        key_flag = 0U;
    }
    else if(key_flag == Button_2)
    {
        next_threshold = (threshold >= BINARY_ADJUST_THRESHOLD_MAX) ?
                         (uint16)BINARY_ADJUST_THRESHOLD_MAX :
                         (uint16)(threshold + 1U);
        key_flag = 0U;
    }
    else if(key_flag == Button_3)
    {
        zone = (uint8)((zone + 1U) % VISION_BINARY_ZONE_COUNT);
        threshold = CrossCoreCamera_ControlCoreGetBinaryThreshold(zone);
        next_threshold = threshold;
        key_flag = 0U;
    }

    if(next_threshold != threshold)
    {
        threshold = (uint8)next_threshold;
        CrossCoreCamera_ControlCoreSetBinaryThreshold(zone, threshold);
    }

    ips200_show_gray_image(0U, 0U, mt9v03x_image[0],
                           MT9V03X_W, MT9V03X_H,
                           MT9V03X_W, MT9V03X_H, threshold);

    y = BINARY_ADJUST_THRESHOLD_Y;
    test_page_clear_line(y);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(TEST_PAGE_X, y, "THRESHOLD:");
    ips200_show_uint(TEST_VALUE_X, y, threshold, 3U);

    y = BINARY_ADJUST_STEP_Y;
    test_page_clear_line(y);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    zone_text = (zone == VISION_BINARY_ZONE_INNER) ? "INNER" :
                ((zone == VISION_BINARY_ZONE_MIDDLE) ? "MIDDLE" : "OUTER");
    ips200_show_string(TEST_PAGE_X, y, "ZONE:");
    ips200_show_string(TEST_VALUE_X, y, zone_text);

    y = BINARY_ADJUST_MODE_Y;
    test_page_clear_line(y);
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    ips200_show_string(TEST_PAGE_X, y, "STEP: 1");

    y = BINARY_ADJUST_HINT_Y;
    test_page_clear_line(y);
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    ips200_show_string(TEST_PAGE_X, y, "K1- K2+ K3 ZONE K4 BACK");
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

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

static void test_page_show_uint(uint16 y, const char *label, uint32 value, uint8 digits)
{
    test_page_clear_line(y);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(TEST_PAGE_X, y, (char *)label);
    ips200_show_uint(TEST_VALUE_X, y, value, digits);
}

static void test_page_show_float(uint16 y, const char *label, float value, uint8 digits, uint8 point_digits)
{
    test_page_clear_line(y);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(TEST_PAGE_X, y, (char *)label);
    ips200_show_float(TEST_VALUE_X, y, value, digits, point_digits);
}

static void sensor_page_clear_line(uint16 y)
{
    ips200_set_font(IPS200_6X8_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0U, y, "                                        ");
}

static void sensor_page_show_text(uint16 label_x, uint16 value_x, uint16 y,
                                  const char *label, const char *value, uint16 color)
{
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(label_x, y, (char *)label);
    ips200_set_color(color, RGB565_BLACK);
    ips200_show_string(value_x, y, (char *)value);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

static void sensor_page_show_float(uint16 label_x, uint16 value_x, uint16 y,
                                   const char *label, float value,
                                   uint8 digits, uint8 point_digits, uint16 color)
{
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(label_x, y, (char *)label);
    ips200_set_color(color, RGB565_BLACK);
    ips200_show_float(value_x, y, value, digits, point_digits);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

static void sensor_page_show_int(uint16 label_x, uint16 value_x, uint16 y,
                                 const char *label, int32 value, uint8 digits, uint16 color)
{
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(label_x, y, (char *)label);
    ips200_set_color(color, RGB565_BLACK);
    ips200_show_int(value_x, y, value, digits);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

static uint16 sensor_page_bool_color(uint8 value)
{
    return value ? RGB565_GREEN : RGB565_RED;
}

static const char *sensor_page_yes_no(uint8 value)
{
    return value ? "YES" : "NO";
}

static uint16 sensor_page_age_color(uint16 age_ms, uint16 warn_ms, uint16 stale_ms)
{
    if(age_ms >= stale_ms)
    {
        return RGB565_RED;
    }
    if(age_ms >= warn_ms)
    {
        return RGB565_YELLOW;
    }
    return RGB565_GREEN;
}

static const char *sensor_page_tof_source_name(uint8 source)
{
    switch((DualTofSource_t)source)
    {
        case DUAL_TOF_SOURCE_TOF1: return "T1";
        case DUAL_TOF_SOURCE_TOF2: return "T2";
        case DUAL_TOF_SOURCE_EQUAL: return "T1=T2";
        default: return "NONE";
    }
}

static const char *sensor_page_tof_state_name(uint8 state)
{
    switch((HeightTofState_t)state)
    {
        case HEIGHT_TOF_STATE_LIVE: return "LIVE";
        case HEIGHT_TOF_STATE_HELD_REJECTED: return "HELD";
        case HEIGHT_TOF_STATE_REACQUIRING: return "REACQ";
        default: return "INVALID";
    }
}

static uint16 sensor_page_tof_state_color(uint8 state)
{
    if(state == (uint8)HEIGHT_TOF_STATE_LIVE)
    {
        return RGB565_GREEN;
    }
    if(state == (uint8)HEIGHT_TOF_STATE_HELD_REJECTED ||
       state == (uint8)HEIGHT_TOF_STATE_REACQUIRING)
    {
        return RGB565_YELLOW;
    }
    return RGB565_RED;
}

static void sensor_page_draw_header(void)
{
    ips200_set_font(IPS200_8X16_FONT);
    ui_menu_draw_function_header(s_sensor_status_view ? "Sensor IMU/Power" : "Sensor ToF/Flow");
    ips200_set_font(IPS200_6X8_FONT);
}

static void sensor_page_draw_hint(void)
{
    uint16 y = TEST_ROW_Y(11U);

    sensor_page_clear_line(y);
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    ips200_show_string(SENSOR_LEFT_LABEL_X, y, "K1/K2 VIEW    K4 BACK");
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

static void sensor_status_service_link(void)
{
    if(Car_Copy_Screen_Flag)
    {
        Car_Copy_Screen_Flag = 0U;
        core_link_user_tick((uint16)(FLOW_SENSOR_UPDATE_US / 1000U));
        core_link_user_service_beacon_link();
    }
}

static void sensor_status_sample_frame_rate(void)
{
    VehicleMarkerLossDebug_t loss_debug = {0};
    uint32 frame_delta = 0U;

    VehicleMarkerLocalization_GetLossDebug(&loss_debug);
    if(loss_debug.process_count >= s_sensor_status_frame_previous_count)
    {
        frame_delta = loss_debug.process_count - s_sensor_status_frame_previous_count;
    }
    else
    {
        s_sensor_status_frame_accumulated = 0U;
        s_sensor_status_frame_window_ms = 0U;
    }

    s_sensor_status_frame_previous_count = loss_debug.process_count;
    s_sensor_status_frame_accumulated += frame_delta;
    if(s_sensor_status_frame_window_ms < (uint16)(SENSOR_STATUS_FRAME_WINDOW_MS - SENSOR_STATUS_LOOP_PERIOD_MS))
    {
        s_sensor_status_frame_window_ms += SENSOR_STATUS_LOOP_PERIOD_MS;
    }
    else
    {
        s_sensor_status_frame_window_ms = SENSOR_STATUS_FRAME_WINDOW_MS;
    }

    if(s_sensor_status_frame_window_ms >= SENSOR_STATUS_FRAME_WINDOW_MS)
    {
        s_sensor_status_image_fps = ((float)s_sensor_status_frame_accumulated * 1000.0f) /
                                    (float)s_sensor_status_frame_window_ms;
        s_sensor_status_frame_rate_valid = 1U;
        s_sensor_status_frame_accumulated = 0U;
        s_sensor_status_frame_window_ms = 0U;
    }
}

static void vision_view_send_snapshot(void)
{
    uint8 wait_count = 0U;
    uint8 sent = 0U;
    HeightTofSnapshot_t height_snapshot = {0};

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
        HeightManager_GetTofSnapshot(&height_snapshot);
        (void)CrossCoreCamera_ControlCoreService(current_roll,
                                                 current_pitch,
                                                 current_yaw,
                                                 height_snapshot.filtered_height_cm,
                                                 height_snapshot.filtered_height_cm);
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

static void sensor_status_draw_tof_flow(void)
{
    HeightManagerDiag_t height_diag = {0};
    PMWFlowState_t flow_state = {0};
    uint8 tof1_online;
    uint8 tof2_online;
    uint16 tof1_color;
    uint16 tof2_color;
    uint16 flow_color;
    uint16 fused_color;
    uint16 y;

    HeightManager_Get_Diag(&height_diag);
    PMW_Flow_GetState(&flow_state);

    tof1_online = (uint8)(height_diag.tof1_initialized &&
                          height_diag.tof1_valid &&
                          height_diag.tof1_age_ms < SENSOR_TOF_STALE_AGE_MS);
    tof2_online = (uint8)(height_diag.tof2_initialized &&
                          height_diag.tof2_valid &&
                          height_diag.tof2_age_ms < SENSOR_TOF_STALE_AGE_MS);
    tof1_color = tof1_online ? sensor_page_age_color(height_diag.tof1_age_ms,
                                                     SENSOR_TOF_WARN_AGE_MS,
                                                     SENSOR_TOF_STALE_AGE_MS) : RGB565_RED;
    tof2_color = tof2_online ? sensor_page_age_color(height_diag.tof2_age_ms,
                                                     SENSOR_TOF_WARN_AGE_MS,
                                                     SENSOR_TOF_STALE_AGE_MS) : RGB565_RED;
    flow_color = sensor_page_bool_color(flow_state.valid);
    fused_color = sensor_page_bool_color(flow_state.fused_velocity_valid);

    y = TEST_ROW_Y(0U);
    sensor_page_clear_line(y);
    sensor_page_show_float(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, y,
                           "T1 CM", height_diag.tof1_raw_cm, 4U, 1U, tof1_color);
    sensor_page_show_float(SENSOR_RIGHT_LABEL_X, SENSOR_RIGHT_VALUE_X, y,
                           "T2 CM", height_diag.tof2_raw_cm, 4U, 1U, tof2_color);

    y = TEST_ROW_Y(1U);
    sensor_page_clear_line(y);
    sensor_page_show_float(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, y,
                           "SEL CM", height_diag.tof_selected_cm, 4U, 1U,
                           sensor_page_bool_color(height_diag.tof_valid));
    sensor_page_show_text(SENSOR_RIGHT_LABEL_X, SENSOR_RIGHT_VALUE_X, y,
                          "SOURCE", sensor_page_tof_source_name(height_diag.tof_source), RGB565_CYAN);

    y = TEST_ROW_Y(2U);
    sensor_page_clear_line(y);
    sensor_page_show_text(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, y,
                          "T1 ON", sensor_page_yes_no(tof1_online), sensor_page_bool_color(tof1_online));
    sensor_page_show_text(SENSOR_RIGHT_LABEL_X, SENSOR_RIGHT_VALUE_X, y,
                          "T2 ON", sensor_page_yes_no(tof2_online), sensor_page_bool_color(tof2_online));

    y = TEST_ROW_Y(3U);
    sensor_page_clear_line(y);
    sensor_page_show_int(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, y,
                         "T1 AGE", (int32)height_diag.tof1_age_ms, 5U,
                         sensor_page_age_color(height_diag.tof1_age_ms,
                                               SENSOR_TOF_WARN_AGE_MS,
                                               SENSOR_TOF_STALE_AGE_MS));
    sensor_page_show_int(SENSOR_RIGHT_LABEL_X, SENSOR_RIGHT_VALUE_X, y,
                         "T2 AGE", (int32)height_diag.tof2_age_ms, 5U,
                         sensor_page_age_color(height_diag.tof2_age_ms,
                                               SENSOR_TOF_WARN_AGE_MS,
                                               SENSOR_TOF_STALE_AGE_MS));

    y = TEST_ROW_Y(4U);
    sensor_page_clear_line(y);
    sensor_page_show_float(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, y,
                           "VX RAW", flow_state.velocity_x_raw_cms, 4U, 1U, flow_color);
    sensor_page_show_float(SENSOR_RIGHT_LABEL_X, SENSOR_RIGHT_VALUE_X, y,
                           "VY RAW", flow_state.velocity_y_raw_cms, 4U, 1U, flow_color);

    y = TEST_ROW_Y(5U);
    sensor_page_clear_line(y);
    sensor_page_show_float(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, y,
                           "VX FUSE", flow_state.velocity_x_fused_cms, 4U, 1U, fused_color);
    sensor_page_show_float(SENSOR_RIGHT_LABEL_X, SENSOR_RIGHT_VALUE_X, y,
                           "VY FUSE", flow_state.velocity_y_fused_cms, 4U, 1U, fused_color);

    y = TEST_ROW_Y(6U);
    sensor_page_clear_line(y);
    sensor_page_show_text(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, y,
                          "FLOW OK", sensor_page_yes_no(flow_state.valid), flow_color);
    sensor_page_show_text(SENSOR_RIGHT_LABEL_X, SENSOR_RIGHT_VALUE_X, y,
                          "FUSED OK", sensor_page_yes_no(flow_state.fused_velocity_valid), fused_color);

    y = TEST_ROW_Y(7U);
    sensor_page_clear_line(y);
    sensor_page_show_text(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, y,
                          "FLOW USE", sensor_page_yes_no(flow_state.flow_used),
                          flow_state.flow_used ? RGB565_GREEN : RGB565_YELLOW);
    sensor_page_show_float(SENSOR_RIGHT_LABEL_X, SENSOR_RIGHT_VALUE_X, y,
                           "CONF", flow_state.flow_confidence, 1U, 3U,
                           flow_state.flow_used ? RGB565_GREEN : RGB565_YELLOW);

    y = TEST_ROW_Y(8U);
    sensor_page_clear_line(y);
    sensor_page_show_float(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, y,
                           "FLOW DT", flow_state.dt_s * 1000.0f, 3U, 1U,
                           flow_state.valid ? RGB565_GREEN : RGB565_RED);
    sensor_page_show_float(SENSOR_RIGHT_LABEL_X, SENSOR_RIGHT_VALUE_X, y,
                           "FLOW HEIGHT", flow_state.range_cm, 4U, 1U,
                           (flow_state.range_cm > 0.0f) ? RGB565_GREEN : RGB565_RED);

    y = TEST_ROW_Y(9U);
    sensor_page_clear_line(y);
    sensor_page_show_text(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, y,
                          "TOF STATE", sensor_page_tof_state_name(height_diag.tof_state),
                          sensor_page_tof_state_color(height_diag.tof_state));
    sensor_page_show_int(SENSOR_RIGHT_LABEL_X, SENSOR_RIGHT_VALUE_X, y,
                         "TOF AGE", (int32)height_diag.tof_raw_age_ms, 5U,
                         sensor_page_age_color(height_diag.tof_raw_age_ms,
                                               SENSOR_TOF_WARN_AGE_MS,
                                               SENSOR_TOF_STALE_AGE_MS));

    sensor_page_clear_line(TEST_ROW_Y(10U));
    sensor_page_draw_hint();
}

static void sensor_status_draw_imu_power(void)
{
    BatteryStatus battery_status = {0};
    ICM_Health_t imu_health = {0};
    SmallDriverEscStatus esc_status = {0};
    CoreLinkBeaconDiag_t car_diag = {0};
    PositionLoopDiag_t pos_diag = {0};
    char esc_code_text[24];
    uint8 imu_fresh;
    uint8 esc_online;
    uint8 esc_fault_active = 0U;
    uint8 i;
    uint16 battery_color;
    uint16 esc_age_color;
    uint16 y;

    BatteryMonitor_GetStatus(&battery_status);
    ICM_GetHealth(&imu_health);
    small_driver_get_esc_status(&esc_status);
    core_link_user_get_beacon_diag(&car_diag);
    Position_Loop_Get_Diag(&pos_diag);

    imu_fresh = (uint8)(!imu_health.sample_stale && imu_health.sample_sequence != 0U);
    esc_online = (uint8)(esc_status.valid && !esc_status.stale);
    for(i = 0U; i < 4U; i++)
    {
        if(esc_status.active_fault[i] != (uint8)SMALL_DRIVER_ESC_FAULT_NONE)
        {
            esc_fault_active = 1U;
        }
    }

    if(!battery_status.local_valid || battery_status.low_warning)
    {
        battery_color = RGB565_RED;
    }
    else if(battery_status.local_filtered_voltage_v < battery_status.low_recover_voltage_v)
    {
        battery_color = RGB565_YELLOW;
    }
    else
    {
        battery_color = RGB565_GREEN;
    }

    if(!esc_status.valid || esc_status.age_ms >= SMALL_DRIVER_ESC_STATUS_STALE_MS)
    {
        esc_age_color = RGB565_RED;
    }
    else if(esc_status.age_ms >= SENSOR_ESC_WARN_AGE_MS)
    {
        esc_age_color = RGB565_YELLOW;
    }
    else
    {
        esc_age_color = RGB565_GREEN;
    }

    (void)snprintf(esc_code_text, sizeof(esc_code_text),
                   "M1:%u M2:%u M3:%u M4:%u",
                   (unsigned int)esc_status.active_fault[0],
                   (unsigned int)esc_status.active_fault[1],
                   (unsigned int)esc_status.active_fault[2],
                   (unsigned int)esc_status.active_fault[3]);

    y = TEST_ROW_Y(0U);
    sensor_page_clear_line(y);
    sensor_page_show_float(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, y,
                           "ROLL", current_roll, 3U, 1U, RGB565_CYAN);
    sensor_page_show_float(SENSOR_RIGHT_LABEL_X, SENSOR_RIGHT_VALUE_X, y,
                           "PITCH", current_pitch, 3U, 1U, RGB565_CYAN);

    y = TEST_ROW_Y(1U);
    sensor_page_clear_line(y);
    sensor_page_show_float(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, y,
                           "YAW", current_yaw, 3U, 1U, RGB565_CYAN);
    sensor_page_show_text(SENSOR_RIGHT_LABEL_X, SENSOR_RIGHT_VALUE_X, y,
                          "IMU FRESH", sensor_page_yes_no(imu_fresh), sensor_page_bool_color(imu_fresh));

    y = TEST_ROW_Y(2U);
    sensor_page_clear_line(y);
    sensor_page_show_float(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, y,
                           "ACC R", Roll_acc_F, 4U, 1U, RGB565_WHITE);
    sensor_page_show_float(SENSOR_RIGHT_LABEL_X, SENSOR_RIGHT_VALUE_X, y,
                           "ACC P", Pitch_acc_F, 4U, 1U, RGB565_WHITE);

    y = TEST_ROW_Y(3U);
    sensor_page_clear_line(y);
    sensor_page_show_float(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, y,
                           "ACC Y", Yaw_acc_F, 4U, 1U, RGB565_WHITE);
    sensor_page_show_float(SENSOR_RIGHT_LABEL_X, SENSOR_RIGHT_VALUE_X, y,
                           "ACC NORM", acc_norm_filtered, 4U, 1U, RGB565_WHITE);

    y = TEST_ROW_Y(4U);
    sensor_page_clear_line(y);
    sensor_page_show_float(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, y,
                           "GYRO R", current_gyro_roll, 4U, 1U, RGB565_WHITE);
    sensor_page_show_float(SENSOR_RIGHT_LABEL_X, SENSOR_RIGHT_VALUE_X, y,
                           "GYRO P", current_gyro_pitch, 4U, 1U, RGB565_WHITE);

    y = TEST_ROW_Y(5U);
    sensor_page_clear_line(y);
    sensor_page_show_float(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, y,
                           "GYRO Y", current_gyro_yaw, 4U, 1U, RGB565_WHITE);
    sensor_page_show_float(SENSOR_RIGHT_LABEL_X, SENSOR_RIGHT_VALUE_X, y,
                           "DT US", imu_health.sample_interval_us, 4U, 1U,
                           imu_fresh ? RGB565_GREEN : RGB565_RED);

    y = TEST_ROW_Y(6U);
    sensor_page_clear_line(y);
    sensor_page_show_float(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, y,
                           "BAT V", battery_status.local_filtered_voltage_v, 2U, 2U, battery_color);
    sensor_page_show_text(SENSOR_RIGHT_LABEL_X, SENSOR_RIGHT_VALUE_X, y,
                          "ESC ON", sensor_page_yes_no(esc_online), sensor_page_bool_color(esc_online));

    y = TEST_ROW_Y(7U);
    sensor_page_clear_line(y);
    sensor_page_show_int(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, y,
                         "ESC MS", (int32)esc_status.age_ms, 5U, esc_age_color);

    y = TEST_ROW_Y(8U);
    sensor_page_clear_line(y);
    sensor_page_show_text(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, y,
                          "ESC CODE", esc_code_text,
                          esc_fault_active ? RGB565_RED : RGB565_GREEN);

    sensor_page_clear_line(TEST_ROW_Y(9U));
    sensor_page_show_float(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, TEST_ROW_Y(9U),
                           "POS ERR X", pos_diag.error_x_cm, 4U, 1U, RGB565_CYAN);
    sensor_page_show_float(SENSOR_RIGHT_LABEL_X, SENSOR_RIGHT_VALUE_X, TEST_ROW_Y(9U),
                           "POS ERR Y", pos_diag.error_y_cm, 4U, 1U, RGB565_CYAN);

    sensor_page_clear_line(TEST_ROW_Y(10U));
    sensor_page_show_text(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, TEST_ROW_Y(10U),
                          "CAR LINK", sensor_page_yes_no(car_diag.valid),
                          sensor_page_bool_color(car_diag.valid));
    sensor_page_show_float(SENSOR_RIGHT_LABEL_X, SENSOR_RIGHT_VALUE_X, TEST_ROW_Y(10U),
                           "IMG FPS", s_sensor_status_image_fps, 3U, 1U,
                           (s_sensor_status_frame_rate_valid && s_sensor_status_image_fps > 0.0f) ?
                           RGB565_GREEN : RGB565_RED);
    sensor_page_draw_hint();
}

static void sensor_status_page(void)
{
    uint8 refresh_now = 0U;

    if(!s_sensor_status_page_active)
    {
        s_sensor_status_page_active = 1U;
        s_sensor_status_view = 0U;
        s_sensor_status_refresh_ms = SENSOR_STATUS_REFRESH_MS;
        s_sensor_status_frame_window_ms = 0U;
        s_sensor_status_frame_accumulated = 0U;
        s_sensor_status_image_fps = 0.0f;
        s_sensor_status_frame_rate_valid = 0U;
        {
            VehicleMarkerLossDebug_t loss_debug = {0};
            VehicleMarkerLocalization_GetLossDebug(&loss_debug);
            s_sensor_status_frame_previous_count = loss_debug.process_count;
        }
        sensor_page_draw_header();
        refresh_now = 1U;
    }

    if(key_flag == Button_1 || key_flag == Button_2)
    {
        s_sensor_status_view ^= 1U;
        key_flag = 0U;
        sensor_page_draw_header();
        refresh_now = 1U;
    }
    else if(key_flag == Button_4 || g_menu_exit_flag || menu_is_monitor_view())
    {
        s_sensor_status_page_active = 0U;
        s_sensor_status_view = 0U;
        s_sensor_status_refresh_ms = 0U;
        s_sensor_status_frame_window_ms = 0U;
        s_sensor_status_frame_accumulated = 0U;
        s_sensor_status_image_fps = 0.0f;
        s_sensor_status_frame_rate_valid = 0U;
        ips200_set_font(IPS200_8X16_FONT);
        return;
    }

    sensor_status_service_link();
    sensor_status_sample_frame_rate();

    if(!refresh_now)
    {
        if(s_sensor_status_refresh_ms < (uint16)(SENSOR_STATUS_REFRESH_MS - SENSOR_STATUS_LOOP_PERIOD_MS))
        {
            s_sensor_status_refresh_ms += SENSOR_STATUS_LOOP_PERIOD_MS;
        }
        else
        {
            s_sensor_status_refresh_ms = SENSOR_STATUS_REFRESH_MS;
        }

        if(s_sensor_status_refresh_ms >= SENSOR_STATUS_REFRESH_MS)
        {
            s_sensor_status_refresh_ms = 0U;
            refresh_now = 1U;
        }
    }

    if(!refresh_now)
    {
        return;
    }

    s_sensor_status_refresh_ms = 0U;
    ips200_set_font(IPS200_6X8_FONT);
    if(s_sensor_status_view == 0U)
    {
        sensor_status_draw_tof_flow();
    }
    else
    {
        sensor_status_draw_imu_power();
    }
}

static void vofa_status_page(void)
{
    VofaDiag_t diag = {0};
    char command_text[4] = {'-', '-', ' ', '\0'};
    const char *mux_text = "IMAGE";

    Send_test();
    USART_PID_Adjust();
    Vofa_GetDiag(&diag);

    if(diag.send_back_command != 0U)
    {
        command_text[0] = (char)diag.send_back_command;
        command_text[1] = ' ';
    }

    if(diag.comm_mode == COMM_MUX_MODE_VOFA)
    {
        mux_text = "VOFA ";
    }
    else if(diag.comm_mode == COMM_MUX_MODE_CAMERA_BOARD)
    {
        mux_text = "BOARD";
    }

    test_page_show_text(TEST_ROW_Y(0U), "MUX:", mux_text,
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

static void rear_board_show_pair_i32(uint8 row,
                                     const char *left_label,
                                     int32 left_value,
                                     const char *right_label,
                                     int32 right_value,
                                     uint8 digits)
{
    uint16 y = TEST_ROW_Y(row);

    test_page_clear_line(y);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(TEST_PAGE_X, y, (char *)left_label);
    ips200_show_int(72U, y, left_value, digits);
    ips200_show_string(132U, y, (char *)right_label);
    ips200_show_int(180U, y, right_value, digits);
}

static void rear_board_draw_link(void)
{
    CameraBoardLinkDiag_t diag = {0};

    CameraBoardLink_GetDiag(&diag);
    test_page_show_text(TEST_ROW_Y(0U), "MODE:", diag.mode_active ? "BOARD" : "OTHER",
                        diag.mode_active ? RGB565_GREEN : RGB565_YELLOW);
    test_page_show_text(TEST_ROW_Y(1U), "ONLINE:", diag.online ? "YES  " : "NO   ",
                        diag.online ? RGB565_GREEN : RGB565_RED);
    test_page_show_text(TEST_ROW_Y(2U), "CAMERA:", diag.rear_camera_ready ? "READY" : "WAIT ",
                        diag.rear_camera_ready ? RGB565_GREEN : RGB565_RED);
    test_page_show_text(TEST_ROW_Y(3U), "CM7_1:", diag.rear_camera_core_alive ? "RUN  " : "WAIT ",
                        diag.rear_camera_core_alive ? RGB565_GREEN : RGB565_RED);
    test_page_show_text(TEST_ROW_Y(4U), "SYNC:", diag.rear_time_sync_valid ? "VALID" : "BAD  ",
                        diag.rear_time_sync_valid ? RGB565_GREEN : RGB565_RED);
    rear_board_show_pair_i32(5U, "RES:", diag.last_result_age_ms,
                             "STAT:", diag.last_status_age_ms, 5U);
    rear_board_show_pair_i32(6U, "STATE:", diag.rear_state_age_ms,
                             "SYNC:", diag.rear_sync_error_us, 5U);
    test_page_show_uint(TEST_ROW_Y(7U), "TX FRAME:", diag.tx_frame_count, 8U);
    test_page_show_uint(TEST_ROW_Y(8U), "RX FRAME:", diag.rx_frame_count, 8U);
    test_page_show_uint(TEST_ROW_Y(9U), "LOCAL CRC:", diag.crc_error_count, 8U);
    test_page_show_uint(TEST_ROW_Y(10U), "REAR CRC:", diag.rear_crc_error_count, 5U);
    test_page_show_text(TEST_ROW_Y(11U), "KEY:", "1< 2> 3B 4X", RGB565_CYAN);
}

static void rear_board_draw_result(void)
{
    CameraBoardVisionFrame_t frame = {0};
    CameraBoardBeaconObservation_t observation = {0};
    uint8 received = CameraBoardLink_GetLatestRearFrame(&frame);
    uint8 beacon_valid = (uint8)(received &&
        (s_rear_board_rx_beacon_index < frame.beacon_count));

    if(beacon_valid)
    {
        observation = frame.beacon[s_rear_board_rx_beacon_index];
    }

    test_page_show_text(TEST_ROW_Y(0U), "KEY:", "1< 2> 3B 4X", RGB565_CYAN);
    test_page_show_uint(TEST_ROW_Y(1U), "FRAME:", frame.camera_frame_seq, 8U);
    test_page_show_uint(TEST_ROW_Y(2U), "CAPTURE US:", frame.capture_time_us, 10U);
    rear_board_show_pair_i32(3U, "SEQ:", frame.state_seq_used,
                             "RESET:", frame.reset_generation, 5U);
    rear_board_show_pair_i32(4U, "AGE:", frame.state_age_us,
                             "PROC:", frame.recognition_time_us, 5U);
    test_page_show_uint(TEST_ROW_Y(5U), "FLAGS:", frame.result_flags, 3U);
    rear_board_show_pair_i32(6U, "CAR X:", frame.car_x_mm,
                             "Y:", frame.car_y_mm, 6U);
    rear_board_show_pair_i32(7U, "RAW X:", frame.car_raw_x_mm,
                             "Y:", frame.car_raw_y_mm, 6U);
    rear_board_show_pair_i32(8U, "COUNT:", frame.beacon_count,
                             "SEL:", (int32)s_rear_board_rx_beacon_index + 1, 2U);
    rear_board_show_pair_i32(9U, "B X:", observation.x_mm,
                             "Y:", observation.y_mm, 6U);
    rear_board_show_pair_i32(10U, "CONF:", observation.detection_confidence,
                             "SIG:", observation.position_sigma_cm, 3U);
    if(received)
    {
        test_page_show_uint(TEST_ROW_Y(11U), "B FLAGS:", observation.flags, 3U);
    }
    else
    {
        test_page_show_text(TEST_ROW_Y(11U), "RESULT:", "WAIT ", RGB565_RED);
    }
}

static void rear_board_draw_diag(void)
{
    CameraBoardLinkDiag_t diag = {0};

    CameraBoardLink_GetDiag(&diag);
    test_page_show_text(TEST_ROW_Y(0U), "KEY:", "1< 2> 3B 4X", RGB565_CYAN);
    test_page_show_uint(TEST_ROW_Y(1U), "LOCAL PROTO:", diag.protocol_error_count, 8U);
    test_page_show_uint(TEST_ROW_Y(2U), "LOCAL GAP:", diag.sequence_gap_count, 8U);
    test_page_show_uint(TEST_ROW_Y(3U), "LOCAL FIFO:", diag.rx_overflow_count, 8U);
    test_page_show_uint(TEST_ROW_Y(4U), "LOCAL TXDROP:", diag.tx_busy_drop_count, 8U);
    test_page_show_uint(TEST_ROW_Y(5U), "REAR GAP:", diag.rear_sequence_gap_count, 5U);
    test_page_show_uint(TEST_ROW_Y(6U), "REAR FIFO:", diag.rear_rx_overflow_count, 5U);
    test_page_show_uint(TEST_ROW_Y(7U), "REAR TXDROP:", diag.rear_tx_busy_drop_count, 5U);
    test_page_show_uint(TEST_ROW_Y(8U), "CAPTURE:", diag.rear_captured_frame_count, 8U);
    test_page_show_uint(TEST_ROW_Y(9U), "PROCESS:", diag.rear_processed_frame_count, 8U);
    test_page_show_uint(TEST_ROW_Y(10U), "DROP:", diag.rear_dropped_frame_count, 8U);
    test_page_show_uint(TEST_ROW_Y(11U), "IPC RX:", diag.rear_ipc_result_count, 8U);
}

static void rear_board_rx_page(void)
{
    uint8 refresh_now = 0U;

    if(!s_rear_board_rx_page_active)
    {
        s_rear_board_rx_page_active = 1U;
        s_rear_board_rx_view = 0U;
        s_rear_board_rx_beacon_index = 0U;
        s_rear_board_rx_refresh_ticks = 0U;
        key_flag = 0U;
        refresh_now = 1U;
    }

    if(key_flag == Button_4 || g_menu_exit_flag || menu_is_monitor_view())
    {
        s_rear_board_rx_page_active = 0U;
        s_rear_board_rx_refresh_ticks = 0U;
        return;
    }
    if(key_flag == Button_1)
    {
        s_rear_board_rx_view = (s_rear_board_rx_view == 0U) ?
                               2U : (uint8)(s_rear_board_rx_view - 1U);
        key_flag = 0U;
        refresh_now = 1U;
    }
    else if(key_flag == Button_2)
    {
        s_rear_board_rx_view = (uint8)((s_rear_board_rx_view + 1U) % 3U);
        key_flag = 0U;
        refresh_now = 1U;
    }
    else if(key_flag == Button_3)
    {
        s_rear_board_rx_beacon_index = (uint8)(
            (s_rear_board_rx_beacon_index + 1U) % CAMERA_BOARD_MAX_BEACON_COUNT);
        s_rear_board_rx_view = 1U;
        key_flag = 0U;
        refresh_now = 1U;
    }

    s_rear_board_rx_refresh_ticks++;
    if(s_rear_board_rx_refresh_ticks >= REAR_BOARD_RX_REFRESH_TICKS)
    {
        s_rear_board_rx_refresh_ticks = 0U;
        refresh_now = 1U;
    }
    if(!refresh_now)
    {
        return;
    }

    if(s_rear_board_rx_view == 0U)
    {
        rear_board_draw_link();
    }
    else if(s_rear_board_rx_view == 1U)
    {
        rear_board_draw_result();
    }
    else
    {
        rear_board_draw_diag();
    }
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
        HeightTofSnapshot_t height_snapshot = {0};
        uint8 ir_track_valid;

        s_track_light_status_tick_div = 0U;
        VehicleMarkerLocalization_ServiceLostProtection();
        BatteryMonitor_GetStatus(&battery_status);
        ICM_GetHealth(&imu_health);
        HeightManager_GetTofSnapshot(&height_snapshot);
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
        snapshot.tof_height_cm = height_snapshot.filtered_height_cm;
        snapshot.fused_height_cm = height_snapshot.filtered_height_cm;
        snapshot.fused_velocity_cms = height_snapshot.fused_velocity_cms;
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
    menu_create_item(&testMenu, "Sensor Status", sensor_status_page, NULL);
    menu_create_item(&testMenu, "Vision View", vision_view_page, NULL);
    menu_create_item(&testMenu, "Track Light", track_light_test_page, NULL);
    menu_create_item(&testMenu, "VOFA Status", vofa_status_page, NULL);
    menu_create_item(&testMenu, "Rear Board RX", rear_board_rx_page, NULL);
    menu_create_item(&testMenu, "MT9V03X Adjust", mt9v03x_adjust_page, NULL);
    menu_create_item(&testMenu, "Binary Adjust", binary_threshold_adjust_page, NULL);
    
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
#if CY_CORE_CM7_0
    HeightTofSnapshot_t height_snapshot = {0};
#endif

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
        CameraBoardLink_MainTask();
        // 处理菜单逻辑
        g_currentMenu = menu_process(g_currentMenu);
#if CY_CORE_CM7_0
        HeightManager_GetTofSnapshot(&height_snapshot);
        (void)CrossCoreCamera_ControlCoreService(current_roll,
                                                 current_pitch,
                                                 current_yaw,
                                                 height_snapshot.filtered_height_cm,
                                                 height_snapshot.filtered_height_cm);
#endif
        Send_test();
        USART_PID_Adjust();
        if (g_currentMenu == NULL)
        {
            // ?????????
            break;
        }
        
        // ????????????????
        system_delay_ms(5U);
        CameraBoardLink_MainTask();
        system_delay_ms(5U);
    }
    
    if (!menu_is_monitor_view())
    {
        // 退出菜单后清屏，避免残留旧页面内容
        ips200_clear();
    }
}
