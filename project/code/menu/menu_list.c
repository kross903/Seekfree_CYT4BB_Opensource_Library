#include "common/project_headfile.h"
#include "motor_hardware/small_driver_uart_control.h"

#include <math.h>
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
#define VISION_FUSION_REFRESH_US          50000U
#define VISION_FUSION_ROW_COUNT           12U
#define VISION_FUSION_VIEW_COUNT          3U
#define VISION_FUSION_TARGETS_PER_SOURCE  3U
#define VISION_FUSION_DIVIDER_Y           ((uint16)(TEST_ROW_Y(8U) + 17U))
#define REAR_BOARD_RX_ROW_COUNT           12U
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
static uint32 s_track_light_fusion_last_refresh_us = 0U;
static uint8 s_track_light_row_cursor = 0U;
static uint8 s_track_light_snapshot_valid = 0U;
static DualCameraFusionSnapshot_t s_track_light_snapshot;
static DualCameraFusionDiag_t s_track_light_fusion_diag;
static CameraBoardLinkDiag_t s_track_light_link_diag;
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
static uint8 s_rear_board_rx_row_cursor = 0U;
static uint8 s_rear_board_rx_frame_received = 0U;
static CameraBoardLinkDiag_t s_rear_board_rx_diag_snapshot;
static CameraBoardVisionFrame_t s_rear_board_rx_frame_snapshot;
static uint8 s_vision_fusion_page_active = 0U;
static uint8 s_vision_fusion_view = 0U;
static uint32 s_vision_fusion_last_refresh_us = 0U;
static uint8 s_vision_fusion_row_cursor = 0U;
static DualCameraFusionSnapshot_t s_vision_fusion_snapshot;
static DualCameraFusionDiag_t s_vision_fusion_diag;

static void test_page_clear_line(uint16 y);
static void test_page_show_text(uint16 y, const char *label,
                                const char *value, uint16 color);
static void test_page_show_int(uint16 y, const char *label,
                               int32 value, uint8 digits);

static const char *vision_fusion_source_text(uint8 source)
{
    switch(source)
    {
        case DUAL_CAMERA_FUSION_SOURCE_FRONT:
        case DUAL_CAMERA_FUSION_SOURCE_BOTH_FRONT:
            return "F ";
        case DUAL_CAMERA_FUSION_SOURCE_REAR:
        case DUAL_CAMERA_FUSION_SOURCE_BOTH_REAR:
            return "R ";
        default: return "--";
    }
}

static uint8 vision_fusion_source_is_front(uint8 source)
{
    return (uint8)((source == DUAL_CAMERA_FUSION_SOURCE_FRONT) ||
                   (source == DUAL_CAMERA_FUSION_SOURCE_BOTH_FRONT));
}

static uint8 vision_fusion_target_index(uint8 display_index)
{
    uint8 want_front;
    uint8 source_index;
    uint8 target_index;
    uint8 matched_index = 0U;

    if(display_index < VISION_FUSION_TARGETS_PER_SOURCE)
    {
        want_front = 1U;
        source_index = display_index;
    }
    else
    {
        want_front = 0U;
        source_index = (uint8)(display_index -
                               VISION_FUSION_TARGETS_PER_SOURCE);
    }

    for(target_index = 0U;
        target_index < s_vision_fusion_snapshot.target_count;
        target_index++)
    {
        const DualCameraFusionTarget_t *target =
            &s_vision_fusion_snapshot.target[target_index];

        if(!target->valid ||
           (vision_fusion_source_is_front(target->source) != want_front))
        {
            continue;
        }
        if(matched_index == source_index)
        {
            return target_index;
        }
        matched_index++;
    }
    return 0xFFU;
}

static void vision_fusion_draw_title(void)
{
    ips200_clear();
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    if(s_vision_fusion_view == 0U)
    {
        ips200_show_string(TEST_PAGE_X, 20U,
                           "FUS P1 BCN-UAV K1< K2> K4");
    }
    else if(s_vision_fusion_view == 1U)
    {
        ips200_show_string(TEST_PAGE_X, 20U,
                           "FUS P2 BCN-CAR K1< K2> K4");
    }
    else
    {
        ips200_show_string(TEST_PAGE_X, 20U,
                           "FUS P3 CAR-UAV K1< K2> K4");
    }
}

static void vision_fusion_capture_snapshot(uint32 now_us)
{
    s_vision_fusion_last_refresh_us = now_us;
    DualCameraFusion_BuildSnapshot(now_us);
    DualCameraFusion_GetSnapshot(&s_vision_fusion_snapshot);
    DualCameraFusion_GetDiag(&s_vision_fusion_diag);
}

static void vision_fusion_draw_row(uint8 row)
{
    uint16 y = TEST_ROW_Y(row);
    uint16 extrinsic_color;
    uint8 target_index;

    test_page_clear_line(y);
    if(s_vision_fusion_view == 2U)
    {
        int32 front_x = (int32)s_vision_fusion_snapshot.front_car_x_cm;
        int32 front_y = (int32)s_vision_fusion_snapshot.front_car_y_cm;
        int32 rear_x = (int32)s_vision_fusion_snapshot.rear_car_x_cm;
        int32 rear_y = (int32)s_vision_fusion_snapshot.rear_car_y_cm;

        ips200_set_font(IPS200_8X16_FONT);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        if(row == 0U)
        {
            ips200_show_string(20U, y, "SRC");
            ips200_show_string(52U, y, "VALID");
            ips200_show_string(108U, y, "X");
            ips200_show_string(164U, y, "Y");
        }
        else if((row == 1U) || (row == 2U))
        {
            uint8 front = (row == 1U) ? 1U : 0U;
            uint8 valid = front ? s_vision_fusion_snapshot.front_car_valid :
                                  s_vision_fusion_snapshot.rear_car_valid;

            ips200_set_color(front ? RGB565_CYAN : RGB565_YELLOW,
                             RGB565_BLACK);
            ips200_show_string(20U, y, front ? "F" : "R");
            ips200_set_color(valid ? RGB565_GREEN : RGB565_RED,
                             RGB565_BLACK);
            ips200_show_string(52U, y, valid ? "OK" : "--");
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);
            if(valid)
            {
                ips200_show_int(92U, y, front ? front_x : rear_x, 5U);
                ips200_show_int(148U, y, front ? front_y : rear_y, 5U);
            }
            else
            {
                ips200_show_string(108U, y, "--");
                ips200_show_string(164U, y, "--");
            }
        }
        else if(row == 3U)
        {
            ips200_draw_line(4U, (uint16)(y - 3U), 235U,
                             (uint16)(y - 3U), RGB565_CYAN);
            ips200_show_string(20U, y, "REF");
            if(s_vision_fusion_snapshot.reference_source ==
               DUAL_CAMERA_FUSION_SOURCE_FRONT)
            {
                ips200_set_color(RGB565_CYAN, RGB565_BLACK);
                ips200_show_string(52U, y, "F");
            }
            else if(s_vision_fusion_snapshot.reference_source ==
                    DUAL_CAMERA_FUSION_SOURCE_REAR)
            {
                ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
                ips200_show_string(52U, y, "R");
            }
            else
            {
                ips200_set_color(RGB565_RED, RGB565_BLACK);
                ips200_show_string(52U, y, "--");
            }
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);
            if(s_vision_fusion_snapshot.car_valid)
            {
                ips200_show_int(92U, y,
                    (int32)s_vision_fusion_snapshot.car_x_cm, 5U);
                ips200_show_int(148U, y,
                    (int32)s_vision_fusion_snapshot.car_y_cm, 5U);
            }
            else
            {
                ips200_show_string(108U, y, "--");
                ips200_show_string(164U, y, "--");
            }
        }
        else if(row == 4U)
        {
            ips200_show_string(20U, y, "DIFF");
            if(s_vision_fusion_snapshot.front_car_valid &&
               s_vision_fusion_snapshot.rear_car_valid)
            {
                ips200_show_int(92U, y, rear_x - front_x, 5U);
                ips200_show_int(148U, y, rear_y - front_y, 5U);
            }
            else
            {
                ips200_show_string(108U, y, "--");
                ips200_show_string(164U, y, "--");
            }
        }
        else if(row == 5U)
        {
            ips200_show_string(20U, y, "DIST");
            if(s_vision_fusion_snapshot.car_valid)
            {
                float car_x = s_vision_fusion_snapshot.car_x_cm;
                float car_y = s_vision_fusion_snapshot.car_y_cm;
                ips200_show_int(92U, y,
                    (int32)sqrtf(car_x * car_x + car_y * car_y), 5U);
                ips200_show_string(148U, y, "cm");
            }
            else
            {
                ips200_show_string(108U, y, "--");
            }
        }
        else if(row == 6U)
        {
            ips200_show_string(20U, y, "AGE F");
            if(s_vision_fusion_diag.front_age_ms == 0xFFFFU)
            {
                ips200_show_string(68U, y, "---");
            }
            else
            {
                ips200_show_uint(68U, y,
                    (s_vision_fusion_diag.front_age_ms > 999U) ? 999U :
                        s_vision_fusion_diag.front_age_ms, 3U);
            }
            ips200_show_string(116U, y, "R");
            if(s_vision_fusion_diag.rear_age_ms == 0xFFFFU)
            {
                ips200_show_string(132U, y, "---");
            }
            else
            {
                ips200_show_uint(132U, y,
                    (s_vision_fusion_diag.rear_age_ms > 999U) ? 999U :
                        s_vision_fusion_diag.rear_age_ms, 3U);
            }
            ips200_show_string(172U, y, "ms");
        }
        else if(row == 7U)
        {
            ips200_show_string(20U, y, "PAIR");
            ips200_set_color(s_vision_fusion_diag.pair_valid ? RGB565_GREEN :
                             RGB565_YELLOW, RGB565_BLACK);
            ips200_show_string(60U, y,
                s_vision_fusion_diag.pair_valid ? "OK" : "--");
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);
            ips200_show_string(92U, y, "DT");
            ips200_show_int(116U, y, s_vision_fusion_diag.pair_dt_ms, 4U);
            ips200_show_string(164U, y, "ms");
        }
        else if(row == 8U)
        {
            ips200_set_color(RGB565_CYAN, RGB565_BLACK);
            ips200_show_string(20U, y, "AXIS UAV X:FWD Y:LEFT");
        }
        return;
    }

    ips200_draw_line(4U, VISION_FUSION_DIVIDER_Y,
                     235U, VISION_FUSION_DIVIDER_Y, RGB565_CYAN);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    if(row == 0U)
    {
        ips200_show_string(20U, y, "M");
#if (VISION_FUSION_OUTPUT_MODE == VISION_FUSION_OUTPUT_LIVE)
        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(44U, y, "LIVE");
        extrinsic_color = s_vision_fusion_diag.extrinsic_nominal ? RGB565_RED :
            (s_vision_fusion_diag.extrinsic_valid ? RGB565_GREEN : RGB565_RED);
#else
        ips200_set_color(RGB565_CYAN, RGB565_BLACK);
        ips200_show_string(44U, y, "SHD");
        extrinsic_color = s_vision_fusion_diag.extrinsic_nominal ? RGB565_YELLOW :
            (s_vision_fusion_diag.extrinsic_valid ? RGB565_GREEN : RGB565_RED);
#endif
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(100U, y, "E");
        ips200_set_color(extrinsic_color, RGB565_BLACK);
        ips200_show_string(124U, y,
            s_vision_fusion_diag.extrinsic_nominal ? "NOM" :
            (s_vision_fusion_diag.extrinsic_valid ? "VAL" : "INV"));
    }
    else if((row == 1U) || (row == 2U))
    {
        uint8 front = (row == 1U) ? 1U : 0U;
        uint8 healthy = front ? s_vision_fusion_diag.front_healthy :
                                s_vision_fusion_diag.rear_healthy;
        uint8 car_valid = front ? s_vision_fusion_diag.front_car_valid :
                                  s_vision_fusion_diag.rear_car_valid;
        uint8 beacon_count = front ? s_vision_fusion_diag.front_beacon_count :
                                     s_vision_fusion_diag.rear_beacon_count;
        uint16 age_ms = front ? s_vision_fusion_diag.front_age_ms :
                                s_vision_fusion_diag.rear_age_ms;

        ips200_show_string(20U, y, front ? "F" : "R");
        ips200_set_color(healthy ? RGB565_GREEN : RGB565_RED, RGB565_BLACK);
        ips200_show_string(44U, y, healthy ? "OK" : "--");
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(76U, y, "C");
        ips200_show_int(92U, y, car_valid, 1U);
        ips200_show_string(116U, y, "N");
        ips200_show_int(132U, y, beacon_count, 1U);
        ips200_show_string(156U, y, "A");
        ips200_show_uint(172U, y, (age_ms > 999U) ? 999U : age_ms, 3U);
    }
    else if(row == 3U)
    {
        ips200_show_string(20U, y, "P");
        ips200_set_color(s_vision_fusion_diag.pair_valid ? RGB565_GREEN :
                         RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(44U, y,
                           s_vision_fusion_diag.pair_valid ? "OK" : "--");
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(76U, y, "DT");
        ips200_show_int(100U, y, s_vision_fusion_diag.pair_dt_ms, 4U);
        ips200_show_string(148U, y, "Y");
        ips200_show_float(164U, y,
                          s_vision_fusion_diag.pair_yaw_delta_deg, 4U, 1U);
    }
    else if(row == 4U)
    {
        ips200_show_string(20U, y, "OUT");
        ips200_show_int(52U, y, s_vision_fusion_diag.output_count, 2U);
        ips200_show_string(100U, y, "DUP");
        ips200_show_int(132U, y, s_vision_fusion_diag.duplicate_count, 2U);
    }
    else if(row == 5U)
    {
        ips200_show_string(20U, y, "ID");
        ips200_show_string(44U, y, "SRC");
        ips200_show_string(76U, y,
            (s_vision_fusion_view == 0U) ? "X" : "DX");
        ips200_show_string(124U, y,
            (s_vision_fusion_view == 0U) ? "Y" : "DY");
        ips200_show_string(176U, y, "S");
        ips200_show_string(208U, y, "C");
    }
    else
    {
        target_index = vision_fusion_target_index((uint8)(row - 6U));
        if((target_index == 0xFFU) ||
           !s_vision_fusion_snapshot.target[target_index].valid)
        {
            ips200_show_string(20U, y, "--");
            ips200_set_color(((row - 6U) <
                              VISION_FUSION_TARGETS_PER_SOURCE) ?
                                 RGB565_CYAN : RGB565_YELLOW,
                             RGB565_BLACK);
            ips200_show_string(44U, y,
                ((row - 6U) < VISION_FUSION_TARGETS_PER_SOURCE) ?
                    "F " : "R ");
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);
            ips200_show_string(76U, y, "--    --   --  ---");
            return;
        }
        ips200_show_uint(20U, y,
                         s_vision_fusion_snapshot.target[target_index].track_id,
                         2U);
        ips200_set_color(vision_fusion_source_is_front(
            s_vision_fusion_snapshot.target[target_index].source) ?
                RGB565_CYAN : RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(44U, y,
            vision_fusion_source_text(
                s_vision_fusion_snapshot.target[target_index].source));
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        if((s_vision_fusion_view != 0U) &&
           !s_vision_fusion_snapshot.car_valid)
        {
            ips200_show_string(76U, y, "--");
            ips200_show_string(124U, y, "--");
        }
        else
        {
            ips200_show_int(76U, y, (int32)((s_vision_fusion_view == 0U) ?
                s_vision_fusion_snapshot.target[target_index].x_cm :
                s_vision_fusion_snapshot.target[target_index].dx_cm), 5U);
            ips200_show_int(124U, y, (int32)((s_vision_fusion_view == 0U) ?
                s_vision_fusion_snapshot.target[target_index].y_cm :
                s_vision_fusion_snapshot.target[target_index].dy_cm), 5U);
        }
        ips200_show_uint(176U, y,
            s_vision_fusion_snapshot.target[target_index].position_sigma_cm, 2U);
        ips200_show_uint(208U, y,
            s_vision_fusion_snapshot.target[target_index].detection_confidence,
            3U);
    }
}

static void vision_fusion_draw_page(void)
{
    uint32 now_us = CameraBoardLink_TimeNowUs();

    DualCameraFusion_ServiceInputs();
    if((s_vision_fusion_last_refresh_us == 0U) ||
       ((uint32)(now_us - s_vision_fusion_last_refresh_us) >=
        VISION_FUSION_REFRESH_US))
    {
        vision_fusion_capture_snapshot(now_us);
    }
    vision_fusion_draw_row(s_vision_fusion_row_cursor);
    s_vision_fusion_row_cursor++;
    if(s_vision_fusion_row_cursor >= VISION_FUSION_ROW_COUNT)
    {
        s_vision_fusion_row_cursor = 0U;
    }
}

static void vision_fusion_page(void)
{
    if(!s_vision_fusion_page_active)
    {
        s_vision_fusion_page_active = 1U;
        s_vision_fusion_view = 0U;
        s_vision_fusion_last_refresh_us = 0U;
        s_vision_fusion_row_cursor = 0U;
        memset(&s_vision_fusion_snapshot, 0,
               sizeof(s_vision_fusion_snapshot));
        memset(&s_vision_fusion_diag, 0, sizeof(s_vision_fusion_diag));
        vision_fusion_draw_title();
        key_flag = 0U;
    }
    if(key_flag == Button_4 || g_menu_exit_flag || menu_is_monitor_view())
    {
        s_vision_fusion_page_active = 0U;
        s_vision_fusion_last_refresh_us = 0U;
        s_vision_fusion_row_cursor = 0U;
        return;
    }
    if(key_flag == Button_1)
    {
        s_vision_fusion_view = (s_vision_fusion_view == 0U) ?
            2U : (uint8)(s_vision_fusion_view - 1U);
        s_vision_fusion_row_cursor = 0U;
        vision_fusion_draw_title();
        key_flag = 0U;
    }
    else if(key_flag == Button_2)
    {
        s_vision_fusion_view = (uint8)((s_vision_fusion_view + 1U) %
                                       VISION_FUSION_VIEW_COUNT);
        s_vision_fusion_row_cursor = 0U;
        vision_fusion_draw_title();
        key_flag = 0U;
    }
    key_flag = 0U;
    vision_fusion_draw_page();
}

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
                           "T1 H", height_diag.tof1_comp_cm, 4U, 1U, tof1_color);
    sensor_page_show_float(SENSOR_RIGHT_LABEL_X, SENSOR_RIGHT_VALUE_X, y,
                           "T2 H", height_diag.tof2_comp_cm, 4U, 1U, tof2_color);

    y = TEST_ROW_Y(1U);
    sensor_page_clear_line(y);
    sensor_page_show_float(SENSOR_LEFT_LABEL_X, SENSOR_LEFT_VALUE_X, y,
                           "SEL H", height_diag.tof_selected_comp_cm, 4U, 1U,
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
    CoreLinkCarMotionDiag_t car_diag = {0};
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
    core_link_user_get_car_motion_diag(&car_diag);
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
                          "CAR LINK", sensor_page_yes_no(
                              car_diag.link_state !=
                              CORE_LINK_CAR_MOTION_STATE_INVALID),
                          sensor_page_bool_color(
                              car_diag.link_state !=
                              CORE_LINK_CAR_MOTION_STATE_INVALID));
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

static void rear_board_capture_snapshot(void)
{
    CameraBoardLink_GetDiag(&s_rear_board_rx_diag_snapshot);
    memset(&s_rear_board_rx_frame_snapshot, 0,
           sizeof(s_rear_board_rx_frame_snapshot));
    s_rear_board_rx_frame_received = CameraBoardLink_GetLatestRearFrame(
        &s_rear_board_rx_frame_snapshot);
}

static void rear_board_draw_link_row(uint8 row)
{
    const CameraBoardLinkDiag_t *diag = &s_rear_board_rx_diag_snapshot;

    switch(row)
    {
        case 0U:
            test_page_show_text(TEST_ROW_Y(row), "MODE:",
                diag->mode_active ? "BOARD" : "OTHER",
                diag->mode_active ? RGB565_GREEN : RGB565_YELLOW);
            break;
        case 1U:
            test_page_show_text(TEST_ROW_Y(row), "ONLINE:",
                diag->online ? "YES  " : "NO   ",
                diag->online ? RGB565_GREEN : RGB565_RED);
            break;
        case 2U:
            test_page_show_text(TEST_ROW_Y(row), "CAMERA:",
                diag->rear_camera_ready ? "READY" : "WAIT ",
                diag->rear_camera_ready ? RGB565_GREEN : RGB565_RED);
            break;
        case 3U:
            test_page_show_text(TEST_ROW_Y(row), "CM7_1:",
                diag->rear_camera_core_alive ? "RUN  " : "WAIT ",
                diag->rear_camera_core_alive ? RGB565_GREEN : RGB565_RED);
            break;
        case 4U:
            test_page_show_text(TEST_ROW_Y(row), "SYNC:",
                diag->rear_time_sync_valid ? "VALID" : "BAD  ",
                diag->rear_time_sync_valid ? RGB565_GREEN : RGB565_RED);
            break;
        case 5U:
            rear_board_show_pair_i32(row, "RES:", diag->last_result_age_ms,
                                     "STAT:", diag->last_status_age_ms, 5U);
            break;
        case 6U:
            rear_board_show_pair_i32(row, "STATE:", diag->rear_state_age_ms,
                                     "SYNC:", diag->rear_sync_error_us, 5U);
            break;
        case 7U:
            test_page_show_uint(TEST_ROW_Y(row), "TX FRAME:",
                                diag->tx_frame_count, 8U);
            break;
        case 8U:
            test_page_show_uint(TEST_ROW_Y(row), "RX FRAME:",
                                diag->rx_frame_count, 8U);
            break;
        case 9U:
            test_page_show_uint(TEST_ROW_Y(row), "LOCAL CRC:",
                                diag->crc_error_count, 8U);
            break;
        case 10U:
            test_page_show_uint(TEST_ROW_Y(row), "REAR CRC:",
                                diag->rear_crc_error_count, 5U);
            break;
        default:
            test_page_show_text(TEST_ROW_Y(row), "KEY:",
                                "1< 2> 3B 4X", RGB565_CYAN);
            break;
    }
}

static void rear_board_draw_result_row(uint8 row)
{
    const CameraBoardVisionFrame_t *frame = &s_rear_board_rx_frame_snapshot;
    CameraBoardBeaconObservation_t observation = {0};
    uint8 beacon_valid = (uint8)(s_rear_board_rx_frame_received &&
        (s_rear_board_rx_beacon_index < frame->beacon_count));

    if(beacon_valid)
    {
        observation = frame->beacon[s_rear_board_rx_beacon_index];
    }

    switch(row)
    {
        case 0U:
            test_page_show_text(TEST_ROW_Y(row), "KEY:",
                                "1< 2> 3B 4X", RGB565_CYAN);
            break;
        case 1U:
            test_page_show_uint(TEST_ROW_Y(row), "FRAME:",
                                frame->camera_frame_seq, 8U);
            break;
        case 2U:
            test_page_show_uint(TEST_ROW_Y(row), "CAPTURE US:",
                                frame->capture_time_us, 10U);
            break;
        case 3U:
            rear_board_show_pair_i32(row, "SEQ:", frame->state_seq_used,
                                     "RESET:", frame->reset_generation, 5U);
            break;
        case 4U:
            rear_board_show_pair_i32(row, "AGE:", frame->state_age_us,
                                     "PROC:", frame->recognition_time_us, 5U);
            break;
        case 5U:
            test_page_show_uint(TEST_ROW_Y(row), "FLAGS:",
                                frame->result_flags, 3U);
            break;
        case 6U:
            rear_board_show_pair_i32(row, "CAR X:", frame->car_x_mm,
                                     "Y:", frame->car_y_mm, 6U);
            break;
        case 7U:
            rear_board_show_pair_i32(row, "RAW X:", frame->car_raw_x_mm,
                                     "Y:", frame->car_raw_y_mm, 6U);
            break;
        case 8U:
            rear_board_show_pair_i32(row, "COUNT:", frame->beacon_count,
                "SEL:", (int32)s_rear_board_rx_beacon_index + 1, 2U);
            break;
        case 9U:
            rear_board_show_pair_i32(row, "B X:", observation.x_mm,
                                     "Y:", observation.y_mm, 6U);
            break;
        case 10U:
            rear_board_show_pair_i32(row, "CONF:",
                observation.detection_confidence,
                "SIG:", observation.position_sigma_cm, 3U);
            break;
        default:
            if(s_rear_board_rx_frame_received)
            {
                test_page_show_uint(TEST_ROW_Y(row), "B FLAGS:",
                                    observation.flags, 3U);
            }
            else
            {
                test_page_show_text(TEST_ROW_Y(row), "RESULT:",
                                    "WAIT ", RGB565_RED);
            }
            break;
    }
}

static void rear_board_draw_diag_row(uint8 row)
{
    const CameraBoardLinkDiag_t *diag = &s_rear_board_rx_diag_snapshot;

    switch(row)
    {
        case 0U:
            test_page_show_text(TEST_ROW_Y(row), "KEY:",
                                "1< 2> 3B 4X", RGB565_CYAN);
            break;
        case 1U:
            test_page_show_uint(TEST_ROW_Y(row), "LOCAL PROTO:",
                                diag->protocol_error_count, 8U);
            break;
        case 2U:
            test_page_show_uint(TEST_ROW_Y(row), "LOCAL GAP:",
                                diag->sequence_gap_count, 8U);
            break;
        case 3U:
            test_page_show_uint(TEST_ROW_Y(row), "LOCAL FIFO:",
                                diag->rx_overflow_count, 8U);
            break;
        case 4U:
            test_page_show_uint(TEST_ROW_Y(row), "LOCAL TXDROP:",
                                diag->tx_busy_drop_count, 8U);
            break;
        case 5U:
            test_page_show_uint(TEST_ROW_Y(row), "REAR GAP:",
                                diag->rear_sequence_gap_count, 5U);
            break;
        case 6U:
            test_page_show_uint(TEST_ROW_Y(row), "REAR FIFO:",
                                diag->rear_rx_overflow_count, 5U);
            break;
        case 7U:
            test_page_show_uint(TEST_ROW_Y(row), "REAR TXDROP:",
                                diag->rear_tx_busy_drop_count, 5U);
            break;
        case 8U:
            test_page_show_uint(TEST_ROW_Y(row), "CAPTURE:",
                                diag->rear_captured_frame_count, 8U);
            break;
        case 9U:
            test_page_show_uint(TEST_ROW_Y(row), "PROCESS:",
                                diag->rear_processed_frame_count, 8U);
            break;
        case 10U:
            test_page_show_uint(TEST_ROW_Y(row), "DROP:",
                                diag->rear_dropped_frame_count, 8U);
            break;
        default:
            test_page_show_uint(TEST_ROW_Y(row), "IPC RX:",
                                diag->rear_ipc_result_count, 8U);
            break;
    }
}

static void rear_board_rx_page(void)
{
    if(!s_rear_board_rx_page_active)
    {
        s_rear_board_rx_page_active = 1U;
        s_rear_board_rx_view = 0U;
        s_rear_board_rx_beacon_index = 0U;
        s_rear_board_rx_row_cursor = 0U;
        memset(&s_rear_board_rx_diag_snapshot, 0,
               sizeof(s_rear_board_rx_diag_snapshot));
        memset(&s_rear_board_rx_frame_snapshot, 0,
               sizeof(s_rear_board_rx_frame_snapshot));
        s_rear_board_rx_frame_received = 0U;
        key_flag = 0U;
    }

    if(key_flag == Button_4 || g_menu_exit_flag || menu_is_monitor_view())
    {
        s_rear_board_rx_page_active = 0U;
        s_rear_board_rx_row_cursor = 0U;
        return;
    }
    if(key_flag == Button_1)
    {
        s_rear_board_rx_view = (s_rear_board_rx_view == 0U) ?
                               2U : (uint8)(s_rear_board_rx_view - 1U);
        key_flag = 0U;
        s_rear_board_rx_row_cursor = 0U;
    }
    else if(key_flag == Button_2)
    {
        s_rear_board_rx_view = (uint8)((s_rear_board_rx_view + 1U) % 3U);
        key_flag = 0U;
        s_rear_board_rx_row_cursor = 0U;
    }
    else if(key_flag == Button_3)
    {
        s_rear_board_rx_beacon_index = (uint8)(
            (s_rear_board_rx_beacon_index + 1U) % CAMERA_BOARD_MAX_BEACON_COUNT);
        s_rear_board_rx_view = 1U;
        key_flag = 0U;
        s_rear_board_rx_row_cursor = 0U;
    }

    if(s_rear_board_rx_row_cursor == 0U)
    {
        rear_board_capture_snapshot();
    }

    if(s_rear_board_rx_view == 0U)
    {
        rear_board_draw_link_row(s_rear_board_rx_row_cursor);
    }
    else if(s_rear_board_rx_view == 1U)
    {
        rear_board_draw_result_row(s_rear_board_rx_row_cursor);
    }
    else
    {
        rear_board_draw_diag_row(s_rear_board_rx_row_cursor);
    }
    s_rear_board_rx_row_cursor++;
    if(s_rear_board_rx_row_cursor >= REAR_BOARD_RX_ROW_COUNT)
    {
        s_rear_board_rx_row_cursor = 0U;
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

static uint8 track_light_service_vision_fusion(void)
{
    uint32 now_us = CameraBoardLink_TimeNowUs();

    /* 与 Vision Fusion 页面一致：先接收输入，再按固定周期构建快照。 */
    DualCameraFusion_ServiceInputs();
    if((s_track_light_fusion_last_refresh_us != 0U) &&
       ((uint32)(now_us - s_track_light_fusion_last_refresh_us) <
        VISION_FUSION_REFRESH_US))
    {
        return 0U;
    }

    DualCameraFusion_BuildSnapshot(now_us);
    s_track_light_snapshot_valid =
        DualCameraFusion_GetSnapshot(&s_track_light_snapshot);
    DualCameraFusion_GetDiag(&s_track_light_fusion_diag);
    CameraBoardLink_GetDiag(&s_track_light_link_diag);
    s_track_light_fusion_last_refresh_us = now_us;
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
    s_track_light_fusion_last_refresh_us = 0U;
    s_track_light_row_cursor = 0U;
    s_track_light_snapshot_valid = 0U;
    memset(&s_track_light_snapshot, 0, sizeof(s_track_light_snapshot));
    memset(&s_track_light_fusion_diag, 0,
           sizeof(s_track_light_fusion_diag));
    memset(&s_track_light_link_diag, 0, sizeof(s_track_light_link_diag));
}

static void track_light_restore_local_screen(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_clear();
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(TEST_PAGE_X, 20U, "Track Vision K3 IMG K4 BACK");
}

static const char *track_light_fusion_source_text(uint8 source)
{
    switch(source)
    {
        case DUAL_CAMERA_FUSION_SOURCE_FRONT: return "F ";
        case DUAL_CAMERA_FUSION_SOURCE_REAR: return "R ";
        case DUAL_CAMERA_FUSION_SOURCE_BOTH_FRONT:
        case DUAL_CAMERA_FUSION_SOURCE_BOTH_REAR: return "B ";
        default: return "--";
    }
}

static void track_light_draw_age(uint16 x, uint16 y, uint16 age_ms)
{
    if(age_ms == 0xFFFFU)
    {
        ips200_show_string(x, y, "---");
    }
    else
    {
        ips200_show_uint(x, y,
                         (age_ms > 999U) ? 999U : age_ms, 3U);
    }
}

static const char *track_light_rear_use_text(void)
{
    if(s_track_light_fusion_diag.rear_healthy)
    {
        return "READY";
    }
    if(s_track_light_link_diag.last_result_age_ms == 0xFFFFU)
    {
        return "NO RX";
    }
    if((s_track_light_link_diag.last_result_age_ms > 300U) ||
       (s_track_light_fusion_diag.rear_age_ms == 0xFFFFU) ||
       (s_track_light_fusion_diag.rear_age_ms > 300U))
    {
        return "STALE";
    }
    if(!s_track_light_link_diag.rear_extrinsic_valid)
    {
        return "EXT";
    }
    if(!s_track_light_link_diag.rear_time_sync_valid)
    {
        return "SYNC";
    }
    return "WAIT";
}

static void track_light_draw_front_link(uint8 row)
{
    uint16 y = TEST_ROW_Y(row);

    test_page_clear_line(y);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(TEST_PAGE_X, y, "FRONT FRAME:");
    track_light_draw_age((uint16)(TEST_PAGE_X + 96U), y,
                         s_track_light_fusion_diag.front_age_ms);
    ips200_set_color(s_track_light_fusion_diag.front_healthy ?
                     RGB565_GREEN : RGB565_RED, RGB565_BLACK);
    ips200_show_string((uint16)(TEST_PAGE_X + 128U), y,
                       s_track_light_fusion_diag.front_healthy ?
                       "READY" : "STALE");
}

static void track_light_draw_rear_link(uint8 row)
{
    uint16 y = TEST_ROW_Y(row);
    const char *use_text = track_light_rear_use_text();

    test_page_clear_line(y);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(TEST_PAGE_X, y, "REAR RX:");
    track_light_draw_age((uint16)(TEST_PAGE_X + 64U), y,
                         s_track_light_link_diag.last_result_age_ms);
    ips200_show_string((uint16)(TEST_PAGE_X + 88U), y, " F:");
    track_light_draw_age((uint16)(TEST_PAGE_X + 112U), y,
                         s_track_light_fusion_diag.rear_age_ms);
    ips200_show_string((uint16)(TEST_PAGE_X + 136U), y, " USE:");
    ips200_set_color(s_track_light_fusion_diag.rear_healthy ?
                     RGB565_GREEN : RGB565_RED, RGB565_BLACK);
    ips200_show_string((uint16)(TEST_PAGE_X + 176U), y, (char *)use_text);
}

static void track_light_draw_fusion_target(
    uint8 row, uint8 slot, const DualCameraFusionTarget_t *target)
{
    uint16 y = TEST_ROW_Y(row);

    test_page_clear_line(y);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_char(TEST_PAGE_X, y, 'B');
    ips200_show_uint((uint16)(TEST_PAGE_X + 8U), y, (uint32)(slot + 1U), 1U);
    if(target == NULL || !target->valid)
    {
        ips200_set_color(RGB565_RED, RGB565_BLACK);
        ips200_show_string((uint16)(TEST_PAGE_X + 24U), y, "OFF");
        return;
    }

    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    ips200_show_string((uint16)(TEST_PAGE_X + 24U), y,
                       track_light_fusion_source_text(target->source));
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string((uint16)(TEST_PAGE_X + 48U), y, "X:");
    ips200_show_float((uint16)(TEST_PAGE_X + 64U), y,
                      target->x_cm, 3U, 1U);
    ips200_show_string((uint16)(TEST_PAGE_X + 112U), y, "Y:");
    ips200_show_float((uint16)(TEST_PAGE_X + 128U), y,
                      target->y_cm, 3U, 1U);
    ips200_show_string((uint16)(TEST_PAGE_X + 176U), y, "C:");
    ips200_show_uint((uint16)(TEST_PAGE_X + 192U), y,
                     target->detection_confidence, 3U);
}

static void track_light_draw_row(uint8 row)
{
    const DualCameraFusionSnapshot_t *snapshot = &s_track_light_snapshot;
    uint8 car_valid;
    const char *car_source = "LOST ";
    float car_x_cm = 0.0f;
    float car_y_cm = 0.0f;

    car_valid = (uint8)(s_track_light_snapshot_valid &&
        (snapshot->car_valid || snapshot->front_car_valid ||
         snapshot->rear_car_valid));
    if(car_valid)
    {
        if(snapshot->front_car_valid && snapshot->rear_car_valid)
        {
            car_source = "BOTH ";
        }
        else if(snapshot->rear_car_valid)
        {
            car_source = "REAR ";
        }
        else
        {
            car_source = "FRONT";
        }

        if(snapshot->car_valid)
        {
            car_x_cm = snapshot->car_x_cm;
            car_y_cm = snapshot->car_y_cm;
        }
        else if(snapshot->rear_car_valid)
        {
            car_x_cm = snapshot->rear_car_x_cm;
            car_y_cm = snapshot->rear_car_y_cm;
        }
        else
        {
            car_x_cm = snapshot->front_car_x_cm;
            car_y_cm = snapshot->front_car_y_cm;
        }
    }

    ips200_set_font(IPS200_8X16_FONT);
    switch(row)
    {
        case 0U:
            track_light_draw_front_link(row);
            break;
        case 1U:
            track_light_draw_rear_link(row);
            break;
        case 2U:
            test_page_show_text(TEST_ROW_Y(row), "CAR:", car_source,
                                car_valid ? RGB565_GREEN : RGB565_RED);
            break;
        case 3U:
            test_page_show_float(TEST_ROW_Y(row), "CAR X CM:",
                                 car_valid ? car_x_cm : 0.0f, 3U, 1U);
            break;
        case 4U:
            test_page_show_float(TEST_ROW_Y(row), "CAR Y CM:",
                                 car_valid ? car_y_cm : 0.0f, 3U, 1U);
            break;
        case 5U:
            test_page_show_int(TEST_ROW_Y(row), "BEACONS:",
                s_track_light_snapshot_valid ?
                    (int32)snapshot->target_count : 0, 2U);
            break;
        default:
        {
            uint8 target_index = (uint8)(row - 6U);
            const DualCameraFusionTarget_t *target =
                (s_track_light_snapshot_valid &&
                 target_index < snapshot->target_count) ?
                    &snapshot->target[target_index] : NULL;
            track_light_draw_fusion_target(row, target_index, target);
            break;
        }
    }
}

static void track_light_draw_next_row(void)
{
    track_light_draw_row(s_track_light_row_cursor);
    s_track_light_row_cursor++;
    if(s_track_light_row_cursor >= 12U)
    {
        s_track_light_row_cursor = 0U;
    }
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
    if(!s_track_light_page_active)
    {
        track_light_reset_page_state();
        s_track_light_page_active = 1U;
        track_light_restore_local_screen();
        key_flag = 0U;
    }

    (void)track_light_service_downlink();
    (void)track_light_service_vision_fusion();

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
        s_track_light_row_cursor = 0U;
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

    /* 每轮只绘制一行，确保两次 UART5 解析之间不会被整页SPI刷新阻塞。 */
    track_light_draw_next_row();
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
    enum
    {
        MOTOR_TEST_ROW_MOTOR = 0U,
        MOTOR_TEST_ROW_THROTTLE,
        MOTOR_TEST_ROW_OUTPUT,
        MOTOR_TEST_ROW_COUNT,
        MOTOR_TEST_EDIT_NONE = 0xFFU
    };

    static uint8 selected_row = MOTOR_TEST_ROW_MOTOR;
    static uint8 edit_row = MOTOR_TEST_EDIT_NONE;
    static uint8 backup_motor = 0U;
    static uint16 backup_throttle = 0U;
    uint8 selected_motor = 0U;
    uint8 output_enable = 0U;
    uint16 throttle = 0U;
    uint16 output_limit = 0U;
    uint16 test_limit = 0U;
    const char *key_text = "K1/K2 SEL K3";
    uint16 key_color = RGB565_WHITE;

    if(!Flight_Motor_Test_Is_Active())
    {
        Flight_Motor_Test_Start();
        selected_row = MOTOR_TEST_ROW_MOTOR;
        edit_row = MOTOR_TEST_EDIT_NONE;
    }

    selected_motor = Flight_Motor_Test_Get_Selected();
    throttle = Flight_Motor_Test_Get_Throttle();
    output_enable = Flight_Motor_Test_Get_Output_Enable();
    output_limit = Flight_Motor_Test_Get_Output_Limit();
    test_limit = (MOTOR_TEST_MAX_THROTTLE < output_limit) ? MOTOR_TEST_MAX_THROTTLE : output_limit;

    /* 电机输出期间任意实体按键只执行本地急停，本次按键不再触发原功能。 */
    if(output_enable && key_flag != 0U)
    {
        Flight_Motor_Test_Set_Output_Enable(0U);
        key_flag = 0U;
    }
    else if(key_flag == Button_4)
    {
        if(edit_row != MOTOR_TEST_EDIT_NONE)
        {
            if(edit_row == MOTOR_TEST_ROW_MOTOR)
            {
                Flight_Motor_Test_Set_Selected(backup_motor);
            }
            else if(edit_row == MOTOR_TEST_ROW_THROTTLE)
            {
                Flight_Motor_Test_Set_Throttle(backup_throttle);
            }
            edit_row = MOTOR_TEST_EDIT_NONE;
            key_flag = 0;
        }
        else
        {
            Flight_Motor_Test_Stop();
            return;
        }
    }
    else if(edit_row != MOTOR_TEST_EDIT_NONE)
    {
        if(key_flag == Button_1)
        {
            if(edit_row == MOTOR_TEST_ROW_MOTOR)
            {
                selected_motor = (selected_motor == 0U) ? 3U : (selected_motor - 1U);
                Flight_Motor_Test_Set_Selected(selected_motor);
            }
            else if(edit_row == MOTOR_TEST_ROW_THROTTLE)
            {
                throttle = (throttle + MOTOR_TEST_STEP > test_limit) ? test_limit : (throttle + MOTOR_TEST_STEP);
                Flight_Motor_Test_Set_Throttle(throttle);
            }
            key_flag = 0;
        }
        else if(key_flag == Button_2)
        {
            if(edit_row == MOTOR_TEST_ROW_MOTOR)
            {
                selected_motor = (selected_motor + 1U) & 0x03U;
                Flight_Motor_Test_Set_Selected(selected_motor);
            }
            else if(edit_row == MOTOR_TEST_ROW_THROTTLE)
            {
                throttle = (throttle > MOTOR_TEST_STEP) ? (throttle - MOTOR_TEST_STEP) : 0U;
                Flight_Motor_Test_Set_Throttle(throttle);
            }
            key_flag = 0;
        }
        else if(key_flag == Button_3)
        {
            edit_row = MOTOR_TEST_EDIT_NONE;
            key_flag = 0;
        }
    }
    else
    {
        if(key_flag == Button_1)
        {
            selected_row = (selected_row == 0U) ? (uint8)(MOTOR_TEST_ROW_COUNT - 1U) : (uint8)(selected_row - 1U);
            key_flag = 0;
        }
        else if(key_flag == Button_2)
        {
            selected_row = (uint8)((selected_row + 1U) % MOTOR_TEST_ROW_COUNT);
            key_flag = 0;
        }
        else if(key_flag == Button_3)
        {
            if(selected_row == MOTOR_TEST_ROW_OUTPUT)
            {
                if(output_enable)
                {
                    Flight_Motor_Test_Set_Output_Enable(0U);
                }
                else
                {
                    Flight_Motor_Test_Set_Output_Enable(1U);
                }
            }
            else if(!output_enable)
            {
                edit_row = selected_row;
                backup_motor = selected_motor;
                backup_throttle = throttle;
            }
            key_flag = 0;
        }
    }

    selected_motor = Flight_Motor_Test_Get_Selected();
    throttle = Flight_Motor_Test_Get_Throttle();
    output_enable = Flight_Motor_Test_Get_Output_Enable();

    if(output_enable)
    {
        key_text = "ANY KEY STOP";
        key_color = RGB565_RED;
    }
    else if(edit_row == MOTOR_TEST_ROW_MOTOR)
    {
        key_text = "K1< K2> K3";
        key_color = RGB565_CYAN;
    }
    else if(edit_row == MOTOR_TEST_ROW_THROTTLE)
    {
        key_text = "K1+ K2- K3";
        key_color = RGB565_CYAN;
    }
    test_page_show_text(TEST_ROW_Y(0U), "KEY:", key_text, key_color);
    test_page_show_text(TEST_ROW_Y(1U),
                        (selected_row == MOTOR_TEST_ROW_MOTOR) ? "> MOTOR:" : "  MOTOR:",
                        "M", (edit_row == MOTOR_TEST_ROW_MOTOR) ? RGB565_CYAN : RGB565_WHITE);
    ips200_show_int((uint16)(TEST_VALUE_X + 16U), TEST_ROW_Y(1U), (int32)(selected_motor + 1U), 1U);
    test_page_show_int(TEST_ROW_Y(2U),
                       (selected_row == MOTOR_TEST_ROW_THROTTLE) ? "> THR:" : "  THR:",
                       (int32)throttle, 4U);
    test_page_show_text(TEST_ROW_Y(3U),
                        (selected_row == MOTOR_TEST_ROW_OUTPUT) ?
                            (output_enable ? "> STOP:" : "> START:") :
                            (output_enable ? "  STOP:" : "  START:"),
                        output_enable ? "RUN  " : "READY",
                        output_enable ? RGB565_RED : RGB565_GREEN);
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
    menu_create_item(&testMenu, "Vision Fusion", vision_fusion_page, NULL);
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
