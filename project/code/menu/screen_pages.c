#include "screen_pages.h"
#include "common/project_headfile.h"
#include <stdio.h>
#include <string.h>

/* ===================== 通用页面辅助 ===================== */

#define UI_MENU_START_X              10U
#define UI_MENU_START_Y              40U
#define UI_MENU_ITEM_HEIGHT          20U
#define UI_MENU_ITEM_WIDTH           200U
#define UI_MENU_TITLE_Y              10U

#define UI_MENU_BG_COLOR             RGB565_BLACK
#define UI_MENU_TEXT_COLOR           RGB565_WHITE
#define UI_MENU_SELECT_COLOR         RGB565_BLUE
#define UI_MENU_TITLE_COLOR          RGB565_YELLOW

#define UI_PARAM_TITLE_Y             10U
#define UI_PARAM_START_Y             40U
#define UI_PARAM_LINE_HEIGHT         18U
#define UI_PARAM_LEFT_X              10U
#define UI_PARAM_VALUE_X             152U
#define UI_PARAM_STATUS_Y            258U
#define UI_PARAM_HINT_Y              284U

#define UI_ORIGIN_LINE_HEIGHT        16U
#define UI_ORIGIN_VALUE_X            120U

#define UI_MONITOR_LEFT_VALUE_X      48U
#define UI_MONITOR_RIGHT_VALUE_X     172U

#define UI_MONITOR_TITLE_Y           0U
#define UI_MONITOR_MODE_Y            24U
#define UI_MONITOR_BATTERY_Y         48U
#define UI_MONITOR_THROTTLE_Y        72U
#define UI_MONITOR_ATTITUDE_Y        96U
#define UI_MONITOR_YAW_Y             120U
#define UI_MONITOR_NEAREST_Y         144U
#define UI_MONITOR_HEIGHT_Y          168U
#define UI_MONITOR_IR_Y              192U
#define UI_MONITOR_HINT_TOP_Y        288U
#define UI_MONITOR_HINT_BOTTOM_Y     304U
#define MONITOR_ARM_START_THROTTLE   400.0f

static uint16 ui_origin_line_y(uint16 row)
{
    return (uint16)(row * UI_ORIGIN_LINE_HEIGHT);
}

static void ui_show_padded_text(uint16 x, uint16 y, const char *text, uint8 width, uint16 color)
{
    char buffer[32];
    uint8 i = 0;

    if(width >= sizeof(buffer))
    {
        width = (uint8)(sizeof(buffer) - 1U);
    }

    for(i = 0; i < width; i++)
    {
        buffer[i] = ' ';
    }
    buffer[width] = '\0';

    if(text != NULL)
    {
        for(i = 0; i < width && text[i] != '\0'; i++)
        {
            buffer[i] = text[i];
        }
    }

    ips200_set_color(color, RGB565_BLACK);
    ips200_show_string(x, y, buffer);
}

/* ===================== 通用菜单页面 ===================== */

void ui_menu_init_display(void)
{
    ips200_clear();
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(UI_MENU_TEXT_COLOR, UI_MENU_BG_COLOR);
}

void ui_menu_display(Menu *menu)
{
    uint8 i = 0;

    if(menu == NULL)
    {
        return;
    }

    ips200_clear();
    ips200_set_color(UI_MENU_TITLE_COLOR, UI_MENU_BG_COLOR);
    ips200_show_string(UI_MENU_START_X, UI_MENU_TITLE_Y, menu->title);

    if(menu->itemCount == 0U)
    {
        ips200_set_color(UI_MENU_TEXT_COLOR, UI_MENU_BG_COLOR);
        ips200_show_string(UI_MENU_START_X, UI_MENU_START_Y, "No menu items");
        return;
    }

    if(menu->currentSelection >= menu->itemCount)
    {
        menu->currentSelection = 0U;
    }

    for(i = 0; i < menu->itemCount; i++)
    {
        if(i == menu->currentSelection)
        {
            ips200_set_color(UI_MENU_SELECT_COLOR, UI_MENU_BG_COLOR);
            ips200_draw_line(UI_MENU_START_X - 5U, UI_MENU_START_Y + i * UI_MENU_ITEM_HEIGHT,
                             UI_MENU_START_X + UI_MENU_ITEM_WIDTH, UI_MENU_START_Y + i * UI_MENU_ITEM_HEIGHT, UI_MENU_SELECT_COLOR);
            ips200_draw_line(UI_MENU_START_X - 5U, UI_MENU_START_Y + (i + 1U) * UI_MENU_ITEM_HEIGHT,
                             UI_MENU_START_X + UI_MENU_ITEM_WIDTH, UI_MENU_START_Y + (i + 1U) * UI_MENU_ITEM_HEIGHT, UI_MENU_SELECT_COLOR);
            ips200_draw_line(UI_MENU_START_X - 5U, UI_MENU_START_Y + i * UI_MENU_ITEM_HEIGHT,
                             UI_MENU_START_X - 5U, UI_MENU_START_Y + (i + 1U) * UI_MENU_ITEM_HEIGHT, UI_MENU_SELECT_COLOR);
            ips200_draw_line(UI_MENU_START_X + UI_MENU_ITEM_WIDTH, UI_MENU_START_Y + i * UI_MENU_ITEM_HEIGHT,
                             UI_MENU_START_X + UI_MENU_ITEM_WIDTH, UI_MENU_START_Y + (i + 1U) * UI_MENU_ITEM_HEIGHT, UI_MENU_SELECT_COLOR);
        }
        else
        {
            ips200_set_color(UI_MENU_TEXT_COLOR, UI_MENU_BG_COLOR);
        }

        ips200_show_string(UI_MENU_START_X, UI_MENU_START_Y + i * UI_MENU_ITEM_HEIGHT, menu->items[i].name);
    }

    ips200_set_color(UI_MENU_TEXT_COLOR, UI_MENU_BG_COLOR);
}

void ui_menu_update_selection(Menu *menu, uint8 old_selection, uint8 new_selection)
{
    if(menu == NULL || menu->itemCount == 0U)
    {
        return;
    }

    if(old_selection >= menu->itemCount || new_selection >= menu->itemCount)
    {
        ui_menu_display(menu);
        return;
    }

    ips200_set_color(UI_MENU_TEXT_COLOR, UI_MENU_BG_COLOR);
    ips200_draw_line(UI_MENU_START_X - 5U, UI_MENU_START_Y + old_selection * UI_MENU_ITEM_HEIGHT,
                     UI_MENU_START_X + UI_MENU_ITEM_WIDTH, UI_MENU_START_Y + old_selection * UI_MENU_ITEM_HEIGHT, UI_MENU_BG_COLOR);
    ips200_draw_line(UI_MENU_START_X - 5U, UI_MENU_START_Y + (old_selection + 1U) * UI_MENU_ITEM_HEIGHT,
                     UI_MENU_START_X + UI_MENU_ITEM_WIDTH, UI_MENU_START_Y + (old_selection + 1U) * UI_MENU_ITEM_HEIGHT, UI_MENU_BG_COLOR);
    ips200_draw_line(UI_MENU_START_X - 5U, UI_MENU_START_Y + old_selection * UI_MENU_ITEM_HEIGHT,
                     UI_MENU_START_X - 5U, UI_MENU_START_Y + (old_selection + 1U) * UI_MENU_ITEM_HEIGHT, UI_MENU_BG_COLOR);
    ips200_draw_line(UI_MENU_START_X + UI_MENU_ITEM_WIDTH, UI_MENU_START_Y + old_selection * UI_MENU_ITEM_HEIGHT,
                     UI_MENU_START_X + UI_MENU_ITEM_WIDTH, UI_MENU_START_Y + (old_selection + 1U) * UI_MENU_ITEM_HEIGHT, UI_MENU_BG_COLOR);
    ips200_show_string(UI_MENU_START_X, UI_MENU_START_Y + old_selection * UI_MENU_ITEM_HEIGHT, menu->items[old_selection].name);

    ips200_set_color(UI_MENU_SELECT_COLOR, UI_MENU_BG_COLOR);
    ips200_draw_line(UI_MENU_START_X - 5U, UI_MENU_START_Y + new_selection * UI_MENU_ITEM_HEIGHT,
                     UI_MENU_START_X + UI_MENU_ITEM_WIDTH, UI_MENU_START_Y + new_selection * UI_MENU_ITEM_HEIGHT, UI_MENU_SELECT_COLOR);
    ips200_draw_line(UI_MENU_START_X - 5U, UI_MENU_START_Y + (new_selection + 1U) * UI_MENU_ITEM_HEIGHT,
                     UI_MENU_START_X + UI_MENU_ITEM_WIDTH, UI_MENU_START_Y + (new_selection + 1U) * UI_MENU_ITEM_HEIGHT, UI_MENU_SELECT_COLOR);
    ips200_draw_line(UI_MENU_START_X - 5U, UI_MENU_START_Y + new_selection * UI_MENU_ITEM_HEIGHT,
                     UI_MENU_START_X - 5U, UI_MENU_START_Y + (new_selection + 1U) * UI_MENU_ITEM_HEIGHT, UI_MENU_SELECT_COLOR);
    ips200_draw_line(UI_MENU_START_X + UI_MENU_ITEM_WIDTH, UI_MENU_START_Y + new_selection * UI_MENU_ITEM_HEIGHT,
                     UI_MENU_START_X + UI_MENU_ITEM_WIDTH, UI_MENU_START_Y + (new_selection + 1U) * UI_MENU_ITEM_HEIGHT, UI_MENU_SELECT_COLOR);
    ips200_show_string(UI_MENU_START_X, UI_MENU_START_Y + new_selection * UI_MENU_ITEM_HEIGHT, menu->items[new_selection].name);

    ips200_set_color(UI_MENU_TEXT_COLOR, UI_MENU_BG_COLOR);
}

void ui_menu_draw_function_header(const char *title)
{
    ips200_clear();
    ips200_set_color(UI_MENU_TITLE_COLOR, UI_MENU_BG_COLOR);
    ips200_show_string(20U, 20U, (char *)title);
    ips200_set_color(UI_MENU_TEXT_COLOR, UI_MENU_BG_COLOR);
    ips200_show_string(20U, 40U, "Press BACK to return");
}

/* ===================== 参数页面 ===================== */

static void ui_parameter_draw_frame(const char *title, const char *hint, uint8 dirty)
{
    ips200_clear();
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(UI_PARAM_LEFT_X, UI_PARAM_TITLE_Y, (char *)title);

    if(dirty)
    {
        ui_show_padded_text(UI_PARAM_LEFT_X, UI_PARAM_STATUS_Y, "UNSAVED CHANGES", 20U, RGB565_CYAN);
    }
    else
    {
        ui_show_padded_text(UI_PARAM_LEFT_X, UI_PARAM_STATUS_Y, "FLASH SYNCHRONIZED", 20U, RGB565_GREEN);
    }

    ips200_set_color(RGB565_GRAY, RGB565_BLACK);
    ips200_show_string(UI_PARAM_LEFT_X, UI_PARAM_HINT_Y, (char *)hint);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

void ui_parameter_draw_root_menu(uint8 dirty, const char *const *menu_items, uint8 item_count, uint8 selection)
{
    uint8 i = 0;

    ui_parameter_draw_frame("Parameter Config", "1:Up 2:Down 3:Enter 4:Back", dirty);

    for(i = 0; i < item_count; i++)
    {
        ips200_set_color((i == selection) ? RGB565_BLUE : RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(UI_PARAM_LEFT_X,
                           (uint16)(UI_PARAM_START_Y + i * UI_PARAM_LINE_HEIGHT),
                           (char *)menu_items[i]);
    }

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

void ui_parameter_draw_object_menu(uint8 dirty, const char *title, const char *const *object_labels, const uint8 *object_ids, uint8 object_count, uint8 selection)
{
    uint8 i = 0;

    ui_parameter_draw_frame(title, "1:Up 2:Down 3:Enter 4:Back", dirty);

    for(i = 0; i < object_count; i++)
    {
        ips200_set_color((i == selection) ? RGB565_BLUE : RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(UI_PARAM_LEFT_X,
                           (uint16)(UI_PARAM_START_Y + i * UI_PARAM_LINE_HEIGHT),
                           (char *)object_labels[object_ids[i]]);
    }

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

void ui_parameter_draw_item_menu(uint8 dirty, const char *title, const char *const *item_labels, const float *item_values, const uint8 *item_available, uint8 item_count, uint8 selection, uint8 edit_mode)
{
    uint8 i = 0;
    char value_text[24];

    ui_parameter_draw_frame(title,
                            edit_mode ? "1:+ 2:- 3:OK 4:Cancel"
                                      : "1:Up 2:Down 3:Edit 4:Back",
                            dirty);

    for(i = 0; i < item_count; i++)
    {
        uint16 color = RGB565_WHITE;

        if(!item_available[i])
        {
            continue;
        }

        if(i == selection)
        {
            color = edit_mode ? RGB565_GREEN : RGB565_BLUE;
        }

        ips200_set_color(color, RGB565_BLACK);
        ips200_show_string(UI_PARAM_LEFT_X,
                           (uint16)(UI_PARAM_START_Y + i * UI_PARAM_LINE_HEIGHT),
                           (char *)item_labels[i]);

        format_float_truncate(value_text, item_values[i], 8U, 2U);
        ips200_show_string(UI_PARAM_VALUE_X,
                           (uint16)(UI_PARAM_START_Y + i * UI_PARAM_LINE_HEIGHT),
                           value_text);
    }

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

void ui_parameter_draw_confirm(uint8 dirty, const char *title, const char *message, uint16 color)
{
    ui_parameter_draw_frame(title, "3:Yes 4:No", dirty);
    ui_show_padded_text(UI_PARAM_LEFT_X, UI_PARAM_START_Y, message, 28U, color);
}

void ui_parameter_draw_status(const char *message, uint16 color, uint16 delay_ms)
{
    ips200_clear();
    ui_show_padded_text(UI_PARAM_LEFT_X, (uint16)(ips200_height_max / 2U - 8U), message, 28U, color);
    system_delay_ms(delay_ms);
}

/* ===================== 原点标定页面 ===================== */

void ui_origin_draw_calibration_layout(void)
{
    ips200_clear();

    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(0U, ui_origin_line_y(0U), "ICM INIT / ORIGIN");

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0U, ui_origin_line_y(2U), "Keep drone still");
    ips200_show_string(0U, ui_origin_line_y(3U), "Start in 2s");
    ips200_show_string(0U, ui_origin_line_y(5U), "LIVE ROLL:");
    ips200_show_string(0U, ui_origin_line_y(6U), "LIVE PITCH:");
    ips200_show_string(0U, ui_origin_line_y(7U), "LIVE YAW:");
    ips200_show_string(0U, ui_origin_line_y(10U), "AVG ROLL:");
    ips200_show_string(0U, ui_origin_line_y(11U), "AVG PITCH:");
    ips200_show_string(0U, ui_origin_line_y(12U), "AVG YAW:");
}

void ui_origin_show_phase(const char *text)
{
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0U, ui_origin_line_y(3U), (char *)text);
}

void ui_origin_show_live_attitude(float roll, float pitch, float yaw)
{
    ips200_show_float(UI_ORIGIN_VALUE_X, ui_origin_line_y(5U), roll, 3U, 3U);
    ips200_show_float(UI_ORIGIN_VALUE_X, ui_origin_line_y(6U), pitch, 3U, 3U);
    ips200_show_float(UI_ORIGIN_VALUE_X, ui_origin_line_y(7U), yaw, 3U, 3U);
}

void ui_origin_show_average_attitude(float averaged_roll, float averaged_pitch, float averaged_yaw)
{
    ips200_show_float(UI_ORIGIN_VALUE_X, ui_origin_line_y(10U), averaged_roll, 3U, 3U);
    ips200_show_float(UI_ORIGIN_VALUE_X, ui_origin_line_y(11U), averaged_pitch, 3U, 3U);
    ips200_show_float(UI_ORIGIN_VALUE_X, ui_origin_line_y(12U), averaged_yaw, 3U, 3U);
}

void ui_origin_show_saved(void)
{
    ui_origin_show_phase("Origin saved");
}

/* ===================== ??? HUD ??? ===================== */

typedef struct
{
    const char *mode_text;
    const char *hint_text;
    uint8 armed;
    uint8 low_voltage;
    uint8 has_nearest;
    float voltage;
    int32 target_throttle;
    int32 base_throttle;
    float roll;
    float pitch;
    int32 beacon_count;
    float nearest_x;
    float nearest_y;
    float tof_height_cm;
    float fused_height_cm;
    float fused_velocity_cms;
} MonitorViewHudData;

static const char *ui_monitor_mode_text(uint8 control_mode)
{
    return (control_mode == CONTROL_MODE_ANGLE) ? "ANGLE" : "MANUAL";
}

static const char *ui_monitor_hint_text(uint8 armed)
{
    return armed ? "ANY KEY: EMERGENCY STOP" : "K1:PREV K2:NEXT K3:ARM";
}

static void ui_monitor_fill_view_hud(const MonitorRuntimeSnapshot *snapshot, MonitorViewHudData *data)
{
    if(snapshot == NULL || data == NULL)
    {
        return;
    }

    data->mode_text = ui_monitor_mode_text(snapshot->control_mode);
    data->hint_text = ui_monitor_hint_text(snapshot->armed);
    data->armed = snapshot->armed;
    data->low_voltage = snapshot->low_voltage;
    data->has_nearest = snapshot->has_nearest;
    data->voltage = snapshot->voltage;
    data->target_throttle = snapshot->target_throttle;
    data->base_throttle = snapshot->base_throttle;
    data->roll = snapshot->roll;
    data->pitch = snapshot->pitch;
    data->beacon_count = snapshot->beacon_count;
    data->nearest_x = snapshot->nearest_x;
    data->nearest_y = snapshot->nearest_y;
    data->tof_height_cm = snapshot->tof_height_cm;
    data->fused_height_cm = snapshot->fused_height_cm;
    data->fused_velocity_cms = snapshot->fused_velocity_cms;
}

static void ui_monitor_draw_view_layout(void)
{
    ips200_clear();

    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(0U, UI_MONITOR_TITLE_Y, "FLIGHT SAFETY");
    ips200_show_string(0U, UI_MONITOR_MODE_Y, "MODE:");
    ips200_show_string(120U, UI_MONITOR_MODE_Y, "ARM:");
    ips200_show_string(0U, UI_MONITOR_BATTERY_Y, "BAT:");
    ips200_show_string(120U, UI_MONITOR_BATTERY_Y, "LOW:");
    ips200_show_string(0U, UI_MONITOR_THROTTLE_Y, "TTHR:");
    ips200_show_string(120U, UI_MONITOR_THROTTLE_Y, "BTHR:");
    ips200_show_string(0U, UI_MONITOR_ATTITUDE_Y, "HFH:");
    ips200_show_string(120U, UI_MONITOR_ATTITUDE_Y, "TOF:");
    ips200_show_string(0U, UI_MONITOR_YAW_Y, "VZ:");
    ips200_show_string(120U, UI_MONITOR_YAW_Y, "IR:");
    ips200_show_string(0U, UI_MONITOR_NEAREST_Y, "IRX:");
    ips200_show_string(120U, UI_MONITOR_NEAREST_Y, "IRY:");
    ips200_show_string(0U, UI_MONITOR_HEIGHT_Y, "ROLL:");
    ips200_show_string(120U, UI_MONITOR_HEIGHT_Y, "PITCH:");
    ips200_show_string(0U, UI_MONITOR_IR_Y, "BCN:");
    ips200_show_string(0U, UI_MONITOR_HINT_BOTTOM_Y, "HOLD K4:MENU WHEN STOPPED");
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

static void ui_monitor_draw_view_hud(const MonitorViewHudData *data)
{
    if(data == NULL)
    {
        return;
    }

    ui_show_padded_text(UI_MONITOR_LEFT_VALUE_X, UI_MONITOR_MODE_Y, data->mode_text, 8U, RGB565_WHITE);
    ips200_show_int(UI_MONITOR_RIGHT_VALUE_X, UI_MONITOR_MODE_Y, data->armed, 1U);

    ips200_show_float(UI_MONITOR_LEFT_VALUE_X, UI_MONITOR_BATTERY_Y, data->voltage, 2U, 2U);
    ui_show_padded_text(UI_MONITOR_RIGHT_VALUE_X, UI_MONITOR_BATTERY_Y, data->low_voltage ? "LOW" : "OK", 4U,
                        data->low_voltage ? RGB565_RED : RGB565_GREEN);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    ips200_show_int(UI_MONITOR_LEFT_VALUE_X, UI_MONITOR_THROTTLE_Y, data->target_throttle, 4U);
    ips200_show_int(UI_MONITOR_RIGHT_VALUE_X, UI_MONITOR_THROTTLE_Y, data->base_throttle, 4U);

    ips200_show_float(UI_MONITOR_LEFT_VALUE_X, UI_MONITOR_ATTITUDE_Y, data->fused_height_cm, 4U, 1U);
    ips200_show_float(UI_MONITOR_RIGHT_VALUE_X, UI_MONITOR_ATTITUDE_Y, data->tof_height_cm, 4U, 1U);
    ips200_show_float(UI_MONITOR_LEFT_VALUE_X, UI_MONITOR_YAW_Y, data->fused_velocity_cms, 4U, 1U);
    ui_show_padded_text(UI_MONITOR_RIGHT_VALUE_X, UI_MONITOR_YAW_Y, data->has_nearest ? "TRACK" : "LOST", 6U,
                        data->has_nearest ? RGB565_GREEN : RGB565_RED);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    if(data->has_nearest)
    {
        ips200_show_float(UI_MONITOR_LEFT_VALUE_X, UI_MONITOR_NEAREST_Y, data->nearest_x, 4U, 1U);
        ips200_show_float(UI_MONITOR_RIGHT_VALUE_X, UI_MONITOR_NEAREST_Y, data->nearest_y, 4U, 1U);
    }
    else
    {
        ui_show_padded_text(UI_MONITOR_LEFT_VALUE_X, UI_MONITOR_NEAREST_Y, "NONE", 6U, RGB565_WHITE);
        ui_show_padded_text(UI_MONITOR_RIGHT_VALUE_X, UI_MONITOR_NEAREST_Y, "NONE", 6U, RGB565_WHITE);
    }

    ips200_show_float(UI_MONITOR_LEFT_VALUE_X, UI_MONITOR_HEIGHT_Y, data->roll, 4U, 1U);
    ips200_show_float(UI_MONITOR_RIGHT_VALUE_X, UI_MONITOR_HEIGHT_Y, data->pitch, 4U, 1U);
    ips200_show_int(UI_MONITOR_LEFT_VALUE_X, UI_MONITOR_IR_Y, data->beacon_count, 2U);
    ui_show_padded_text(0U, UI_MONITOR_HINT_TOP_Y, data->hint_text, 28U, RGB565_WHITE);
}

static void ui_monitor_show_view_page(const MonitorRuntimeSnapshot *snapshot, uint8 draw_layout)
{
    MonitorViewHudData data;

    if(snapshot == NULL)
    {
        return;
    }

    if(draw_layout)
    {
        ui_monitor_draw_view_layout();
    }

    ui_monitor_fill_view_hud(snapshot, &data);
    ui_monitor_draw_view_hud(&data);
}

/* ===================== ?????????? ===================== */

static uint8 screen_consume_monitor_button_event(uint8 button_id, key_index_enum key_n)
{
    if(key_flag == button_id)
    {
        key_flag = 0;
        key_clear_state(key_n);
        return 1U;
    }

    return 0U;
}

static void screen_monitor_request_stop(void)
{
    Flight_Runtime_Emergency_Stop(EMERGENCY_STOP_REASON_MANUAL_BUTTON);
}

static uint8 s_monitor_estop_wait_release = 0U;

static uint8 screen_monitor_any_key_state_active(void)
{
    return (uint8)((key_get_state(KEY_1) != KEY_RELEASE) ||
                   (key_get_state(KEY_2) != KEY_RELEASE) ||
                   (key_get_state(KEY_3) != KEY_RELEASE) ||
                   (key_get_state(KEY_4) != KEY_RELEASE));
}

static uint8 screen_monitor_handle_armed_estop_key(void)
{
    if(s_monitor_estop_wait_release)
    {
        key_flag = 0U;
        if(!screen_monitor_any_key_state_active())
        {
            s_monitor_estop_wait_release = 0U;
        }
        return 1U;
    }

    if(armed && key_flag >= Button_1 && key_flag <= Button_4)
    {
        key_flag = 0U;
        s_monitor_estop_wait_release = 1U;
        screen_monitor_request_stop();
        return 1U;
    }

    return 0U;
}

static void screen_monitor_arm_with_initial_target(void)
{
    Flight_Runtime_Clear_Emergency_Stop_Latch();
    if(Flight_Control_Arm(NULL))
    {
        Flight_Control_Set_Throttle(MONITOR_ARM_START_THROTTLE);
    }
}

static void screen_monitor_switch_page(monitor_page_enum *page, int8 direction)
{
    int8 next_page = 0;

    if(page == NULL)
    {
        return;
    }

    next_page = (int8)(*page) + direction;

    if(next_page < 0)
    {
        next_page = (int8)MONITOR_PAGE_COUNT - 1;
    }
    else if(next_page >= (int8)MONITOR_PAGE_COUNT)
    {
        next_page = 0;
    }

    *page = (monitor_page_enum)next_page;
}

void screen_monitor_enter_page(monitor_page_enum page, const MonitorRuntimeSnapshot *snapshot)
{
    if(page == MONITOR_PAGE_DATA)
    {
        ui_monitor_show_view_page(snapshot, 1U);
    }
}

void screen_monitor_refresh_page(monitor_page_enum page, const MonitorRuntimeSnapshot *snapshot)
{
    if(page == MONITOR_PAGE_DATA)
    {
        ui_monitor_show_view_page(snapshot, 0U);
    }
}

uint8 screen_monitor_handle_input(monitor_page_enum *page,
                                  const MonitorRuntimeSnapshot *snapshot,
                                  float throttle_step)
{
    uint8 remote_exit_requested = 0U;

    if(throttle_step < 50.0f)
    {
        throttle_step = 50.0f;
    }

    if(page == NULL)
    {
        return 0U;
    }

    /* 已解锁后，Flight View 的任意实体按键都只执行急停。
     * 急停后等待本次按键真正释放，避免长按事件被再次解释为菜单操作。 */
    if(screen_monitor_handle_armed_estop_key())
    {
        return 1U;
    }

    remote_exit_requested = core_link_user_consume_monitor_exit_request();

    /* ?? KEY_4?????????????? */
    if((key_get_state(KEY_4) == KEY_LONG_PRESS) || remote_exit_requested)
    {
        screen_monitor_request_stop();
        if(key_get_state(KEY_4) == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_4);
        }
        key_flag = 0;
        menu_enter_main_menu_view();
        return 1U;
    }

    if(screen_consume_monitor_button_event(Button_1, KEY_1))
    {
        if(armed)
        {
            Flight_Control_Set_Throttle(Flight_Control_Get_Target_Throttle() + throttle_step);
        }
        else
        {
            screen_monitor_switch_page(page, -1);
            screen_monitor_enter_page(*page, snapshot);
        }
    }

    if(screen_consume_monitor_button_event(Button_2, KEY_2))
    {
        if(armed)
        {
            Flight_Control_Set_Throttle(Flight_Control_Get_Target_Throttle() - throttle_step);
        }
        else
        {
            screen_monitor_switch_page(page, 1);
            screen_monitor_enter_page(*page, snapshot);
        }
    }

    if(screen_consume_monitor_button_event(Button_3, KEY_3))
    {
        if(control_mode == CONTROL_MODE_ANGLE)
        {
            /* 角度闭环模式：走自动水平标定 + 起飞爬坡流程，不走立即解锁分支。
             * 采样阶段保持 disarmed，状态机到 RAMP 时自行 Arm。 */
            Flight_Runtime_Clear_Emergency_Stop_Latch();
            Flight_Takeoff_Start();
        }
        else if(!armed)
        {
            screen_monitor_arm_with_initial_target();
        }
    }

    if(screen_consume_monitor_button_event(Button_4, KEY_4))
    {
        screen_monitor_request_stop();
    }

    return 0U;
}

/* ===================== 菜单特殊页面 ===================== */

void ui_menu_show_icm_init_result_page(void)
{
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(0U, 0U, "ICM INIT DONE");
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0U, 56U, "ROLL:");
    ips200_show_string(0U, 80U, "PITCH:");
    ips200_show_string(0U, 104U, "YAW:");
    ips200_show_float(96U, 56U, Roll_a, 3U, 3U);
    ips200_show_float(96U, 80U, Pitch_a, 3U, 3U);
    ips200_show_float(96U, 104U, Yaw_a, 3U, 3U);

    ips200_show_string(0U, 124U, "GYRO X:");
    ips200_show_string(0U, 148U, "GYRO Y:");
    ips200_show_string(0U, 172U, "GYRO Z:");
    ips200_show_float(96U, 124U, icm42688_gyro_x, 3U, 3U);
    ips200_show_float(96U, 148U, icm42688_gyro_y, 3U, 3U);
    ips200_show_float(96U, 172U, icm42688_gyro_z, 3U, 3U);
}

void ui_menu_show_parameter_conflict_warning(void)
{
    ips200_clear();
    ips200_set_color(RGB565_RED, RGB565_BLACK);
    ips200_show_string(10U, 20U, "Param Index Conflict");
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(10U, 50U, "Check parameter_flash.c");
    ips200_show_string(10U, 70U, "Review index settings");
}

void ui_menu_draw_control_mode_page(uint8 selection)
{
    ips200_clear();
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(20U, 20U, "SELECT CTRL MODE");
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(40U, 60U, "0. MANUAL");
    ips200_show_string(40U, 80U, "1. ANGLE");
    ips200_set_color(RGB565_BLUE, RGB565_BLACK);
    ips200_show_string(20U, selection ? 80U : 60U, ">");
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

void ui_menu_show_control_mode_saved(void)
{
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    ips200_show_string(40U, 110U, "Mode Saved!");
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

void ui_menu_draw_progress_bar(uint16 x, uint16 y, uint16 width, uint16 height, uint8 progress)
{
    static uint8 init_flag = 0U;
    static uint16 prev_x = 0U;
    static uint16 prev_y = 0U;
    static uint16 prev_width = 0U;
    static uint16 prev_height = 0U;
    static uint16 prev_filled_w = 0U;
    static uint16 current_color = RGB565_RED;
    uint16 i = 0U;
    uint16 filled_width = 0U;
    uint16 new_color = RGB565_RED;

    if(x + width >= ips200_width_max)
    {
        width = (uint16)(ips200_width_max - x - 5U);
    }

    if(y + height >= ips200_height_max)
    {
        height = (uint16)(ips200_height_max - y - 5U);
    }

    if(progress > 100U)
    {
        progress = 100U;
    }

    filled_width = (uint16)((width * progress) / 100U);
    if(filled_width > width - 2U)
    {
        filled_width = (uint16)(width - 2U);
    }

    if(!init_flag || prev_x != x || prev_y != y || prev_width != width || prev_height != height)
    {
        init_flag = 1U;
        prev_x = x;
        prev_y = y;
        prev_width = width;
        prev_height = height;
        prev_filled_w = 0U;
        current_color = RGB565_RED;

        ips200_draw_line(x, y, x + width, y, RGB565_WHITE);
        ips200_draw_line(x, y + height, x + width, y + height, RGB565_WHITE);
        ips200_draw_line(x, y, x, y + height, RGB565_WHITE);
        ips200_draw_line(x + width, y, x + width, y + height, RGB565_WHITE);
    }

    if(progress < 30U)
    {
        new_color = RGB565_RED;
    }
    else if(progress < 70U)
    {
        new_color = RGB565_YELLOW;
    }
    else
    {
        new_color = RGB565_GREEN;
    }

    if(new_color != current_color)
    {
        uint16 start_x = (uint16)(x + 1U);
        uint16 end_x = (uint16)(x + filled_width);

        current_color = new_color;
        for(i = 1U; i < height && i < 19U; i++)
        {
            if(start_x < ips200_width_max && end_x < ips200_width_max && start_x <= end_x)
            {
                ips200_draw_line(start_x, y + i, end_x, y + i, current_color);
            }
        }
    }
    else if(filled_width != prev_filled_w)
    {
        if(filled_width > prev_filled_w)
        {
            uint16 start_x = (uint16)(x + 1U + prev_filled_w);
            uint16 end_x = (uint16)(x + filled_width);

            for(i = 1U; i < height && i < 19U; i++)
            {
                if(start_x < ips200_width_max && end_x < ips200_width_max && start_x <= end_x)
                {
                    ips200_draw_line(start_x, y + i, end_x, y + i, current_color);
                }
            }
        }
        else
        {
            uint16 start_x = (uint16)(x + 1U + filled_width);
            uint16 end_x = (uint16)(x + prev_filled_w);

            for(i = 1U; i < height && i < 19U; i++)
            {
                if(start_x < ips200_width_max && end_x < ips200_width_max && start_x <= end_x)
                {
                    ips200_draw_line(start_x, y + i, end_x, y + i, RGB565_BLACK);
                }
            }
        }
    }

    prev_filled_w = filled_width;

    if(x + width + 30U < ips200_width_max && y + height / 2U < ips200_height_max)
    {
        ips200_draw_line(x + width + 5U, (uint16)(y + height / 2U - 1U), x + width + 29U, (uint16)(y + height / 2U - 1U), RGB565_BLACK);
        ips200_draw_line(x + width + 5U, (uint16)(y + height / 2U + 8U), x + width + 29U, (uint16)(y + height / 2U + 8U), RGB565_BLACK);
        ips200_show_string((uint16)(x + width + 10U), (uint16)(y + height / 2U - 8U), "%");
        ips200_show_uint((uint16)(x + width + 5U), (uint16)(y + height / 2U - 8U), progress, 3U);
    }
}

void ui_menu_play_exit_animation(void)
{
    uint8 progress = 0U;
    char status_text[20];
    uint16 screen_width = ips200_width_max;
    uint16 screen_height = ips200_height_max;
    uint16 bar_x = 20U;
    uint16 bar_y = (uint16)(screen_height / 2U);
    uint16 bar_width = (uint16)(screen_width - 80U);
    uint16 bar_height = 20U;

    if(bar_width > (uint16)(screen_width - 40U))
    {
        bar_width = (uint16)(screen_width - 40U);
    }
    if((uint16)(bar_y + 50U) > screen_height)
    {
        bar_y = (uint16)(screen_height - 80U);
    }

    ui_menu_draw_exit_animation_page(bar_y);

    while(progress <= 100U)
    {
        if(progress < 30U)
        {
            strcpy(status_text, "Saving settings...");
        }
        else if(progress < 60U)
        {
            strcpy(status_text, "Cleaning up...");
        }
        else if(progress < 90U)
        {
            strcpy(status_text, "Finalizing...");
        }
        else
        {
            strcpy(status_text, "Complete!");
        }

        if(progress == 0U || progress == 30U || progress == 60U || progress == 90U)
        {
            ui_menu_update_exit_animation_status(bar_y, bar_height, screen_width, screen_height, status_text);
        }

        ui_menu_draw_progress_bar(bar_x, bar_y, bar_width, bar_height, progress);
        progress++;
        system_delay_ms(10);
    }

    ui_menu_show_exit_completed(bar_y, bar_height, screen_height);
    system_delay_ms(1000);
}

void ui_menu_draw_exit_animation_page(uint16 bar_y)
{
    ips200_clear();
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(20U, 20U, "Exiting menu system...");
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(20U, (uint16)(bar_y - 20U), "Progress:");
}

void ui_menu_update_exit_animation_status(uint16 bar_y, uint16 bar_height, uint16 screen_width, uint16 screen_height, const char *status_text)
{
    uint8 i = 0U;

    for(i = 0U; i < 20U && (uint16)(bar_y + bar_height + 10U + i) < screen_height; i++)
    {
        if(180U < screen_width)
        {
            ips200_draw_line(20U,
                             (uint16)(bar_y + bar_height + 10U + i),
                             180U,
                             (uint16)(bar_y + bar_height + 10U + i),
                             RGB565_BLACK);
        }
    }

    if((uint16)(bar_y + bar_height + 10U) < (uint16)(screen_height - 20U))
    {
        ips200_show_string(20U, (uint16)(bar_y + bar_height + 10U), (char *)status_text);
    }
}

void ui_menu_show_exit_completed(uint16 bar_y, uint16 bar_height, uint16 screen_height)
{
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    if((uint16)(bar_y + bar_height + 40U) < (uint16)(screen_height - 10U))
    {
        ips200_show_string(20U, (uint16)(bar_y + bar_height + 40U), "Exit completed!");
    }
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}
