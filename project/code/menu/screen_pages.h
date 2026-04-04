#ifndef _SCREEN_PAGES_H_
#define _SCREEN_PAGES_H_

#include "zf_common_headfile.h"

struct Menu;

typedef enum
{
    MONITOR_PAGE_DATA = 0,
    MONITOR_PAGE_IMAGE,
    MONITOR_PAGE_TOF_ATTITUDE,
    MONITOR_PAGE_OPTICAL_FLOW,
    MONITOR_PAGE_COUNT
} monitor_page_enum;

typedef struct
{
    uint8 *image_buffer;
    uint8 control_mode;
    uint8 armed;
    uint8 low_voltage;
    uint8 has_nearest;
    float voltage;
    int32 target_throttle;
    int32 base_throttle;
    float roll;
    float pitch;
    float yaw;
    int32 beacon_count;
    float nearest_x;
    float nearest_y;
    float tof_raw_cm;
    float tof_height_cm;
    float fused_height_cm;
    float ir_height_cm;
    float fused_velocity_cms;
    uint8 optical_flow_valid;
    int32 optical_flow_dx;
    int32 optical_flow_dy;
    int32 optical_flow_x_integral;
    float optical_flow_dt_s;
    float optical_flow_tof_los_cm;
    float optical_flow_range_cm;
    float optical_flow_roll_deg;
    float optical_flow_pitch_deg;
    float optical_flow_x_rad;
    float optical_flow_y_rad;
    float optical_flow_gyro_x_delta_rad;
    float optical_flow_gyro_y_delta_rad;
    float optical_flow_comp_x_rad;
    float optical_flow_comp_y_rad;
    float optical_flow_vx_raw_cms;
    float optical_flow_vy_raw_cms;
    float optical_flow_vx_cms;
    float optical_flow_vy_cms;
    float optical_flow_speed_cms;
} MonitorRuntimeSnapshot;

void ui_menu_init_display(void);
void ui_menu_display(struct Menu *menu);
void ui_menu_update_selection(struct Menu *menu, uint8 old_selection, uint8 new_selection);
void ui_menu_draw_function_header(const char *title);

void ui_parameter_draw_root_menu(uint8 dirty, const char *const *menu_items, uint8 item_count, uint8 selection);
void ui_parameter_draw_object_menu(uint8 dirty, const char *title, const char *const *object_labels, const uint8 *object_ids, uint8 object_count, uint8 selection);
void ui_parameter_draw_item_menu(uint8 dirty, const char *title, const char *const *item_labels, const float *item_values, const uint8 *item_available, uint8 item_count, uint8 selection, uint8 edit_mode);
void ui_parameter_draw_confirm(uint8 dirty, const char *title, const char *message, uint16 color);
void ui_parameter_draw_status(const char *message, uint16 color, uint16 delay_ms);

void ui_origin_draw_calibration_layout(void);
void ui_origin_show_phase(const char *text);
void ui_origin_show_live_attitude(float roll, float pitch, float yaw);
void ui_origin_show_average_attitude(float averaged_roll, float averaged_pitch, float averaged_yaw);
void ui_origin_show_saved(void);

void ui_monitor_show_view_page(const MonitorRuntimeSnapshot *snapshot, uint8 draw_layout);
void ui_monitor_show_image_page(const MonitorRuntimeSnapshot *snapshot, uint8 draw_layout);
uint8 screen_monitor_handle_input(monitor_page_enum *page,
                                  const MonitorRuntimeSnapshot *snapshot,
                                  float throttle_step);
void screen_monitor_enter_page(monitor_page_enum page, const MonitorRuntimeSnapshot *snapshot);
void screen_monitor_refresh_page(monitor_page_enum page, const MonitorRuntimeSnapshot *snapshot);

void ui_menu_show_icm_init_result_page(void);
void ui_menu_show_parameter_conflict_warning(void);
void ui_menu_draw_control_mode_page(uint8 selection);
void ui_menu_show_control_mode_saved(void);
void ui_menu_draw_progress_bar(uint16 x, uint16 y, uint16 width, uint16 height, uint8 progress);
void ui_menu_play_exit_animation(void);
void ui_menu_draw_exit_animation_page(uint16 bar_y);
void ui_menu_update_exit_animation_status(uint16 bar_y, uint16 bar_height, uint16 screen_width, uint16 screen_height, const char *status_text);
void ui_menu_show_exit_completed(uint16 bar_y, uint16 bar_height, uint16 screen_height);

#endif
