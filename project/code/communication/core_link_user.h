#ifndef _core_link_user_h_
#define _core_link_user_h_

#include "core_link.h"
#include "menu/screen_pages.h"

void core_link_user_init (void);
void core_link_user_task (void);
void core_link_user_poll (void);
void core_link_user_tick (uint16 elapsed_ms);
void core_link_user_service_beacon_link(void);
typedef struct
{
    uint8 valid;
    uint16 age_ms;
} CoreLinkBeaconDiag_t;

typedef struct
{
    uint8 valid;
    uint8 flags;
    uint8 target_valid;
    uint8 sequence;
    uint16 age_ms;
    uint16 protocol_error_count;
    float forward_speed_cms;
    float left_speed_cms;
    float car_yaw_deg;
} CoreLinkCarMotionDiag_t;

void core_link_user_get_beacon_diag(CoreLinkBeaconDiag_t *diag);
void core_link_user_get_car_motion_diag(CoreLinkCarMotionDiag_t *diag);
/* 用最新有效的小车绝对Yaw与指定飞机Yaw计算车机相对Yaw。 */
uint8 core_link_user_get_car_relative_yaw_deg(float uav_yaw_deg,
                                              float *relative_yaw_deg);
uint8 core_link_user_is_screen_mirror_enabled (void);
uint8 core_link_user_consume_screen_mirror_enabled_changed (void);
uint8 core_link_user_consume_monitor_exit_request (void);
void core_link_user_send_drone_status_from_snapshot(const MonitorRuntimeSnapshot *snapshot);

#endif
