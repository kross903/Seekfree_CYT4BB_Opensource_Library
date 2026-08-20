#ifndef _core_link_user_h_
#define _core_link_user_h_

#include "core_link.h"
#include "menu/screen_pages.h"

void core_link_user_init (void);
void core_link_user_task (void);
void core_link_user_poll (void);
void core_link_user_tick (uint16 elapsed_ms);
void core_link_user_motion_tick(uint16 elapsed_ms);
void core_link_user_service_beacon_link(void);
typedef struct
{
    uint8 valid;
    uint8 acceleration_valid;
    uint8 yaw_valid;
    uint8 search_requested;
    uint8 link_state;
    uint8 flags;
    uint16 sequence;
    uint16 age_ms;
    uint16 arrival_interval_ms;
    uint16 protocol_error_count;
    uint32 sample_time_ms;
    float raw_world_vx_cms;
    float raw_world_vy_cms;
    float raw_world_ax_cms2;
    float raw_world_ay_cms2;
    float world_vx_cms;
    float world_vy_cms;
    float world_ax_cms2;
    float world_ay_cms2;
    float car_yaw_deg;
} CoreLinkCarMotionDiag_t;

#define CORE_LINK_CAR_MOTION_STATE_INVALID (0U)
#define CORE_LINK_CAR_MOTION_STATE_FRESH   (1U)
#define CORE_LINK_CAR_MOTION_STATE_STALE   (2U)
void core_link_user_get_car_motion_diag(CoreLinkCarMotionDiag_t *diag);
uint8 core_link_user_is_screen_mirror_enabled (void);
uint8 core_link_user_consume_screen_mirror_enabled_changed (void);
uint8 core_link_user_consume_monitor_exit_request (void);
void core_link_user_send_drone_status_from_snapshot(const MonitorRuntimeSnapshot *snapshot);

#endif
