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
    uint16 sequence;
    uint16 age_ms;
    uint16 protocol_error_count;
    float forward_speed_cms;
    float left_speed_cms;
    float car_yaw_deg;
} CoreLinkCarMotionDiag_t;

typedef struct
{
    uint8 valid;
    uint8 version;
    uint8 flags;
    uint8 current_segment;
    uint8 predicted_beacon_source;
    uint8 predicted_beacon_id;
    uint16 sequence;
    uint16 plan_generation;
    uint16 age_ms;
    uint16 protocol_error_count;
    float car_yaw_deg;
    float actual_forward_speed_cms;
    float actual_left_speed_cms;
    float command_forward_speed_cms;
    float command_left_speed_cms;
    float planned_forward_acc_cms2;
    float planned_left_acc_cms2;
    float planned_forward_jerk_cms3;
    float planned_left_jerk_cms3;
    float trajectory_error_forward_cm;
    float trajectory_error_left_cm;
    float target_forward_cm;
    float target_left_cm;
    float predicted_beacon_forward_cm;
    float predicted_beacon_left_cm;
} CoreLinkCarTrajectoryDiag_t;

void core_link_user_get_beacon_diag(CoreLinkBeaconDiag_t *diag);
void core_link_user_get_car_motion_diag(CoreLinkCarMotionDiag_t *diag);
void core_link_user_get_car_trajectory_diag(CoreLinkCarTrajectoryDiag_t *diag);
uint8 core_link_user_is_screen_mirror_enabled (void);
uint8 core_link_user_consume_screen_mirror_enabled_changed (void);
uint8 core_link_user_consume_monitor_exit_request (void);
void core_link_user_send_drone_status_from_snapshot(const MonitorRuntimeSnapshot *snapshot);

#endif
