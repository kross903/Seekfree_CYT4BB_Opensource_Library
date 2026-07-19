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

void core_link_user_get_beacon_diag(CoreLinkBeaconDiag_t *diag);
uint8 core_link_user_is_screen_mirror_enabled (void);
uint8 core_link_user_consume_screen_mirror_enabled_changed (void);
uint8 core_link_user_consume_monitor_exit_request (void);
void core_link_user_send_drone_status_from_snapshot(const MonitorRuntimeSnapshot *snapshot);

#endif
