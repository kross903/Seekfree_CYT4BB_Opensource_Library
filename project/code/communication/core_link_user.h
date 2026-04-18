#ifndef _core_link_user_h_
#define _core_link_user_h_

#include "core_link.h"
#include "menu/screen_pages.h"

void core_link_user_init (void);
void core_link_user_task (void);
void core_link_user_poll (void);
uint8 core_link_user_consume_monitor_exit_request (void);
void core_link_user_send_drone_status_from_snapshot(const MonitorRuntimeSnapshot *snapshot);

#endif
