#ifndef PROJECT_HEADFILE_H_
#define PROJECT_HEADFILE_H_

#include "zf_common_headfile.h"

/*
 * Project-wide aggregation header for the application layer.
 * Keep public project headers reachable from here so new modules have
 * a single place to extend when exposing application-level interfaces.
 */

// ===================================================== Core =====================================================
#include "common/All_Init.h"
#include "common/battery_monitor.h"
#include "common/flight_safety.h"
#include "common/PIT_Demo.h"
#include "common/shared_data.h"

// ================================================= Communication =================================================
#include "communication/comm_mux.h"
#include "communication/camera_board_link.h"
#include "communication/core_link.h"
#include "communication/core_link_user.h"
#include "communication/cross_core_camera.h"
#include "communication/dshot600.h"
#include "communication/vofa.h"

// =================================================== Control ====================================================
#include "control/ControlMath.h"
#include "control/beacon_search.h"
#include "control/dual_tof_manager.h"
#include "control/flight_auto_land.h"
#include "control/angle_loop.h"
#include "control/flight_control.h"
#include "control/ground_remote_input.h"
#include "control/motor_mixer.h"
#include "control/pid_controller.h"
#include "control/position_loop.h"
#include "control/rate_loop.h"
#include "control/throttle_runtime.h"
#include "control/velocity_loop.h"

// ===================================================== ICM ======================================================
#include "icm/icm.h"
#include "icm/icm42688.h"

// ============================================== Height Control ==================================================
#include "control/height_manager.h"

// ==================================================== Menu ======================================================
#include "menu/key.h"
#include "menu/menu.h"
#include "menu/menu_fly_test.h"
#include "menu/menu_motor_test.h"
#include "menu/parameter_flash.h"
#include "menu/parameter_scheme.h"
#include "menu/screen_pages.h"

// =============================================== Motor Hardware =================================================
#include "motor_hardware/motor_hardware.h"

// ===================================================== PMW ======================================================
#include "pmw/optical_flow_feed.h"
#include "pmw/pmw_flow.h"

// ==================================================== Vision ====================================================
#include "vision/beacon_detect.h"
#include "vision/drone_ipm.h"
#include "vision/drone_ipm_embedded.h"
#include "vision/ipm_interface.h"
#include "vision/vehicle_marker_localization.h"
#include "vision/vision_overlay.h"
#include "vision/vision_runtime.h"

#endif
