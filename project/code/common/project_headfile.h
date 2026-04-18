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
#include "common/PIT_Demo.h"
#include "common/shared_data.h"

// ================================================= Communication =================================================
#include "communication/core_link.h"
#include "communication/core_link_user.h"
#include "communication/cross_core_camera.h"
#include "communication/dshot600.h"
#include "communication/vofa.h"

// =================================================== Control ====================================================
#include "control/ControlMath.h"
#include "control/flight_control.h"
#include "control/joystick.h"
#include "control/pid_controller.h"

// ===================================================== ICM ======================================================
#include "icm/icm.h"
#include "icm/icm42688.h"

// ============================================== Kalman Filter ===================================================
#include "kalman_filter/gyro_kalman.h"
#include "kalman_filter/height_fusion.h"
#include "kalman_filter/height_manager.h"

// ==================================================== Menu ======================================================
#include "menu/key.h"
#include "menu/menu.h"
#include "menu/parameter_flash.h"
#include "menu/parameter_scheme.h"
#include "menu/screen_pages.h"

// ===================================================== PMW ======================================================
#include "pmw/pmw_flow.h"

// ==================================================== Vision ====================================================
#include "vision/beacon_detect.h"
#include "vision/drone_ipm.h"
#include "vision/drone_ipm_embedded.h"
#include "vision/ipm_interface.h"
#include "vision/ir_localization.h"

#endif
