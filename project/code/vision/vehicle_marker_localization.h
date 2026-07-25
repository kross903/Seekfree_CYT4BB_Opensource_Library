#ifndef VEHICLE_MARKER_LOCALIZATION_H_
#define VEHICLE_MARKER_LOCALIZATION_H_

#include "zf_common_headfile.h"
#include "vision/vision_runtime.h"

#ifdef __cplusplus
extern "C" {
#endif

// ==================== 配置参数 ====================
/* Pose storage keeps four slots for the existing cross-core data layout;
 * chevron recognition uses apex, left tip and right tip. */
#define VEHICLE_MARKER_POINT_COUNT             (4)
#define VEHICLE_MARKER_POSITION_LOST_HOLD_MS   (500U)
#define VEHICLE_MARKER_TASK_PERIOD_MS          (5U)
#define VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM (17.0f)

#define VEHICLE_MARKER_CHEVRON_REJECT_NONE              (0U)
#define VEHICLE_MARKER_CHEVRON_REJECT_NO_CANDIDATE      (1U)
#define VEHICLE_MARKER_CHEVRON_REJECT_IMAGE_GEOMETRY    (2U)
#define VEHICLE_MARKER_CHEVRON_REJECT_WORLD_PROJECTION  (3U)
#define VEHICLE_MARKER_CHEVRON_REJECT_PHYSICAL_GEOMETRY (4U)

typedef struct
{
    float u;
    float v;
    float intensity;
} VehicleMarkerPoint2D_t;

typedef struct
{
    float x;
    float y;
} VehicleMarkerWorldPoint_t;

typedef struct
{
    float x;
    float y;
    float raw_x;
    float raw_y;
    float z;
    float heading_deg;
    float square_side_cm;
    VehicleMarkerPoint2D_t img_pts[VEHICLE_MARKER_POINT_COUNT];
    VehicleMarkerWorldPoint_t world_pts[VEHICLE_MARKER_POINT_COUNT];
    uint8 detected_point_count;
    uint8 reconstructed_point;
    uint8 valid;
} VehicleMarkerPose_t;

typedef struct
{
    uint8 valid;
    uint8 left_valid;
    uint8 right_valid;
    VehicleMarkerPoint2D_t apex_img;
    VehicleMarkerPoint2D_t left_tip_img;
    VehicleMarkerPoint2D_t right_tip_img;
    VehicleMarkerPoint2D_t left_line_start_img;
    VehicleMarkerPoint2D_t left_line_end_img;
    VehicleMarkerPoint2D_t right_line_start_img;
    VehicleMarkerPoint2D_t right_line_end_img;
    float heading_deg;
    float physical_area_cm2;
    float physical_occupancy_ratio;
    float physical_circularity;
    float left_arm_cm;
    float right_arm_cm;
    float arm_angle_deg;
    float base_cm;
    float triangle_height_cm;
    uint8 candidate_count;
    uint8 geometry_reject_count;
    uint8 reject_reason;
} VehicleMarkerChevronDebug_t;

typedef struct
{
    uint8 raw_detect_valid;
    uint8 has_trackable_pose;
    uint8 lost_flag;
    uint8 pose_valid;
    uint16 lost_elapsed_ms;
    uint32 task_5ms_count;
    uint32 process_count;
    uint32 process_success_count;
    uint32 process_fail_count;
    uint32 mark_lost_count;
    uint32 timeout_count;
} VehicleMarkerLossDebug_t;

void VehicleMarkerLocalization_Init(void);
void VehicleMarkerLocalization_Task5ms(void);
void VehicleMarkerLocalization_ServiceLostProtection(void);
void VehicleMarkerLocalization_GetLossDebug(VehicleMarkerLossDebug_t *debug);
void VehicleMarkerLocalization_SetLossDebug(const VehicleMarkerLossDebug_t *debug);
void VehicleMarkerLocalization_GetChevronDebug(VehicleMarkerChevronDebug_t *debug);
void VehicleMarkerLocalization_GetBlobRejectDebug(int *area_low_count,
                                                  int *area_high_count,
                                                  int *size_count,
                                                  int *aspect_count,
                                                  int *insert_count);
void VehicleMarkerLocalization_BeginBlobFrame(void);
uint8 VehicleMarkerLocalization_AppendBlob(const VisionBlob_t *blob, const VisionContour_t *contour);
/* Appends the blob and performs the full chevron geometry check without
 * committing a pose. A true result is safe for candidate short-circuiting. */
uint8 VehicleMarkerLocalization_AppendAndValidateBlob(
    const VisionBlob_t *blob, const VisionContour_t *contour);
uint8 VehicleMarkerLocalization_EndBlobFrame(VehicleMarkerPose_t *out_pose);

extern VehicleMarkerPose_t g_vehicle_marker_pose;
extern volatile uint8 g_vehicle_marker_position_lost_flag;
extern volatile uint16 g_vehicle_marker_position_lost_elapsed_ms;
extern volatile uint32 g_vehicle_marker_pose_update_seq;

#ifdef __cplusplus
}
#endif

#endif /* VEHICLE_MARKER_LOCALIZATION_H_ */
