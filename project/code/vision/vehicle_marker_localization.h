#ifndef VEHICLE_MARKER_LOCALIZATION_H_
#define VEHICLE_MARKER_LOCALIZATION_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VEHICLE_MARKER_POINT_COUNT             (4U)
#define VEHICLE_MARKER_POSITION_LOST_HOLD_MS   (150U)
#define VEHICLE_MARKER_TASK_PERIOD_MS          (5U)
#define VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM (17.0f)
#define VEHICLE_MARKER_FOLLOW_REAR_OFFSET_CM   (10.0f)

#define VEHICLE_MARKER_CHEVRON_REJECT_NONE          (0U)
#define VEHICLE_MARKER_CHEVRON_REJECT_NO_CANDIDATE  (1U)
#define VEHICLE_MARKER_CHEVRON_REJECT_CONTOUR       (2U)
#define VEHICLE_MARKER_CHEVRON_REJECT_YAW_DIRECTION (3U)

typedef struct { float u; float v; float intensity; } VehicleMarkerPoint2D_t;
typedef struct { float x; float y; } VehicleMarkerWorldPoint_t;

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

/* Layout retained for the existing display and cross-core diagnostics. */
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
    float mean_arm_cm;
    float arm_symmetry;
    float opening_ratio;
    float concavity_depth_px;
    float concavity_ratio;
    float arm_symmetry_score;
    float opening_ratio_score;
    float concavity_score;
    float basic_blob_score;
    float direction_expected_deg;
    float direction_error_deg;
    float direction_score;
    float geometry_score;
    float arm_angle_deg;
    float base_cm;
    float triangle_height_cm;
    float open_v_arm_support;
    float open_v_center_gap;
    float open_v_arm_continuity;
    float open_v_score;
    float open_v_hypothesis_score;
    float open_v_hypothesis_second_score;
    float open_v_hypothesis_margin;
    float open_v_gate_score;
    float open_v_gate_center_gap;
    uint8 open_v_apex_index;
    uint8 open_v_gate_from_primary;
    uint8 direction_prior_valid;
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
void VehicleMarkerLocalization_SetOpeningDirectionPrior(float car_yaw_rel_uav_deg,
                                                        uint8 valid);
void VehicleMarkerLocalization_BeginFrame(void);
uint8 VehicleMarkerLocalization_CommitContourArrow(
    const VehicleMarkerPoint2D_t *apex_img,
    const VehicleMarkerPoint2D_t *left_tip_img,
    const VehicleMarkerPoint2D_t *right_tip_img,
    float direction_error_deg,
    uint8 yaw_assisted);
void VehicleMarkerLocalization_EndFrame(void);
uint8 VehicleMarkerLocalization_GetFollowPoint(
    const VehicleMarkerPose_t *pose,
    float *filtered_x, float *filtered_y,
    float *raw_x, float *raw_y);
void VehicleMarkerLocalization_GetBlobRejectDebug(int *area_low_count,
                                                  int *area_high_count,
                                                  int *size_count,
                                                  int *aspect_count,
                                                  int *insert_count);

extern VehicleMarkerPose_t g_vehicle_marker_pose;
extern volatile uint8 g_vehicle_marker_position_lost_flag;
extern volatile uint16 g_vehicle_marker_position_lost_elapsed_ms;
extern volatile uint32 g_vehicle_marker_pose_update_seq;

#ifdef __cplusplus
}
#endif

#endif /* VEHICLE_MARKER_LOCALIZATION_H_ */
