#include "vision/vehicle_marker_localization.h"
#include "vision/ipm_interface.h"
#include <math.h>
#include <string.h>

#define MARKER_KALMAN_INITIAL_P (1.0f)
#define MARKER_KALMAN_Q         (0.10f)
#define MARKER_KALMAN_R         (6.0f)
#define MARKER_EPSILON          (1e-3f)

typedef struct
{
    float value;
    float covariance;
    uint8 initialized;
} MarkerFilter_t;

VehicleMarkerPose_t g_vehicle_marker_pose = {0};
volatile uint8 g_vehicle_marker_position_lost_flag = 1U;
volatile uint16 g_vehicle_marker_position_lost_elapsed_ms =
    VEHICLE_MARKER_POSITION_LOST_HOLD_MS;
volatile uint32 g_vehicle_marker_pose_update_seq = 0U;

static VehicleMarkerChevronDebug_t s_debug;
static VehicleMarkerLossDebug_t s_loss;
static MarkerFilter_t s_filter_x;
static MarkerFilter_t s_filter_y;
static float s_yaw_prior_deg;
static uint8 s_yaw_prior_valid;
static uint8 s_detected_this_frame;
static uint32 s_accept_count;

static void marker_filter_reset(MarkerFilter_t *filter)
{
    if(filter == NULL) return;
    filter->value = 0.0f;
    filter->covariance = MARKER_KALMAN_INITIAL_P;
    filter->initialized = 0U;
}

static float marker_filter_update(MarkerFilter_t *filter, float measurement)
{
    float gain;
    if(filter == NULL) return measurement;
    if(!filter->initialized)
    {
        filter->value = measurement;
        filter->covariance = MARKER_KALMAN_INITIAL_P;
        filter->initialized = 1U;
        return measurement;
    }
    filter->covariance += MARKER_KALMAN_Q;
    gain = filter->covariance / (filter->covariance + MARKER_KALMAN_R);
    filter->value += gain * (measurement - filter->value);
    filter->covariance *= 1.0f - gain;
    return filter->value;
}

static void marker_clear_track(void)
{
    g_vehicle_marker_pose.x = 0.0f;
    g_vehicle_marker_pose.y = 0.0f;
    g_vehicle_marker_pose.raw_x = 0.0f;
    g_vehicle_marker_pose.raw_y = 0.0f;
    g_vehicle_marker_pose.valid = 0U;
    g_vehicle_marker_pose.detected_point_count = 0U;
    s_loss.has_trackable_pose = 0U;
}

static void marker_mark_lost(void)
{
    g_vehicle_marker_position_lost_flag = 1U;
    g_vehicle_marker_position_lost_elapsed_ms =
        VEHICLE_MARKER_POSITION_LOST_HOLD_MS;
    s_loss.lost_flag = 1U;
    s_loss.lost_elapsed_ms = VEHICLE_MARKER_POSITION_LOST_HOLD_MS;
    ++s_loss.mark_lost_count;
    marker_filter_reset(&s_filter_x);
    marker_filter_reset(&s_filter_y);
    marker_clear_track();
}

void VehicleMarkerLocalization_Init(void)
{
    memset(&g_vehicle_marker_pose, 0, sizeof(g_vehicle_marker_pose));
    memset(&s_debug, 0, sizeof(s_debug));
    memset(&s_loss, 0, sizeof(s_loss));
    marker_filter_reset(&s_filter_x);
    marker_filter_reset(&s_filter_y);
    s_yaw_prior_deg = 0.0f;
    s_yaw_prior_valid = 1U;
    s_detected_this_frame = 0U;
    s_accept_count = 0U;
    g_vehicle_marker_pose_update_seq = 0U;
    marker_mark_lost();
}

void VehicleMarkerLocalization_Task5ms(void)
{
    uint32 elapsed;
    ++s_loss.task_5ms_count;
    if(!s_loss.has_trackable_pose) return;
    elapsed = (uint32)g_vehicle_marker_position_lost_elapsed_ms +
              VEHICLE_MARKER_TASK_PERIOD_MS;
    if(elapsed >= VEHICLE_MARKER_POSITION_LOST_HOLD_MS)
    {
        ++s_loss.timeout_count;
        marker_mark_lost();
        return;
    }
    g_vehicle_marker_position_lost_elapsed_ms = (uint16)elapsed;
    s_loss.lost_elapsed_ms = (uint16)elapsed;
}

void VehicleMarkerLocalization_ServiceLostProtection(void)
{
    if(g_vehicle_marker_position_lost_flag) marker_clear_track();
}

void VehicleMarkerLocalization_GetLossDebug(VehicleMarkerLossDebug_t *debug)
{
    if(debug == NULL) return;
    s_loss.lost_flag = g_vehicle_marker_position_lost_flag;
    s_loss.pose_valid = g_vehicle_marker_pose.valid;
    s_loss.lost_elapsed_ms = g_vehicle_marker_position_lost_elapsed_ms;
    *debug = s_loss;
}

void VehicleMarkerLocalization_SetLossDebug(const VehicleMarkerLossDebug_t *debug)
{
    if(debug == NULL) return;
    s_loss = *debug;
    g_vehicle_marker_position_lost_flag = debug->lost_flag;
    g_vehicle_marker_position_lost_elapsed_ms = debug->lost_elapsed_ms;
    g_vehicle_marker_pose.valid = debug->pose_valid;
}

void VehicleMarkerLocalization_GetChevronDebug(VehicleMarkerChevronDebug_t *debug)
{
    if(debug != NULL) *debug = s_debug;
}

void VehicleMarkerLocalization_SetOpeningDirectionPrior(float yaw_deg,
                                                        uint8 valid)
{
    s_yaw_prior_valid = (uint8)(valid && isfinite(yaw_deg));
    s_yaw_prior_deg = s_yaw_prior_valid ? yaw_deg : 0.0f;
}

void VehicleMarkerLocalization_BeginFrame(void)
{
    memset(&s_debug, 0, sizeof(s_debug));
    s_debug.reject_reason = VEHICLE_MARKER_CHEVRON_REJECT_NO_CANDIDATE;
    s_debug.direction_prior_valid = s_yaw_prior_valid;
    s_debug.direction_expected_deg = s_yaw_prior_deg;
    s_detected_this_frame = 0U;
    s_loss.raw_detect_valid = 0U;
    ++s_loss.process_count;
}

uint8 VehicleMarkerLocalization_CommitContourArrow(
    const VehicleMarkerPoint2D_t *apex_img,
    const VehicleMarkerPoint2D_t *left_tip_img,
    const VehicleMarkerPoint2D_t *right_tip_img,
    float direction_error_deg,
    uint8 yaw_assisted)
{
    VehicleMarkerWorldPoint_t apex;
    VehicleMarkerWorldPoint_t left;
    VehicleMarkerWorldPoint_t right;
    float raw_x;
    float raw_y;
    float rear_x;
    float rear_y;
    float base_x;
    float base_y;
    float left_dx;
    float left_dy;
    float right_dx;
    float right_dy;

    if(apex_img == NULL || left_tip_img == NULL || right_tip_img == NULL ||
       !IPM_FullRectifiedImage2PlaneF(apex_img->u, apex_img->v,
           VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM, &apex.x, &apex.y) ||
       !IPM_FullRectifiedImage2PlaneF(left_tip_img->u, left_tip_img->v,
           VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM, &left.x, &left.y) ||
       !IPM_FullRectifiedImage2PlaneF(right_tip_img->u, right_tip_img->v,
           VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM, &right.x, &right.y))
    {
        s_debug.reject_reason = VEHICLE_MARKER_CHEVRON_REJECT_CONTOUR;
        return 0U;
    }
    raw_x = (apex.x + left.x + right.x) / 3.0f;
    raw_y = (apex.y + left.y + right.y) / 3.0f;
    rear_x = 0.5f * (left.x + right.x);
    rear_y = 0.5f * (left.y + right.y);
    base_x = right.x - left.x;
    base_y = right.y - left.y;
    left_dx = apex.x - left.x;
    left_dy = apex.y - left.y;
    right_dx = apex.x - right.x;
    right_dy = apex.y - right.y;

    memset(&g_vehicle_marker_pose, 0, sizeof(g_vehicle_marker_pose));
    g_vehicle_marker_pose.raw_x = raw_x;
    g_vehicle_marker_pose.raw_y = raw_y;
    g_vehicle_marker_pose.x = marker_filter_update(&s_filter_x, raw_x);
    g_vehicle_marker_pose.y = marker_filter_update(&s_filter_y, raw_y);
    g_vehicle_marker_pose.z = VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM;
    g_vehicle_marker_pose.heading_deg =
        atan2f(apex.y - rear_y, apex.x - rear_x) *
        (180.0f / 3.14159265358979323846f);
    g_vehicle_marker_pose.square_side_cm = sqrtf(base_x * base_x + base_y * base_y);
    g_vehicle_marker_pose.img_pts[0] = *apex_img;
    g_vehicle_marker_pose.img_pts[1] = *left_tip_img;
    g_vehicle_marker_pose.img_pts[2] = *right_tip_img;
    g_vehicle_marker_pose.world_pts[0] = apex;
    g_vehicle_marker_pose.world_pts[1] = left;
    g_vehicle_marker_pose.world_pts[2] = right;
    g_vehicle_marker_pose.detected_point_count = 3U;
    g_vehicle_marker_pose.valid = 1U;

    memset(&s_debug, 0, sizeof(s_debug));
    s_debug.valid = 1U;
    s_debug.left_valid = 1U;
    s_debug.right_valid = 1U;
    s_debug.apex_img = *apex_img;
    s_debug.left_tip_img = *left_tip_img;
    s_debug.right_tip_img = *right_tip_img;
    s_debug.left_line_start_img = *left_tip_img;
    s_debug.left_line_end_img = *apex_img;
    s_debug.right_line_start_img = *apex_img;
    s_debug.right_line_end_img = *right_tip_img;
    s_debug.heading_deg = g_vehicle_marker_pose.heading_deg;
    s_debug.left_arm_cm = sqrtf(left_dx * left_dx + left_dy * left_dy);
    s_debug.right_arm_cm = sqrtf(right_dx * right_dx + right_dy * right_dy);
    s_debug.mean_arm_cm = 0.5f * (s_debug.left_arm_cm + s_debug.right_arm_cm);
    s_debug.base_cm = g_vehicle_marker_pose.square_side_cm;
    s_debug.triangle_height_cm = sqrtf(
        (apex.x - rear_x) * (apex.x - rear_x) +
        (apex.y - rear_y) * (apex.y - rear_y));
    s_debug.direction_expected_deg = s_yaw_prior_deg;
    s_debug.direction_error_deg = direction_error_deg;
    s_debug.direction_score = 1.0f - direction_error_deg / 180.0f;
    if(s_debug.direction_score < 0.0f) s_debug.direction_score = 0.0f;
    s_debug.geometry_score = 1.0f;
    s_debug.open_v_gate_from_primary = (uint8)!yaw_assisted;
    s_debug.direction_prior_valid = s_yaw_prior_valid;
    s_debug.candidate_count = 1U;
    s_debug.reject_reason = VEHICLE_MARKER_CHEVRON_REJECT_NONE;

    s_detected_this_frame = 1U;
    s_loss.raw_detect_valid = 1U;
    s_loss.has_trackable_pose = 1U;
    s_loss.pose_valid = 1U;
    s_loss.lost_flag = 0U;
    s_loss.lost_elapsed_ms = 0U;
    ++s_loss.process_success_count;
    ++s_accept_count;
    g_vehicle_marker_position_lost_flag = 0U;
    g_vehicle_marker_position_lost_elapsed_ms = 0U;
    ++g_vehicle_marker_pose_update_seq;
    return 1U;
}

void VehicleMarkerLocalization_EndFrame(void)
{
    if(!s_detected_this_frame)
    {
        ++s_loss.process_fail_count;
        s_debug.reject_reason = VEHICLE_MARKER_CHEVRON_REJECT_NO_CANDIDATE;
    }
}

uint8 VehicleMarkerLocalization_GetFollowPoint(
    const VehicleMarkerPose_t *pose,
    float *filtered_x, float *filtered_y,
    float *raw_x, float *raw_y)
{
    float rear_x;
    float rear_y;
    float length;
    if(pose == NULL || filtered_x == NULL || filtered_y == NULL ||
       ((raw_x == NULL) != (raw_y == NULL)) ||
       !pose->valid || pose->detected_point_count < 3U) return 0U;
    rear_x = 0.5f * (pose->world_pts[1].x + pose->world_pts[2].x) -
             pose->world_pts[0].x;
    rear_y = 0.5f * (pose->world_pts[1].y + pose->world_pts[2].y) -
             pose->world_pts[0].y;
    length = sqrtf(rear_x * rear_x + rear_y * rear_y);
    if(!isfinite(length) || length <= MARKER_EPSILON) return 0U;
    rear_x *= VEHICLE_MARKER_FOLLOW_REAR_OFFSET_CM / length;
    rear_y *= VEHICLE_MARKER_FOLLOW_REAR_OFFSET_CM / length;
    *filtered_x = pose->x + rear_x;
    *filtered_y = pose->y + rear_y;
    if(raw_x != NULL)
    {
        *raw_x = pose->raw_x + rear_x;
        *raw_y = pose->raw_y + rear_y;
    }
    return (uint8)(isfinite(*filtered_x) && isfinite(*filtered_y));
}

void VehicleMarkerLocalization_GetBlobRejectDebug(int *area_low_count,
                                                  int *area_high_count,
                                                  int *size_count,
                                                  int *aspect_count,
                                                  int *insert_count)
{
    if(area_low_count != NULL) *area_low_count = 0;
    if(area_high_count != NULL) *area_high_count = 0;
    if(size_count != NULL) *size_count = 0;
    if(aspect_count != NULL) *aspect_count = 0;
    if(insert_count != NULL) *insert_count = (int)s_accept_count;
}
