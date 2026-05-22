#include "vision/vision_runtime.h"

#include "vision/beacon_detect.h"
#include "vision/ipm_interface.h"
#include "vision/ir_localization.h"
#include "zf_device_mt9v03x.h"
#include <string.h>

static VisionRuntimeDiag_t s_vision_runtime_diag;

static void VisionRuntime_UpdateDiagFromCurrentFrame(int beacon_count,
                                                     const BeaconResult_t *beacon_results)
{
    IR_LossDebug_t ir_loss_debug = {0};
    int nearest_index = -1;

    IR_Localization_GetLossDebug(&ir_loss_debug);

    s_vision_runtime_diag.ir_raw_detect_valid = ir_loss_debug.raw_detect_valid;
    s_vision_runtime_diag.ir_track_valid = (uint8)((ir_pose_estimate.valid && !ir_position_lost_flag) ? 1U : 0U);
    s_vision_runtime_diag.ir_detected_point_count = ir_pose_estimate.detected_point_count;
    s_vision_runtime_diag.ir_x_cm = ir_pose_estimate.x;
    s_vision_runtime_diag.ir_y_cm = ir_pose_estimate.y;
    s_vision_runtime_diag.ir_square_side_cm = ir_pose_estimate.square_side_cm;
    s_vision_runtime_diag.ir_height_cm = ir_pose_estimate.z;

    s_vision_runtime_diag.beacon_count = beacon_count;
    s_vision_runtime_diag.beacon_has_nearest = 0U;
    s_vision_runtime_diag.beacon_nearest_x_cm = 0.0f;
    s_vision_runtime_diag.beacon_nearest_y_cm = 0.0f;
    s_vision_runtime_diag.beacon_nearest_distance_cm = 0.0f;
    s_vision_runtime_diag.beacon_nearest_area = 0.0f;
    s_vision_runtime_diag.beacon_nearest_brightness = 0.0f;
    s_vision_runtime_diag.beacon_nearest_occupancy = 0.0f;
    s_vision_runtime_diag.beacon_nearest_aspect = 0.0f;

    if(beacon_count > 0 && beacon_results != NULL)
    {
        nearest_index = BeaconDetect_GetNearestIndex(beacon_results, beacon_count);
        if(nearest_index >= 0)
        {
            const BeaconResult_t *nearest = &beacon_results[nearest_index];

            s_vision_runtime_diag.beacon_has_nearest = 1U;
            s_vision_runtime_diag.beacon_nearest_x_cm = nearest->world_x;
            s_vision_runtime_diag.beacon_nearest_y_cm = nearest->world_y;
            s_vision_runtime_diag.beacon_nearest_distance_cm = nearest->distance_cm;
            s_vision_runtime_diag.beacon_nearest_area = (float)nearest->area;
            s_vision_runtime_diag.beacon_nearest_brightness = nearest->average_brightness;
            s_vision_runtime_diag.beacon_nearest_occupancy = nearest->occupancy_ratio;
            s_vision_runtime_diag.beacon_nearest_aspect = nearest->aspect_ratio;
        }
    }
}

void VisionRuntime_ProcessFrame(float height_cm)
{
    static BeaconResult_t beacon_results[BEACON_MAX_DETECTIONS];
    int beacon_count = 0;

    IR_Localization_ServiceLostProtection();

    if(!mt9v03x_finish_flag)
    {
        return;
    }

    mt9v03x_finish_flag = 0;
    IPM_Interface_UpdateFromIMU(height_cm);
    IR_Localization_Process(mt9v03x_image, &ir_pose_estimate);
    beacon_count = BeaconDetect_Process(mt9v03x_image, beacon_results, BEACON_MAX_DETECTIONS);
    VisionRuntime_UpdateDiagFromCurrentFrame(beacon_count, beacon_results);
}

void VisionRuntime_GetDiag(VisionRuntimeDiag_t *diag)
{
    if(diag == NULL)
    {
        return;
    }

    memcpy(diag, &s_vision_runtime_diag, sizeof(VisionRuntimeDiag_t));
}
