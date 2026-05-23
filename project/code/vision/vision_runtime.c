#include "vision/vision_runtime.h"

#include "vision/beacon_detect.h"
#include "vision/ipm_interface.h"
#include "vision/ir_localization.h"
#include "zf_device_mt9v03x.h"
#include <string.h>

static VisionRuntimeDiag_t s_vision_runtime_diag;
static uint8 s_vision_binary[MT9V03X_H * MT9V03X_W];
static uint8 s_vision_visited[MT9V03X_H * MT9V03X_W];
static uint16 s_vision_queue[MT9V03X_H * MT9V03X_W];

static float VisionRuntime_NormalizedAspect(int width, int height)
{
    float w = (float)width;
    float h = (float)height;

    if(w < 1.0f)
    {
        w = 1.0f;
    }
    if(h < 1.0f)
    {
        h = 1.0f;
    }

    return (w > h) ? (w / h) : (h / w);
}

static void VisionRuntime_Binarize(const uint8 image[MT9V03X_H][MT9V03X_W])
{
    int index = 0;

    for(index = 0; index < (MT9V03X_H * MT9V03X_W); index++)
    {
        s_vision_binary[index] = (((const uint8 *)image)[index] >= VISION_BINARY_THRESHOLD) ? 1U : 0U;
    }
}

static int VisionRuntime_GetScaledLargeAreaThreshold(void)
{
    int threshold = IR_Localization_GetScaledMaxBlobArea();

    if(threshold < g_beacon_config.area_threshold)
    {
        threshold = g_beacon_config.area_threshold;
    }

    return threshold;
}

static void VisionRuntime_DispatchBlob(const VisionBlob_t *blob, int large_area_threshold)
{
    if(blob == NULL)
    {
        return;
    }

    if(blob->area > large_area_threshold)
    {
        s_vision_runtime_diag.beacon_split_count++;
        if(BeaconDetect_AppendBlob(blob))
        {
            s_vision_runtime_diag.beacon_blob_count++;
        }
    }
    else
    {
        s_vision_runtime_diag.ir_split_count++;
        if(IR_Localization_AppendBlob(blob))
        {
            s_vision_runtime_diag.ir_blob_count++;
        }
    }
}

static void VisionRuntime_ProcessBlobs(const uint8 image[MT9V03X_H][MT9V03X_W])
{
    int row = 0;
    int col = 0;
    int large_area_threshold = VisionRuntime_GetScaledLargeAreaThreshold();

    memset(s_vision_visited, 0, sizeof(s_vision_visited));
    VisionRuntime_Binarize(image);

    for(row = 0; row < MT9V03X_H; row++)
    {
        for(col = 0; col < MT9V03X_W; col++)
        {
            int start_index = row * MT9V03X_W + col;
            int front = 0;
            int rear = 0;
            int pixel_count = 0;
            int row_sum = 0;
            int col_sum = 0;
            int brightness_sum = 0;
            int min_row = row;
            int max_row = row;
            int min_col = col;
            int max_col = col;

            if(s_vision_binary[start_index] == 0U || s_vision_visited[start_index])
            {
                continue;
            }

            s_vision_queue[rear++] = (uint16)start_index;
            s_vision_visited[start_index] = 1U;

            while(front < rear)
            {
                int idx = s_vision_queue[front++];
                int current_row = idx / MT9V03X_W;
                int current_col = idx % MT9V03X_W;
                int dr = 0;
                int dc = 0;

                pixel_count++;
                row_sum += current_row;
                col_sum += current_col;
                brightness_sum += image[current_row][current_col];

                if(current_row < min_row) min_row = current_row;
                if(current_row > max_row) max_row = current_row;
                if(current_col < min_col) min_col = current_col;
                if(current_col > max_col) max_col = current_col;

                for(dr = -1; dr <= 1; dr++)
                {
                    for(dc = -1; dc <= 1; dc++)
                    {
                        int next_row = 0;
                        int next_col = 0;
                        int next_index = 0;

                        if(dr == 0 && dc == 0)
                        {
                            continue;
                        }

                        next_row = current_row + dr;
                        next_col = current_col + dc;

                        if(next_row < 0 || next_row >= MT9V03X_H ||
                           next_col < 0 || next_col >= MT9V03X_W)
                        {
                            continue;
                        }

                        next_index = next_row * MT9V03X_W + next_col;
                        if(s_vision_binary[next_index] != 0U && !s_vision_visited[next_index])
                        {
                            s_vision_visited[next_index] = 1U;
                            if(rear < (MT9V03X_H * MT9V03X_W))
                            {
                                s_vision_queue[rear++] = (uint16)next_index;
                            }
                        }
                    }
                }
            }

            if(pixel_count > 0)
            {
                VisionBlob_t blob;
                int width = max_col - min_col + 1;
                int height = max_row - min_row + 1;

                s_vision_runtime_diag.blob_count++;
                memset(&blob, 0, sizeof(blob));
                blob.centroid_row = (float)row_sum / (float)pixel_count;
                blob.centroid_col = (float)col_sum / (float)pixel_count;
                blob.area = pixel_count;
                blob.min_row = min_row;
                blob.max_row = max_row;
                blob.min_col = min_col;
                blob.max_col = max_col;
                blob.average_brightness = (float)brightness_sum / (float)pixel_count;
                blob.aspect_ratio = VisionRuntime_NormalizedAspect(width, height);
                blob.occupancy_ratio = (width > 0 && height > 0) ?
                                       ((float)pixel_count / (float)(width * height)) : 0.0f;
                IPM_Image2World((int)blob.centroid_col, (int)blob.centroid_row,
                                &blob.world_x, &blob.world_y);
                VisionRuntime_DispatchBlob(&blob, large_area_threshold);
            }
        }
    }
}

static void VisionRuntime_UpdateDiagFromCurrentFrame(int beacon_count,
                                                     const BeaconResult_t *beacon_results)
{
    IR_LossDebug_t ir_loss_debug = {0};
    int nearest_index = -1;

    IR_Localization_GetLossDebug(&ir_loss_debug);
    IR_Localization_GetBlobRejectDebug(&s_vision_runtime_diag.ir_reject_area_low_count,
                                       &s_vision_runtime_diag.ir_reject_area_high_count,
                                       &s_vision_runtime_diag.ir_reject_size_count,
                                       &s_vision_runtime_diag.ir_reject_aspect_count,
                                       &s_vision_runtime_diag.ir_reject_insert_count);

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
    s_vision_runtime_diag.blob_count = 0;
    s_vision_runtime_diag.ir_split_count = 0;
    s_vision_runtime_diag.beacon_split_count = 0;
    s_vision_runtime_diag.ir_blob_count = 0;
    s_vision_runtime_diag.beacon_blob_count = 0;
    IR_Localization_BeginBlobFrame();
    BeaconDetect_BeginBlobFrame(beacon_results, BEACON_MAX_DETECTIONS);
    VisionRuntime_ProcessBlobs(mt9v03x_image);
    (void)IR_Localization_EndBlobFrame(&ir_pose_estimate);
    beacon_count = BeaconDetect_EndBlobFrame();
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
