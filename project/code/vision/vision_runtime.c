#include "vision/vision_runtime.h"
#include <string.h>
#include <math.h>

static VisionRuntimeDiag_t s_vision_runtime_diag;
static VisionRuntimeBeaconSlot_t s_vision_runtime_beacon_slots[VISION_RUNTIME_BEACON_SLOT_COUNT];
static uint8 s_vision_binary[MT9V03X_H * MT9V03X_W];
static uint8 s_vision_binary_thresholds[VISION_BINARY_ZONE_COUNT] = {
    (uint8)VISION_BINARY_THRESHOLD_INNER,
    (uint8)VISION_BINARY_THRESHOLD_MIDDLE,
    (uint8)VISION_BINARY_THRESHOLD_OUTER};

void VisionRuntime_SetBinaryThreshold(uint8 threshold)
{
    uint8 value = (threshold == 0U) ? 1U : threshold;
    VisionRuntime_SetBinaryThresholds(value, value, value);
}

void VisionRuntime_SetBinaryThresholds(uint8 inner, uint8 middle, uint8 outer)
{
    s_vision_binary_thresholds[VISION_BINARY_ZONE_INNER] =
        (inner == 0U) ? 1U : inner;
    s_vision_binary_thresholds[VISION_BINARY_ZONE_MIDDLE] =
        (middle == 0U) ? 1U : middle;
    s_vision_binary_thresholds[VISION_BINARY_ZONE_OUTER] =
        (outer == 0U) ? 1U : outer;
}

void VisionRuntime_GetBinaryThresholds(uint8 thresholds[VISION_BINARY_ZONE_COUNT])
{
    if(thresholds != NULL)
    {
        memcpy(thresholds, s_vision_binary_thresholds,
               sizeof(s_vision_binary_thresholds));
    }
}

#if CY_CORE_CM7_1
#include "vision/beacon_detect.h"
#include "vision/camera_calibration.h"
#include "vision/ipm_interface.h"
#include "vision/vehicle_marker_localization.h"
#include "zf_device_mt9v03x.h"

#define VISION_RUNTIME_IMAGE_PIXEL_COUNT \
    (MT9V03X_H * MT9V03X_W)
#define VISION_RUNTIME_IMAGE_BITSET_BYTES \
    ((VISION_RUNTIME_IMAGE_PIXEL_COUNT + 7U) / 8U)
#define VISION_RUNTIME_LOCAL_BIRDVIEW_PIXEL_COUNT \
    (VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_WIDTH * VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_HEIGHT)
#define VISION_RUNTIME_LOCAL_BIRDVIEW_BITSET_BYTES \
    ((VISION_RUNTIME_LOCAL_BIRDVIEW_PIXEL_COUNT + 7U) / 8U)
#define VISION_RUNTIME_BIT_GET(buffer, index) \
    (((buffer)[(index) >> 3] >> ((index) & 7)) & 1U)
#define VISION_RUNTIME_BIT_SET(buffer, index) \
    ((buffer)[(index) >> 3] |= (uint8)(1U << ((index) & 7)))
#define VISION_RUNTIME_BIT_CLEAR(buffer, index) \
    ((buffer)[(index) >> 3] &= (uint8)~(uint8)(1U << ((index) & 7)))

static uint8 s_vision_visited[VISION_RUNTIME_IMAGE_BITSET_BYTES];
static uint16 s_vision_queue[MT9V03X_H * MT9V03X_W];
static uint8 s_vision_component_mask[VISION_RUNTIME_IMAGE_BITSET_BYTES];
static uint8 s_vision_local_birdview_mask[VISION_RUNTIME_LOCAL_BIRDVIEW_BITSET_BYTES];
static uint8 s_vision_undistorted[MT9V03X_H][MT9V03X_W];

static float VisionRuntime_NormalizedAspect(int width, int height);

static void VisionRuntime_ClearComponentMask(int pixel_count)
{
    int i;

    if(pixel_count < 0)
    {
        pixel_count = 0;
    }
    if(pixel_count > (MT9V03X_H * MT9V03X_W))
    {
        pixel_count = MT9V03X_H * MT9V03X_W;
    }

    for(i = 0; i < pixel_count; ++i)
    {
        VISION_RUNTIME_BIT_CLEAR(s_vision_component_mask, s_vision_queue[i]);
    }
}

/*
 * 将当前连通域投影到固定物理比例的局部鸟瞰栅格。
 * 栅格中的每个前景单元代表 SCALE^2 cm^2，面积不使用鞋带公式。
 */
static void VisionRuntime_BuildPhysicalBlobMetrics(VisionBlob_t *blob, int pixel_count)
{
    const float scale = VISION_RUNTIME_LOCAL_BIRDVIEW_SCALE_CM_PER_PIXEL;
    float min_x = 0.0f;
    float max_x = 0.0f;
    float min_y = 0.0f;
    float max_y = 0.0f;
    int valid_points = 0;
    int local_width;
    int local_height;
    int i;

    if(blob == NULL || pixel_count <= 0 || scale <= 0.0f)
    {
        return;
    }

    for(i = 0; i < pixel_count; ++i)
    {
        int index = (int)s_vision_queue[i];
        int row = index / MT9V03X_W;
        int col = index % MT9V03X_W;
        float world_x = 0.0f;
        float world_y = 0.0f;

        if(!IPM_RectifiedImage2WorldF((float)col, (float)row, &world_x, &world_y))
        {
            continue;
        }

        if(valid_points == 0)
        {
            min_x = max_x = world_x;
            min_y = max_y = world_y;
        }
        else
        {
            if(world_x < min_x) min_x = world_x;
            if(world_x > max_x) max_x = world_x;
            if(world_y < min_y) min_y = world_y;
            if(world_y > max_y) max_y = world_y;
        }
        valid_points++;
    }

    if(valid_points <= 0 || valid_points * 2 < pixel_count)
    {
        return;
    }

    /* 为每个原图像素的投影足迹预留至少一个物理栅格单元。 */
    min_x -= scale;
    max_x += scale;
    min_y -= scale;
    max_y += scale;

    local_width = (int)ceilf((max_y - min_y) / scale) + 1;
    local_height = (int)ceilf((max_x - min_x) / scale) + 1;
    if(local_width < 1) local_width = 1;
    if(local_height < 1) local_height = 1;

    if(local_width > (int)VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_WIDTH ||
       local_height > (int)VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_HEIGHT)
    {
        return;
    }

    memset(s_vision_local_birdview_mask, 0,
           sizeof(s_vision_local_birdview_mask));

    /* 先按鸟瞰图的逆采样方向读取当前连通域，形成物理比例栅格。 */
    {
        int local_y;
        int local_x;

        for(local_y = 0; local_y < local_height; ++local_y)
        {
            for(local_x = 0; local_x < local_width; ++local_x)
            {
                float world_x = min_x + ((float)local_y + 0.5f) * scale;
                float world_y = max_y - ((float)local_x + 0.5f) * scale;
                int src_x = 0;
                int src_y = 0;

                if(IPM_World2RectifiedImage(world_x, world_y, &src_x, &src_y) &&
                   VISION_RUNTIME_BIT_GET(s_vision_component_mask,
                                          src_y * MT9V03X_W + src_x) != 0U)
                {
                    VISION_RUNTIME_BIT_SET(
                        s_vision_local_birdview_mask,
                        local_y * VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_WIDTH + local_x);
                }
            }
        }
    }

    /* 按原图像素四角的投影足迹补栅格，避免高度较高时局部图出现空洞。 */
    for(i = 0; i < pixel_count; ++i)
    {
        int index = (int)s_vision_queue[i];
        int row = index / MT9V03X_W;
        int col = index % MT9V03X_W;
        const float corner_x[4] = {
            (float)col - 0.5f, (float)col + 0.5f,
            (float)col - 0.5f, (float)col + 0.5f};
        const float corner_y[4] = {
            (float)row - 0.5f, (float)row - 0.5f,
            (float)row + 0.5f, (float)row + 0.5f};
        float footprint_min_x = 0.0f;
        float footprint_max_x = 0.0f;
        float footprint_min_y = 0.0f;
        float footprint_max_y = 0.0f;
        int footprint_valid = 0;
        int corner;

        for(corner = 0; corner < 4; ++corner)
        {
            float world_x = 0.0f;
            float world_y = 0.0f;

            if(!IPM_RectifiedImage2WorldF(corner_x[corner], corner_y[corner],
                                         &world_x, &world_y))
            {
                continue;
            }
            if(footprint_valid == 0)
            {
                footprint_min_x = footprint_max_x = world_x;
                footprint_min_y = footprint_max_y = world_y;
            }
            else
            {
                if(world_x < footprint_min_x) footprint_min_x = world_x;
                if(world_x > footprint_max_x) footprint_max_x = world_x;
                if(world_y < footprint_min_y) footprint_min_y = world_y;
                if(world_y > footprint_max_y) footprint_max_y = world_y;
            }
            footprint_valid++;
        }

        if(footprint_valid == 0)
        {
            continue;
        }

        {
            int min_local_x = (int)floorf((footprint_min_y - min_y) / scale);
            int max_local_x = (int)floorf((footprint_max_y - min_y) / scale);
            int min_local_y = (int)floorf((max_x - footprint_max_x) / scale);
            int max_local_y = (int)floorf((max_x - footprint_min_x) / scale);
            int local_y;
            int local_x;

            if(min_local_x < 0) min_local_x = 0;
            if(min_local_y < 0) min_local_y = 0;
            if(max_local_x >= local_width) max_local_x = local_width - 1;
            if(max_local_y >= local_height) max_local_y = local_height - 1;
            for(local_y = min_local_y; local_y <= max_local_y; ++local_y)
            {
                for(local_x = min_local_x; local_x <= max_local_x; ++local_x)
                {
                    VISION_RUNTIME_BIT_SET(
                        s_vision_local_birdview_mask,
                        local_y * VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_WIDTH + local_x);
                }
            }
        }
    }

    {
        uint32 occupied = 0U;
        uint32 exposed_edges = 0U;
        int row;
        int col;

        for(row = 0; row < local_height; ++row)
        {
            for(col = 0; col < local_width; ++col)
            {
                int index = row * VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_WIDTH + col;
                int neighbor;

                if(VISION_RUNTIME_BIT_GET(s_vision_local_birdview_mask, index) == 0U)
                {
                    continue;
                }
                occupied++;

                neighbor = (row > 0) ? index - (int)VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_WIDTH : -1;
                if(neighbor < 0 || VISION_RUNTIME_BIT_GET(s_vision_local_birdview_mask, neighbor) == 0U) exposed_edges++;
                neighbor = (row + 1 < local_height) ? index + (int)VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_WIDTH : -1;
                if(neighbor < 0 || VISION_RUNTIME_BIT_GET(s_vision_local_birdview_mask, neighbor) == 0U) exposed_edges++;
                neighbor = (col > 0) ? index - 1 : -1;
                if(neighbor < 0 || VISION_RUNTIME_BIT_GET(s_vision_local_birdview_mask, neighbor) == 0U) exposed_edges++;
                neighbor = (col + 1 < local_width) ? index + 1 : -1;
                if(neighbor < 0 || VISION_RUNTIME_BIT_GET(s_vision_local_birdview_mask, neighbor) == 0U) exposed_edges++;
            }
        }

        if(occupied == 0U || exposed_edges == 0U)
        {
            return;
        }

        blob->physical_pixel_count = (occupied > 65535U) ? 65535U : (uint16)occupied;
        blob->physical_area_cm2 = (float)occupied * scale * scale;
        blob->physical_perimeter_cm = (float)exposed_edges * scale;
        blob->physical_circularity =
            (4.0f * 3.14159265f * blob->physical_area_cm2) /
            (blob->physical_perimeter_cm * blob->physical_perimeter_cm);
        if(blob->physical_circularity > 1.0f)
        {
            blob->physical_circularity = 1.0f;
        }
        blob->physical_aspect_ratio = VisionRuntime_NormalizedAspect(local_width, local_height);
        blob->physical_occupancy_ratio =
            (float)occupied / (float)(local_width * local_height);
        blob->physical_valid = 1U;
    }
}

static void VisionRuntime_InsertBlobAreaRank(int area)
{
    uint8 index = 0U;

    if(area <= 0)
    {
        return;
    }

    while(index < VISION_RUNTIME_BLOB_AREA_COUNT)
    {
        if(area > s_vision_runtime_diag.blob_area_rank[index])
        {
            uint8 move_index = (uint8)(VISION_RUNTIME_BLOB_AREA_COUNT - 1U);

            while(move_index > index)
            {
                s_vision_runtime_diag.blob_area_rank[move_index] =
                    s_vision_runtime_diag.blob_area_rank[move_index - 1U];
                move_index--;
            }

            s_vision_runtime_diag.blob_area_rank[index] = area;
            break;
        }

        index++;
    }
}

static void VisionRuntime_ClearBeaconSlots(void)
{
    memset(s_vision_runtime_beacon_slots, 0, sizeof(s_vision_runtime_beacon_slots));
}

static void VisionRuntime_UpdateBeaconSlotsFromCurrentFrame(int beacon_count,
                                                            const BeaconResult_t *beacon_results)
{
    uint8 slot_count = 0U;
    uint8 selected[BEACON_MAX_DETECTIONS] = {0};

    VisionRuntime_ClearBeaconSlots();

    if(beacon_count <= 0 || beacon_results == NULL)
    {
        return;
    }

    while(slot_count < VISION_RUNTIME_BEACON_SLOT_COUNT)
    {
        int best_index = -1;
        float best_distance = 0.0f;
        int i;

        for(i = 0; i < beacon_count; ++i)
        {
            if(selected[i])
            {
                continue;
            }

            if(best_index < 0 || beacon_results[i].distance_cm < best_distance)
            {
                best_index = i;
                best_distance = beacon_results[i].distance_cm;
            }
        }

        if(best_index < 0)
        {
            break;
        }

        selected[best_index] = 1U;
        s_vision_runtime_beacon_slots[slot_count].valid = 1U;
        s_vision_runtime_beacon_slots[slot_count].x_cm = beacon_results[best_index].world_x;
        s_vision_runtime_beacon_slots[slot_count].y_cm = beacon_results[best_index].world_y;
        s_vision_runtime_beacon_slots[slot_count].min_row = beacon_results[best_index].min_row;
        s_vision_runtime_beacon_slots[slot_count].max_row = beacon_results[best_index].max_row;
        s_vision_runtime_beacon_slots[slot_count].min_col = beacon_results[best_index].min_col;
        s_vision_runtime_beacon_slots[slot_count].max_col = beacon_results[best_index].max_col;
        s_vision_runtime_beacon_slots[slot_count].area_cm2 = beacon_results[best_index].physical_area_cm2;
        s_vision_runtime_beacon_slots[slot_count].perimeter_cm = beacon_results[best_index].physical_perimeter_cm;
        s_vision_runtime_beacon_slots[slot_count].circularity = beacon_results[best_index].physical_circularity;
        s_vision_runtime_beacon_slots[slot_count].physical_pixel_count = beacon_results[best_index].physical_pixel_count;
        s_vision_runtime_beacon_slots[slot_count].physical_valid = beacon_results[best_index].physical_valid;
        slot_count++;
    }
}

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
    const float inner_ratio_sq = 0.40f * 0.40f;
    const float middle_ratio_sq = 0.72f * 0.72f;
    const sint32 center_x_q8 = (sint32)(CAMERA_CALIBRATION_CX * 256.0f + 0.5f);
    const sint32 center_y_q8 = (sint32)(CAMERA_CALIBRATION_CY * 256.0f + 0.5f);
    const sint32 far_x_q8 = (center_x_q8 >
        (((sint32)MT9V03X_W - 1) * 256 - center_x_q8)) ?
        center_x_q8 : (((sint32)MT9V03X_W - 1) * 256 - center_x_q8);
    const sint32 far_y_q8 = (center_y_q8 >
        (((sint32)MT9V03X_H - 1) * 256 - center_y_q8)) ?
        center_y_q8 : (((sint32)MT9V03X_H - 1) * 256 - center_y_q8);
    const uint32 max_radius_sq =
        (uint32)(far_x_q8 * far_x_q8) + (uint32)(far_y_q8 * far_y_q8);
    const uint32 inner_radius_sq = (uint32)((float)max_radius_sq * inner_ratio_sq);
    const uint32 middle_radius_sq = (uint32)((float)max_radius_sq * middle_ratio_sq);
    int row;
    int col;

    IPM_GenerateUndistortedDebugImage(image, s_vision_undistorted);
    for(row = 0; row < MT9V03X_H; row++)
    {
        for(col = 0; col < MT9V03X_W; col++)
        {
            uint16 source_x_q8 = 0U;
            uint16 source_y_q8 = 0U;
            uint8 threshold;
            int index = row * MT9V03X_W + col;

            if(!CameraCalibration_RectifiedPixelToSourceQ8(col, row,
                                                           &source_x_q8,
                                                           &source_y_q8))
            {
                s_vision_binary[index] = 0U;
                continue;
            }

            {
                sint32 dx = (sint32)source_x_q8 - center_x_q8;
                sint32 dy = (sint32)source_y_q8 - center_y_q8;
                uint32 radius_sq = (uint32)(dx * dx) + (uint32)(dy * dy);

                if(radius_sq <= inner_radius_sq)
                {
                    threshold = s_vision_binary_thresholds[VISION_BINARY_ZONE_INNER];
                }
                else if(radius_sq <= middle_radius_sq)
                {
                    threshold = s_vision_binary_thresholds[VISION_BINARY_ZONE_MIDDLE];
                }
                else
                {
                    threshold = s_vision_binary_thresholds[VISION_BINARY_ZONE_OUTER];
                }
            }
            s_vision_binary[index] =
                (s_vision_undistorted[row][col] >= threshold) ? 255U : 0U;
        }
    }
}

static int VisionRuntime_GetScaledLargeAreaThreshold(void)
{
    return VISION_RUNTIME_BEACON_SPLIT_AREA_THRESHOLD;
}

static uint8 VisionRuntime_IsBoundaryPixel(int row, int col)
{
    int dr = 0;
    int dc = 0;

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

            next_row = row + dr;
            next_col = col + dc;
            if(next_row < 0 || next_row >= MT9V03X_H ||
               next_col < 0 || next_col >= MT9V03X_W)
            {
                return 1U;
            }

            next_index = next_row * MT9V03X_W + next_col;
            if(s_vision_binary[next_index] == 0U)
            {
                return 1U;
            }
        }
    }

    return 0U;
}

static void VisionRuntime_DispatchBlob(const VisionBlob_t *blob,
                                       const VisionContour_t *contour,
                                       int large_area_threshold)
{
    if(blob == NULL)
    {
        return;
    }

#if VEHICLE_MARKER_MODE == VEHICLE_MARKER_MODE_CHEVRON
    if(blob->area > large_area_threshold)
    {
        s_vision_runtime_diag.beacon_split_count++;
        if(BeaconDetect_AppendBlob(blob))
        {
            s_vision_runtime_diag.beacon_blob_count++;
            return;
        }
    }

    s_vision_runtime_diag.ir_split_count++;
    if(VehicleMarkerLocalization_AppendBlob(blob, contour))
    {
        s_vision_runtime_diag.ir_blob_count++;
    }
#else
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
        if(VehicleMarkerLocalization_AppendBlob(blob, contour))
        {
            s_vision_runtime_diag.ir_blob_count++;
        }
    }
#endif
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

            if(s_vision_binary[start_index] == 0U ||
               VISION_RUNTIME_BIT_GET(s_vision_visited, start_index))
            {
                continue;
            }

            s_vision_queue[rear++] = (uint16)start_index;
            VISION_RUNTIME_BIT_SET(s_vision_visited, start_index);
            VISION_RUNTIME_BIT_SET(s_vision_component_mask, start_index);

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
                brightness_sum += s_vision_undistorted[current_row][current_col];

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
                        if(s_vision_binary[next_index] != 0U &&
                           !VISION_RUNTIME_BIT_GET(s_vision_visited, next_index))
                        {
                            VISION_RUNTIME_BIT_SET(s_vision_visited, next_index);
                            VISION_RUNTIME_BIT_SET(s_vision_component_mask, next_index);
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
                VisionContour_t contour;
                int width = max_col - min_col + 1;
                int height = max_row - min_row + 1;
                int boundary_count = 0;
                int i = 0;

                s_vision_runtime_diag.blob_count++;
                VisionRuntime_InsertBlobAreaRank(pixel_count);
                memset(&blob, 0, sizeof(blob));
                memset(&contour, 0, sizeof(contour));
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
                for(i = 0; i < rear; ++i)
                {
                    int current_index = (int)s_vision_queue[i];
                    int current_row = current_index / MT9V03X_W;
                    int current_col = current_index % MT9V03X_W;

                    if(!VisionRuntime_IsBoundaryPixel(current_row, current_col))
                    {
                        continue;
                    }

                    if(boundary_count < (int)VISION_RUNTIME_CONTOUR_MAX_POINTS)
                    {
                        contour.point_x[boundary_count] = (uint16)current_col;
                        contour.point_y[boundary_count] = (uint16)current_row;
                    }
                    boundary_count++;
                }
                contour.point_count = (uint16)((boundary_count > (int)VISION_RUNTIME_CONTOUR_MAX_POINTS) ?
                                               (int)VISION_RUNTIME_CONTOUR_MAX_POINTS : boundary_count);
                if(boundary_count > 0)
                {
                    float perimeter = (float)boundary_count;

                    blob.circularity = (4.0f * 3.14159265f * (float)pixel_count) / (perimeter * perimeter);
                    if(blob.circularity > 1.0f)
                    {
                        blob.circularity = 1.0f;
                    }
                }
                else
                {
                    blob.circularity = 0.0f;
                }
                VisionRuntime_BuildPhysicalBlobMetrics(&blob, rear);
                blob.world_valid = IPM_RectifiedImage2WorldF(blob.centroid_col,
                                                             blob.centroid_row,
                                                             &blob.world_x,
                                                             &blob.world_y);
                if(blob.world_valid)
                {
                    VisionRuntime_DispatchBlob(&blob, &contour, large_area_threshold);
                }
                VisionRuntime_ClearComponentMask(rear);
            }
        }
    }
}

static void VisionRuntime_UpdateDiagFromCurrentFrame(int beacon_count,
                                                     const BeaconResult_t *beacon_results)
{
    VehicleMarkerLossDebug_t ir_loss_debug = {0};
    int nearest_index = -1;

    VehicleMarkerLocalization_GetLossDebug(&ir_loss_debug);
    VehicleMarkerLocalization_GetBlobRejectDebug(&s_vision_runtime_diag.ir_reject_area_low_count,
                                       &s_vision_runtime_diag.ir_reject_area_high_count,
                                       &s_vision_runtime_diag.ir_reject_size_count,
                                       &s_vision_runtime_diag.ir_reject_aspect_count,
                                       &s_vision_runtime_diag.ir_reject_insert_count);

    s_vision_runtime_diag.ir_raw_detect_valid = ir_loss_debug.raw_detect_valid;
    s_vision_runtime_diag.ir_track_valid = (uint8)((g_vehicle_marker_pose.valid && !g_vehicle_marker_position_lost_flag) ? 1U : 0U);
    s_vision_runtime_diag.ir_detected_point_count = g_vehicle_marker_pose.detected_point_count;
    s_vision_runtime_diag.ir_x_cm = g_vehicle_marker_pose.x;
    s_vision_runtime_diag.ir_y_cm = g_vehicle_marker_pose.y;
    s_vision_runtime_diag.ir_square_side_cm = g_vehicle_marker_pose.square_side_cm;
    s_vision_runtime_diag.ir_height_cm = g_vehicle_marker_pose.z;

    s_vision_runtime_diag.beacon_count = beacon_count;
    s_vision_runtime_diag.beacon_has_nearest = 0U;
    s_vision_runtime_diag.beacon_nearest_x_cm = 0.0f;
    s_vision_runtime_diag.beacon_nearest_y_cm = 0.0f;
    s_vision_runtime_diag.beacon_nearest_distance_cm = 0.0f;
    s_vision_runtime_diag.beacon_nearest_area = 0.0f;
    s_vision_runtime_diag.beacon_nearest_area_cm2 = 0.0f;
    s_vision_runtime_diag.beacon_nearest_perimeter_cm = 0.0f;
    s_vision_runtime_diag.beacon_nearest_circularity = 0.0f;
    s_vision_runtime_diag.beacon_nearest_brightness = 0.0f;
    s_vision_runtime_diag.beacon_nearest_occupancy = 0.0f;
    s_vision_runtime_diag.beacon_nearest_aspect = 0.0f;
    VisionRuntime_UpdateBeaconSlotsFromCurrentFrame(beacon_count, beacon_results);

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
            s_vision_runtime_diag.beacon_nearest_area_cm2 = nearest->physical_area_cm2;
            s_vision_runtime_diag.beacon_nearest_perimeter_cm = nearest->physical_perimeter_cm;
            s_vision_runtime_diag.beacon_nearest_circularity = nearest->physical_circularity;
            s_vision_runtime_diag.beacon_nearest_brightness = nearest->average_brightness;
            s_vision_runtime_diag.beacon_nearest_occupancy = nearest->occupancy_ratio;
            s_vision_runtime_diag.beacon_nearest_aspect = nearest->aspect_ratio;
        }
    }
}

static void VisionRuntime_ProcessFrameInternal(float roll_deg,
                                               float pitch_deg,
                                               float height_cm,
                                               uint8 use_explicit_pose)
{
    static BeaconResult_t beacon_results[BEACON_MAX_DETECTIONS];
    int beacon_count = 0;

    VehicleMarkerLocalization_ServiceLostProtection();

    if(!mt9v03x_finish_flag)
    {
        return;
    }

    mt9v03x_finish_flag = 0;
    if(use_explicit_pose)
    {
        IPM_Interface_Update(roll_deg, pitch_deg, height_cm);
    }
    else
    {
        IPM_Interface_UpdateFromIMU(height_cm);
    }
    s_vision_runtime_diag.blob_count = 0;
    s_vision_runtime_diag.ir_split_count = 0;
    s_vision_runtime_diag.beacon_split_count = 0;
    s_vision_runtime_diag.ir_blob_count = 0;
    s_vision_runtime_diag.beacon_blob_count = 0;
    memset(s_vision_runtime_diag.blob_area_rank, 0, sizeof(s_vision_runtime_diag.blob_area_rank));
#if VEHICLE_MARKER_MODE == VEHICLE_MARKER_MODE_CHEVRON
    VehicleMarkerLocalization_BeginBlobFrame();
#endif
    BeaconDetect_BeginBlobFrame(beacon_results, BEACON_MAX_DETECTIONS);
    VisionRuntime_ProcessBlobs(mt9v03x_image);
#if VEHICLE_MARKER_MODE == VEHICLE_MARKER_MODE_CHEVRON
    (void)VehicleMarkerLocalization_EndBlobFrame(&g_vehicle_marker_pose);
#else
    (void)VehicleMarkerLocalization_ProcessBinary(VisionRuntime_GetBinaryImage(),
                                                  &g_vehicle_marker_pose);
#endif
    beacon_count = BeaconDetect_EndBlobFrame();
    VisionRuntime_UpdateDiagFromCurrentFrame(beacon_count, beacon_results);
}

void VisionRuntime_ProcessFrame(float height_cm)
{
    VisionRuntime_ProcessFrameInternal(0.0f, 0.0f, height_cm, 0U);
}

void VisionRuntime_ProcessFrameWithPose(float roll_deg, float pitch_deg, float height_cm)
{
    VisionRuntime_ProcessFrameInternal(roll_deg, pitch_deg, height_cm, 1U);
}
#else
void VisionRuntime_ProcessFrame(float height_cm)
{
    (void)height_cm;
}

void VisionRuntime_ProcessFrameWithPose(float roll_deg, float pitch_deg, float height_cm)
{
    (void)roll_deg;
    (void)pitch_deg;
    (void)height_cm;
}
#endif

const uint8 *VisionRuntime_GetBinaryImage(void)
{
    return s_vision_binary;
}

const uint8 *VisionRuntime_GetBinaryDisplayImage(void)
{
    return s_vision_binary;
}

const uint8 *VisionRuntime_GetUndistortedImage(void)
{
#if CY_CORE_CM7_1
    return &s_vision_undistorted[0][0];
#else
    return NULL;
#endif
}

void VisionRuntime_GetDiag(VisionRuntimeDiag_t *diag)
{
    if(diag == NULL)
    {
        return;
    }

    memcpy(diag, &s_vision_runtime_diag, sizeof(VisionRuntimeDiag_t));
}

void VisionRuntime_SetDiag(const VisionRuntimeDiag_t *diag)
{
    if(diag == NULL)
    {
        return;
    }

    memcpy(&s_vision_runtime_diag, diag, sizeof(VisionRuntimeDiag_t));
}

void VisionRuntime_GetBeaconSlots(VisionRuntimeBeaconSlot_t *slots, uint8 max_slots)
{
    uint8 copy_count;

    if(slots == NULL || max_slots == 0U)
    {
        return;
    }

    copy_count = (max_slots < VISION_RUNTIME_BEACON_SLOT_COUNT) ? max_slots : VISION_RUNTIME_BEACON_SLOT_COUNT;
    memcpy(slots, s_vision_runtime_beacon_slots, sizeof(VisionRuntimeBeaconSlot_t) * copy_count);
}

void VisionRuntime_SetBeaconSlots(const VisionRuntimeBeaconSlot_t *slots, uint8 max_slots)
{
    uint8 copy_count;

    memset(s_vision_runtime_beacon_slots, 0, sizeof(s_vision_runtime_beacon_slots));
    if(slots == NULL || max_slots == 0U)
    {
        return;
    }

    copy_count = (max_slots < VISION_RUNTIME_BEACON_SLOT_COUNT) ? max_slots : VISION_RUNTIME_BEACON_SLOT_COUNT;
    memcpy(s_vision_runtime_beacon_slots, slots, sizeof(VisionRuntimeBeaconSlot_t) * copy_count);
}
