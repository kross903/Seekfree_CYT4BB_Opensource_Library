#include "vision/vision_runtime.h"
#include "vision/camera_calibration.h"
#include <string.h>
#include <math.h>

static VisionRuntimeDiag_t s_vision_runtime_diag;
static VisionRuntimeBeaconSlot_t s_vision_runtime_beacon_slots[VISION_RUNTIME_BEACON_SLOT_COUNT];
static uint8 s_vision_binary[VISION_RUNTIME_BINARY_BYTES];
static uint32 s_vision_rle_overflow_count;
static uint8 s_vision_vehicle_threshold =
    (uint8)VISION_BINARY_THRESHOLD_VEHICLE;
static uint8 s_vision_beacon_threshold =
    (uint8)VISION_BINARY_THRESHOLD_BEACON;
static uint8 s_vision_coarse_threshold = (uint8)VISION_RUNTIME_COARSE_THRESHOLD;

void VisionRuntime_SetBinaryThreshold(uint8 threshold)
{
    uint8 value = (threshold == 0U) ? 1U : threshold;
    VisionRuntime_SetRecognitionThresholds(value, value);
}

void VisionRuntime_SetCoarseThreshold(uint8 threshold)
{
    s_vision_coarse_threshold = (threshold == 0U) ? 1U : threshold;
}

uint8 VisionRuntime_GetCoarseThreshold(void)
{
    return s_vision_coarse_threshold;
}

void VisionRuntime_SetBinaryThresholds(uint8 inner, uint8 middle, uint8 outer)
{
    (void)outer;
    VisionRuntime_SetRecognitionThresholds(inner, middle);
}

void VisionRuntime_GetBinaryThresholds(uint8 thresholds[VISION_BINARY_ZONE_COUNT])
{
    if(thresholds != NULL)
    {
        thresholds[VISION_BINARY_ZONE_INNER] = s_vision_vehicle_threshold;
        thresholds[VISION_BINARY_ZONE_MIDDLE] = s_vision_beacon_threshold;
        thresholds[VISION_BINARY_ZONE_OUTER] = s_vision_beacon_threshold;
    }
}

void VisionRuntime_SetRecognitionThresholds(uint8 vehicle, uint8 beacon)
{
    s_vision_vehicle_threshold = (vehicle == 0U) ? 1U : vehicle;
    s_vision_beacon_threshold = (beacon == 0U) ? 1U : beacon;
}

void VisionRuntime_GetRecognitionThresholds(uint8 *vehicle, uint8 *beacon)
{
    if(vehicle != NULL) *vehicle = s_vision_vehicle_threshold;
    if(beacon != NULL) *beacon = s_vision_beacon_threshold;
}

#if CY_CORE_CM7_1
#include "vision/beacon_detect.h"
#include "vision/camera_calibration.h"
#include "vision/ipm_interface.h"
#include "vision/vehicle_marker_localization.h"
#include "zf_device_mt9v03x.h"

#define VISION_RUNTIME_IMAGE_PIXEL_COUNT \
    (MT9V03X_H * MT9V03X_W)
#define VISION_RUNTIME_IMAGE_STRIDE \
    ((MT9V03X_W + 7U) / 8U)
#define VISION_RUNTIME_IMAGE_BITSET_BYTES \
    (VISION_RUNTIME_IMAGE_STRIDE * MT9V03X_H)
#define VISION_RUNTIME_LOCAL_BIRDVIEW_PIXEL_COUNT \
    (VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_WIDTH * VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_HEIGHT)
#define VISION_RUNTIME_LOCAL_BIRDVIEW_BITSET_BYTES \
    ((VISION_RUNTIME_LOCAL_BIRDVIEW_PIXEL_COUNT + 7U) / 8U)
#define VISION_RUNTIME_LOCAL_BIT_GET(buffer, index) \
    (((buffer)[(index) >> 3] >> ((index) & 7)) & 1U)
#define VISION_RUNTIME_LOCAL_BIT_SET(buffer, index) \
    ((buffer)[(index) >> 3] |= (uint8)(1U << ((index) & 7)))
#define VISION_RUNTIME_LOCAL_BIT_CLEAR(buffer, index) \
    ((buffer)[(index) >> 3] &= (uint8)~(uint8)(1U << ((index) & 7)))

#define VISION_RUNTIME_IMAGE_BYTE(index) \
    (((index) / MT9V03X_W) * VISION_RUNTIME_IMAGE_STRIDE + \
     (((index) % MT9V03X_W) >> 3))
#define VISION_RUNTIME_IMAGE_MASK(index) \
    ((uint8)(0x80U >> ((index) % MT9V03X_W & 7)))
#define VISION_RUNTIME_IMAGE_BIT_GET(buffer, index) \
    (((buffer)[VISION_RUNTIME_IMAGE_BYTE(index)] & \
      VISION_RUNTIME_IMAGE_MASK(index)) != 0U)
#define VISION_RUNTIME_IMAGE_BIT_SET(buffer, index) \
    ((buffer)[VISION_RUNTIME_IMAGE_BYTE(index)] |= \
     VISION_RUNTIME_IMAGE_MASK(index))
#define VISION_RUNTIME_IMAGE_BIT_CLEAR(buffer, index) \
    ((buffer)[VISION_RUNTIME_IMAGE_BYTE(index)] &= \
     (uint8)~VISION_RUNTIME_IMAGE_MASK(index))

#define VISION_RUNTIME_FINE_BYTE(x, y) \
    ((y) * VISION_RUNTIME_FINE_STRIDE + ((x) >> 3))
#define VISION_RUNTIME_FINE_MASK(x) \
    ((uint8)(0x80U >> ((x) & 7)))
#define VISION_RUNTIME_FINE_BIT_GET(buffer, x, y) \
    (((buffer)[VISION_RUNTIME_FINE_BYTE((x), (y))] & \
      VISION_RUNTIME_FINE_MASK(x)) != 0U)
#define VISION_RUNTIME_FINE_BIT_SET(buffer, x, y) \
    ((buffer)[VISION_RUNTIME_FINE_BYTE((x), (y))] |= \
     VISION_RUNTIME_FINE_MASK(x))
#define VISION_RUNTIME_FINE_BIT_CLEAR(buffer, x, y) \
    ((buffer)[VISION_RUNTIME_FINE_BYTE((x), (y))] &= \
     (uint8)~VISION_RUNTIME_FINE_MASK(x))

static uint8 s_vision_visited[VISION_RUNTIME_IMAGE_BITSET_BYTES];
static uint8 s_vision_fine_visited[VISION_RUNTIME_FINE_BYTES];
static uint8 s_vision_fine_component_mask[VISION_RUNTIME_FINE_BYTES];
static uint8 s_vision_local_birdview_mask[VISION_RUNTIME_LOCAL_BIRDVIEW_BITSET_BYTES];
static uint8 s_vision_undistorted[MT9V03X_H][MT9V03X_W];

static float VisionRuntime_NormalizedAspect(int width, int height);

#if 0
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
        VISION_RUNTIME_IMAGE_BIT_CLEAR(s_vision_component_mask, s_vision_queue[i]);
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
                   VISION_RUNTIME_IMAGE_BIT_GET(s_vision_component_mask,
                                          src_y * MT9V03X_W + src_x) != 0U)
                {
                    VISION_RUNTIME_LOCAL_BIT_SET(
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
                    VISION_RUNTIME_LOCAL_BIT_SET(
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

                if(VISION_RUNTIME_LOCAL_BIT_GET(s_vision_local_birdview_mask, index) == 0U)
                {
                    continue;
                }
                occupied++;

                neighbor = (row > 0) ? index - (int)VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_WIDTH : -1;
                if(neighbor < 0 || VISION_RUNTIME_LOCAL_BIT_GET(s_vision_local_birdview_mask, neighbor) == 0U) exposed_edges++;
                neighbor = (row + 1 < local_height) ? index + (int)VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_WIDTH : -1;
                if(neighbor < 0 || VISION_RUNTIME_LOCAL_BIT_GET(s_vision_local_birdview_mask, neighbor) == 0U) exposed_edges++;
                neighbor = (col > 0) ? index - 1 : -1;
                if(neighbor < 0 || VISION_RUNTIME_LOCAL_BIT_GET(s_vision_local_birdview_mask, neighbor) == 0U) exposed_edges++;
                neighbor = (col + 1 < local_width) ? index + 1 : -1;
                if(neighbor < 0 || VISION_RUNTIME_LOCAL_BIT_GET(s_vision_local_birdview_mask, neighbor) == 0U) exposed_edges++;
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
        s_vision_runtime_beacon_slots[slot_count].raw_min_row = beacon_results[best_index].raw_min_row;
        s_vision_runtime_beacon_slots[slot_count].raw_max_row = beacon_results[best_index].raw_max_row;
        s_vision_runtime_beacon_slots[slot_count].raw_min_col = beacon_results[best_index].raw_min_col;
        s_vision_runtime_beacon_slots[slot_count].raw_max_col = beacon_results[best_index].raw_max_col;
        s_vision_runtime_beacon_slots[slot_count].area_cm2 = beacon_results[best_index].physical_area_cm2;
        s_vision_runtime_beacon_slots[slot_count].perimeter_cm = beacon_results[best_index].physical_perimeter_cm;
        s_vision_runtime_beacon_slots[slot_count].circularity = beacon_results[best_index].physical_circularity;
        s_vision_runtime_beacon_slots[slot_count].physical_pixel_count = beacon_results[best_index].physical_pixel_count;
        s_vision_runtime_beacon_slots[slot_count].physical_valid = beacon_results[best_index].physical_valid;
        s_vision_runtime_beacon_slots[slot_count].raw_box_area_pixels =
            beacon_results[best_index].raw_box_area_pixels;
        s_vision_runtime_beacon_slots[slot_count].detection_confidence =
            beacon_results[best_index].detection_confidence;
        s_vision_runtime_beacon_slots[slot_count].confidence_box_size =
            beacon_results[best_index].confidence_box_size;
        s_vision_runtime_beacon_slots[slot_count].confidence_stability =
            beacon_results[best_index].confidence_stability;
        s_vision_runtime_beacon_slots[slot_count].confidence_area =
            beacon_results[best_index].confidence_area;
        s_vision_runtime_beacon_slots[slot_count].confidence_edge =
            beacon_results[best_index].confidence_edge;
        slot_count++;
    }
}

#endif

/* These helpers are shared by the full-resolution connected-component path.
 * Keep the legacy 188x120 implementation disabled above. */
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

static void VisionRuntime_UpdateBeaconSlotsFromCurrentFrame(
    int beacon_count, const BeaconResult_t *beacon_results)
{
    uint8 slot_count = 0U;
    uint8 selected[BEACON_MAX_DETECTIONS] = {0};
    int selectable_count;

    VisionRuntime_ClearBeaconSlots();

    if(beacon_count <= 0 || beacon_results == NULL)
    {
        return;
    }

    selectable_count = beacon_count;
    if(selectable_count > BEACON_MAX_DETECTIONS)
    {
        selectable_count = BEACON_MAX_DETECTIONS;
    }

    while(slot_count < VISION_RUNTIME_BEACON_SLOT_COUNT)
    {
        int best_index = -1;
        float best_distance = 0.0f;
        int i;

        for(i = 0; i < selectable_count; ++i)
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
        s_vision_runtime_beacon_slots[slot_count].raw_min_row = beacon_results[best_index].raw_min_row;
        s_vision_runtime_beacon_slots[slot_count].raw_max_row = beacon_results[best_index].raw_max_row;
        s_vision_runtime_beacon_slots[slot_count].raw_min_col = beacon_results[best_index].raw_min_col;
        s_vision_runtime_beacon_slots[slot_count].raw_max_col = beacon_results[best_index].raw_max_col;
        s_vision_runtime_beacon_slots[slot_count].area_cm2 = beacon_results[best_index].physical_area_cm2;
        s_vision_runtime_beacon_slots[slot_count].perimeter_cm = beacon_results[best_index].physical_perimeter_cm;
        s_vision_runtime_beacon_slots[slot_count].circularity = beacon_results[best_index].physical_circularity;
        s_vision_runtime_beacon_slots[slot_count].physical_pixel_count = beacon_results[best_index].physical_pixel_count;
        s_vision_runtime_beacon_slots[slot_count].physical_valid = beacon_results[best_index].physical_valid;
        s_vision_runtime_beacon_slots[slot_count].raw_box_area_pixels =
            beacon_results[best_index].raw_box_area_pixels;
        s_vision_runtime_beacon_slots[slot_count].detection_confidence =
            beacon_results[best_index].detection_confidence;
        s_vision_runtime_beacon_slots[slot_count].confidence_box_size =
            beacon_results[best_index].confidence_box_size;
        s_vision_runtime_beacon_slots[slot_count].confidence_stability =
            beacon_results[best_index].confidence_stability;
        s_vision_runtime_beacon_slots[slot_count].confidence_area =
            beacon_results[best_index].confidence_area;
        s_vision_runtime_beacon_slots[slot_count].confidence_edge =
            beacon_results[best_index].confidence_edge;
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

#define VISION_RUNTIME_MAX_CANDIDATES (16U)
#define VISION_RUNTIME_CANDIDATE_MIN_AREA (10)
#define VISION_RUNTIME_CANDIDATE_MARGIN (2)

typedef struct
{
    int area;
    int min_row;
    int max_row;
    int min_col;
    int max_col;
} VisionRuntimeCandidate_t;

typedef struct
{
    uint8 vehicle_valid;
    uint8 beacon_valid;
    int raw_box_area_pixels;
    uint8 detection_confidence;
    float beacon_x_cm;
    float beacon_y_cm;
    float beacon_area_cm2;
    float beacon_circularity;
    float confidence_box_size;
    float confidence_stability;
    float confidence_area;
    float confidence_edge;
} VisionRuntimeCandidateAnnotation_t;

static uint8 s_vision_coarse_binary[VISION_RUNTIME_IMAGE_BITSET_BYTES];
static VisionRuntimeCandidate_t s_vision_candidates[VISION_RUNTIME_MAX_CANDIDATES];
static VisionRuntimeCandidateAnnotation_t
    s_vision_candidate_annotations[VISION_RUNTIME_MAX_CANDIDATES];
static uint8 s_vision_candidate_count;
typedef struct
{
    uint16 row;
    uint16 start_col;
    uint16 end_col;
} VisionRuntimeFineRun_t;

static VisionRuntimeFineRun_t
    s_vision_fine_runs[VISION_RUNTIME_RLE_MAX_RUNS];

static void VisionRuntime_GetFineCandidateBounds(
    const VisionRuntimeCandidate_t *candidate,
    int *min_x, int *max_x, int *min_y, int *max_y)
{
    float min_rect_x = 0.0f;
    float max_rect_x = 0.0f;
    float min_rect_y = 0.0f;
    float max_rect_y = 0.0f;
    int corner;
    const int raw_x[4] = {0, 0, 1, 1};
    const int raw_y[4] = {0, 1, 0, 1};

    if(candidate == NULL || min_x == NULL || max_x == NULL ||
       min_y == NULL || max_y == NULL)
    {
        return;
    }
    for(corner = 0; corner < 4; ++corner)
    {
        float source_x = (float)(raw_x[corner] ? candidate->max_col : candidate->min_col);
        float source_y = (float)(raw_y[corner] ? candidate->max_row : candidate->min_row);
        float rect_x = source_x;
        float rect_y = source_y;

        if(!CameraCalibration_SourcePointToRectified(
               source_x, source_y, &rect_x, &rect_y))
        {
            rect_x = source_x;
            rect_y = source_y;
        }
        if(corner == 0 || rect_x < min_rect_x) min_rect_x = rect_x;
        if(corner == 0 || rect_x > max_rect_x) max_rect_x = rect_x;
        if(corner == 0 || rect_y < min_rect_y) min_rect_y = rect_y;
        if(corner == 0 || rect_y > max_rect_y) max_rect_y = rect_y;
    }
    *min_x = (int)floorf(min_rect_x * 2.0f) - 4;
    *max_x = (int)ceilf(max_rect_x * 2.0f) + 4;
    *min_y = (int)floorf(min_rect_y * 2.0f) - 4;
    *max_y = (int)ceilf(max_rect_y * 2.0f) + 4;
    if(*min_x < 0) *min_x = 0;
    if(*min_y < 0) *min_y = 0;
    if(*max_x >= (int)VISION_RUNTIME_FINE_WIDTH)
        *max_x = VISION_RUNTIME_FINE_WIDTH - 1;
    if(*max_y >= (int)VISION_RUNTIME_FINE_HEIGHT)
        *max_y = VISION_RUNTIME_FINE_HEIGHT - 1;
}

static void VisionRuntime_BinarizeCoarse(const uint8 image[MT9V03X_H][MT9V03X_W])
{
    int row;
    int col;

    memset(s_vision_coarse_binary, 0, sizeof(s_vision_coarse_binary));
    for(row = 0; row < MT9V03X_H; ++row)
    {
        for(col = 0; col < MT9V03X_W; ++col)
        {
            int index = row * MT9V03X_W + col;
            if(image[row][col] >= s_vision_coarse_threshold)
            {
                VISION_RUNTIME_IMAGE_BIT_SET(s_vision_coarse_binary, index);
            }
        }
    }
}

static uint8 VisionRuntime_CoarseIsForeground(int x, int y)
{
    if(x < 0 || x >= MT9V03X_W || y < 0 || y >= MT9V03X_H) return 0U;
    return (uint8)VISION_RUNTIME_IMAGE_BIT_GET(
        s_vision_coarse_binary, y * MT9V03X_W + x);
}

static uint8 VisionRuntime_CoarseAppendRun(int row, int seed_col,
                                           int min_col, int max_col,
                                           uint16 *run_count)
{
    int start_col = seed_col;
    int end_col = seed_col;
    int x;

    if(run_count == NULL || *run_count >= VISION_RUNTIME_RLE_MAX_RUNS)
    {
        ++s_vision_rle_overflow_count;
        return 0U;
    }
    while(start_col > min_col &&
          VisionRuntime_CoarseIsForeground(start_col - 1, row) &&
          !VISION_RUNTIME_IMAGE_BIT_GET(s_vision_visited,
                                        row * MT9V03X_W + start_col - 1))
    {
        --start_col;
    }
    while(end_col < max_col &&
          VisionRuntime_CoarseIsForeground(end_col + 1, row) &&
          !VISION_RUNTIME_IMAGE_BIT_GET(s_vision_visited,
                                        row * MT9V03X_W + end_col + 1))
    {
        ++end_col;
    }
    for(x = start_col; x <= end_col; ++x)
    {
        VISION_RUNTIME_IMAGE_BIT_SET(s_vision_visited,
                                     row * MT9V03X_W + x);
    }
    s_vision_fine_runs[*run_count].row = (uint16)row;
    s_vision_fine_runs[*run_count].start_col = (uint16)start_col;
    s_vision_fine_runs[*run_count].end_col = (uint16)end_col;
    ++(*run_count);
    return 1U;
}

static uint8 VisionRuntime_CoarseAppendNeighborRuns(
    const VisionRuntimeFineRun_t *run, int neighbor_row,
    int min_col, int max_col, uint16 *run_count)
{
    int scan_col;
    int begin_col;
    int end_col;

    if(run == NULL || run_count == NULL ||
       neighbor_row < 0 || neighbor_row >= MT9V03X_H)
    {
        return 1U;
    }
    begin_col = (int)run->start_col - 1;
    end_col = (int)run->end_col + 1;
    if(begin_col < min_col) begin_col = min_col;
    if(end_col > max_col) end_col = max_col;
    scan_col = begin_col;
    while(scan_col <= end_col)
    {
        int index = neighbor_row * MT9V03X_W + scan_col;
        if(VisionRuntime_CoarseIsForeground(scan_col, neighbor_row) &&
           !VISION_RUNTIME_IMAGE_BIT_GET(s_vision_visited, index))
        {
            if(!VisionRuntime_CoarseAppendRun(neighbor_row, scan_col,
                                               min_col, max_col, run_count))
            {
                return 0U;
            }
            scan_col = (int)s_vision_fine_runs[*run_count - 1U].end_col + 1;
        }
        else
        {
            ++scan_col;
        }
    }
    return 1U;
}

static uint8 VisionRuntime_FindCoarseCandidates(void)
{
    int row;
    int col;
    uint8 candidate_count = 0U;

    memset(s_vision_visited, 0, sizeof(s_vision_visited));
    memset(s_vision_candidates, 0, sizeof(s_vision_candidates));
    memset(s_vision_candidate_annotations, 0,
           sizeof(s_vision_candidate_annotations));
    for(row = 0; row < MT9V03X_H; ++row)
    {
        for(col = 0; col < MT9V03X_W; ++col)
        {
            int start_index = row * MT9V03X_W + col;
            uint16 run_count = 0U;
            int front = 0;
            int area = 0;
            int min_row = row;
            int max_row = row;
            int min_col = col;
            int max_col = col;
            uint8 complete = 1U;

            if(!VISION_RUNTIME_IMAGE_BIT_GET(s_vision_coarse_binary, start_index) ||
               VISION_RUNTIME_IMAGE_BIT_GET(s_vision_visited, start_index))
            {
                continue;
            }
            if(!VisionRuntime_CoarseAppendRun(row, col, 0, MT9V03X_W - 1,
                                              &run_count))
            {
                continue;
            }
            while(front < (int)run_count)
            {
                VisionRuntimeFineRun_t run = s_vision_fine_runs[front++];
                int run_length = (int)run.end_col - (int)run.start_col + 1;
                area += run_length;
                {
                    int current_row = (int)run.row;
                if(current_row < min_row) min_row = current_row;
                if(current_row > max_row) max_row = current_row;
                if((int)run.start_col < min_col) min_col = run.start_col;
                if((int)run.end_col > max_col) max_col = run.end_col;
                }
                if(!VisionRuntime_CoarseAppendNeighborRuns(
                       &run, (int)run.row - 1, 0, MT9V03X_W - 1, &run_count) ||
                   !VisionRuntime_CoarseAppendNeighborRuns(
                       &run, (int)run.row + 1, 0, MT9V03X_W - 1, &run_count))
                {
                    complete = 0U;
                    break;
                }
            }
            if(complete && area >= VISION_RUNTIME_CANDIDATE_MIN_AREA)
            {
                int slot = (int)candidate_count;

                if(candidate_count >= VISION_RUNTIME_MAX_CANDIDATES)
                {
                    uint8 i;
                    slot = 0;
                    for(i = 1U; i < candidate_count; ++i)
                    {
                        if(s_vision_candidates[i].area <
                           s_vision_candidates[slot].area) slot = (int)i;
                    }
                    if(area <= s_vision_candidates[slot].area) continue;
                }
                else
                {
                    ++candidate_count;
                }
                s_vision_candidates[slot].area = area;
                s_vision_candidates[slot].min_row = min_row;
                s_vision_candidates[slot].max_row = max_row;
                s_vision_candidates[slot].min_col = min_col;
                s_vision_candidates[slot].max_col = max_col;
            }
        }
    }
    s_vision_candidate_count = candidate_count;
    return candidate_count;
}

#if 0
static void VisionRuntime_RefineCandidateUndistorted(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    const VisionRuntimeCandidate_t *candidate)
{
    const float inner_ratio_sq = 0.40f * 0.40f;
    const float middle_ratio_sq = 0.72f * 0.72f;
    const sint32 center_x_q8 = (sint32)(CAMERA_CALIBRATION_CX * 256.0f + 0.5f);
    const sint32 center_y_q8 = (sint32)(CAMERA_CALIBRATION_CY * 256.0f + 0.5f);
    const sint32 far_x_q8 = (center_x_q8 >
        (((sint32)MT9V03X_W - 1) * 256 - center_x_q8)) ? center_x_q8 :
        (((sint32)MT9V03X_W - 1) * 256 - center_x_q8);
    const sint32 far_y_q8 = (center_y_q8 >
        (((sint32)MT9V03X_H - 1) * 256 - center_y_q8)) ? center_y_q8 :
        (((sint32)MT9V03X_H - 1) * 256 - center_y_q8);
    const uint32 max_radius_sq = (uint32)(far_x_q8 * far_x_q8) +
                                 (uint32)(far_y_q8 * far_y_q8);
    const uint32 inner_radius_sq = (uint32)((float)max_radius_sq * inner_ratio_sq);
    const uint32 middle_radius_sq = (uint32)((float)max_radius_sq * middle_ratio_sq);
    int min_row;
    int max_row;
    int min_col;
    int max_col;
    int row;
    int col;

    if(candidate == NULL) return;
    min_row = candidate->min_row - VISION_RUNTIME_CANDIDATE_MARGIN;
    max_row = candidate->max_row + VISION_RUNTIME_CANDIDATE_MARGIN;
    min_col = candidate->min_col - VISION_RUNTIME_CANDIDATE_MARGIN;
    max_col = candidate->max_col + VISION_RUNTIME_CANDIDATE_MARGIN;
    if(min_row < 0) min_row = 0;
    if(min_col < 0) min_col = 0;
    if(max_row >= MT9V03X_H) max_row = MT9V03X_H - 1;
    if(max_col >= MT9V03X_W) max_col = MT9V03X_W - 1;
    for(row = min_row; row <= max_row; ++row)
    {
        for(col = min_col; col <= max_col; ++col)
        {
            uint8 foreground_count = 0U;
            int sub_row;
            int sub_col;
            int index = row * MT9V03X_W + col;
            for(sub_row = 0; sub_row < 2; ++sub_row)
            {
                for(sub_col = 0; sub_col < 2; ++sub_col)
                {
                    uint16 source_x_q8 = 0U;
                    uint16 source_y_q8 = 0U;
                    uint8 sample = 0U;
                    uint8 threshold;

                    if(!IPM_SampleFullUndistortedGrayQ8(
                           image, col * 2 + sub_col, row * 2 + sub_row,
                           &source_x_q8, &source_y_q8, &sample)) continue;
                    threshold = VisionRuntime_GetZoneThreshold(
                        source_x_q8, source_y_q8, center_x_q8, center_y_q8,
                        inner_radius_sq, middle_radius_sq);
                    if(sample >= threshold) ++foreground_count;
                }
            }
            if(foreground_count >= 2U)
            {
                VISION_RUNTIME_IMAGE_BIT_SET(s_vision_binary, index);
            }
        }
    }
}

#endif

#if 0
static void VisionRuntime_RefineCandidateIPM(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    const VisionRuntimeCandidate_t *candidate)
{
    const float scale = VISION_RUNTIME_LOCAL_BIRDVIEW_SCALE_CM_PER_PIXEL;
    float min_world_x = 0.0f;
    float max_world_x = 0.0f;
    float min_world_y = 0.0f;
    float max_world_y = 0.0f;
    int local_width;
    int local_height;
    int valid = 0;
    int corner;
    int corner_x[4];
    int corner_y[4];

    if(candidate == NULL || scale <= 0.0f) return;
    corner_x[0] = candidate->min_col;
    corner_x[1] = candidate->max_col;
    corner_x[2] = candidate->min_col;
    corner_x[3] = candidate->max_col;
    corner_y[0] = candidate->min_row;
    corner_y[1] = candidate->min_row;
    corner_y[2] = candidate->max_row;
    corner_y[3] = candidate->max_row;
    for(corner = 0; corner < 4; ++corner)
    {
        float world_x;
        float world_y;

        if(!IPM_RectifiedImage2WorldF((float)corner_x[corner],
                                      (float)corner_y[corner],
                                      &world_x, &world_y)) continue;
        if(!valid)
        {
            min_world_x = max_world_x = world_x;
            min_world_y = max_world_y = world_y;
            valid = 1;
        }
        else
        {
            if(world_x < min_world_x) min_world_x = world_x;
            if(world_x > max_world_x) max_world_x = world_x;
            if(world_y < min_world_y) min_world_y = world_y;
            if(world_y > max_world_y) max_world_y = world_y;
        }
    }
    if(!valid) return;
    min_world_x -= scale;
    max_world_x += scale;
    min_world_y -= scale;
    max_world_y += scale;
    local_width = (int)ceilf((max_world_y - min_world_y) / scale) + 1;
    local_height = (int)ceilf((max_world_x - min_world_x) / scale) + 1;
    if(local_width < 1 || local_height < 1 ||
       local_width > (int)VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_WIDTH ||
       local_height > (int)VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_HEIGHT) return;

    memset(s_vision_local_birdview_mask, 0,
           sizeof(s_vision_local_birdview_mask));
    {
        int local_row;
        int local_col;

        for(local_row = 0; local_row < local_height; ++local_row)
        {
            for(local_col = 0; local_col < local_width; ++local_col)
            {
                float world_x = min_world_x + ((float)local_row + 0.5f) * scale;
                float world_y = max_world_y - ((float)local_col + 0.5f) * scale;
                int source_x;
                int source_y;
                int rect_x;
                int rect_y;
                int local_index = local_row *
                    VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_WIDTH + local_col;

                if(!IPM_World2Image(world_x, world_y, &source_x, &source_y) ||
                   !IPM_World2RectifiedImage(world_x, world_y, &rect_x, &rect_y) ||
                   source_x < 0 || source_x >= MT9V03X_W ||
                   source_y < 0 || source_y >= MT9V03X_H) continue;
                if(image[source_y][source_x] >=
                   VisionRuntime_GetSourceThreshold(source_x, source_y))
                {
                    VISION_RUNTIME_LOCAL_BIT_SET(s_vision_local_birdview_mask,
                                                 local_index);
                    VISION_RUNTIME_IMAGE_BIT_SET(s_vision_binary,
                        rect_y * MT9V03X_W + rect_x);
                }
            }
        }
    }
}

#endif

static void VisionRuntime_FillFineCandidateUndistorted(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    const VisionRuntimeCandidate_t *candidate, uint8 threshold);
static uint8 VisionRuntime_BuildFineCandidateIPMBinary(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    const VisionRuntimeCandidate_t *candidate, uint8 threshold);

static void VisionRuntime_Binarize(const uint8 image[MT9V03X_H][MT9V03X_W])
{
    VisionRuntime_BinarizeCoarse(image);
    VisionRuntime_FindCoarseCandidates();
    memset(s_vision_binary, 0, sizeof(s_vision_binary));
    memset(s_vision_fine_component_mask, 0,
           sizeof(s_vision_fine_component_mask));
    IPM_GenerateUndistortedDebugImage(image, s_vision_undistorted);
}

static void VisionRuntime_ClearFineCandidate(
    const VisionRuntimeCandidate_t *candidate)
{
    int min_x;
    int max_x;
    int min_y;
    int max_y;
    int x;
    int y;

    if(candidate == NULL) return;
    VisionRuntime_GetFineCandidateBounds(candidate, &min_x, &max_x,
                                         &min_y, &max_y);
    for(y = min_y; y <= max_y; ++y)
    {
        for(x = min_x; x <= max_x; ++x)
        {
            VISION_RUNTIME_FINE_BIT_CLEAR(s_vision_binary, x, y);
        }
    }
}

static uint8 VisionRuntime_BuildFineCandidateBinary(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    const VisionRuntimeCandidate_t *candidate, uint8 threshold)
{
    if(candidate == NULL) return 0U;
    VisionRuntime_ClearFineCandidate(candidate);
#if VISION_RUNTIME_PIPELINE_MODE == VISION_RUNTIME_PIPELINE_IPM_THEN_BINARY
    return VisionRuntime_BuildFineCandidateIPMBinary(image, candidate,
                                                      threshold);
#else
    VisionRuntime_FillFineCandidateUndistorted(image, candidate, threshold);
    return 1U;
#endif
}

static void VisionRuntime_FillFineCandidateUndistorted(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    const VisionRuntimeCandidate_t *candidate, uint8 threshold)
{
    int min_x;
    int max_x;
    int min_y;
    int max_y;
    int x;
    int y;

    if(candidate == NULL) return;
    VisionRuntime_GetFineCandidateBounds(candidate, &min_x, &max_x, &min_y, &max_y);

    for(y = min_y; y <= max_y; ++y)
    {
        for(x = min_x; x <= max_x; ++x)
        {
            uint16 source_x_q8 = 0U;
            uint16 source_y_q8 = 0U;
            uint8 sample = 0U;

            if(!IPM_SampleFullUndistortedGrayQ8(
                   image, x, y, &source_x_q8, &source_y_q8, &sample))
            {
                continue;
            }
            if(sample >= threshold)
            {
                VISION_RUNTIME_FINE_BIT_SET(s_vision_binary, x, y);
            }
        }
    }
}

static uint8 VisionRuntime_BuildFineCandidateIPMBinary(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    const VisionRuntimeCandidate_t *candidate, uint8 threshold)
{
    const float scale = VISION_RUNTIME_LOCAL_BIRDVIEW_SCALE_CM_PER_PIXEL;
    float min_world_x = 0.0f;
    float max_world_x = 0.0f;
    float min_world_y = 0.0f;
    float max_world_y = 0.0f;
    int corner_x[4];
    int corner_y[4];
    int local_width;
    int local_height;
    int valid = 0;
    int corner;

    if(candidate == NULL || scale <= 0.0f) return 0U;
    corner_x[0] = candidate->min_col;
    corner_x[1] = candidate->max_col;
    corner_x[2] = candidate->min_col;
    corner_x[3] = candidate->max_col;
    corner_y[0] = candidate->min_row;
    corner_y[1] = candidate->min_row;
    corner_y[2] = candidate->max_row;
    corner_y[3] = candidate->max_row;
    for(corner = 0; corner < 4; ++corner)
    {
        float world_x;
        float world_y;

        if(!IPM_RectifiedImage2WorldF((float)corner_x[corner],
                                      (float)corner_y[corner],
                                      &world_x, &world_y)) continue;
        if(!valid)
        {
            min_world_x = max_world_x = world_x;
            min_world_y = max_world_y = world_y;
            valid = 1;
        }
        else
        {
            if(world_x < min_world_x) min_world_x = world_x;
            if(world_x > max_world_x) max_world_x = world_x;
            if(world_y < min_world_y) min_world_y = world_y;
            if(world_y > max_world_y) max_world_y = world_y;
        }
    }
    if(!valid) return 0U;
    min_world_x -= scale;
    max_world_x += scale;
    min_world_y -= scale;
    max_world_y += scale;
    local_width = (int)ceilf((max_world_y - min_world_y) / scale) + 1;
    local_height = (int)ceilf((max_world_x - min_world_x) / scale) + 1;
    if(local_width < 1 || local_height < 1 ||
       local_width > (int)VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_WIDTH ||
       local_height > (int)VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_HEIGHT)
    {
        return 0U;
    }

    memset(s_vision_local_birdview_mask, 0,
           sizeof(s_vision_local_birdview_mask));
    for(corner = 0; corner < local_height; ++corner)
    {
        int local_col;
        for(local_col = 0; local_col < local_width; ++local_col)
        {
            float world_x = min_world_x + ((float)corner + 0.5f) * scale;
            float world_y = max_world_y - ((float)local_col + 0.5f) * scale;
            int source_x;
            int source_y;
            int full_x;
            int full_y;
            int local_index = corner * VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_WIDTH +
                              local_col;

            if(!IPM_World2Image(world_x, world_y, &source_x, &source_y) ||
               !IPM_World2FullRectifiedImage(world_x, world_y,
                                             &full_x, &full_y) ||
               source_x < 0 || source_x >= MT9V03X_W ||
               source_y < 0 || source_y >= MT9V03X_H)
            {
                continue;
            }
            if(image[source_y][source_x] >= threshold)
            {
                VISION_RUNTIME_LOCAL_BIT_SET(s_vision_local_birdview_mask,
                                             local_index);
                VISION_RUNTIME_FINE_BIT_SET(s_vision_binary, full_x, full_y);
            }
        }
    }
    return 1U;
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
            if(!VISION_RUNTIME_IMAGE_BIT_GET(s_vision_binary, next_index))
            {
                return 1U;
            }
        }
    }

    return 0U;
}

#if 0
static void VisionRuntime_ProcessBlobsLegacy(const uint8 image[MT9V03X_H][MT9V03X_W])
{
    int row = 0;
    int col = 0;
    int large_area_threshold = VisionRuntime_GetScaledLargeAreaThreshold();

    memset(s_vision_visited, 0, sizeof(s_vision_visited));
    VisionRuntime_Binarize(image);
    memset(s_vision_visited, 0, sizeof(s_vision_visited));

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

            if(!VISION_RUNTIME_IMAGE_BIT_GET(s_vision_binary, start_index) ||
               VISION_RUNTIME_IMAGE_BIT_GET(s_vision_visited, start_index))
            {
                continue;
            }

            s_vision_queue[rear++] = (uint16)start_index;
            VISION_RUNTIME_IMAGE_BIT_SET(s_vision_visited, start_index);
            VISION_RUNTIME_IMAGE_BIT_SET(s_vision_component_mask, start_index);

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
                        if(VISION_RUNTIME_IMAGE_BIT_GET(s_vision_binary, next_index) &&
                           !VISION_RUNTIME_IMAGE_BIT_GET(s_vision_visited, next_index))
                        {
                            VISION_RUNTIME_IMAGE_BIT_SET(s_vision_visited, next_index);
                            VISION_RUNTIME_IMAGE_BIT_SET(s_vision_component_mask, next_index);
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

#endif

static uint8 VisionRuntime_FineIsForeground(int x, int y)
{
    return (uint8)(x >= 0 && x < (int)VISION_RUNTIME_FINE_WIDTH &&
                   y >= 0 && y < (int)VISION_RUNTIME_FINE_HEIGHT &&
                   VISION_RUNTIME_FINE_BIT_GET(s_vision_binary, x, y));
}

static uint8 VisionRuntime_FineIsBoundary(int x, int y)
{
    int dr;
    int dc;

    for(dr = -1; dr <= 1; ++dr)
    {
        for(dc = -1; dc <= 1; ++dc)
        {
            if(dr == 0 && dc == 0) continue;
            if(!VisionRuntime_FineIsForeground(x + dc, y + dr)) return 1U;
        }
    }
    return 0U;
}

static uint8 VisionRuntime_FineAppendRun(int row, int seed_col,
                                         int min_col, int max_col,
                                         uint16 *run_count)
{
    int start_col = seed_col;
    int end_col = seed_col;
    int x;

    if(run_count == NULL || *run_count >= VISION_RUNTIME_RLE_MAX_RUNS)
    {
        ++s_vision_rle_overflow_count;
        return 0U;
    }
    while(start_col > min_col &&
          VisionRuntime_FineIsForeground(start_col - 1, row) &&
          !VISION_RUNTIME_FINE_BIT_GET(s_vision_fine_visited,
                                       start_col - 1, row))
    {
        --start_col;
    }
    while(end_col < max_col &&
          VisionRuntime_FineIsForeground(end_col + 1, row) &&
          !VISION_RUNTIME_FINE_BIT_GET(s_vision_fine_visited,
                                       end_col + 1, row))
    {
        ++end_col;
    }
    for(x = start_col; x <= end_col; ++x)
    {
        VISION_RUNTIME_FINE_BIT_SET(s_vision_fine_visited, x, row);
        VISION_RUNTIME_FINE_BIT_SET(s_vision_fine_component_mask, x, row);
    }
    s_vision_fine_runs[*run_count].row = (uint16)row;
    s_vision_fine_runs[*run_count].start_col = (uint16)start_col;
    s_vision_fine_runs[*run_count].end_col = (uint16)end_col;
    ++(*run_count);
    return 1U;
}

static uint8 VisionRuntime_FineAppendNeighborRuns(
    const VisionRuntimeFineRun_t *run, int neighbor_row,
    int min_col, int max_col, uint16 *run_count)
{
    int scan_col;
    int begin_col;
    int end_col;

    if(run == NULL || run_count == NULL ||
       neighbor_row < 0 || neighbor_row >= (int)VISION_RUNTIME_FINE_HEIGHT)
    {
        return 1U;
    }
    begin_col = (int)run->start_col - 1;
    end_col = (int)run->end_col + 1;
    if(begin_col < min_col) begin_col = min_col;
    if(end_col > max_col) end_col = max_col;
    scan_col = begin_col;
    while(scan_col <= end_col)
    {
        if(VisionRuntime_FineIsForeground(scan_col, neighbor_row) &&
           !VISION_RUNTIME_FINE_BIT_GET(s_vision_fine_visited,
                                        scan_col, neighbor_row))
        {
            if(!VisionRuntime_FineAppendRun(neighbor_row, scan_col,
                                            min_col, max_col, run_count))
            {
                return 0U;
            }
            scan_col = (int)s_vision_fine_runs[*run_count - 1U].end_col + 1;
        }
        else
        {
            ++scan_col;
        }
    }
    return 1U;
}

static uint8 VisionRuntime_FineSampleBrightness(
    const uint8 image[MT9V03X_H][MT9V03X_W], int x, int y, uint8 *sample)
{
    uint16 source_x_q8;
    uint16 source_y_q8;

    if(sample == NULL || x < 0 || x >= (int)VISION_RUNTIME_FINE_WIDTH ||
       y < 0 || y >= (int)VISION_RUNTIME_FINE_HEIGHT)
    {
        return 0U;
    }
    return IPM_SampleFullUndistortedGrayQ8(
        image, x, y, &source_x_q8, &source_y_q8, sample);
}

static float VisionRuntime_Clamp01(float value)
{
    if(value < 0.0f) return 0.0f;
    if(value > 1.0f) return 1.0f;
    return value;
}

static void VisionRuntime_ComputeConfidenceInputs(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    const VisionRuntimeCandidate_t *candidate,
    uint8 threshold,
    VisionBlob_t *blob)
{
    int x;
    int y;
    int threshold_low = (threshold > 8U) ? (int)threshold - 8 : 1;
    int threshold_high = (threshold < 247U) ? (int)threshold + 8 : 255;
    uint32 low_area = 0U;
    uint32 high_area = 0U;
    uint32 low_x_sum = 0U;
    uint32 low_y_sum = 0U;
    uint32 high_x_sum = 0U;
    uint32 high_y_sum = 0U;
    float max_center_drift = 0.0f;
    float max_area_change = 0.0f;
    float delta;
    float base_area;

    if(blob == NULL) return;
    blob->raw_box_area_pixels = 0;
    blob->confidence_box_size = 0.0f;
    blob->confidence_stability = 0.0f;
    blob->confidence_area = 0.0f;
    blob->confidence_edge = 0.0f;

    if(candidate != NULL)
    {
        blob->raw_min_row = candidate->min_row;
        blob->raw_max_row = candidate->max_row;
        blob->raw_min_col = candidate->min_col;
        blob->raw_max_col = candidate->max_col;
        blob->raw_box_area_pixels =
            (candidate->max_col - candidate->min_col + 1) *
            (candidate->max_row - candidate->min_row + 1);
        blob->confidence_box_size = VisionRuntime_Clamp01(
            (sqrtf((float)blob->raw_box_area_pixels) -
             sqrtf(VISION_RUNTIME_BEACON_BOX_AREA_MIN_PIXELS)) /
            (sqrtf(VISION_RUNTIME_BEACON_BOX_AREA_MAX_PIXELS) -
             sqrtf(VISION_RUNTIME_BEACON_BOX_AREA_MIN_PIXELS)));
    }

    for(y = blob->min_row; y <= blob->max_row; ++y)
    {
        for(x = blob->min_col; x <= blob->max_col; ++x)
        {
            uint8 sample = 0U;
            if(!VisionRuntime_FineSampleBrightness(image, x, y, &sample)) continue;
            if((int)sample >= threshold_low)
            {
                ++low_area;
                low_x_sum += (uint32)x;
                low_y_sum += (uint32)y;
            }
            if((int)sample >= threshold_high)
            {
                ++high_area;
                high_x_sum += (uint32)x;
                high_y_sum += (uint32)y;
            }
        }
    }
    base_area = (blob->area > 0) ? (float)blob->area : 1.0f;
    max_area_change = fabsf((float)low_area - base_area) / base_area;
    delta = fabsf((float)high_area - base_area) / base_area;
    if(delta > max_area_change) max_area_change = delta;
    if(low_area > 0U)
    {
        delta = sqrtf(
            ((float)low_x_sum / (float)low_area - blob->centroid_col) *
            ((float)low_x_sum / (float)low_area - blob->centroid_col) +
            ((float)low_y_sum / (float)low_area - blob->centroid_row) *
            ((float)low_y_sum / (float)low_area - blob->centroid_row));
        if(delta > max_center_drift) max_center_drift = delta;
    }
    if(high_area > 0U)
    {
        delta = sqrtf(
            ((float)high_x_sum / (float)high_area - blob->centroid_col) *
            ((float)high_x_sum / (float)high_area - blob->centroid_col) +
            ((float)high_y_sum / (float)high_area - blob->centroid_row) *
            ((float)high_y_sum / (float)high_area - blob->centroid_row));
        if(delta > max_center_drift) max_center_drift = delta;
    }
    blob->confidence_stability =
        0.70f * expf(-max_center_drift / 1.5f) +
        0.30f * expf(-max_area_change / 0.25f);
    blob->confidence_stability = VisionRuntime_Clamp01(blob->confidence_stability);

    {
        int edge_distance = blob->raw_min_col;
        int value;
        value = blob->raw_min_row;
        if(value < edge_distance) edge_distance = value;
        value = (MT9V03X_W - 1) - blob->raw_max_col;
        if(value < edge_distance) edge_distance = value;
        value = (MT9V03X_H - 1) - blob->raw_max_row;
        if(value < edge_distance) edge_distance = value;
        blob->confidence_edge = VisionRuntime_Clamp01((float)edge_distance / 8.0f);
    }
}

static void VisionRuntime_BuildPhysicalBlobMetricsFine(
    VisionBlob_t *blob, uint16 run_count)
{
    const float scale = VISION_RUNTIME_LOCAL_BIRDVIEW_SCALE_CM_PER_PIXEL;
    float min_x = 0.0f;
    float max_x = 0.0f;
    float min_y = 0.0f;
    float max_y = 0.0f;
    int valid_points = 0;
    int local_width;
    int local_height;
    uint32 occupied = 0U;
    uint32 exposed_edges = 0U;
    uint16 run_index;

    if(blob == NULL || run_count == 0U || scale <= 0.0f) return;
    for(run_index = 0U; run_index < run_count; ++run_index)
    {
        int x;
        int y = (int)s_vision_fine_runs[run_index].row;
        for(x = (int)s_vision_fine_runs[run_index].start_col;
            x <= (int)s_vision_fine_runs[run_index].end_col; ++x)
        {
            float world_x;
            float world_y;
            if(!IPM_FullRectifiedImage2WorldF((float)x, (float)y,
                                              &world_x, &world_y)) continue;
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
            ++valid_points;
        }
    }
    if(valid_points == 0) return;

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
    {
        int local_row;
        int local_col;
        for(local_row = 0; local_row < local_height; ++local_row)
        {
            for(local_col = 0; local_col < local_width; ++local_col)
            {
                float world_x = min_x + ((float)local_row + 0.5f) * scale;
                float world_y = max_y - ((float)local_col + 0.5f) * scale;
                int source_x;
                int source_y;
                int index = local_row * VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_WIDTH +
                            local_col;

                if(IPM_World2FullRectifiedImage(world_x, world_y,
                                                &source_x, &source_y) &&
                   VISION_RUNTIME_FINE_BIT_GET(
                       s_vision_fine_component_mask, source_x, source_y))
                {
                    VISION_RUNTIME_LOCAL_BIT_SET(s_vision_local_birdview_mask,
                                                 index);
                }
            }
        }
    }

    /* Fill the projected footprint of each full-resolution foreground pixel. */
    for(run_index = 0U; run_index < run_count; ++run_index)
    {
        int x;
        int y = (int)s_vision_fine_runs[run_index].row;
        for(x = (int)s_vision_fine_runs[run_index].start_col;
            x <= (int)s_vision_fine_runs[run_index].end_col; ++x)
        {
            const float corner_x[4] = {
                (float)x - 0.5f, (float)x + 0.5f,
                (float)x - 0.5f, (float)x + 0.5f};
            const float corner_y[4] = {
                (float)y - 0.5f, (float)y - 0.5f,
                (float)y + 0.5f, (float)y + 0.5f};
            float footprint_min_x = 0.0f;
            float footprint_max_x = 0.0f;
            float footprint_min_y = 0.0f;
            float footprint_max_y = 0.0f;
            int footprint_valid = 0;
            int corner;

            for(corner = 0; corner < 4; ++corner)
            {
                float world_x;
                float world_y;
                if(!IPM_FullRectifiedImage2WorldF(
                       corner_x[corner], corner_y[corner],
                       &world_x, &world_y)) continue;
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
                ++footprint_valid;
            }
            if(footprint_valid == 0) continue;
            {
                int min_local_row = (int)floorf((footprint_min_x - min_x) / scale);
                int max_local_row = (int)floorf((footprint_max_x - min_x) / scale);
                int min_local_col = (int)floorf((max_y - footprint_max_y) / scale);
                int max_local_col = (int)floorf((max_y - footprint_min_y) / scale);
                int local_row;
                int local_col;

                if(min_local_row < 0) min_local_row = 0;
                if(min_local_col < 0) min_local_col = 0;
                if(max_local_row >= local_height) max_local_row = local_height - 1;
                if(max_local_col >= local_width) max_local_col = local_width - 1;
                for(local_row = min_local_row; local_row <= max_local_row; ++local_row)
                {
                    for(local_col = min_local_col; local_col <= max_local_col; ++local_col)
                    {
                        VISION_RUNTIME_LOCAL_BIT_SET(
                            s_vision_local_birdview_mask,
                            local_row * VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_WIDTH +
                            local_col);
                    }
                }
            }
        }
    }

    {
        int row;
        int col;
        for(row = 0; row < local_height; ++row)
        {
            for(col = 0; col < local_width; ++col)
            {
                int index = row * VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_WIDTH + col;
                int neighbor;
                if(!VISION_RUNTIME_LOCAL_BIT_GET(s_vision_local_birdview_mask,
                                                 index)) continue;
                ++occupied;
                neighbor = (row > 0) ? index - VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_WIDTH : -1;
                if(neighbor < 0 || !VISION_RUNTIME_LOCAL_BIT_GET(s_vision_local_birdview_mask, neighbor)) ++exposed_edges;
                neighbor = (row + 1 < local_height) ? index + VISION_RUNTIME_LOCAL_BIRDVIEW_MAX_WIDTH : -1;
                if(neighbor < 0 || !VISION_RUNTIME_LOCAL_BIT_GET(s_vision_local_birdview_mask, neighbor)) ++exposed_edges;
                neighbor = (col > 0) ? index - 1 : -1;
                if(neighbor < 0 || !VISION_RUNTIME_LOCAL_BIT_GET(s_vision_local_birdview_mask, neighbor)) ++exposed_edges;
                neighbor = (col + 1 < local_width) ? index + 1 : -1;
                if(neighbor < 0 || !VISION_RUNTIME_LOCAL_BIT_GET(s_vision_local_birdview_mask, neighbor)) ++exposed_edges;
            }
        }
    }
    if(occupied == 0U || exposed_edges == 0U) return;
    blob->physical_pixel_count = (occupied > 65535U) ? 65535U : (uint16)occupied;
    blob->physical_area_cm2 = (float)occupied * scale * scale;
    blob->physical_perimeter_cm = (float)exposed_edges * scale;
    blob->physical_circularity =
        (4.0f * 3.14159265f * blob->physical_area_cm2) /
        ((float)exposed_edges * scale * (float)exposed_edges * scale);
    if(blob->physical_circularity > 1.0f) blob->physical_circularity = 1.0f;
    blob->physical_aspect_ratio = VisionRuntime_NormalizedAspect(local_width,
                                                                  local_height);
    blob->physical_occupancy_ratio =
        (float)occupied / (float)(local_width * local_height);
    blob->physical_valid = 1U;
}

static void VisionRuntime_ClearFineComponent(uint16 run_count)
{
    uint16 run_index;
    for(run_index = 0U; run_index < run_count; ++run_index)
    {
        int x;
        int y = (int)s_vision_fine_runs[run_index].row;
        for(x = (int)s_vision_fine_runs[run_index].start_col;
            x <= (int)s_vision_fine_runs[run_index].end_col; ++x)
        {
            VISION_RUNTIME_FINE_BIT_CLEAR(s_vision_fine_component_mask, x, y);
        }
    }
}

typedef enum
{
    VISION_RUNTIME_FINE_TARGET_VEHICLE = 0,
    VISION_RUNTIME_FINE_TARGET_BEACON = 1
} VisionRuntimeFineTarget_t;

static uint8 VisionRuntime_DispatchFineComponent(
    const uint8 image[MT9V03X_H][MT9V03X_W], uint16 run_count,
    VisionRuntimeFineTarget_t target, const VisionRuntimeCandidate_t *candidate,
    uint8 candidate_index, uint8 threshold)
{
    VisionBlob_t blob;
    VisionContour_t contour;
    uint32 area = 0U;
    uint32 row_sum = 0U;
    uint32 col_sum = 0U;
    uint32 brightness_sum = 0U;
    uint32 boundary_count = 0U;
    uint32 boundary_seen = 0U;
    uint16 run_index;
    int min_row = VISION_RUNTIME_FINE_HEIGHT;
    int max_row = -1;
    int min_col = VISION_RUNTIME_FINE_WIDTH;
    int max_col = -1;
    int boundary_step;
    int width;
    int height;

    memset(&blob, 0, sizeof(blob));
    memset(&contour, 0, sizeof(contour));
    for(run_index = 0U; run_index < run_count; ++run_index)
    {
        int x;
        int y = (int)s_vision_fine_runs[run_index].row;
        for(x = (int)s_vision_fine_runs[run_index].start_col;
            x <= (int)s_vision_fine_runs[run_index].end_col; ++x)
        {
            uint8 sample = 0U;
            ++area;
            row_sum += (uint32)y;
            col_sum += (uint32)x;
            if(VisionRuntime_FineSampleBrightness(image, x, y, &sample))
                brightness_sum += sample;
            if(y < min_row) min_row = y;
            if(y > max_row) max_row = y;
            if(x < min_col) min_col = x;
            if(x > max_col) max_col = x;
            if(VisionRuntime_FineIsBoundary(x, y)) ++boundary_count;
        }
    }
    if(area == 0U || boundary_count == 0U) return 0U;
    width = max_col - min_col + 1;
    height = max_row - min_row + 1;
    blob.centroid_row = (float)row_sum / (float)area;
    blob.centroid_col = (float)col_sum / (float)area;
    blob.area = (area > 2147483647U) ? 2147483647 : (int)area;
    blob.min_row = min_row;
    blob.max_row = max_row;
    blob.min_col = min_col;
    blob.max_col = max_col;
    blob.average_brightness = (float)brightness_sum / (float)area;
    blob.aspect_ratio = VisionRuntime_NormalizedAspect(width, height);
    blob.occupancy_ratio = (float)area / (float)(width * height);
    blob.circularity = (4.0f * 3.14159265f * (float)area) /
                       ((float)boundary_count * (float)boundary_count);
    if(blob.circularity > 1.0f) blob.circularity = 1.0f;

    boundary_step = (boundary_count > VISION_RUNTIME_CONTOUR_MAX_POINTS) ?
                    (int)((boundary_count + VISION_RUNTIME_CONTOUR_MAX_POINTS - 1U) /
                          VISION_RUNTIME_CONTOUR_MAX_POINTS) : 1;
    for(run_index = 0U; run_index < run_count; ++run_index)
    {
        int x;
        int y = (int)s_vision_fine_runs[run_index].row;
        for(x = (int)s_vision_fine_runs[run_index].start_col;
            x <= (int)s_vision_fine_runs[run_index].end_col; ++x)
        {
            if(!VisionRuntime_FineIsBoundary(x, y)) continue;
            if((boundary_seen % (uint32)boundary_step) == 0U &&
               contour.point_count < VISION_RUNTIME_CONTOUR_MAX_POINTS)
            {
                contour.point_x[contour.point_count] = (uint16)x;
                contour.point_y[contour.point_count] = (uint16)y;
                ++contour.point_count;
            }
            ++boundary_seen;
        }
    }
    VisionRuntime_BuildPhysicalBlobMetricsFine(&blob, run_count);
    blob.world_valid = IPM_FullRectifiedImage2WorldF(
        blob.centroid_col, blob.centroid_row, &blob.world_x, &blob.world_y);
    ++s_vision_runtime_diag.blob_count;
    VisionRuntime_InsertBlobAreaRank(blob.area);
    if(blob.world_valid)
    {
        if(target == VISION_RUNTIME_FINE_TARGET_VEHICLE)
        {
            ++s_vision_runtime_diag.ir_split_count;
            if(VehicleMarkerLocalization_AppendAndValidateBlob(&blob, &contour))
            {
                ++s_vision_runtime_diag.ir_blob_count;
                return 1U;
            }
        }
        else
        {
            ++s_vision_runtime_diag.beacon_split_count;
            VisionRuntime_ComputeConfidenceInputs(image, candidate, threshold, &blob);
            if(BeaconDetect_AppendBlob(&blob))
            {
                ++s_vision_runtime_diag.beacon_blob_count;
                if(candidate_index < VISION_RUNTIME_MAX_CANDIDATES)
                {
                    s_vision_candidate_annotations[candidate_index].beacon_valid = 1U;
                    s_vision_candidate_annotations[candidate_index].raw_box_area_pixels =
                        blob.raw_box_area_pixels;
                    s_vision_candidate_annotations[candidate_index].detection_confidence =
                        blob.detection_confidence;
                    s_vision_candidate_annotations[candidate_index].beacon_x_cm = blob.world_x;
                    s_vision_candidate_annotations[candidate_index].beacon_y_cm = blob.world_y;
                    s_vision_candidate_annotations[candidate_index].beacon_area_cm2 =
                        blob.physical_area_cm2;
                    s_vision_candidate_annotations[candidate_index].beacon_circularity =
                        blob.physical_circularity;
                    s_vision_candidate_annotations[candidate_index].confidence_box_size =
                        blob.confidence_box_size;
                    s_vision_candidate_annotations[candidate_index].confidence_stability =
                        blob.confidence_stability;
                    s_vision_candidate_annotations[candidate_index].confidence_area =
                        blob.confidence_area;
                    s_vision_candidate_annotations[candidate_index].confidence_edge =
                        blob.confidence_edge;
                }
            }
        }
    }
    return 0U;
}

static uint8 VisionRuntime_ProcessFineROI(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    const VisionRuntimeCandidate_t *candidate,
    VisionRuntimeFineTarget_t target, uint8 candidate_index, uint8 threshold)
{
    int min_col;
    int max_col;
    int min_row;
    int max_row;
    int row;
    int col;

    if(candidate == NULL) return 0U;
    VisionRuntime_GetFineCandidateBounds(candidate, &min_col, &max_col,
                                         &min_row, &max_row);
    memset(s_vision_fine_visited, 0, sizeof(s_vision_fine_visited));

    for(row = min_row; row <= max_row; ++row)
    {
        for(col = min_col; col <= max_col; ++col)
        {
            uint16 run_count = 0U;
            int front = 0;

            if(!VisionRuntime_FineIsForeground(col, row) ||
               VISION_RUNTIME_FINE_BIT_GET(s_vision_fine_visited, col, row))
            {
                continue;
            }
            if(!VisionRuntime_FineAppendRun(row, col, min_col, max_col,
                                            &run_count))
            {
                VisionRuntime_ClearFineComponent(run_count);
                return 0U;
            }
            while(front < (int)run_count)
            {
                VisionRuntimeFineRun_t run = s_vision_fine_runs[front++];
                if(!VisionRuntime_FineAppendNeighborRuns(
                       &run, (int)run.row - 1, min_col, max_col, &run_count) ||
                   !VisionRuntime_FineAppendNeighborRuns(
                       &run, (int)run.row + 1, min_col, max_col, &run_count))
                {
                    VisionRuntime_ClearFineComponent(run_count);
                    return 0U;
                }
            }
            if(VisionRuntime_DispatchFineComponent(
                   image, run_count, target, candidate, candidate_index, threshold))
            {
                VisionRuntime_ClearFineComponent(run_count);
                return 1U;
            }
            VisionRuntime_ClearFineComponent(run_count);
        }
    }
    return 0U;
}

static void VisionRuntime_ProcessBlobs(
    const uint8 image[MT9V03X_H][MT9V03X_W])
{
    uint8 i;

    VisionRuntime_Binarize(image);
    for(i = 0U; i < s_vision_candidate_count; ++i)
    {
        uint8 vehicle_confirmed = 0U;

        if(VisionRuntime_BuildFineCandidateBinary(
               image, &s_vision_candidates[i], s_vision_vehicle_threshold))
        {
            vehicle_confirmed = VisionRuntime_ProcessFineROI(
                image, &s_vision_candidates[i],
                VISION_RUNTIME_FINE_TARGET_VEHICLE, i,
                s_vision_vehicle_threshold);
            if(vehicle_confirmed)
            {
                s_vision_candidate_annotations[i].vehicle_valid = 1U;
            }
        }
        if(vehicle_confirmed)
        {
            continue;
        }
        if(VisionRuntime_BuildFineCandidateBinary(
               image, &s_vision_candidates[i], s_vision_beacon_threshold))
        {
            (void)VisionRuntime_ProcessFineROI(
                image, &s_vision_candidates[i],
                VISION_RUNTIME_FINE_TARGET_BEACON, i,
                s_vision_beacon_threshold);
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
            s_vision_runtime_diag.beacon_nearest_confidence =
                nearest->detection_confidence;
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
    s_vision_rle_overflow_count = 0U;
    memset(s_vision_runtime_diag.blob_area_rank, 0, sizeof(s_vision_runtime_diag.blob_area_rank));
    VehicleMarkerLocalization_BeginBlobFrame();
    BeaconDetect_BeginBlobFrame(beacon_results, BEACON_MAX_DETECTIONS);
    VisionRuntime_ProcessBlobs(mt9v03x_image);
    (void)VehicleMarkerLocalization_EndBlobFrame(&g_vehicle_marker_pose);
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

uint32 VisionRuntime_GetFineRleOverflowCount(void)
{
    return s_vision_rle_overflow_count;
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

uint8 VisionRuntime_GetCandidateBoxes(VisionRuntimeCandidateBox_t *boxes, uint8 max_boxes)
{
#if CY_CORE_CM7_1
    uint8 count;
    uint8 i;

    if(boxes == NULL || max_boxes == 0U)
    {
        return 0U;
    }
    count = (max_boxes < s_vision_candidate_count) ? max_boxes : s_vision_candidate_count;
    for(i = 0U; i < count; ++i)
    {
        int min_x;
        int max_x;
        int min_y;
        int max_y;

        VisionRuntime_GetFineCandidateBounds(
            &s_vision_candidates[i], &min_x, &max_x, &min_y, &max_y);
        boxes[i].raw_min_row = s_vision_candidates[i].min_row;
        boxes[i].raw_max_row = s_vision_candidates[i].max_row;
        boxes[i].raw_min_col = s_vision_candidates[i].min_col;
        boxes[i].raw_max_col = s_vision_candidates[i].max_col;
        boxes[i].fine_min_row = min_y;
        boxes[i].fine_max_row = max_y;
        boxes[i].fine_min_col = min_x;
        boxes[i].fine_max_col = max_x;
        boxes[i].area = s_vision_candidates[i].area;
        boxes[i].raw_box_area_pixels =
            s_vision_candidate_annotations[i].raw_box_area_pixels;
        boxes[i].vehicle_valid = s_vision_candidate_annotations[i].vehicle_valid;
        boxes[i].beacon_valid = s_vision_candidate_annotations[i].beacon_valid;
        boxes[i].detection_confidence =
            s_vision_candidate_annotations[i].detection_confidence;
        boxes[i].beacon_x_cm = s_vision_candidate_annotations[i].beacon_x_cm;
        boxes[i].beacon_y_cm = s_vision_candidate_annotations[i].beacon_y_cm;
        boxes[i].beacon_area_cm2 = s_vision_candidate_annotations[i].beacon_area_cm2;
        boxes[i].beacon_circularity =
            s_vision_candidate_annotations[i].beacon_circularity;
        boxes[i].confidence_box_size =
            s_vision_candidate_annotations[i].confidence_box_size;
        boxes[i].confidence_stability =
            s_vision_candidate_annotations[i].confidence_stability;
        boxes[i].confidence_area =
            s_vision_candidate_annotations[i].confidence_area;
        boxes[i].confidence_edge =
            s_vision_candidate_annotations[i].confidence_edge;
    }
    return count;
#else
    (void)boxes;
    (void)max_boxes;
    return 0U;
#endif
}
