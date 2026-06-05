#include "beacon_detect.h"
#include "common/project_headfile.h"
#include <math.h>

BeaconConfig_t g_beacon_config = {
    .binary_threshold = 170,
    .area_threshold   = 15,
    .min_aspect       = 0.3f,
    .max_aspect       = 6.0f,
    .min_occupancy    = 0.15f
};

static BeaconResult_t *s_beacon_frame_results = NULL;
static int s_beacon_frame_max_results = 0;
static int s_beacon_frame_count = 0;

static void BeaconDetect_FillResultFromBlob(const VisionBlob_t *blob, BeaconResult_t *result)
{
    float dist = 0.0f;
    float angle = 0.0f;

    if(blob == NULL || result == NULL)
    {
        return;
    }

    dist = sqrtf(blob->world_x * blob->world_x + blob->world_y * blob->world_y);
    angle = atan2f(blob->world_y, blob->world_x) * (180.0f / 3.14159265f);

    result->centroid_row = blob->centroid_row;
    result->centroid_col = blob->centroid_col;
    result->area = blob->area;
    result->average_brightness = blob->average_brightness;
    result->aspect_ratio = blob->aspect_ratio;
    result->occupancy_ratio = blob->occupancy_ratio;
    result->world_x = blob->world_x;
    result->world_y = blob->world_y;
    result->distance_cm = dist;
    result->angle_deg = angle;
}

void BeaconDetect_Init(void)
{
}

void BeaconDetect_BeginBlobFrame(BeaconResult_t *results, int max_results)
{
    s_beacon_frame_results = results;
    s_beacon_frame_max_results = max_results;
    s_beacon_frame_count = 0;
}

uint8 BeaconDetect_AppendBlob(const VisionBlob_t *blob)
{
    if(blob == NULL || s_beacon_frame_results == NULL || s_beacon_frame_max_results <= 0)
    {
        return 0U;
    }

    if(blob->aspect_ratio < g_beacon_config.min_aspect ||
       blob->aspect_ratio > g_beacon_config.max_aspect)
    {
        return 0U;
    }
    if(blob->occupancy_ratio < g_beacon_config.min_occupancy)
    {
        return 0U;
    }
    if(blob->centroid_row > (BEACON_IMG_ROWS / 2) && blob->area < 10)
    {
        return 0U;
    }

    if(s_beacon_frame_count >= s_beacon_frame_max_results)
    {
        return 0U;
    }

    BeaconDetect_FillResultFromBlob(blob, &s_beacon_frame_results[s_beacon_frame_count]);
    s_beacon_frame_count++;
    return 1U;
}

int BeaconDetect_EndBlobFrame(void)
{
    return s_beacon_frame_count;
}

int BeaconDetect_GetNearestIndex(const BeaconResult_t *results, int count)
{
    int best = 0;
    int i = 0;
    float best_dist = 0.0f;

    if(results == NULL || count <= 0)
    {
        return -1;
    }

    best_dist = results[0].distance_cm;
    for(i = 1; i < count; i++)
    {
        if(results[i].distance_cm < best_dist)
        {
            best_dist = results[i].distance_cm;
            best = i;
        }
    }

    return best;
}
