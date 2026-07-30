#include "zf_device_mt9v03x.h"
#include "vision/beacon_detect.h"
#include "vision/camera_calibration.h"
#include "vision/ipm_interface.h"
#include "vision/vehicle_marker_localization.h"
#include "vision/vision_runtime.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
volatile uint8 mt9v03x_finish_flag = 0U;
static uint8 s_runtime_undistorted_mask[MT9V03X_H][MT9V03X_W];
static uint8 s_runtime_binary_fine[VISION_RUNTIME_FINE_HEIGHT][VISION_RUNTIME_FINE_WIDTH];
static uint8 s_runtime_binary_coarse[MT9V03X_H][MT9V03X_W];

#define BIRDVIEW_MAX_ALLOCATION_BYTES (256U * 1024U * 1024U)
#define BIRDVIEW_HALF_RANGE_CM        (500.0f)
#define BINARY_MODE_GRAY              (0)
#define BINARY_MODE_PRE_TRANSFORM     (1)
#define BINARY_MODE_UNDISTORT_BINARY  (2)
#define BINARY_MODE_POST_TRANSFORM    (3)

static float parse_float(const char *text, float fallback)
{
    char *end = NULL;
    float value;

    if(text == NULL)
    {
        return fallback;
    }
    value = (float)strtod(text, &end);
    return (end == text) ? fallback : value;
}

static const char *get_option(int argc, char **argv, const char *name)
{
    int i;

    for(i = 1; i + 1 < argc; i++)
    {
        if(strcmp(argv[i], name) == 0)
        {
            return argv[i + 1];
        }
    }
    return NULL;
}

static int read_raw(const char *path)
{
    FILE *file = NULL;
    size_t read_count;

    if(path == NULL)
    {
        return 0;
    }
    file = fopen(path, "rb");
    if(file == NULL)
    {
        return 0;
    }
    read_count = fread(mt9v03x_image[0], 1U, MT9V03X_IMAGE_SIZE, file);
    fclose(file);
    return read_count == MT9V03X_IMAGE_SIZE;
}

static int write_pgm_image(const char *path, const uint8 *image, int width, int height)
{
    FILE *file = NULL;

    if(path == NULL || image == NULL)
    {
        return 0;
    }
    file = fopen(path, "wb");
    if(file == NULL)
    {
        return 0;
    }
    fprintf(file, "P5\n%d %d\n255\n", width, height);
    fwrite(image, 1U, (size_t)(width * height), file);
    fclose(file);
    return 1;
}

static int parse_int(const char *text, int fallback)
{
    char *end = NULL;
    long value;

    if(text == NULL)
    {
        return fallback;
    }
    value = strtol(text, &end, 10);
    return (end == text) ? fallback : (int)value;
}

static int write_pgm(const char *path, const uint8 *image)
{
    return write_pgm_image(path, image, MT9V03X_W, MT9V03X_H);
}

static int write_fine_pgm(const char *path, const uint8 *image)
{
    return write_pgm_image(path, image, VISION_RUNTIME_FINE_WIDTH,
                            VISION_RUNTIME_FINE_HEIGHT);
}

static void unpack_runtime_binary(const uint8 *packed)
{
    int y;
    int x;

    memset(s_runtime_binary_fine, 0, sizeof(s_runtime_binary_fine));
    memset(s_runtime_binary_coarse, 0, sizeof(s_runtime_binary_coarse));
    if(packed == NULL)
    {
        return;
    }
    for(y = 0; y < (int)VISION_RUNTIME_FINE_HEIGHT; y++)
    {
        for(x = 0; x < (int)VISION_RUNTIME_FINE_WIDTH; x++)
        {
            size_t byte_index = (size_t)y * (size_t)VISION_RUNTIME_FINE_STRIDE +
                                (size_t)(x >> 3);
            uint8 mask = (uint8)(0x80U >> (x & 7));
            uint8 value = (packed[byte_index] & mask) ? 255U : 0U;
            s_runtime_binary_fine[y][x] = value;
            if(value != 0U)
            {
                s_runtime_binary_coarse[y >> 1][x >> 1] = 255U;
            }
        }
    }
}

static void threshold_image(uint8 *image, size_t pixel_count, uint8 threshold)
{
    size_t i;

    if(image == NULL)
    {
        return;
    }
    for(i = 0U; i < pixel_count; i++)
    {
        image[i] = (image[i] >= threshold) ? 255U : 0U;
    }
}

typedef struct
{
    float min_u;
    float min_v;
    float max_u;
    float max_v;
    float center_u;
    float center_v;
    uint8 valid;
} RawOverlayBox_t;

static uint8 full_rectified_xy_to_raw(float full_u, float full_v,
                                      float *raw_u, float *raw_v)
{
    uint16 source_x_q8 = 0U;
    uint16 source_y_q8 = 0U;
    int full_x = (int)(full_u + 0.5f);
    int full_y = (int)(full_v + 0.5f);

    if(raw_u == NULL || raw_v == NULL)
    {
        return 0U;
    }
    if(CameraCalibration_FullRectifiedPixelToSourceQ8(
           full_x, full_y, &source_x_q8, &source_y_q8))
    {
        *raw_u = (float)source_x_q8 / 256.0f;
        *raw_v = (float)source_y_q8 / 256.0f;
        return 1U;
    }
    return 0U;
}

static void full_rectified_point_to_raw(const VehicleMarkerPoint2D_t *full,
                                        float *raw_u, float *raw_v)
{
    if(full == NULL || raw_u == NULL || raw_v == NULL)
    {
        return;
    }
    if(!full_rectified_xy_to_raw(full->u, full->v, raw_u, raw_v))
    {
        *raw_u = full->u * 0.5f;
        *raw_v = full->v * 0.5f;
    }
}

static void full_rectified_box_to_raw(const VisionRuntimeBeaconSlot_t *slot,
                                      RawOverlayBox_t *box)
{
    int center_x;
    int center_y;
    int sample;
    int point;
    int valid_count = 0;

    if(box == NULL) return;
    memset(box, 0, sizeof(*box));
    if(slot == NULL || !slot->valid) return;

    for(sample = 0; sample <= 8; ++sample)
    {
        float t = (float)sample / 8.0f;
        float x = (float)slot->min_col +
                  ((float)slot->max_col - (float)slot->min_col) * t;
        float y = (float)slot->min_row +
                  ((float)slot->max_row - (float)slot->min_row) * t;
        const float full_u[4] = {
            x, x, (float)slot->min_col, (float)slot->max_col};
        const float full_v[4] = {
            (float)slot->min_row, (float)slot->max_row, y, y};

        for(point = 0; point < 4; ++point)
        {
            float raw_u;
            float raw_v;
            if(!full_rectified_xy_to_raw(full_u[point], full_v[point],
                                         &raw_u, &raw_v))
            {
                continue;
            }
            if(valid_count == 0)
            {
                box->min_u = box->max_u = raw_u;
                box->min_v = box->max_v = raw_v;
            }
            else
            {
                if(raw_u < box->min_u) box->min_u = raw_u;
                if(raw_u > box->max_u) box->max_u = raw_u;
                if(raw_v < box->min_v) box->min_v = raw_v;
                if(raw_v > box->max_v) box->max_v = raw_v;
            }
            ++valid_count;
        }
    }
    if(valid_count == 0) return;
    if(IPM_World2Image(slot->x_cm, slot->y_cm, &center_x, &center_y))
    {
        box->center_u = (float)center_x;
        box->center_v = (float)center_y;
    }
    else if(!full_rectified_xy_to_raw(
                0.5f * (float)(slot->min_col + slot->max_col),
                0.5f * (float)(slot->min_row + slot->max_row),
                &box->center_u, &box->center_v))
    {
        box->center_u = 0.5f * (box->min_u + box->max_u);
        box->center_v = 0.5f * (box->min_v + box->max_v);
    }
    box->valid = 1U;
}

static int write_result(const char *path, const char *binary_path,
                        float roll_deg, float pitch_deg, float height_cm,
                        uint8 vehicle_threshold, uint8 beacon_threshold,
                        int binary_mode,
                        const VisionRuntimeDiag_t *diag,
                        const IPM_UndistortCanvas_t *undistort_canvas,
                        const IPM_BirdViewBounds_t *birdview_bounds)
{
    FILE *file = NULL;
    VisionRuntimeBeaconSlot_t slots[VISION_RUNTIME_BEACON_SLOT_COUNT] = {0};
    VisionRuntimeCandidateBox_t candidates[16U] = {0};
    VehicleMarkerChevronDebug_t chevron = {0};
    uint8 coarse_thresholds[VISION_RUNTIME_COARSE_ZONE_COUNT] = {0};
    float apex_raw_u = 0.0f;
    float apex_raw_v = 0.0f;
    float left_tip_raw_u = 0.0f;
    float left_tip_raw_v = 0.0f;
    float right_tip_raw_u = 0.0f;
    float right_tip_raw_v = 0.0f;
    float left_line_start_raw_u = 0.0f;
    float left_line_start_raw_v = 0.0f;
    float left_line_end_raw_u = 0.0f;
    float left_line_end_raw_v = 0.0f;
    float right_line_start_raw_u = 0.0f;
    float right_line_start_raw_v = 0.0f;
    float right_line_end_raw_u = 0.0f;
    float right_line_end_raw_v = 0.0f;
    uint8 candidate_count;
    int i;

    if(path == NULL || diag == NULL)
    {
        return 0;
    }
    VisionRuntime_GetBeaconSlots(slots, VISION_RUNTIME_BEACON_SLOT_COUNT);
    candidate_count = VisionRuntime_GetCandidateBoxes(
        candidates, 16U);
    VehicleMarkerLocalization_GetChevronDebug(&chevron);
    VisionRuntime_GetCoarseZoneThresholds(coarse_thresholds);
    file = fopen(path, "w");
    if(file == NULL)
    {
        return 0;
    }
    fprintf(file, "{\n");
    fprintf(file, "  \"width\":%d,\n  \"height\":%d,\n", MT9V03X_W, MT9V03X_H);
    fprintf(file, "  \"roll_deg\":%.6f,\n  \"pitch_deg\":%.6f,\n  \"height_cm\":%.6f,\n",
            roll_deg, pitch_deg, height_cm);
    if(birdview_bounds != NULL)
    {
        fprintf(file,
                "  \"birdview\":{\"width\":%d,\"height\":%d,\"scale_cm_per_pixel\":%.6f,"
                "\"min_world_x\":%.6f,\"max_world_x\":%.6f,"
                "\"min_world_y\":%.6f,\"max_world_y\":%.6f},\n",
                birdview_bounds->width, birdview_bounds->height,
                birdview_bounds->scale,
                birdview_bounds->min_world_x, birdview_bounds->max_world_x,
                birdview_bounds->min_world_y, birdview_bounds->max_world_y);
    }
    else
    {
        fprintf(file, "  \"birdview\":null,\n");
    }
    fprintf(file, "  \"blob_count\":%d,\n  \"beacon_count\":%d,\n  \"beacon_blob_count\":%d,\n",
            diag->blob_count, diag->beacon_count, diag->beacon_blob_count);
    fprintf(file,
            "  \"vehicle_blob_rejects\":{\"area_low\":%d,\"area_high\":%d,"
            "\"size_or_occupancy\":%d,\"circularity\":%d,\"accepted\":%d},\n",
            diag->ir_reject_area_low_count,
            diag->ir_reject_area_high_count,
            diag->ir_reject_size_count,
            diag->ir_reject_aspect_count,
            diag->ir_reject_insert_count);
    fprintf(file,
            "  \"beacon_nearest_metrics\":{\"raw_area_pixels\":%.6f,\"area_cm2\":%.6f,"
            "\"perimeter_cm\":%.6f,\"circularity\":%.6f,\"detection_confidence\":%u},\n",
            diag->beacon_nearest_area,
            diag->beacon_nearest_area_cm2,
            diag->beacon_nearest_perimeter_cm,
            diag->beacon_nearest_circularity,
            (unsigned)diag->beacon_nearest_confidence);
    fprintf(file, "  \"binary_threshold\":%u,\n",
            (unsigned)beacon_threshold);
    fprintf(file, "  \"coarse_threshold\":%u,\n",
            (unsigned)VisionRuntime_GetCoarseThreshold());
    fprintf(file,
            "  \"coarse_thresholds\":{"
            "\"center\":%u,\"near\":%u,\"middle\":%u,\"far\":%u,\"edge\":%u},\n",
            (unsigned)coarse_thresholds[0],
            (unsigned)coarse_thresholds[1],
            (unsigned)coarse_thresholds[2],
            (unsigned)coarse_thresholds[3],
            (unsigned)coarse_thresholds[4]);
    fprintf(file, "  \"vehicle_min_raw_box_area\":%u,\n",
            (unsigned)VISION_RUNTIME_VEHICLE_MIN_RAW_BOX_AREA);
    fprintf(file,
            "  \"vehicle_small_box_gate\":{\"min_area\":%u,"
            "\"max_fill_percent\":%u},\n",
            (unsigned)VISION_RUNTIME_VEHICLE_SMALL_BOX_MIN_AREA,
            (unsigned)VISION_RUNTIME_VEHICLE_SMALL_BOX_MAX_FILL_PCT);
    fprintf(file,
            "  \"coarse_candidate_min_area\":{\"normal\":%u,"
            "\"outer\":%u,\"first_outer_zone\":%u},\n",
            (unsigned)VISION_RUNTIME_COARSE_CANDIDATE_MIN_AREA,
            (unsigned)VISION_RUNTIME_OUTER_CANDIDATE_MIN_AREA,
            (unsigned)VISION_RUNTIME_OUTER_CANDIDATE_FIRST_ZONE);
    fprintf(file,
            "  \"beacon_edge_rescue\":{\"min_radius_sq\":%.3f,"
            "\"min_area_px\":%.1f,\"min_circularity\":%.2f,"
            "\"min_stability\":%.2f,\"box_min_scale\":%.2f},\n",
            (double)BEACON_EDGE_RESCUE_MIN_RADIUS_SQ,
            (double)BEACON_EDGE_RESCUE_MIN_AREA_PX,
            (double)BEACON_EDGE_RESCUE_MIN_CIRCULARITY,
            (double)BEACON_EDGE_RESCUE_MIN_STABILITY,
            (double)BEACON_EDGE_RESCUE_BOX_MIN_SCALE);
    fprintf(file,
            "  \"binary_thresholds\":{\"inner\":%u,\"middle\":%u,\"outer\":%u},\n",
            (unsigned)vehicle_threshold,
            (unsigned)beacon_threshold,
            (unsigned)beacon_threshold);
    fprintf(file,
            "  \"recognition_thresholds\":{\"vehicle\":%u,"
            "\"vehicle_fallback\":%u,\"beacon\":%u},\n",
            (unsigned)vehicle_threshold,
            (unsigned)VISION_BINARY_THRESHOLD_VEHICLE_FALLBACK,
            (unsigned)beacon_threshold);
    fprintf(file,
            "  \"recognition_pipeline\":\"vehicle_first_fallback_threshold_short_circuit\",\n");
    fprintf(file, "  \"physical_grid_enabled\":%u,\n",
            (unsigned)VISION_RUNTIME_PHYSICAL_GRID_ENABLE);
    fprintf(file, "  \"binary_mode\":\"%s\",\n",
            (binary_mode == BINARY_MODE_PRE_TRANSFORM) ? "pre_transform" :
            ((binary_mode == BINARY_MODE_UNDISTORT_BINARY) ? "undistort_binary_ipm" :
             ((binary_mode == BINARY_MODE_POST_TRANSFORM) ? "post_transform" : "gray")));
    if(undistort_canvas != NULL)
    {
        if(binary_mode == BINARY_MODE_UNDISTORT_BINARY)
        {
            fprintf(file,
                    "  \"undistorted\":{\"width\":%d,\"height\":%d,"
                    "\"runtime_full_fov_downsample\":1},\n",
                    MT9V03X_W, MT9V03X_H);
        }
        else
        {
            fprintf(file,
                    "  \"undistorted\":{\"width\":%d,\"height\":%d,"
                    "\"min_rectified_x\":%.6f,\"max_rectified_x\":%.6f,"
                    "\"min_rectified_y\":%.6f,\"max_rectified_y\":%.6f},\n",
                    undistort_canvas->width, undistort_canvas->height,
                    undistort_canvas->min_rectified_x, undistort_canvas->max_rectified_x,
                    undistort_canvas->min_rectified_y, undistort_canvas->max_rectified_y);
        }
    }
    else
    {
        fprintf(file, "  \"undistorted\":null,\n");
    }
    fprintf(file, "  \"binary_path\":\"");
    if(binary_path != NULL)
    {
        const char *cursor = binary_path;
        while(*cursor != '\0')
        {
            if(*cursor == '\\') fputc('/', file); else fputc(*cursor, file);
            cursor++;
        }
    }
    fprintf(file, "\",\n  \"beacons\":[\n");
    for(i = 0; i < VISION_RUNTIME_BEACON_SLOT_COUNT; i++)
    {
        RawOverlayBox_t overlay_box;
        full_rectified_box_to_raw(&slots[i], &overlay_box);
        fprintf(file,
                "    {\"valid\":%u,\"x_cm\":%.6f,\"y_cm\":%.6f,\"min_row\":%d,\"max_row\":%d,\"min_col\":%d,\"max_col\":%d,"
                "\"raw_min_row\":%d,\"raw_max_row\":%d,\"raw_min_col\":%d,\"raw_max_col\":%d,"
                "\"overlay_raw_valid\":%u,\"overlay_raw_min_row\":%.3f,\"overlay_raw_max_row\":%.3f,"
                "\"overlay_raw_min_col\":%.3f,\"overlay_raw_max_col\":%.3f,"
                "\"overlay_raw_centroid_row\":%.3f,\"overlay_raw_centroid_col\":%.3f,"
                "\"area_cm2\":%.6f,\"perimeter_cm\":%.6f,\"circularity\":%.6f,"
                "\"physical_pixel_count\":%u,\"physical_valid\":%u,"
                "\"raw_box_area_pixels\":%d,\"detection_confidence\":%u,\"position_sigma_cm\":%u,"
                "\"confidence_box_size\":%.6f,"
                "\"confidence_box_size_min\":%.6f,"
                "\"confidence_stability\":%.6f,\"confidence_area\":%.6f,"
                "\"confidence_edge\":%.6f}%s\n",
                (unsigned)slots[i].valid, slots[i].x_cm, slots[i].y_cm,
                slots[i].min_row, slots[i].max_row,
                slots[i].min_col, slots[i].max_col,
                slots[i].raw_min_row, slots[i].raw_max_row,
                slots[i].raw_min_col, slots[i].raw_max_col,
                (unsigned)overlay_box.valid,
                overlay_box.min_v, overlay_box.max_v,
                overlay_box.min_u, overlay_box.max_u,
                overlay_box.center_v, overlay_box.center_u,
                slots[i].area_cm2, slots[i].perimeter_cm, slots[i].circularity,
                (unsigned)slots[i].physical_pixel_count,
                (unsigned)slots[i].physical_valid,
                slots[i].raw_box_area_pixels,
                (unsigned)slots[i].detection_confidence,
                (unsigned)slots[i].position_sigma_cm,
                slots[i].confidence_box_size,
                slots[i].confidence_box_size_min,
                slots[i].confidence_stability,
                slots[i].confidence_area,
                slots[i].confidence_edge,
                (i + 1 < VISION_RUNTIME_BEACON_SLOT_COUNT) ? "," : "");
    }
    fprintf(file, "  ],\n  \"vehicle\":{\"valid\":%u,\"x_cm\":%.6f,\"y_cm\":%.6f,\"heading_deg\":%.6f,\"target_plane_height_cm\":%.6f},\n",
            (unsigned)g_vehicle_marker_pose.valid,
            g_vehicle_marker_pose.x, g_vehicle_marker_pose.y,
            g_vehicle_marker_pose.heading_deg,
            VEHICLE_MARKER_CHEVRON_PLANE_HEIGHT_CM);
    fprintf(file, "  \"coarse_candidates\":[\n");
    for(i = 0; i < (int)candidate_count; ++i)
    {
        fprintf(file,
                "    {\"area\":%d,\"raw_min_row\":%d,\"raw_max_row\":%d,"
                "\"raw_min_col\":%d,\"raw_max_col\":%d,"
                "\"fine_min_row\":%d,\"fine_max_row\":%d,"
                "\"fine_min_col\":%d,\"fine_max_col\":%d,"
                "\"vehicle_valid\":%u,\"vehicle_skipped_small_box\":%u,"
                "\"fine_roi_oversize\":%u,\"beacon_valid\":%u,"
                "\"raw_box_area_pixels\":%d,\"beacon_area_pixels\":%d,"
                "\"detection_confidence\":%u,\"position_sigma_cm\":%u,"
                "\"x_cm\":%.6f,\"y_cm\":%.6f,"
                "\"area_cm2\":%.6f,\"circularity\":%.6f,"
                "\"confidence_box_size\":%.6f,\"confidence_box_size_min\":%.6f,"
                "\"confidence_stability\":%.6f,"
                "\"confidence_area\":%.6f,\"confidence_edge\":%.6f,"
                "\"adaptive_normalized_area_px\":%.6f,"
                "\"adaptive_relaxation\":%.6f,"
                "\"adaptive_min_area_px\":%.6f,\"adaptive_max_area_px\":%.6f,"
                "\"adaptive_min_circularity\":%.6f}%s\n",
                candidates[i].area,
                candidates[i].raw_min_row, candidates[i].raw_max_row,
                candidates[i].raw_min_col, candidates[i].raw_max_col,
                candidates[i].fine_min_row, candidates[i].fine_max_row,
                candidates[i].fine_min_col, candidates[i].fine_max_col,
                (unsigned)candidates[i].vehicle_valid,
                (unsigned)candidates[i].vehicle_skipped_small_box,
                (unsigned)candidates[i].fine_roi_oversize,
                (unsigned)candidates[i].beacon_valid,
                candidates[i].raw_box_area_pixels,
                candidates[i].beacon_area_pixels,
                (unsigned)candidates[i].detection_confidence,
                (unsigned)candidates[i].position_sigma_cm,
                candidates[i].beacon_x_cm,
                candidates[i].beacon_y_cm,
                candidates[i].beacon_area_cm2,
                candidates[i].beacon_circularity,
                candidates[i].confidence_box_size,
                candidates[i].confidence_box_size_min,
                candidates[i].confidence_stability,
                candidates[i].confidence_area,
                candidates[i].confidence_edge,
                candidates[i].adaptive_normalized_area_px,
                candidates[i].adaptive_relaxation,
                candidates[i].adaptive_min_area_px,
                candidates[i].adaptive_max_area_px,
                candidates[i].adaptive_min_circularity,
                (i + 1 < (int)candidate_count) ? "," : "");
    }
    full_rectified_point_to_raw(&chevron.apex_img, &apex_raw_u, &apex_raw_v);
    full_rectified_point_to_raw(&chevron.left_tip_img, &left_tip_raw_u, &left_tip_raw_v);
    full_rectified_point_to_raw(&chevron.right_tip_img, &right_tip_raw_u, &right_tip_raw_v);
    full_rectified_point_to_raw(&chevron.left_line_start_img,
                                &left_line_start_raw_u, &left_line_start_raw_v);
    full_rectified_point_to_raw(&chevron.left_line_end_img,
                                &left_line_end_raw_u, &left_line_end_raw_v);
    full_rectified_point_to_raw(&chevron.right_line_start_img,
                                &right_line_start_raw_u, &right_line_start_raw_v);
    full_rectified_point_to_raw(&chevron.right_line_end_img,
                                &right_line_end_raw_u, &right_line_end_raw_v);
    fprintf(file, "  ],\n");
    fprintf(file, "  \"chevron\":{\"valid\":%u,\"heading_deg\":%.6f,",
            (unsigned)chevron.valid, chevron.heading_deg);
    fprintf(file, "\"apex_img\":{\"u\":%.3f,\"v\":%.3f},",
            chevron.apex_img.u, chevron.apex_img.v);
    fprintf(file, "\"left_tip_img\":{\"u\":%.3f,\"v\":%.3f},",
            chevron.left_tip_img.u, chevron.left_tip_img.v);
    fprintf(file, "\"right_tip_img\":{\"u\":%.3f,\"v\":%.3f},",
            chevron.right_tip_img.u, chevron.right_tip_img.v);
    fprintf(file, "\"left_line_start_img\":{\"u\":%.3f,\"v\":%.3f},",
            chevron.left_line_start_img.u, chevron.left_line_start_img.v);
    fprintf(file, "\"left_line_end_img\":{\"u\":%.3f,\"v\":%.3f},",
            chevron.left_line_end_img.u, chevron.left_line_end_img.v);
    fprintf(file, "\"right_line_start_img\":{\"u\":%.3f,\"v\":%.3f},",
            chevron.right_line_start_img.u, chevron.right_line_start_img.v);
    fprintf(file, "\"right_line_end_img\":{\"u\":%.3f,\"v\":%.3f},",
            chevron.right_line_end_img.u, chevron.right_line_end_img.v);
    fprintf(file, "\"apex_raw_img\":{\"u\":%.3f,\"v\":%.3f},",
            apex_raw_u, apex_raw_v);
    fprintf(file, "\"left_tip_raw_img\":{\"u\":%.3f,\"v\":%.3f},",
            left_tip_raw_u, left_tip_raw_v);
    fprintf(file, "\"right_tip_raw_img\":{\"u\":%.3f,\"v\":%.3f},",
            right_tip_raw_u, right_tip_raw_v);
    fprintf(file, "\"left_line_start_raw_img\":{\"u\":%.3f,\"v\":%.3f},",
            left_line_start_raw_u, left_line_start_raw_v);
    fprintf(file, "\"left_line_end_raw_img\":{\"u\":%.3f,\"v\":%.3f},",
            left_line_end_raw_u, left_line_end_raw_v);
    fprintf(file, "\"right_line_start_raw_img\":{\"u\":%.3f,\"v\":%.3f},",
            right_line_start_raw_u, right_line_start_raw_v);
    fprintf(file, "\"right_line_end_raw_img\":{\"u\":%.3f,\"v\":%.3f},",
            right_line_end_raw_u, right_line_end_raw_v);
    fprintf(file,
            "\"physical\":{\"area_cm2\":%.6f,\"occupancy\":%.6f,\"circularity\":%.6f,"
            "\"left_arm_cm\":%.6f,\"right_arm_cm\":%.6f,\"arm_angle_deg\":%.6f,"
            "\"base_cm\":%.6f,\"triangle_height_cm\":%.6f},",
            chevron.physical_area_cm2,
            chevron.physical_occupancy_ratio,
            chevron.physical_circularity,
            chevron.left_arm_cm,
            chevron.right_arm_cm,
            chevron.arm_angle_deg,
            chevron.base_cm,
            chevron.triangle_height_cm);
    fprintf(file,
            "\"open_v\":{\"arm_support\":%.6f,\"center_gap\":%.6f,"
            "\"arm_continuity\":%.6f,\"score\":%.6f,"
            "\"hypothesis_score\":%.6f,\"hypothesis_second_score\":%.6f,"
            "\"hypothesis_margin\":%.6f,\"gate_score\":%.6f,"
            "\"gate_center_gap\":%.6f,"
            "\"gate_from_primary\":%u,\"apex_index\":%u},",
            chevron.open_v_arm_support,
            chevron.open_v_center_gap,
            chevron.open_v_arm_continuity,
            chevron.open_v_score,
            chevron.open_v_hypothesis_score,
            chevron.open_v_hypothesis_second_score,
            chevron.open_v_hypothesis_margin,
            chevron.open_v_gate_score,
            chevron.open_v_gate_center_gap,
            (unsigned)chevron.open_v_gate_from_primary,
            (unsigned)chevron.open_v_apex_index);
    fprintf(file,
            "\"candidate_count\":%u,\"geometry_reject_count\":%u,\"reject_reason\":%u}\n",
            (unsigned)chevron.candidate_count,
            (unsigned)chevron.geometry_reject_count,
            (unsigned)chevron.reject_reason);
    fprintf(file, "}\n");
    fclose(file);
    return 1;
}

int main(int argc, char **argv)
{
    const char *input_path = get_option(argc, argv, "--input");
    const char *output_path = get_option(argc, argv, "--output");
    const char *binary_path = get_option(argc, argv, "--binary");
    const char *undistorted_path = get_option(argc, argv, "--undistorted");
    const char *undistorted_mask_path = get_option(argc, argv, "--undistorted-mask");
    const char *undistorted_binary_path = get_option(argc, argv, "--undistorted-binary");
    const char *birdview_path = get_option(argc, argv, "--birdview");
    const char *birdview_mask_path = get_option(argc, argv, "--birdview-mask");
    const char *birdview_binary_path = get_option(argc, argv, "--birdview-binary");
    float roll_deg = parse_float(get_option(argc, argv, "--roll"), 0.0f);
    float pitch_deg = parse_float(get_option(argc, argv, "--pitch"), 0.0f);
    float height_cm = parse_float(get_option(argc, argv, "--height"), 100.0f);
    const char *binary_mode_text = get_option(argc, argv, "--binary-mode");
    int threshold_value = parse_int(get_option(argc, argv, "--threshold"),
                                    (int)VISION_BINARY_THRESHOLD_BEACON);
    int vehicle_threshold_value = parse_int(
        get_option(argc, argv, "--vehicle-threshold"),
        parse_int(get_option(argc, argv, "--threshold-inner"),
                  (int)VISION_BINARY_THRESHOLD_VEHICLE));
    int beacon_threshold_value = parse_int(
        get_option(argc, argv, "--beacon-threshold"),
        parse_int(get_option(argc, argv, "--threshold-middle"),
                  threshold_value));
    int legacy_coarse_threshold = parse_int(
        get_option(argc, argv, "--coarse-threshold"), -1);
    int coarse_threshold_values[VISION_RUNTIME_COARSE_ZONE_COUNT] =
    {
        parse_int(get_option(argc, argv, "--coarse-center-threshold"),
                  (legacy_coarse_threshold > 0) ? legacy_coarse_threshold :
                  (int)VISION_RUNTIME_COARSE_THRESHOLD_CENTER),
        parse_int(get_option(argc, argv, "--coarse-near-threshold"),
                  (legacy_coarse_threshold > 0) ? legacy_coarse_threshold :
                  (int)VISION_RUNTIME_COARSE_THRESHOLD_NEAR),
        parse_int(get_option(argc, argv, "--coarse-middle-threshold"),
                  (legacy_coarse_threshold > 0) ? legacy_coarse_threshold :
                  (int)VISION_RUNTIME_COARSE_THRESHOLD_MIDDLE),
        parse_int(get_option(argc, argv, "--coarse-far-threshold"),
                  (legacy_coarse_threshold > 0) ? legacy_coarse_threshold :
                  (int)VISION_RUNTIME_COARSE_THRESHOLD_FAR),
        parse_int(get_option(argc, argv, "--coarse-edge-threshold"),
                  (legacy_coarse_threshold > 0) ? legacy_coarse_threshold :
                  (int)VISION_RUNTIME_COARSE_THRESHOLD_EDGE)
    };
    uint8 coarse_thresholds[VISION_RUNTIME_COARSE_ZONE_COUNT];
    uint8 threshold;
    uint8 vehicle_threshold;
    uint8 beacon_threshold;
    int binary_mode = BINARY_MODE_GRAY;
    VisionRuntimeDiag_t diag = {0};
    uint8 *undistorted_image = NULL;
    uint8 *undistorted_mask = NULL;
    uint8 *undistorted_binary = NULL;
    uint8 *birdview_image = NULL;
    uint8 *birdview_mask = NULL;
    uint8 *birdview_binary = NULL;
    const uint8 (*binary_source)[MT9V03X_W] = NULL;
    IPM_UndistortCanvas_t undistort_canvas = {0};
    IPM_UndistortCanvas_t *undistort_canvas_ptr = NULL;
    IPM_BirdViewBounds_t birdview_bounds = {0};
    IPM_BirdViewBounds_t *birdview_bounds_ptr = NULL;
    size_t undistorted_pixels = 0U;
    size_t birdview_pixels = 0U;

    if(input_path == NULL || output_path == NULL || !read_raw(input_path))
    {
        fprintf(stderr, "usage: vision_sim --input frame.raw --output result.json [--binary binary.pgm] [--vehicle-threshold 145] [--beacon-threshold 50] [--coarse-center-threshold 150] [--coarse-near-threshold 60] [--coarse-middle-threshold 50] [--coarse-far-threshold 50] [--coarse-edge-threshold 40] [--binary-mode gray|pre_transform|undistort_binary_ipm|post_transform]\n");
        return 2;
    }

    if(threshold_value < 1) threshold_value = 1;
    if(threshold_value > 255) threshold_value = 255;
    if(vehicle_threshold_value < 1) vehicle_threshold_value = 1;
    if(vehicle_threshold_value > 255) vehicle_threshold_value = 255;
    if(beacon_threshold_value < 1) beacon_threshold_value = 1;
    if(beacon_threshold_value > 255) beacon_threshold_value = 255;
    {
        int zone;
        for(zone = 0; zone < (int)VISION_RUNTIME_COARSE_ZONE_COUNT; ++zone)
        {
            if(coarse_threshold_values[zone] < 1) coarse_threshold_values[zone] = 1;
            if(coarse_threshold_values[zone] > 255) coarse_threshold_values[zone] = 255;
            coarse_thresholds[zone] = (uint8)coarse_threshold_values[zone];
        }
    }
    vehicle_threshold = (uint8)vehicle_threshold_value;
    beacon_threshold = (uint8)beacon_threshold_value;
    threshold = beacon_threshold;
    if(binary_mode_text != NULL && strcmp(binary_mode_text, "pre_transform") == 0)
    {
        binary_mode = BINARY_MODE_PRE_TRANSFORM;
    }
    else if(binary_mode_text != NULL && strcmp(binary_mode_text, "post_transform") == 0)
    {
        binary_mode = BINARY_MODE_POST_TRANSFORM;
    }
    else if(binary_mode_text != NULL && strcmp(binary_mode_text, "undistort_binary_ipm") == 0)
    {
        binary_mode = BINARY_MODE_UNDISTORT_BINARY;
    }
    else if(parse_int(get_option(argc, argv, "--binary-output"), 0) != 0)
    {
        binary_mode = BINARY_MODE_PRE_TRANSFORM;
    }

    IPM_Interface_Init();
    VehicleMarkerLocalization_Init();
    BeaconDetect_Init();
    VisionRuntime_SetRecognitionThresholds(vehicle_threshold,
                                           beacon_threshold);
    VisionRuntime_SetCoarseZoneThresholds(coarse_thresholds);
    mt9v03x_finish_flag = 1U;
    VisionRuntime_ProcessFrameWithPose(roll_deg, pitch_deg, height_cm);
    VisionRuntime_GetDiag(&diag);
    unpack_runtime_binary(VisionRuntime_GetBinaryImage());
    binary_source = (const uint8 (*)[MT9V03X_W])s_runtime_binary_coarse;
    IPM_GenerateUndistortedDebugMask(s_runtime_undistorted_mask);

    if(binary_path != NULL)
    {
        write_fine_pgm(binary_path, &s_runtime_binary_fine[0][0]);
    }
    if(undistorted_path != NULL ||
       (birdview_path != NULL && binary_mode == BINARY_MODE_UNDISTORT_BINARY))
    {
        if(!IPM_SetUndistortCanvas(&undistort_canvas) ||
           undistort_canvas.width <= 0 || undistort_canvas.height <= 0 ||
           (size_t)undistort_canvas.width > ((size_t)-1) / (size_t)undistort_canvas.height)
        {
            fprintf(stderr, "undistort canvas setup failed\n");
            return 4;
        }
        undistorted_pixels = (size_t)undistort_canvas.width * (size_t)undistort_canvas.height;
        if(undistorted_pixels > (size_t)BIRDVIEW_MAX_ALLOCATION_BYTES)
        {
            fprintf(stderr,
                    "undistort dimensions %dx%d require %zu bytes, exceeding the %u-byte limit\n",
                    undistort_canvas.width, undistort_canvas.height, undistorted_pixels,
                    (unsigned)BIRDVIEW_MAX_ALLOCATION_BYTES);
            return 4;
        }

        undistorted_image = (uint8 *)malloc(undistorted_pixels);
        undistorted_mask = (uint8 *)malloc(undistorted_pixels);
        if(binary_mode != BINARY_MODE_GRAY)
        {
            undistorted_binary = (uint8 *)malloc(undistorted_pixels);
        }
        if(undistorted_image == NULL || undistorted_mask == NULL ||
           (binary_mode != BINARY_MODE_GRAY && undistorted_binary == NULL))
        {
            fprintf(stderr, "undistort allocation failed for %dx%d\n",
                    undistort_canvas.width, undistort_canvas.height);
            free(undistorted_image);
            free(undistorted_mask);
            free(undistorted_binary);
            return 4;
        }
        if(!IPM_GenerateFullUndistortedImage(mt9v03x_image, undistorted_image,
                                             undistorted_mask, &undistort_canvas))
        {
            fprintf(stderr, "undistort generation failed\n");
            free(undistorted_image);
            free(undistorted_mask);
            free(undistorted_binary);
            return 4;
        }
        if(undistorted_path != NULL)
        {
            if(binary_mode == BINARY_MODE_UNDISTORT_BINARY)
            {
                write_pgm(undistorted_path,
                          VisionRuntime_GetUndistortedImage());
            }
            else
            {
                write_pgm_image(undistorted_path, undistorted_image,
                                undistort_canvas.width, undistort_canvas.height);
            }
        }
        if(undistorted_mask_path != NULL)
        {
            if(binary_mode == BINARY_MODE_UNDISTORT_BINARY)
            {
                write_pgm(undistorted_mask_path, s_runtime_undistorted_mask[0]);
            }
            else
            {
                write_pgm_image(undistorted_mask_path, undistorted_mask,
                                undistort_canvas.width, undistort_canvas.height);
            }
        }
        if(binary_mode != BINARY_MODE_GRAY)
        {
            if(binary_mode == BINARY_MODE_PRE_TRANSFORM)
            {
                IPM_GenerateFullUndistortedImageNearest(binary_source, undistorted_binary,
                                                        undistorted_mask, &undistort_canvas);
            }
            else
            {
                memcpy(undistorted_binary, undistorted_image, undistorted_pixels);
                threshold_image(undistorted_binary, undistorted_pixels, threshold);
            }
            if(undistorted_binary_path != NULL)
            {
                if(binary_mode == BINARY_MODE_UNDISTORT_BINARY)
                {
                    write_fine_pgm(undistorted_binary_path,
                                   &s_runtime_binary_fine[0][0]);
                }
                else
                {
                    write_pgm_image(undistorted_binary_path, undistorted_binary,
                                    undistort_canvas.width, undistort_canvas.height);
                }
            }
        }
        undistort_canvas_ptr = &undistort_canvas;
    }
    if(birdview_path != NULL)
    {
        if(!IPM_SetFixedBirdViewBounds(IPM_SCALE,
                                       BIRDVIEW_HALF_RANGE_CM,
                                       BIRDVIEW_HALF_RANGE_CM,
                                       &birdview_bounds) ||
           birdview_bounds.width <= 0 || birdview_bounds.height <= 0)
        {
            fprintf(stderr, "birdview has no valid ground projection\n");
            free(undistorted_image);
            free(undistorted_mask);
            free(undistorted_binary);
            return 4;
        }
        if((size_t)birdview_bounds.width > ((size_t)-1) / (size_t)birdview_bounds.height)
        {
            fprintf(stderr, "birdview dimensions overflow\n");
            free(undistorted_image);
            free(undistorted_mask);
            free(undistorted_binary);
            return 4;
        }
        birdview_pixels = (size_t)birdview_bounds.width * (size_t)birdview_bounds.height;
        if(birdview_pixels > (size_t)BIRDVIEW_MAX_ALLOCATION_BYTES)
        {
            fprintf(stderr,
                    "birdview dimensions %dx%d require %zu bytes, exceeding the %u-byte limit; no crop performed\n",
                    birdview_bounds.width, birdview_bounds.height, birdview_pixels,
                    (unsigned)BIRDVIEW_MAX_ALLOCATION_BYTES);
            free(undistorted_image);
            free(undistorted_mask);
            free(undistorted_binary);
            return 4;
        }
        birdview_image = (uint8 *)malloc(birdview_pixels);
        birdview_mask = (uint8 *)malloc(birdview_pixels);
        if(binary_mode != BINARY_MODE_GRAY)
        {
            birdview_binary = (uint8 *)malloc(birdview_pixels);
        }
        if(birdview_image == NULL || birdview_mask == NULL ||
           (binary_mode != BINARY_MODE_GRAY && birdview_binary == NULL))
        {
            fprintf(stderr, "birdview allocation failed for %dx%d (%zu bytes)\n",
                    birdview_bounds.width, birdview_bounds.height, birdview_pixels);
            free(birdview_image);
            free(birdview_mask);
            free(birdview_binary);
            free(undistorted_image);
            free(undistorted_mask);
            free(undistorted_binary);
            return 4;
        }
        if(!IPM_GenerateBirdViewDynamic(mt9v03x_image, birdview_image,
                                        birdview_mask, &birdview_bounds) ||
           !write_pgm_image(birdview_path, birdview_image,
                            birdview_bounds.width, birdview_bounds.height))
        {
            fprintf(stderr, "birdview generation failed\n");
            free(birdview_image);
            free(birdview_mask);
            free(birdview_binary);
            free(undistorted_image);
            free(undistorted_mask);
            free(undistorted_binary);
            return 4;
        }
        if(birdview_mask_path != NULL)
        {
            write_pgm_image(birdview_mask_path, birdview_mask,
                            birdview_bounds.width, birdview_bounds.height);
        }
        if(binary_mode != BINARY_MODE_GRAY && birdview_binary_path != NULL)
        {
            if(binary_mode == BINARY_MODE_PRE_TRANSFORM)
            {
                if(!IPM_GenerateBirdViewDynamicNearest(binary_source, birdview_binary,
                                                       birdview_mask, &birdview_bounds))
                {
                    fprintf(stderr, "binary birdview generation failed\n");
                    free(birdview_image);
                    free(birdview_mask);
                    free(birdview_binary);
                    free(undistorted_image);
                    free(undistorted_mask);
                    free(undistorted_binary);
                    return 4;
                }
            }
            else if(binary_mode == BINARY_MODE_UNDISTORT_BINARY)
            {
                if(!IPM_GenerateBirdViewFromRuntimeBinary(
                       VisionRuntime_GetBinaryImage(), birdview_binary,
                       birdview_mask, &birdview_bounds))
                {
                    fprintf(stderr, "undistorted binary birdview generation failed\n");
                    free(birdview_image);
                    free(birdview_mask);
                    free(birdview_binary);
                    free(undistorted_image);
                    free(undistorted_mask);
                    free(undistorted_binary);
                    return 4;
                }
            }
            else
            {
                memcpy(birdview_binary, birdview_image, birdview_pixels);
                threshold_image(birdview_binary, birdview_pixels, threshold);
            }
            write_pgm_image(birdview_binary_path, birdview_binary,
                            birdview_bounds.width, birdview_bounds.height);
        }
        birdview_bounds_ptr = &birdview_bounds;
    }
    if(!write_result(output_path, binary_path, roll_deg, pitch_deg, height_cm,
                     vehicle_threshold, beacon_threshold, binary_mode, &diag,
                     undistort_canvas_ptr, birdview_bounds_ptr))
    {
        fprintf(stderr, "failed to write result: %s\n", output_path);
        free(birdview_image);
        free(birdview_mask);
        free(birdview_binary);
        free(undistorted_image);
        free(undistorted_mask);
        free(undistorted_binary);
        return 3;
    }
    free(birdview_image);
    free(birdview_mask);
    free(birdview_binary);
    free(undistorted_image);
    free(undistorted_mask);
    free(undistorted_binary);
    return 0;
}
