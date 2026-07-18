#include "zf_device_mt9v03x.h"
#include "vision/beacon_detect.h"
#include "vision/ipm_interface.h"
#include "vision/vehicle_marker_localization.h"
#include "vision/vision_runtime.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
volatile uint8 mt9v03x_finish_flag = 0U;

#define BIRDVIEW_MAX_ALLOCATION_BYTES (256U * 1024U * 1024U)
#define BIRDVIEW_HALF_RANGE_CM        (250.0f)

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

static int write_result(const char *path, const char *binary_path,
                        float roll_deg, float pitch_deg, float height_cm,
                        uint8 threshold,
                        const VisionRuntimeDiag_t *diag,
                        const IPM_BirdViewBounds_t *birdview_bounds)
{
    FILE *file = NULL;
    VisionRuntimeBeaconSlot_t slots[VISION_RUNTIME_BEACON_SLOT_COUNT] = {0};
    VehicleMarkerChevronDebug_t chevron = {0};
    int i;

    if(path == NULL || diag == NULL)
    {
        return 0;
    }
    VisionRuntime_GetBeaconSlots(slots, VISION_RUNTIME_BEACON_SLOT_COUNT);
    VehicleMarkerLocalization_GetChevronDebug(&chevron);
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
            "  \"beacon_nearest_metrics\":{\"raw_area_pixels\":%.6f,\"area_cm2\":%.6f,"
            "\"perimeter_cm\":%.6f,\"circularity\":%.6f},\n",
            diag->beacon_nearest_area,
            diag->beacon_nearest_area_cm2,
            diag->beacon_nearest_perimeter_cm,
            diag->beacon_nearest_circularity);
    fprintf(file, "  \"binary_threshold\":%u,\n", (unsigned)threshold);
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
        fprintf(file,
                "    {\"valid\":%u,\"x_cm\":%.6f,\"y_cm\":%.6f,\"min_row\":%d,\"max_row\":%d,\"min_col\":%d,\"max_col\":%d,"
                "\"area_cm2\":%.6f,\"perimeter_cm\":%.6f,\"circularity\":%.6f,"
                "\"physical_pixel_count\":%u,\"physical_valid\":%u}%s\n",
                (unsigned)slots[i].valid, slots[i].x_cm, slots[i].y_cm,
                slots[i].min_row, slots[i].max_row,
                slots[i].min_col, slots[i].max_col,
                slots[i].area_cm2, slots[i].perimeter_cm, slots[i].circularity,
                (unsigned)slots[i].physical_pixel_count,
                (unsigned)slots[i].physical_valid,
                (i + 1 < VISION_RUNTIME_BEACON_SLOT_COUNT) ? "," : "");
    }
    fprintf(file, "  ],\n  \"vehicle\":{\"valid\":%u,\"x_cm\":%.6f,\"y_cm\":%.6f,\"heading_deg\":%.6f},\n",
            (unsigned)g_vehicle_marker_pose.valid,
            g_vehicle_marker_pose.x, g_vehicle_marker_pose.y,
            g_vehicle_marker_pose.heading_deg);
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
    fprintf(file, "\"right_line_end_img\":{\"u\":%.3f,\"v\":%.3f}}\n",
            chevron.right_line_end_img.u, chevron.right_line_end_img.v);
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
    int threshold_value = parse_int(get_option(argc, argv, "--threshold"), (int)VISION_BINARY_THRESHOLD);
    uint8 threshold;
    int binary_output = parse_int(get_option(argc, argv, "--binary-output"), 0);
    VisionRuntimeDiag_t diag = {0};
    uint8 undistorted_image[MT9V03X_H][MT9V03X_W];
    uint8 undistorted_mask[MT9V03X_H][MT9V03X_W];
    uint8 undistorted_binary[MT9V03X_H][MT9V03X_W];
    uint8 *birdview_image = NULL;
    uint8 *birdview_mask = NULL;
    uint8 *birdview_binary = NULL;
    const uint8 (*binary_source)[MT9V03X_W] = NULL;
    IPM_BirdViewBounds_t birdview_bounds = {0};
    IPM_BirdViewBounds_t *birdview_bounds_ptr = NULL;
    size_t birdview_pixels = 0U;

    if(input_path == NULL || output_path == NULL || !read_raw(input_path))
    {
        fprintf(stderr, "usage: vision_sim --input frame.raw --output result.json [--binary binary.pgm] [--threshold 150] [--binary-output 1] [--undistorted undistorted.pgm] [--undistorted-mask mask.pgm] [--undistorted-binary binary.pgm] [--birdview birdview.pgm] [--birdview-mask mask.pgm] [--birdview-binary binary.pgm]\n");
        return 2;
    }

    if(threshold_value < 1) threshold_value = 1;
    if(threshold_value > 255) threshold_value = 255;
    threshold = (uint8)threshold_value;

    IPM_Interface_Init();
    VehicleMarkerLocalization_Init();
    BeaconDetect_Init();
    VisionRuntime_SetBinaryThreshold(threshold);
    mt9v03x_finish_flag = 1U;
    VisionRuntime_ProcessFrameWithPose(roll_deg, pitch_deg, height_cm);
    VisionRuntime_GetDiag(&diag);
    binary_source = (const uint8 (*)[MT9V03X_W])VisionRuntime_GetBinaryDisplayImage();

    if(binary_path != NULL)
    {
        write_pgm(binary_path, VisionRuntime_GetBinaryDisplayImage());
    }
    if(undistorted_path != NULL)
    {
        IPM_GenerateUndistortedDebugImage(mt9v03x_image, undistorted_image);
        write_pgm(undistorted_path, undistorted_image[0]);
        if(undistorted_mask_path != NULL)
        {
            IPM_GenerateUndistortedDebugMask(undistorted_mask);
            write_pgm(undistorted_mask_path, undistorted_mask[0]);
        }
        if(binary_output && undistorted_binary_path != NULL)
        {
            IPM_GenerateUndistortedDebugImageNearest(binary_source, undistorted_binary);
            write_pgm(undistorted_binary_path, undistorted_binary[0]);
        }
    }
    if(birdview_path != NULL)
    {
        if(!IPM_CalculateBirdViewBounds(IPM_SCALE, &birdview_bounds) ||
           !IPM_ClampBirdViewBounds(&birdview_bounds,
                                    BIRDVIEW_HALF_RANGE_CM,
                                    BIRDVIEW_HALF_RANGE_CM) ||
           birdview_bounds.width <= 0 || birdview_bounds.height <= 0)
        {
            fprintf(stderr, "birdview has no valid ground projection\n");
            return 4;
        }
        if((size_t)birdview_bounds.width > ((size_t)-1) / (size_t)birdview_bounds.height)
        {
            fprintf(stderr, "birdview dimensions overflow\n");
            return 4;
        }
        birdview_pixels = (size_t)birdview_bounds.width * (size_t)birdview_bounds.height;
        if(birdview_pixels > (size_t)BIRDVIEW_MAX_ALLOCATION_BYTES)
        {
            fprintf(stderr,
                    "birdview dimensions %dx%d require %zu bytes, exceeding the %u-byte limit; no crop performed\n",
                    birdview_bounds.width, birdview_bounds.height, birdview_pixels,
                    (unsigned)BIRDVIEW_MAX_ALLOCATION_BYTES);
            return 4;
        }
        birdview_image = (uint8 *)malloc(birdview_pixels);
        birdview_mask = (uint8 *)malloc(birdview_pixels);
        if(binary_output)
        {
            birdview_binary = (uint8 *)malloc(birdview_pixels);
        }
        if(birdview_image == NULL || birdview_mask == NULL ||
           (binary_output && birdview_binary == NULL))
        {
            fprintf(stderr, "birdview allocation failed for %dx%d (%zu bytes)\n",
                    birdview_bounds.width, birdview_bounds.height, birdview_pixels);
            free(birdview_image);
            free(birdview_mask);
            free(birdview_binary);
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
            return 4;
        }
        if(birdview_mask_path != NULL)
        {
            write_pgm_image(birdview_mask_path, birdview_mask,
                            birdview_bounds.width, birdview_bounds.height);
        }
        if(binary_output && birdview_binary_path != NULL)
        {
            if(!IPM_GenerateBirdViewDynamicNearest(binary_source, birdview_binary,
                                                   birdview_mask, &birdview_bounds))
            {
                fprintf(stderr, "binary birdview generation failed\n");
                free(birdview_image);
                free(birdview_mask);
                free(birdview_binary);
                return 4;
            }
            write_pgm_image(birdview_binary_path, birdview_binary,
                            birdview_bounds.width, birdview_bounds.height);
        }
        birdview_bounds_ptr = &birdview_bounds;
    }
    if(!write_result(output_path, binary_path, roll_deg, pitch_deg, height_cm,
                     threshold, &diag, birdview_bounds_ptr))
    {
        fprintf(stderr, "failed to write result: %s\n", output_path);
        free(birdview_image);
        free(birdview_mask);
        free(birdview_binary);
        return 3;
    }
    free(birdview_image);
    free(birdview_mask);
    free(birdview_binary);
    return 0;
}
