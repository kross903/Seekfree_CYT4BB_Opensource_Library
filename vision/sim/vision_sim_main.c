#include "zf_device_mt9v03x.h"
#include "vision/beacon_detect.h"
#include "vision/ipm_interface.h"
#include "vision/vehicle_marker_localization.h"
#include "vision/vision_runtime.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
volatile uint8 mt9v03x_finish_flag = 0U;

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

static int write_pgm(const char *path, const uint8 *image)
{
    return write_pgm_image(path, image, MT9V03X_W, MT9V03X_H);
}

static int write_result(const char *path, const char *binary_path,
                        float roll_deg, float pitch_deg, float height_cm,
                        const VisionRuntimeDiag_t *diag)
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
    fprintf(file, "  \"blob_count\":%d,\n  \"beacon_count\":%d,\n  \"beacon_blob_count\":%d,\n",
            diag->blob_count, diag->beacon_count, diag->beacon_blob_count);
    fprintf(file, "  \"binary_threshold\":%d,\n", VISION_BINARY_THRESHOLD);
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
                "    {\"valid\":%u,\"x_cm\":%.6f,\"y_cm\":%.6f,\"min_row\":%d,\"max_row\":%d,\"min_col\":%d,\"max_col\":%d}%s\n",
                (unsigned)slots[i].valid, slots[i].x_cm, slots[i].y_cm,
                slots[i].min_row, slots[i].max_row,
                slots[i].min_col, slots[i].max_col,
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
    const char *birdview_path = get_option(argc, argv, "--birdview");
    float roll_deg = parse_float(get_option(argc, argv, "--roll"), 0.0f);
    float pitch_deg = parse_float(get_option(argc, argv, "--pitch"), 0.0f);
    float height_cm = parse_float(get_option(argc, argv, "--height"), 100.0f);
    VisionRuntimeDiag_t diag = {0};
    uint8 undistorted_image[MT9V03X_H][MT9V03X_W];
    uint8 birdview_image[IPM_BIRDVIEW_W][IPM_BIRDVIEW_H];
    const uint8 (*birdview_source)[MT9V03X_W] = mt9v03x_image;

    if(input_path == NULL || output_path == NULL || !read_raw(input_path))
    {
        fprintf(stderr, "usage: vision_sim --input frame.raw --output result.json [--binary binary.pgm] [--undistorted undistorted.pgm] [--birdview birdview.pgm]\n");
        return 2;
    }

    IPM_Interface_Init();
    VehicleMarkerLocalization_Init();
    BeaconDetect_Init();
    mt9v03x_finish_flag = 1U;
    VisionRuntime_ProcessFrameWithPose(roll_deg, pitch_deg, height_cm);
    VisionRuntime_GetDiag(&diag);

    if(binary_path != NULL)
    {
        write_pgm(binary_path, VisionRuntime_GetBinaryDisplayImage());
    }
    if(undistorted_path != NULL)
    {
        IPM_GenerateUndistortedDebugImage(mt9v03x_image, undistorted_image);
        write_pgm(undistorted_path, undistorted_image[0]);
        birdview_source = undistorted_image;
    }
    if(birdview_path != NULL)
    {
        IPM_GenerateBirdView(birdview_source,
                             birdview_image[0],
                             IPM_BIRDVIEW_W, IPM_BIRDVIEW_H, IPM_SCALE);
        write_pgm_image(birdview_path, birdview_image[0],
                        IPM_BIRDVIEW_W, IPM_BIRDVIEW_H);
    }
    if(!write_result(output_path, binary_path, roll_deg, pitch_deg, height_cm, &diag))
    {
        fprintf(stderr, "failed to write result: %s\n", output_path);
        return 3;
    }
    return 0;
}
