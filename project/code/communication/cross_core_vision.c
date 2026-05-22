#include "communication/cross_core_vision.h"
#include <string.h>

#define CROSS_CORE_VISION_INPUT_MAGIC        0x56494E31u
#define CROSS_CORE_VISION_OUTPUT_MAGIC       0x564F5531u
#define CROSS_CORE_VISION_CACHE_LINE_SIZE    32U
#define CROSS_CORE_VISION_PAD_BYTES(type)    (CROSS_CORE_VISION_CACHE_LINE_SIZE - (sizeof(type) % CROSS_CORE_VISION_CACHE_LINE_SIZE))

typedef struct
{
    CrossCoreVisionInput_t data;
    uint8 padding[CROSS_CORE_VISION_PAD_BYTES(CrossCoreVisionInput_t)];
} CrossCoreVisionInputSlot_t;

typedef struct
{
    CrossCoreVisionOutput_t data;
    uint8 padding[CROSS_CORE_VISION_PAD_BYTES(CrossCoreVisionOutput_t)];
} CrossCoreVisionOutputSlot_t;

#pragma default_variable_attributes = @ ".shared_vision"
static volatile CrossCoreVisionInputSlot_t s_vision_input_slot __ALIGNED(CROSS_CORE_VISION_CACHE_LINE_SIZE);
static volatile CrossCoreVisionOutputSlot_t s_vision_output_slot __ALIGNED(CROSS_CORE_VISION_CACHE_LINE_SIZE);
#pragma default_variable_attributes =

static void clean_vision_input_cache(void)
{
    SCB_CleanDCache_by_Addr((void *)&s_vision_input_slot, sizeof(s_vision_input_slot));
}

static void invalidate_vision_input_cache(void)
{
    SCB_InvalidateDCache_by_Addr((void *)&s_vision_input_slot, sizeof(s_vision_input_slot));
}

static void clean_vision_output_cache(void)
{
    SCB_CleanDCache_by_Addr((void *)&s_vision_output_slot, sizeof(s_vision_output_slot));
}

static void invalidate_vision_output_cache(void)
{
    SCB_InvalidateDCache_by_Addr((void *)&s_vision_output_slot, sizeof(s_vision_output_slot));
}

void CrossCoreVision_InitShared(void)
{
#if CY_CORE_CM7_0
    memset((void *)&s_vision_input_slot, 0, sizeof(s_vision_input_slot));
    memset((void *)&s_vision_output_slot, 0, sizeof(s_vision_output_slot));
    clean_vision_input_cache();
    clean_vision_output_cache();
#else
    invalidate_vision_input_cache();
    invalidate_vision_output_cache();
#endif
}

void CrossCoreVision_ControlCorePublishInput(float roll_deg,
                                             float pitch_deg,
                                             float yaw_deg,
                                             float height_cm)
{
    s_vision_input_slot.data.magic = CROSS_CORE_VISION_INPUT_MAGIC;
    s_vision_input_slot.data.roll_deg = roll_deg;
    s_vision_input_slot.data.pitch_deg = pitch_deg;
    s_vision_input_slot.data.yaw_deg = yaw_deg;
    s_vision_input_slot.data.height_cm = height_cm;
    s_vision_input_slot.data.seq++;

    clean_vision_input_cache();
}

uint8 CrossCoreVision_CameraCoreFetchInput(CrossCoreVisionInput_t *out_input)
{
    invalidate_vision_input_cache();

    if(out_input == NULL)
    {
        return 0U;
    }

    if(s_vision_input_slot.data.magic != CROSS_CORE_VISION_INPUT_MAGIC)
    {
        return 0U;
    }

    memcpy(out_input, (const void *)&s_vision_input_slot.data, sizeof(CrossCoreVisionInput_t));
    return 1U;
}

void CrossCoreVision_BuildControlResult(const IR_Pose_t *ir_pose,
                                        uint8 position_lost,
                                        const VisionRuntimeDiag_t *diag,
                                        CrossCoreVisionControlResult_t *out_control)
{
    if(out_control == NULL)
    {
        return;
    }

    memset(out_control, 0, sizeof(CrossCoreVisionControlResult_t));

    if(ir_pose != NULL)
    {
        out_control->x_cm = ir_pose->x;
        out_control->y_cm = ir_pose->y;
        out_control->z_cm = ir_pose->z;
        out_control->square_side_cm = ir_pose->square_side_cm;
        out_control->position_lost = position_lost;
        out_control->detected_point_count = ir_pose->detected_point_count;
        out_control->track_valid = (uint8)((ir_pose->valid && !position_lost) ? 1U : 0U);
    }

    if(diag != NULL)
    {
        out_control->raw_detect_valid = diag->ir_raw_detect_valid;
        out_control->track_valid = diag->ir_track_valid;
        out_control->detected_point_count = diag->ir_detected_point_count;
    }
}

void CrossCoreVision_CameraCorePublishOutput(const CrossCoreVisionControlResult_t *control,
                                             const VisionRuntimeDiag_t *diag)
{
    s_vision_output_slot.data.magic = CROSS_CORE_VISION_OUTPUT_MAGIC;

    if(control != NULL)
    {
        memcpy((void *)&s_vision_output_slot.data.control, control, sizeof(CrossCoreVisionControlResult_t));
    }
    else
    {
        memset((void *)&s_vision_output_slot.data.control, 0, sizeof(CrossCoreVisionControlResult_t));
    }

    if(diag != NULL)
    {
        memcpy((void *)&s_vision_output_slot.data.diag, diag, sizeof(VisionRuntimeDiag_t));
    }
    else
    {
        memset((void *)&s_vision_output_slot.data.diag, 0, sizeof(VisionRuntimeDiag_t));
    }

    s_vision_output_slot.data.seq++;
    clean_vision_output_cache();
}

uint8 CrossCoreVision_ControlCoreFetchOutput(CrossCoreVisionOutput_t *out_output)
{
    static uint32 last_seq = 0U;

    invalidate_vision_output_cache();

    if(out_output == NULL)
    {
        return 0U;
    }

    if(s_vision_output_slot.data.magic != CROSS_CORE_VISION_OUTPUT_MAGIC)
    {
        return 0U;
    }

    if(s_vision_output_slot.data.seq == last_seq)
    {
        return 0U;
    }

    last_seq = s_vision_output_slot.data.seq;
    memcpy(out_output, (const void *)&s_vision_output_slot.data, sizeof(CrossCoreVisionOutput_t));
    return 1U;
}

void CrossCoreVision_ControlCoreApplyOutput(const CrossCoreVisionOutput_t *output)
{
    const CrossCoreVisionControlResult_t *control;

    if(output == NULL)
    {
        return;
    }

    control = &output->control;

    ir_pose_estimate.x = control->x_cm;
    ir_pose_estimate.y = control->y_cm;
    ir_pose_estimate.z = control->z_cm;
    ir_pose_estimate.raw_x = control->x_cm;
    ir_pose_estimate.raw_y = control->y_cm;
    ir_pose_estimate.square_side_cm = control->square_side_cm;
    ir_pose_estimate.detected_point_count = control->detected_point_count;
    ir_pose_estimate.reconstructed_point = 0U;
    ir_pose_estimate.valid = control->track_valid;
    ir_position_lost_flag = control->position_lost;
    ir_pose_update_seq = output->seq;

    VisionRuntime_SetDiag(&output->diag);
}

void CrossCoreVision_ControlCoreService(float roll_deg,
                                        float pitch_deg,
                                        float yaw_deg,
                                        float height_cm)
{
    CrossCoreVisionOutput_t vision_output;

    CrossCoreVision_ControlCorePublishInput(roll_deg,
                                            pitch_deg,
                                            yaw_deg,
                                            height_cm);

    if(CrossCoreVision_ControlCoreFetchOutput(&vision_output))
    {
        CrossCoreVision_ControlCoreApplyOutput(&vision_output);
    }
}
