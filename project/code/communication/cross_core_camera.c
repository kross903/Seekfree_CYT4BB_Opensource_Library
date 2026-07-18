#include "communication/cross_core_camera.h"
#include "common/project_headfile.h"

#define SHARED_VISION_INPUT_ADDR                    0x28001000u
#define SHARED_VISION_RESULT_ADDR                   0x28001100u
#define SHARED_VISION_SCREEN_CTRL_ADDR              0x28001300u
#define SHARED_VISION_SNAPSHOT_METADATA_ADDR        0x28001340u
#define SHARED_VISION_MAGIC                         0x56495331u
#define SHARED_VISION_SCREEN_MAGIC                  0x56534331u
#define SHARED_VISION_SNAPSHOT_METADATA_MAGIC       0x56534D31u
#define CROSS_CORE_CAMERA_ASSISTANT_IMAGE_ENABLE    0U
#define CROSS_CORE_CAMERA_SCREEN_FRAME_DIVIDER      2U
#define CROSS_CORE_VISION_IPC_RESULT_READY          0x56495352u
#define CROSS_CORE_CAMERA_DWT_LAR_UNLOCK            0xC5ACCE55UL

#define CROSS_CORE_CAMERA_METADATA_FUNCTION         0x12U
#define CROSS_CORE_CAMERA_METADATA_VERSION          0x01U
#define CROSS_CORE_CAMERA_METADATA_PACKET_SIZE      33U

#define CROSS_CORE_CAMERA_SCREEN_OWNER_NONE         0U
#define CROSS_CORE_CAMERA_SCREEN_OWNER_VISION       1U

typedef struct
{
    uint32 magic;
    uint32 seq;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float tof_height_cm;
    float fused_height_cm;
} SharedVisionInput_t;

typedef struct
{
    uint32 magic;
    uint32 seq;
    VehicleMarkerPose_t pose;
    VisionRuntimeDiag_t diag;
    VisionRuntimeBeaconSlot_t beacon_slots[VISION_RUNTIME_BEACON_SLOT_COUNT];
    VehicleMarkerLossDebug_t loss_debug;
    uint8 lost_flag;
    uint8 reserved[3];
    uint16 lost_elapsed_ms;
    uint16 reserved16;
    uint32 pose_update_seq;
} SharedVisionResult_t;

typedef struct
{
    uint32 magic;
    uint32 request_seq;
    uint32 active_seq;
    uint32 assistant_image_seq;
    uint32 assistant_snapshot_request_seq;
    uint8 requested_owner;
    uint8 active_owner;
    uint8 display_mode;
    uint8 assistant_stream_enable;
    uint8 assistant_image_display_mode;
    uint8 assistant_image_ready;
    uint8 vision_status;
    uint8 reserved;
} SharedVisionScreenCtrl_t;

typedef struct
{
    uint32 magic;
    uint32 image_seq;
    uint32 input_seq;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float tof_height_cm;
    float fused_height_cm;
} SharedVisionSnapshotMetadata_t;

static volatile SharedVisionInput_t *const g_shared_vision_input =
    (volatile SharedVisionInput_t *)SHARED_VISION_INPUT_ADDR;
static volatile SharedVisionResult_t *const g_shared_vision_result =
    (volatile SharedVisionResult_t *)SHARED_VISION_RESULT_ADDR;
static volatile SharedVisionScreenCtrl_t *const g_shared_vision_screen_ctrl =
    (volatile SharedVisionScreenCtrl_t *)SHARED_VISION_SCREEN_CTRL_ADDR;
static volatile SharedVisionSnapshotMetadata_t *const g_shared_vision_snapshot_metadata =
    (volatile SharedVisionSnapshotMetadata_t *)SHARED_VISION_SNAPSHOT_METADATA_ADDR;

static void CrossCoreCamera_CleanSnapshotMetadata(void)
{
    SCB_CleanDCache_by_Addr((void *)g_shared_vision_snapshot_metadata,
                           sizeof(SharedVisionSnapshotMetadata_t));
}

#if CY_CORE_CM7_0
static void CrossCoreCamera_InvalidateSnapshotMetadata(void)
{
    SCB_InvalidateDCache_by_Addr((void *)g_shared_vision_snapshot_metadata,
                                sizeof(SharedVisionSnapshotMetadata_t));
}
#endif

#pragma location = 0x2802bf80
__no_init uint8 g_cross_core_camera_image_copy[MT9V03X_H][MT9V03X_W];

#pragma location = 0x28031800
__no_init uint8 g_cross_core_camera_assistant_image[MT9V03X_H][MT9V03X_W];

#pragma location = 0x28037c55
__no_init uint8 g_cross_core_camera_image_fresh_flag;

#if CY_CORE_CM7_1
static uint8 s_vision_undistorted_image[MT9V03X_H][MT9V03X_W];

static uint8 CrossCoreCamera_CameraCoreAcquireFrame(void)
{
    SCB_CleanInvalidateDCache_by_Addr(&g_cross_core_camera_image_fresh_flag, 1);
    if(g_cross_core_camera_image_fresh_flag == 0U)
    {
        return 0U;
    }

    SCB_CleanInvalidateDCache_by_Addr(g_cross_core_camera_image_copy[0], MT9V03X_IMAGE_SIZE);
    memcpy(mt9v03x_image[0], g_cross_core_camera_image_copy[0], MT9V03X_IMAGE_SIZE);
    g_cross_core_camera_image_fresh_flag = 0U;
    SCB_CleanInvalidateDCache_by_Addr(&g_cross_core_camera_image_fresh_flag, 1);
    mt9v03x_finish_flag = 1U;
    return 1U;
}

static void CrossCoreCamera_CleanInput(void)
{
    SCB_CleanDCache_by_Addr((void *)g_shared_vision_input, sizeof(SharedVisionInput_t));
}

static void CrossCoreCamera_InvalidateInput(void)
{
    SCB_InvalidateDCache_by_Addr((void *)g_shared_vision_input, sizeof(SharedVisionInput_t));
}

static void CrossCoreCamera_CleanResult(void)
{
    SCB_CleanDCache_by_Addr((void *)g_shared_vision_result, sizeof(SharedVisionResult_t));
}

static void CrossCoreCamera_InvalidateScreenCtrl(void)
{
    SCB_InvalidateDCache_by_Addr((void *)g_shared_vision_screen_ctrl, sizeof(SharedVisionScreenCtrl_t));
}

static void CrossCoreCamera_CleanScreenCtrl(void)
{
    SCB_CleanDCache_by_Addr((void *)g_shared_vision_screen_ctrl, sizeof(SharedVisionScreenCtrl_t));
}

static uint8 CrossCoreCamera_DwtInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    if((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0U)
    {
        return 0U;
    }

    DWT->LAR = CROSS_CORE_CAMERA_DWT_LAR_UNLOCK;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    return 1U;
}

static void CrossCoreCamera_AgeVisionLossTimer(void)
{
    static uint8 timer_ready = 0U;
    static uint32 last_cycle = 0U;
    static uint32 residual_ms = 0U;
    uint32 now_cycle = 0U;
    uint32 elapsed_ms = 0U;
    uint32 tick_count = 0U;
    uint32 max_tick_count = 0U;
    uint32 i = 0U;

    if(!CrossCoreCamera_DwtInit() || SystemCoreClock == 0U)
    {
        VehicleMarkerLocalization_Task5ms();
        return;
    }

    now_cycle = DWT->CYCCNT;
    if(!timer_ready)
    {
        last_cycle = now_cycle;
        timer_ready = 1U;
        return;
    }

    elapsed_ms = (uint32)(((uint64)(now_cycle - last_cycle) * 1000ULL) / (uint64)SystemCoreClock);
    last_cycle = now_cycle;
    elapsed_ms += residual_ms;
    tick_count = elapsed_ms / VEHICLE_MARKER_TASK_PERIOD_MS;
    residual_ms = elapsed_ms % VEHICLE_MARKER_TASK_PERIOD_MS;

    max_tick_count = (VEHICLE_MARKER_POSITION_LOST_HOLD_MS / VEHICLE_MARKER_TASK_PERIOD_MS) + 1U;
    if(tick_count > max_tick_count)
    {
        tick_count = max_tick_count;
    }

    for(i = 0U; i < tick_count; i++)
    {
        VehicleMarkerLocalization_Task5ms();
    }
}

static void CrossCoreCamera_SetCameraCoreScreenActive(uint8 active)
{
    g_shared_vision_screen_ctrl->magic = SHARED_VISION_SCREEN_MAGIC;
    g_shared_vision_screen_ctrl->active_owner = active ? CROSS_CORE_CAMERA_SCREEN_OWNER_VISION
                                                       : CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
    g_shared_vision_screen_ctrl->active_seq++;
    CrossCoreCamera_CleanScreenCtrl();
}

static void CrossCoreCamera_CameraCorePublishAssistantSnapshot(const SharedVisionInput_t *input_snapshot)
{
    static uint32 last_snapshot_request_seq = 0U;
    uint32 snapshot_request_seq = 0U;
    uint32 image_seq = 0U;

    CrossCoreCamera_InvalidateScreenCtrl();
    if(g_shared_vision_screen_ctrl->magic != SHARED_VISION_SCREEN_MAGIC ||
       g_shared_vision_screen_ctrl->assistant_stream_enable == 0U ||
       input_snapshot == NULL)
    {
        return;
    }

    snapshot_request_seq = g_shared_vision_screen_ctrl->assistant_snapshot_request_seq;
    if(snapshot_request_seq == last_snapshot_request_seq)
    {
        return;
    }
    last_snapshot_request_seq = snapshot_request_seq;

    memcpy(g_cross_core_camera_assistant_image[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    SCB_CleanDCache_by_Addr(g_cross_core_camera_assistant_image[0], MT9V03X_IMAGE_SIZE);

    image_seq = g_shared_vision_screen_ctrl->assistant_image_seq + 1U;
    g_shared_vision_snapshot_metadata->magic = SHARED_VISION_SNAPSHOT_METADATA_MAGIC;
    g_shared_vision_snapshot_metadata->image_seq = image_seq;
    g_shared_vision_snapshot_metadata->input_seq = input_snapshot->seq;
    g_shared_vision_snapshot_metadata->roll_deg = input_snapshot->roll_deg;
    g_shared_vision_snapshot_metadata->pitch_deg = input_snapshot->pitch_deg;
    g_shared_vision_snapshot_metadata->yaw_deg = input_snapshot->yaw_deg;
    g_shared_vision_snapshot_metadata->tof_height_cm = input_snapshot->tof_height_cm;
    g_shared_vision_snapshot_metadata->fused_height_cm = input_snapshot->fused_height_cm;
    CrossCoreCamera_CleanSnapshotMetadata();

    g_shared_vision_screen_ctrl->assistant_image_display_mode = CROSS_CORE_CAMERA_VISION_DISPLAY_RAW;
    g_shared_vision_screen_ctrl->assistant_image_ready = 1U;
    g_shared_vision_screen_ctrl->assistant_image_seq = image_seq;
    CrossCoreCamera_CleanScreenCtrl();
}

static const char *CrossCoreCamera_CameraCoreVisionStatusText(uint8 status)
{
    switch(status)
    {
        case CROSS_CORE_CAMERA_VISION_STATUS_SENDING:
            return "SNAP SENDING...";
        case CROSS_CORE_CAMERA_VISION_STATUS_SENT:
            return "SNAP SENT";
        case CROSS_CORE_CAMERA_VISION_STATUS_TIMEOUT:
            return "SNAP TIMEOUT";
        case CROSS_CORE_CAMERA_VISION_STATUS_REQUEST_FAILED:
            return "SNAP REQUEST FAILED";
        case CROSS_CORE_CAMERA_VISION_STATUS_ASSISTANT_REQUIRED:
            return "DIP TO ASSISTANT FIRST";
        default:
            return "READY";
    }
}

static uint16 CrossCoreCamera_CameraCoreVisionStatusColor(uint8 status)
{
    switch(status)
    {
        case CROSS_CORE_CAMERA_VISION_STATUS_SENT:
            return RGB565_GREEN;
        case CROSS_CORE_CAMERA_VISION_STATUS_TIMEOUT:
        case CROSS_CORE_CAMERA_VISION_STATUS_REQUEST_FAILED:
            return RGB565_RED;
        case CROSS_CORE_CAMERA_VISION_STATUS_SENDING:
        case CROSS_CORE_CAMERA_VISION_STATUS_ASSISTANT_REQUIRED:
            return RGB565_YELLOW;
        default:
            return RGB565_WHITE;
    }
}

static void CrossCoreCamera_CameraCoreDrawVisionScreen(const VisionRuntimeDiag_t *diag)
{
    uint16 image_y = 20U;
    uint16 image_w = MT9V03X_W;
    uint16 image_h = MT9V03X_H;
    uint8 display_mode = g_shared_vision_screen_ctrl->display_mode;
    uint8 vision_status = g_shared_vision_screen_ctrl->vision_status;
    const uint8 *binary_display_image = VisionRuntime_GetBinaryDisplayImage();
    const uint8 *binary_image = VisionRuntime_GetBinaryImage();
    VisionRuntimeBeaconSlot_t beacon_slots[VISION_RUNTIME_BEACON_SLOT_COUNT] = {0};
    uint8 beacon_count = 0U;
    uint8 has_vehicle = 0U;

    if(display_mode == CROSS_CORE_CAMERA_VISION_DISPLAY_RAW)
    {
        ips200_show_gray_image(0U, image_y, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, image_w, image_h, 0U);
    }
    else if(display_mode == CROSS_CORE_CAMERA_VISION_DISPLAY_UNDISTORT)
    {
        IPM_GenerateUndistortedDebugImage(mt9v03x_image, s_vision_undistorted_image);
        ips200_show_gray_image(0U, image_y, s_vision_undistorted_image[0],
                               MT9V03X_W, MT9V03X_H, image_w, image_h, 0U);
    }
    else if(binary_display_image != NULL)
    {
        ips200_show_gray_image(0U, image_y, (uint8 *)binary_display_image, MT9V03X_W, MT9V03X_H, image_w, image_h, 0U);
    }
    else
    {
        ips200_show_gray_image(0U, image_y, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, image_w, image_h, 0U);
    }
    VisionRuntime_GetBeaconSlots(beacon_slots, VISION_RUNTIME_BEACON_SLOT_COUNT);
    for(uint8 i = 0U; i < VISION_RUNTIME_BEACON_SLOT_COUNT; i++)
    {
        if(!beacon_slots[i].valid)
        {
            continue;
        }

        beacon_count++;
        VisionOverlay_DrawBinaryContour(binary_image,
                                        beacon_slots[i].min_col,
                                        beacon_slots[i].max_col,
                                        beacon_slots[i].min_row,
                                        beacon_slots[i].max_row,
                                        RGB565_CYAN,
                                        image_y);
        VisionOverlay_DrawWorldPoint4x4(beacon_slots[i].x_cm,
                                        beacon_slots[i].y_cm,
                                        RGB565_RED,
                                        image_y);
    }

    {
        VehicleMarkerChevronDebug_t chevron_debug = {0};

        VehicleMarkerLocalization_GetChevronDebug(&chevron_debug);
        if(chevron_debug.valid)
        {
            if(chevron_debug.left_valid)
            {
                ips200_draw_line((uint16)chevron_debug.left_line_start_img.u,
                                 (uint16)(chevron_debug.left_line_start_img.v + image_y),
                                 (uint16)chevron_debug.left_line_end_img.u,
                                 (uint16)(chevron_debug.left_line_end_img.v + image_y),
                                 RGB565_GREEN);
                VisionOverlay_DrawImagePoint4x4((int)chevron_debug.left_tip_img.u,
                                                (int)chevron_debug.left_tip_img.v,
                                                RGB565_BLUE,
                                                image_y);
            }
            if(chevron_debug.right_valid)
            {
                ips200_draw_line((uint16)chevron_debug.right_line_start_img.u,
                                 (uint16)(chevron_debug.right_line_start_img.v + image_y),
                                 (uint16)chevron_debug.right_line_end_img.u,
                                 (uint16)(chevron_debug.right_line_end_img.v + image_y),
                                 RGB565_GREEN);
                VisionOverlay_DrawImagePoint4x4((int)chevron_debug.right_tip_img.u,
                                                (int)chevron_debug.right_tip_img.v,
                                                RGB565_BLUE,
                                                image_y);
            }

            VisionOverlay_DrawImagePoint4x4((int)chevron_debug.apex_img.u,
                                            (int)chevron_debug.apex_img.v,
                                            RGB565_RED,
                                            image_y);
        }
    }

    has_vehicle = (uint8)((g_vehicle_marker_pose.valid && !g_vehicle_marker_position_lost_flag) ? 1U : 0U);
    if(diag != NULL && diag->beacon_count >= 0)
    {
        beacon_count = (uint8)diag->beacon_count;
    }

    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    if(display_mode == CROSS_CORE_CAMERA_VISION_DISPLAY_RAW)
    {
        ips200_show_string(0U, 0U, "RAW IMG CM7_1");
    }
    else if(display_mode == CROSS_CORE_CAMERA_VISION_DISPLAY_UNDISTORT)
    {
        ips200_show_string(0U, 0U, "UNDISTORT CM7_1");
    }
    else
    {
        ips200_show_string(0U, 0U, "BIN IMG CM7_1");
    }
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0U, 152U, "B:");
    ips200_show_int(16U, 152U, beacon_count, 1U);
    ips200_show_string(56U, 152U, "MODE:");
#if VEHICLE_MARKER_MODE == VEHICLE_MARKER_MODE_CHEVRON
    ips200_show_string(104U, 152U, "CHEV");
#else
    ips200_show_string(104U, 152U, "4PT ");
#endif

    ips200_show_string(0U, 176U, "CAR:");
    ips200_show_string(32U, 176U, has_vehicle ? "YES" : "NO ");
    ips200_show_string(88U, 176U, "VALID:");
    ips200_show_int(136U, 176U, g_vehicle_marker_pose.valid ? 1 : 0, 1U);
    ips200_show_string(0U, 200U, "K1 IMG K3 SEND K4 BACK");
    ips200_show_string(0U, 224U, "                              ");
    ips200_set_color(CrossCoreCamera_CameraCoreVisionStatusColor(vision_status), RGB565_BLACK);
    ips200_show_string(0U, 224U, (char *)CrossCoreCamera_CameraCoreVisionStatusText(vision_status));
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

static void CrossCoreCamera_CameraCoreServiceVisionScreen(const VisionRuntimeDiag_t *diag)
{
    static uint8 vision_screen_active = 0U;
    static uint8 frame_divider = 0U;
    uint8 requested = 0U;

    CrossCoreCamera_InvalidateScreenCtrl();
    if(g_shared_vision_screen_ctrl->magic == SHARED_VISION_SCREEN_MAGIC &&
       g_shared_vision_screen_ctrl->requested_owner == CROSS_CORE_CAMERA_SCREEN_OWNER_VISION)
    {
        requested = 1U;
    }

    if(requested && !vision_screen_active)
    {
        ips200_set_dir(IPS200_PORTAIT);
        ips200_init(IPS200_TYPE_SPI);
        ips200_set_font(IPS200_8X16_FONT);
        ips200_clear();
        vision_screen_active = 1U;
        frame_divider = CROSS_CORE_CAMERA_SCREEN_FRAME_DIVIDER;
        CrossCoreCamera_SetCameraCoreScreenActive(1U);
    }
    else if(!requested && vision_screen_active)
    {
        vision_screen_active = 0U;
        frame_divider = 0U;
        CrossCoreCamera_SetCameraCoreScreenActive(0U);
    }

    if(!vision_screen_active)
    {
        return;
    }

    if(diag == NULL)
    {
        return;
    }

    frame_divider++;
    if(frame_divider >= CROSS_CORE_CAMERA_SCREEN_FRAME_DIVIDER)
    {
        frame_divider = 0U;
        CrossCoreCamera_CameraCoreDrawVisionScreen(diag);
    }
}

static void CrossCoreCamera_CameraCoreIpcCallback(uint32 ipc_data)
{
    (void)ipc_data;
}
#endif

#if CY_CORE_CM7_0
static uint8 s_cross_core_camera_capture_ready = 0U;

static void CrossCoreCamera_ControlCorePublishFrame(void)
{
    SCB_CleanInvalidateDCache_by_Addr(&g_cross_core_camera_image_fresh_flag, 1);
    if(g_cross_core_camera_image_fresh_flag != 0U)
    {
        return;
    }

    memcpy(g_cross_core_camera_image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    SCB_CleanInvalidateDCache_by_Addr(g_cross_core_camera_image_copy[0], MT9V03X_IMAGE_SIZE);
    g_cross_core_camera_image_fresh_flag = 1U;
    SCB_CleanInvalidateDCache_by_Addr(&g_cross_core_camera_image_fresh_flag, 1);
}

static void CrossCoreCamera_CleanInput(void)
{
    SCB_CleanDCache_by_Addr((void *)g_shared_vision_input, sizeof(SharedVisionInput_t));
}

static void CrossCoreCamera_InvalidateResult(void)
{
    SCB_InvalidateDCache_by_Addr((void *)g_shared_vision_result, sizeof(SharedVisionResult_t));
}

static void CrossCoreCamera_CleanResult(void)
{
    SCB_CleanDCache_by_Addr((void *)g_shared_vision_result, sizeof(SharedVisionResult_t));
}

static void CrossCoreCamera_InvalidateScreenCtrl(void)
{
    SCB_InvalidateDCache_by_Addr((void *)g_shared_vision_screen_ctrl, sizeof(SharedVisionScreenCtrl_t));
}

static void CrossCoreCamera_CleanScreenCtrl(void)
{
    SCB_CleanDCache_by_Addr((void *)g_shared_vision_screen_ctrl, sizeof(SharedVisionScreenCtrl_t));
}

static volatile uint8 s_vision_ipc_result_pending = 0U;

static void CrossCoreCamera_ControlCoreIpcCallback(uint32 ipc_data)
{
    if(ipc_data == CROSS_CORE_VISION_IPC_RESULT_READY)
    {
        s_vision_ipc_result_pending = 1U;
    }
}
#endif

void CrossCoreCamera_CameraCoreInit(void)
{
#if CY_CORE_CM7_1
    VehicleMarkerLocalization_Init();
    BeaconDetect_Init();
    IPM_Interface_Init();
    ipc_communicate_init(IPC_PORT_2, CrossCoreCamera_CameraCoreIpcCallback);
    g_shared_vision_input->magic = SHARED_VISION_MAGIC;
    g_shared_vision_input->seq = 0U;
    g_shared_vision_input->roll_deg = 0.0f;
    g_shared_vision_input->pitch_deg = 0.0f;
    g_shared_vision_input->yaw_deg = 0.0f;
    g_shared_vision_input->tof_height_cm = 100.0f;
    g_shared_vision_input->fused_height_cm = 100.0f;
    g_shared_vision_snapshot_metadata->magic = 0U;
    g_shared_vision_snapshot_metadata->image_seq = 0U;
    g_shared_vision_snapshot_metadata->input_seq = 0U;
    g_shared_vision_snapshot_metadata->roll_deg = 0.0f;
    g_shared_vision_snapshot_metadata->pitch_deg = 0.0f;
    g_shared_vision_snapshot_metadata->yaw_deg = 0.0f;
    g_shared_vision_snapshot_metadata->tof_height_cm = 100.0f;
    g_shared_vision_snapshot_metadata->fused_height_cm = 100.0f;
    g_shared_vision_result->magic = SHARED_VISION_MAGIC;
    g_shared_vision_result->seq = 0U;
    g_shared_vision_screen_ctrl->magic = SHARED_VISION_SCREEN_MAGIC;
    g_shared_vision_screen_ctrl->request_seq = 0U;
    g_shared_vision_screen_ctrl->active_seq = 0U;
    g_shared_vision_screen_ctrl->assistant_image_seq = 0U;
    g_shared_vision_screen_ctrl->assistant_snapshot_request_seq = 0U;
    g_shared_vision_screen_ctrl->requested_owner = CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
    g_shared_vision_screen_ctrl->active_owner = CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
    g_shared_vision_screen_ctrl->display_mode = CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY;
    g_shared_vision_screen_ctrl->assistant_stream_enable = 0U;
    g_shared_vision_screen_ctrl->assistant_image_display_mode = CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY;
    g_shared_vision_screen_ctrl->assistant_image_ready = 0U;
    g_shared_vision_screen_ctrl->vision_status = CROSS_CORE_CAMERA_VISION_STATUS_IDLE;
    CrossCoreCamera_CleanInput();
    CrossCoreCamera_CleanResult();
    CrossCoreCamera_CleanScreenCtrl();
    CrossCoreCamera_CleanSnapshotMetadata();
#endif
}

void CrossCoreCamera_CameraCoreTask(void)
{
#if CY_CORE_CM7_1
    // 本帧输入快照：由 0 核发布的姿态与高度信息。
    SharedVisionInput_t input_snapshot;
    // 本帧视觉诊断信息：用于屏幕显示和跨核回传。
    VisionRuntimeDiag_t diag_snapshot;
    // 本帧信标结果槽位：用于把 1 核识别结果同步给 0 核。
    VisionRuntimeBeaconSlot_t beacon_slots[VISION_RUNTIME_BEACON_SLOT_COUNT] = {0};
    // 小车定位丢失调试信息。
    VehicleMarkerLossDebug_t loss_debug;

    if(!CrossCoreCamera_CameraCoreAcquireFrame())
    {
        CrossCoreCamera_CameraCoreServiceVisionScreen(NULL);
        return;
    }

    // 按时间推进“小车位置丢失保护”计时，避免长时间无更新时状态失真。
    CrossCoreCamera_AgeVisionLossTimer();

    // 从共享内存读取 0 核发布的姿态与高度输入。
    CrossCoreCamera_InvalidateInput();
    input_snapshot = *g_shared_vision_input;
    // 若共享区尚未初始化，则使用默认姿态和默认高度兜底。
    if(input_snapshot.magic != SHARED_VISION_MAGIC)
    {
        input_snapshot.roll_deg = 0.0f;
        input_snapshot.pitch_deg = 0.0f;
        input_snapshot.yaw_deg = 0.0f;
        input_snapshot.tof_height_cm = 100.0f;
        input_snapshot.fused_height_cm = 100.0f;
    }

    // 执行本帧完整视觉处理：信标识别 + 小车标志识别。
    VisionRuntime_ProcessFrameWithPose(input_snapshot.roll_deg,
                                       input_snapshot.pitch_deg,
                                       input_snapshot.tof_height_cm);
    // 读取本帧诊断、小车丢失调试信息和信标槽位结果。
    VisionRuntime_GetDiag(&diag_snapshot);
    VisionRuntime_GetBeaconSlots(beacon_slots, VISION_RUNTIME_BEACON_SLOT_COUNT);
    VehicleMarkerLocalization_GetLossDebug(&loss_debug);
    CrossCoreCamera_CameraCorePublishAssistantSnapshot(&input_snapshot);

    // 将 1 核本帧结果写回共享区，供 0 核读取并参与控制/显示。
    g_shared_vision_result->magic = SHARED_VISION_MAGIC;
    g_shared_vision_result->pose = g_vehicle_marker_pose;
    g_shared_vision_result->diag = diag_snapshot;
    memcpy((void *)g_shared_vision_result->beacon_slots, beacon_slots, sizeof(beacon_slots));
    g_shared_vision_result->loss_debug = loss_debug;
    g_shared_vision_result->lost_flag = g_vehicle_marker_position_lost_flag;
    g_shared_vision_result->lost_elapsed_ms = g_vehicle_marker_position_lost_elapsed_ms;
    g_shared_vision_result->pose_update_seq = g_vehicle_marker_pose_update_seq;
    g_shared_vision_result->seq++;
    // 写回共享结果前清缓存，并通过 IPC 通知 0 核“新结果已就绪”。
    CrossCoreCamera_CleanResult();
    (void)ipc_send_data(CROSS_CORE_VISION_IPC_RESULT_READY);
    // 按请求刷新 1 核视觉调试屏幕。
    CrossCoreCamera_CameraCoreServiceVisionScreen(&diag_snapshot);

#endif

}

uint8 CrossCoreCamera_ControlCoreFetchFrame(void)
{
#if CY_CORE_CM7_0
    static uint32 last_seq = 0U;
    SharedVisionResult_t result_snapshot;

    CrossCoreCamera_InvalidateResult();
    result_snapshot = *g_shared_vision_result;
    if(result_snapshot.magic != SHARED_VISION_MAGIC)
    {
        return 0U;
    }

    if(result_snapshot.seq == last_seq)
    {
        s_vision_ipc_result_pending = 0U;
        return 0U;
    }

    last_seq = result_snapshot.seq;
    s_vision_ipc_result_pending = 0U;
    g_vehicle_marker_pose = result_snapshot.pose;
    g_vehicle_marker_position_lost_flag = result_snapshot.lost_flag;
    g_vehicle_marker_position_lost_elapsed_ms = result_snapshot.lost_elapsed_ms;
    g_vehicle_marker_pose_update_seq = result_snapshot.pose_update_seq;
    VisionRuntime_SetDiag(&result_snapshot.diag);
    VisionRuntime_SetBeaconSlots(result_snapshot.beacon_slots, VISION_RUNTIME_BEACON_SLOT_COUNT);
    VehicleMarkerLocalization_SetLossDebug(&result_snapshot.loss_debug);
    return 1U;
#else
    return 0U;
#endif
}

#if CY_CORE_CM7_0
void CrossCoreCamera_ControlCoreTask(void)
{
    if(!s_cross_core_camera_capture_ready)
    {
        return;
    }

    if(mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0U;
        CrossCoreCamera_ControlCorePublishFrame();
    }
}

uint8 CrossCoreCamera_ControlCoreService(float roll_deg,
                                         float pitch_deg,
                                         float yaw_deg,
                                         float tof_height_cm,
                                         float fused_height_cm)
{
    static uint8 last_assistant_stream_enable = 0xFFU;
    uint8 camera_new_frame = 0U;
    uint8 assistant_stream_enable = 0U;

    if(CommMux_UpdateFromSwitch())
    {
        Vofa_ClearRxState();
    }

    assistant_stream_enable = CommMux_IsAssistantImageMode();

    if(last_assistant_stream_enable != assistant_stream_enable)
    {
        CrossCoreCamera_ControlCoreSetVisionAssistantStream(assistant_stream_enable);
        last_assistant_stream_enable = assistant_stream_enable;
    }

    CrossCoreCamera_ControlCoreTask();
    CrossCoreCamera_ControlCorePublishInput(roll_deg,
                                            pitch_deg,
                                            yaw_deg,
                                            tof_height_cm,
                                            fused_height_cm);
    camera_new_frame = CrossCoreCamera_ControlCoreFetchFrame();
    return camera_new_frame;
}

void CrossCoreCamera_ControlCorePublishInput(float roll_deg,
                                             float pitch_deg,
                                             float yaw_deg,
                                             float tof_height_cm,
                                             float fused_height_cm)
{
    g_shared_vision_input->magic = SHARED_VISION_MAGIC;
    g_shared_vision_input->roll_deg = roll_deg;
    g_shared_vision_input->pitch_deg = pitch_deg;
    g_shared_vision_input->yaw_deg = yaw_deg;
    g_shared_vision_input->tof_height_cm = tof_height_cm;
    g_shared_vision_input->fused_height_cm = fused_height_cm;
    g_shared_vision_input->seq++;
    CrossCoreCamera_CleanInput();
}

uint32 CrossCoreCamera_ControlCoreResultPending(void)
{
    return (uint32)s_vision_ipc_result_pending;
}

uint32 seekfree_assistant_transfer(const uint8 *buff, uint32 length)
{
    if(!CommMux_IsAssistantImageMode())
    {
        return length;
    }

    uart_write_buffer(UART_INDEX, buff, length);
    return 0U;
}

uint32 seekfree_assistant_receive(uint8 *buff, uint32 length)
{
    (void)buff;
    (void)length;
    return 0U;
}

void CrossCoreCamera_ControlCoreAssistantInit(void)
{
    ipc_communicate_init(IPC_PORT_1, CrossCoreCamera_ControlCoreIpcCallback);
    s_cross_core_camera_capture_ready = (mt9v03x_init() == 0U) ? 1U : 0U;
    g_cross_core_camera_image_fresh_flag = 0U;
    g_shared_vision_input->magic = SHARED_VISION_MAGIC;
    g_shared_vision_input->seq = 0U;
    g_shared_vision_input->roll_deg = 0.0f;
    g_shared_vision_input->pitch_deg = 0.0f;
    g_shared_vision_input->yaw_deg = 0.0f;
    g_shared_vision_input->tof_height_cm = 100.0f;
    g_shared_vision_input->fused_height_cm = 100.0f;
    g_shared_vision_snapshot_metadata->magic = 0U;
    g_shared_vision_snapshot_metadata->image_seq = 0U;
    g_shared_vision_snapshot_metadata->input_seq = 0U;
    g_shared_vision_snapshot_metadata->roll_deg = 0.0f;
    g_shared_vision_snapshot_metadata->pitch_deg = 0.0f;
    g_shared_vision_snapshot_metadata->yaw_deg = 0.0f;
    g_shared_vision_snapshot_metadata->tof_height_cm = 100.0f;
    g_shared_vision_snapshot_metadata->fused_height_cm = 100.0f;
    g_shared_vision_result->magic = 0U;
    g_shared_vision_result->seq = 0U;
    g_shared_vision_screen_ctrl->magic = SHARED_VISION_SCREEN_MAGIC;
    g_shared_vision_screen_ctrl->request_seq = 0U;
    g_shared_vision_screen_ctrl->active_seq = 0U;
    g_shared_vision_screen_ctrl->assistant_image_seq = 0U;
    g_shared_vision_screen_ctrl->assistant_snapshot_request_seq = 0U;
    g_shared_vision_screen_ctrl->requested_owner = CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
    g_shared_vision_screen_ctrl->active_owner = CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
    g_shared_vision_screen_ctrl->display_mode = CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY;
    g_shared_vision_screen_ctrl->assistant_stream_enable = 0U;
    g_shared_vision_screen_ctrl->assistant_image_display_mode = CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY;
    g_shared_vision_screen_ctrl->assistant_image_ready = 0U;
    g_shared_vision_screen_ctrl->vision_status = CROSS_CORE_CAMERA_VISION_STATUS_IDLE;
    CrossCoreCamera_CleanInput();
    CrossCoreCamera_CleanResult();
    CrossCoreCamera_CleanScreenCtrl();
    CrossCoreCamera_CleanSnapshotMetadata();
    SCB_CleanInvalidateDCache_by_Addr(&g_cross_core_camera_image_fresh_flag, 1);
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_CUSTOM);
}

uint8 CrossCoreCamera_ControlCoreRequestAssistantSnapshot(void)
{
    if(!CommMux_IsAssistantImageMode())
    {
        return 0U;
    }

    CrossCoreCamera_InvalidateScreenCtrl();
    if(g_shared_vision_screen_ctrl->magic != SHARED_VISION_SCREEN_MAGIC)
    {
        g_shared_vision_screen_ctrl->magic = SHARED_VISION_SCREEN_MAGIC;
        g_shared_vision_screen_ctrl->request_seq = 0U;
        g_shared_vision_screen_ctrl->active_seq = 0U;
        g_shared_vision_screen_ctrl->assistant_image_seq = 0U;
        g_shared_vision_screen_ctrl->assistant_snapshot_request_seq = 0U;
        g_shared_vision_screen_ctrl->requested_owner = CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
        g_shared_vision_screen_ctrl->active_owner = CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
        g_shared_vision_screen_ctrl->display_mode = CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY;
        g_shared_vision_screen_ctrl->assistant_image_display_mode = CROSS_CORE_CAMERA_VISION_DISPLAY_RAW;
        g_shared_vision_screen_ctrl->vision_status = CROSS_CORE_CAMERA_VISION_STATUS_IDLE;
    }

    g_shared_vision_screen_ctrl->assistant_stream_enable = 1U;
    g_shared_vision_screen_ctrl->assistant_image_ready = 0U;
    g_shared_vision_screen_ctrl->assistant_snapshot_request_seq++;
    g_shared_vision_screen_ctrl->request_seq++;
    CrossCoreCamera_CleanScreenCtrl();
    return 1U;
}

static void CrossCoreCamera_MetadataWriteU32(uint8 *buffer, uint32 value)
{
    buffer[0] = (uint8)(value & 0xFFU);
    buffer[1] = (uint8)((value >> 8) & 0xFFU);
    buffer[2] = (uint8)((value >> 16) & 0xFFU);
    buffer[3] = (uint8)((value >> 24) & 0xFFU);
}

static void CrossCoreCamera_MetadataWriteFloat(uint8 *buffer, float value)
{
    uint32 bits = 0U;

    memcpy(&bits, &value, sizeof(bits));
    CrossCoreCamera_MetadataWriteU32(buffer, bits);
}

static void CrossCoreCamera_SendSnapshotMetadata(const SharedVisionSnapshotMetadata_t *metadata)
{
    uint8 packet[CROSS_CORE_CAMERA_METADATA_PACKET_SIZE] = {0};
    uint8 checksum = 0U;
    uint8 i;

    packet[0] = SEEKFREE_ASSISTANT_SEND_HEAD;
    packet[1] = CROSS_CORE_CAMERA_METADATA_FUNCTION;
    packet[2] = CROSS_CORE_CAMERA_METADATA_VERSION;
    packet[3] = CROSS_CORE_CAMERA_METADATA_PACKET_SIZE;
    CrossCoreCamera_MetadataWriteU32(&packet[4], metadata->image_seq);
    CrossCoreCamera_MetadataWriteU32(&packet[8], metadata->input_seq);
    CrossCoreCamera_MetadataWriteFloat(&packet[12], metadata->roll_deg);
    CrossCoreCamera_MetadataWriteFloat(&packet[16], metadata->pitch_deg);
    CrossCoreCamera_MetadataWriteFloat(&packet[20], metadata->yaw_deg);
    CrossCoreCamera_MetadataWriteFloat(&packet[24], metadata->tof_height_cm);
    CrossCoreCamera_MetadataWriteFloat(&packet[28], metadata->fused_height_cm);

    for(i = 0U; i < (CROSS_CORE_CAMERA_METADATA_PACKET_SIZE - 1U); i++)
    {
        checksum = (uint8)(checksum + packet[i]);
    }
    packet[CROSS_CORE_CAMERA_METADATA_PACKET_SIZE - 1U] = checksum;
    seekfree_assistant_transfer(packet, CROSS_CORE_CAMERA_METADATA_PACKET_SIZE);
}

uint8 CrossCoreCamera_ControlCoreSendAssistantFrame(void)
{
    static uint32 last_sent_seq = 0U;
    uint32 image_seq = 0U;
    SharedVisionSnapshotMetadata_t metadata;

    if(!CommMux_IsAssistantImageMode())
    {
        return 0U;
    }

    CrossCoreCamera_InvalidateScreenCtrl();
    if(g_shared_vision_screen_ctrl->magic != SHARED_VISION_SCREEN_MAGIC ||
       g_shared_vision_screen_ctrl->assistant_stream_enable == 0U ||
       g_shared_vision_screen_ctrl->assistant_image_ready == 0U)
    {
        return 0U;
    }

    image_seq = g_shared_vision_screen_ctrl->assistant_image_seq;
    if(image_seq == last_sent_seq)
    {
        return 0U;
    }

    CrossCoreCamera_InvalidateSnapshotMetadata();
    metadata = *g_shared_vision_snapshot_metadata;
    if(metadata.magic == SHARED_VISION_SNAPSHOT_METADATA_MAGIC &&
       metadata.image_seq == image_seq)
    {
        CrossCoreCamera_SendSnapshotMetadata(&metadata);
    }

    SCB_InvalidateDCache_by_Addr(g_cross_core_camera_assistant_image[0], MT9V03X_IMAGE_SIZE);
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X,
                                                 g_cross_core_camera_assistant_image[0],
                                                 MT9V03X_W,
                                                 MT9V03X_H);
    seekfree_assistant_camera_send();
    last_sent_seq = image_seq;
    return 1U;
}

void CrossCoreCamera_ControlCoreRequestVisionScreen(uint8 enable)
{
    CrossCoreCamera_InvalidateScreenCtrl();
    if(g_shared_vision_screen_ctrl->magic != SHARED_VISION_SCREEN_MAGIC)
    {
        g_shared_vision_screen_ctrl->magic = SHARED_VISION_SCREEN_MAGIC;
        g_shared_vision_screen_ctrl->request_seq = 0U;
        g_shared_vision_screen_ctrl->active_seq = 0U;
        g_shared_vision_screen_ctrl->assistant_image_seq = 0U;
        g_shared_vision_screen_ctrl->assistant_snapshot_request_seq = 0U;
        g_shared_vision_screen_ctrl->active_owner = CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
        g_shared_vision_screen_ctrl->display_mode = CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY;
        g_shared_vision_screen_ctrl->assistant_stream_enable = 0U;
        g_shared_vision_screen_ctrl->assistant_image_display_mode = CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY;
        g_shared_vision_screen_ctrl->assistant_image_ready = 0U;
        g_shared_vision_screen_ctrl->vision_status = CROSS_CORE_CAMERA_VISION_STATUS_IDLE;
    }

    g_shared_vision_screen_ctrl->requested_owner = enable ? CROSS_CORE_CAMERA_SCREEN_OWNER_VISION
                                                          : CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
    g_shared_vision_screen_ctrl->request_seq++;
    CrossCoreCamera_CleanScreenCtrl();
}

void CrossCoreCamera_ControlCoreSetVisionDisplayMode(uint8 mode)
{
    CrossCoreCamera_InvalidateScreenCtrl();
    if(g_shared_vision_screen_ctrl->magic != SHARED_VISION_SCREEN_MAGIC)
    {
        g_shared_vision_screen_ctrl->magic = SHARED_VISION_SCREEN_MAGIC;
        g_shared_vision_screen_ctrl->request_seq = 0U;
        g_shared_vision_screen_ctrl->active_seq = 0U;
        g_shared_vision_screen_ctrl->assistant_image_seq = 0U;
        g_shared_vision_screen_ctrl->assistant_snapshot_request_seq = 0U;
        g_shared_vision_screen_ctrl->requested_owner = CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
        g_shared_vision_screen_ctrl->active_owner = CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
        g_shared_vision_screen_ctrl->assistant_stream_enable = 0U;
        g_shared_vision_screen_ctrl->assistant_image_display_mode = CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY;
        g_shared_vision_screen_ctrl->assistant_image_ready = 0U;
        g_shared_vision_screen_ctrl->vision_status = CROSS_CORE_CAMERA_VISION_STATUS_IDLE;
    }

    if(mode != CROSS_CORE_CAMERA_VISION_DISPLAY_RAW &&
       mode != CROSS_CORE_CAMERA_VISION_DISPLAY_UNDISTORT)
    {
        mode = CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY;
    }

    g_shared_vision_screen_ctrl->display_mode = mode;
    g_shared_vision_screen_ctrl->request_seq++;
    CrossCoreCamera_CleanScreenCtrl();
}

uint8 CrossCoreCamera_ControlCoreGetVisionDisplayMode(void)
{
    CrossCoreCamera_InvalidateScreenCtrl();
    if(g_shared_vision_screen_ctrl->magic != SHARED_VISION_SCREEN_MAGIC)
    {
        return CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY;
    }

    if(g_shared_vision_screen_ctrl->display_mode == CROSS_CORE_CAMERA_VISION_DISPLAY_RAW)
    {
        return CROSS_CORE_CAMERA_VISION_DISPLAY_RAW;
    }
    if(g_shared_vision_screen_ctrl->display_mode == CROSS_CORE_CAMERA_VISION_DISPLAY_UNDISTORT)
    {
        return CROSS_CORE_CAMERA_VISION_DISPLAY_UNDISTORT;
    }
    return CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY;
}

void CrossCoreCamera_ControlCoreSetVisionStatus(uint8 status)
{
    if(status > CROSS_CORE_CAMERA_VISION_STATUS_ASSISTANT_REQUIRED)
    {
        status = CROSS_CORE_CAMERA_VISION_STATUS_IDLE;
    }

    CrossCoreCamera_InvalidateScreenCtrl();
    if(g_shared_vision_screen_ctrl->magic != SHARED_VISION_SCREEN_MAGIC)
    {
        g_shared_vision_screen_ctrl->magic = SHARED_VISION_SCREEN_MAGIC;
        g_shared_vision_screen_ctrl->request_seq = 0U;
        g_shared_vision_screen_ctrl->active_seq = 0U;
        g_shared_vision_screen_ctrl->assistant_image_seq = 0U;
        g_shared_vision_screen_ctrl->assistant_snapshot_request_seq = 0U;
        g_shared_vision_screen_ctrl->requested_owner = CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
        g_shared_vision_screen_ctrl->active_owner = CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
        g_shared_vision_screen_ctrl->display_mode = CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY;
        g_shared_vision_screen_ctrl->assistant_stream_enable = 0U;
        g_shared_vision_screen_ctrl->assistant_image_display_mode = CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY;
        g_shared_vision_screen_ctrl->assistant_image_ready = 0U;
    }

    g_shared_vision_screen_ctrl->vision_status = status;
    g_shared_vision_screen_ctrl->request_seq++;
    CrossCoreCamera_CleanScreenCtrl();
}

void CrossCoreCamera_ControlCoreSetVisionAssistantStream(uint8 enable)
{
    CrossCoreCamera_InvalidateScreenCtrl();
    if(g_shared_vision_screen_ctrl->magic != SHARED_VISION_SCREEN_MAGIC)
    {
        g_shared_vision_screen_ctrl->magic = SHARED_VISION_SCREEN_MAGIC;
        g_shared_vision_screen_ctrl->request_seq = 0U;
        g_shared_vision_screen_ctrl->active_seq = 0U;
        g_shared_vision_screen_ctrl->assistant_image_seq = 0U;
        g_shared_vision_screen_ctrl->assistant_snapshot_request_seq = 0U;
        g_shared_vision_screen_ctrl->requested_owner = CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
        g_shared_vision_screen_ctrl->active_owner = CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
        g_shared_vision_screen_ctrl->display_mode = CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY;
        g_shared_vision_screen_ctrl->assistant_image_display_mode = CROSS_CORE_CAMERA_VISION_DISPLAY_BINARY;
        g_shared_vision_screen_ctrl->assistant_image_ready = 0U;
        g_shared_vision_screen_ctrl->vision_status = CROSS_CORE_CAMERA_VISION_STATUS_IDLE;
    }

    g_shared_vision_screen_ctrl->assistant_stream_enable = enable ? 1U : 0U;
    if(!enable)
    {
        g_shared_vision_screen_ctrl->assistant_image_ready = 0U;
    }
    g_shared_vision_screen_ctrl->request_seq++;
    CrossCoreCamera_CleanScreenCtrl();
}

uint8 CrossCoreCamera_ControlCoreGetVisionAssistantStream(void)
{
    CrossCoreCamera_InvalidateScreenCtrl();
    if(g_shared_vision_screen_ctrl->magic != SHARED_VISION_SCREEN_MAGIC)
    {
        return 0U;
    }

    return g_shared_vision_screen_ctrl->assistant_stream_enable ? 1U : 0U;
}

uint8 CrossCoreCamera_ControlCoreIsVisionScreenActive(void)
{
    CrossCoreCamera_InvalidateScreenCtrl();
    if(g_shared_vision_screen_ctrl->magic != SHARED_VISION_SCREEN_MAGIC)
    {
        return 0U;
    }

    return (g_shared_vision_screen_ctrl->active_owner == CROSS_CORE_CAMERA_SCREEN_OWNER_VISION) ? 1U : 0U;
}
#endif
