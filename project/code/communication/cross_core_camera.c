#include "communication/cross_core_camera.h"
#include "common/project_headfile.h"

#define SHARED_VISION_INPUT_ADDR                    0x28001000u
#define SHARED_VISION_RESULT_ADDR                   0x28001100u
#define SHARED_VISION_SCREEN_CTRL_ADDR              0x28001200u
#define SHARED_VISION_MAGIC                         0x56495331u
#define SHARED_VISION_SCREEN_MAGIC                  0x56534331u
#define CROSS_CORE_CAMERA_ASSISTANT_IMAGE_ENABLE    0U
#define CROSS_CORE_CAMERA_ASSISTANT_FRAME_DIVIDER   10U
#define CROSS_CORE_CAMERA_SCREEN_FRAME_DIVIDER      5U
#define CROSS_CORE_VISION_IPC_RESULT_READY          0x56495352u
#define CROSS_CORE_CAMERA_DWT_LAR_UNLOCK            0xC5ACCE55UL

#define CROSS_CORE_CAMERA_SCREEN_OWNER_NONE         0U
#define CROSS_CORE_CAMERA_SCREEN_OWNER_VISION       1U

typedef struct
{
    uint32 magic;
    uint32 seq;
    float roll_deg;
    float pitch_deg;
    float height_cm;
} SharedVisionInput_t;

typedef struct
{
    uint32 magic;
    uint32 seq;
    IR_Pose_t pose;
    VisionRuntimeDiag_t diag;
    IR_LossDebug_t loss_debug;
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
    uint8 requested_owner;
    uint8 active_owner;
    uint8 reserved[2];
} SharedVisionScreenCtrl_t;

static volatile SharedVisionInput_t *const g_shared_vision_input =
    (volatile SharedVisionInput_t *)SHARED_VISION_INPUT_ADDR;
static volatile SharedVisionResult_t *const g_shared_vision_result =
    (volatile SharedVisionResult_t *)SHARED_VISION_RESULT_ADDR;
static volatile SharedVisionScreenCtrl_t *const g_shared_vision_screen_ctrl =
    (volatile SharedVisionScreenCtrl_t *)SHARED_VISION_SCREEN_CTRL_ADDR;

#if CY_CORE_CM7_1
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
        IR_Localization_Task_5ms();
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
    tick_count = elapsed_ms / IR_POSITION_LOST_TASK_PERIOD_MS;
    residual_ms = elapsed_ms % IR_POSITION_LOST_TASK_PERIOD_MS;

    max_tick_count = (IR_POSITION_LOST_HOLD_MS / IR_POSITION_LOST_TASK_PERIOD_MS) + 1U;
    if(tick_count > max_tick_count)
    {
        tick_count = max_tick_count;
    }

    for(i = 0U; i < tick_count; i++)
    {
        IR_Localization_Task_5ms();
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

static void CrossCoreCamera_CameraCoreDrawVisionScreen(const VisionRuntimeDiag_t *diag)
{
    uint16 image_y = 20U;
    uint16 image_w = MT9V03X_W;
    uint16 image_h = MT9V03X_H;

    ips200_show_gray_image(0U, image_y, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, image_w, image_h, 0U);

    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(0U, 0U, "RAW IMG CM7_1");
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0U, 152U, "IR:");
    ips200_show_string(32U, 152U, ir_position_lost_flag ? "LOST " : "OK   ");
    ips200_show_string(96U, 152U, "PTS:");
    ips200_show_int(136U, 152U, diag != NULL ? diag->ir_detected_point_count : 0, 1U);

    ips200_show_string(0U, 176U, "X:");
    ips200_show_float(32U, 176U, ir_pose_estimate.x, 4U, 1U);
    ips200_show_string(120U, 176U, "Y:");
    ips200_show_float(152U, 176U, ir_pose_estimate.y, 4U, 1U);
    ips200_show_string(0U, 200U, "K4 BACK");
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
    (void)mt9v03x_init();
    IR_Localization_Init();
    BeaconDetect_Init();
    IPM_Interface_Init();
    ipc_communicate_init(IPC_PORT_2, CrossCoreCamera_CameraCoreIpcCallback);
#if CROSS_CORE_CAMERA_ASSISTANT_IMAGE_ENABLE
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
#endif
    g_shared_vision_input->magic = SHARED_VISION_MAGIC;
    g_shared_vision_input->seq = 0U;
    g_shared_vision_input->roll_deg = 0.0f;
    g_shared_vision_input->pitch_deg = 0.0f;
    g_shared_vision_input->height_cm = 100.0f;
    g_shared_vision_result->magic = SHARED_VISION_MAGIC;
    g_shared_vision_result->seq = 0U;
    g_shared_vision_screen_ctrl->magic = SHARED_VISION_SCREEN_MAGIC;
    g_shared_vision_screen_ctrl->request_seq = 0U;
    g_shared_vision_screen_ctrl->active_seq = 0U;
    g_shared_vision_screen_ctrl->requested_owner = CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
    g_shared_vision_screen_ctrl->active_owner = CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
    CrossCoreCamera_CleanInput();
    CrossCoreCamera_CleanResult();
    CrossCoreCamera_CleanScreenCtrl();
#endif
}

void CrossCoreCamera_CameraCoreTask(void)
{
#if CY_CORE_CM7_1
#if CROSS_CORE_CAMERA_ASSISTANT_IMAGE_ENABLE
    static uint8 assistant_frame_divider = 0U;
#endif
    SharedVisionInput_t input_snapshot;
    VisionRuntimeDiag_t diag_snapshot;
    IR_LossDebug_t loss_debug;

    if(!mt9v03x_finish_flag)
    {
        CrossCoreCamera_CameraCoreServiceVisionScreen(NULL);
        return;
    }

    CrossCoreCamera_AgeVisionLossTimer();

    CrossCoreCamera_InvalidateInput();
    input_snapshot = *g_shared_vision_input;
    if(input_snapshot.magic != SHARED_VISION_MAGIC)
    {
        input_snapshot.roll_deg = 0.0f;
        input_snapshot.pitch_deg = 0.0f;
        input_snapshot.height_cm = 100.0f;
    }

    VisionRuntime_ProcessFrameWithPose(input_snapshot.roll_deg,
                                       input_snapshot.pitch_deg,
                                       input_snapshot.height_cm);
    VisionRuntime_GetDiag(&diag_snapshot);
    IR_Localization_GetLossDebug(&loss_debug);

    g_shared_vision_result->magic = SHARED_VISION_MAGIC;
    g_shared_vision_result->pose = ir_pose_estimate;
    g_shared_vision_result->diag = diag_snapshot;
    g_shared_vision_result->loss_debug = loss_debug;
    g_shared_vision_result->lost_flag = ir_position_lost_flag;
    g_shared_vision_result->lost_elapsed_ms = ir_position_lost_elapsed_ms;
    g_shared_vision_result->pose_update_seq = ir_pose_update_seq;
    g_shared_vision_result->seq++;
    CrossCoreCamera_CleanResult();
    (void)ipc_send_data(CROSS_CORE_VISION_IPC_RESULT_READY);
    CrossCoreCamera_CameraCoreServiceVisionScreen(&diag_snapshot);

#if CROSS_CORE_CAMERA_ASSISTANT_IMAGE_ENABLE
    assistant_frame_divider++;
    if(assistant_frame_divider >= CROSS_CORE_CAMERA_ASSISTANT_FRAME_DIVIDER)
    {
        assistant_frame_divider = 0U;
        seekfree_assistant_camera_send();
    }
#endif
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
    ir_pose_estimate = result_snapshot.pose;
    ir_position_lost_flag = result_snapshot.lost_flag;
    ir_position_lost_elapsed_ms = result_snapshot.lost_elapsed_ms;
    ir_pose_update_seq = result_snapshot.pose_update_seq;
    VisionRuntime_SetDiag(&result_snapshot.diag);
    IR_Localization_SetLossDebug(&result_snapshot.loss_debug);
    return 1U;
#else
    return 0U;
#endif
}

#if CY_CORE_CM7_0
void CrossCoreCamera_ControlCorePublishInput(float roll_deg, float pitch_deg, float height_cm)
{
    g_shared_vision_input->magic = SHARED_VISION_MAGIC;
    g_shared_vision_input->roll_deg = roll_deg;
    g_shared_vision_input->pitch_deg = pitch_deg;
    g_shared_vision_input->height_cm = height_cm;
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
    g_shared_vision_input->magic = SHARED_VISION_MAGIC;
    g_shared_vision_input->seq = 0U;
    g_shared_vision_input->roll_deg = 0.0f;
    g_shared_vision_input->pitch_deg = 0.0f;
    g_shared_vision_input->height_cm = 100.0f;
    g_shared_vision_result->magic = 0U;
    g_shared_vision_result->seq = 0U;
    g_shared_vision_screen_ctrl->magic = SHARED_VISION_SCREEN_MAGIC;
    g_shared_vision_screen_ctrl->request_seq = 0U;
    g_shared_vision_screen_ctrl->active_seq = 0U;
    g_shared_vision_screen_ctrl->requested_owner = CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
    g_shared_vision_screen_ctrl->active_owner = CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
    CrossCoreCamera_CleanInput();
    CrossCoreCamera_CleanResult();
    CrossCoreCamera_CleanScreenCtrl();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_CUSTOM);
}

uint8 CrossCoreCamera_ControlCoreSendAssistantFrame(void)
{
    /* Full camera frames now stay on CM7_1; CM7_0 must not send a stale local image. */
    return 0U;
}

void CrossCoreCamera_ControlCoreRequestVisionScreen(uint8 enable)
{
    CrossCoreCamera_InvalidateScreenCtrl();
    if(g_shared_vision_screen_ctrl->magic != SHARED_VISION_SCREEN_MAGIC)
    {
        g_shared_vision_screen_ctrl->magic = SHARED_VISION_SCREEN_MAGIC;
        g_shared_vision_screen_ctrl->request_seq = 0U;
        g_shared_vision_screen_ctrl->active_seq = 0U;
        g_shared_vision_screen_ctrl->active_owner = CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
    }

    g_shared_vision_screen_ctrl->requested_owner = enable ? CROSS_CORE_CAMERA_SCREEN_OWNER_VISION
                                                          : CROSS_CORE_CAMERA_SCREEN_OWNER_NONE;
    g_shared_vision_screen_ctrl->request_seq++;
    CrossCoreCamera_CleanScreenCtrl();
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
