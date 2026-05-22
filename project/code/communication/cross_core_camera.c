#include "communication/cross_core_camera.h"
#include "common/project_headfile.h"
#include <string.h>

#define SHARED_CAMERA_MAGIC                           0x43414D31u
#define CROSS_CORE_CAMERA_ASSISTANT_IMAGE_ENABLE      0U
#define CROSS_CORE_CAMERA_ASSISTANT_FRAME_DIVIDER     10U
#define SHARED_CAMERA_CACHE_LINE_SIZE                 32U
#define SHARED_CAMERA_PAD_BYTES(type)                 (SHARED_CAMERA_CACHE_LINE_SIZE - (sizeof(type) % SHARED_CAMERA_CACHE_LINE_SIZE))

typedef struct
{
    uint32 magic;
    uint16 width;
    uint16 height;
    uint8 reserved[8];
    uint8 image[MT9V03X_H][MT9V03X_W];
    uint32 frame_seq;
} SharedCameraPayload_t;

typedef struct
{
    SharedCameraPayload_t payload;
    uint8 padding[SHARED_CAMERA_PAD_BYTES(SharedCameraPayload_t)];
} SharedCameraFrame_t;

#pragma default_variable_attributes = @ ".shared_camera"
static volatile SharedCameraFrame_t s_shared_cam __ALIGNED(SHARED_CAMERA_CACHE_LINE_SIZE);
#pragma default_variable_attributes =

static volatile SharedCameraPayload_t *const g_shared_cam = &s_shared_cam.payload;

static void clean_shared_camera_cache(void)
{
    SCB_CleanDCache_by_Addr((void *)g_shared_cam, sizeof(SharedCameraFrame_t));
}

static void invalidate_shared_camera_cache(void)
{
    SCB_InvalidateDCache_by_Addr((void *)g_shared_cam, sizeof(SharedCameraFrame_t));
}

void CrossCoreCamera_CameraCoreInit(void)
{
    (void)mt9v03x_init();
#if CROSS_CORE_CAMERA_ASSISTANT_IMAGE_ENABLE
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, (void *)g_shared_cam->image[0], MT9V03X_W, MT9V03X_H);
#endif
    g_shared_cam->magic = SHARED_CAMERA_MAGIC;
    g_shared_cam->frame_seq = 0U;
    g_shared_cam->width = (uint16)MT9V03X_W;
    g_shared_cam->height = (uint16)MT9V03X_H;
    clean_shared_camera_cache();
}

uint8 CrossCoreCamera_CameraCoreTask(void)
{
#if CROSS_CORE_CAMERA_ASSISTANT_IMAGE_ENABLE
    static uint8 assistant_frame_divider = 0U;
#endif

    if(!mt9v03x_finish_flag)
    {
        return 0U;
    }

    mt9v03x_finish_flag = 0;
    memcpy((void *)g_shared_cam->image[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    g_shared_cam->frame_seq++;
    clean_shared_camera_cache();
#if CROSS_CORE_CAMERA_ASSISTANT_IMAGE_ENABLE
    assistant_frame_divider++;
    if(assistant_frame_divider >= CROSS_CORE_CAMERA_ASSISTANT_FRAME_DIVIDER)
    {
        assistant_frame_divider = 0U;
        seekfree_assistant_camera_send();
    }
#endif
    return 1U;
}

uint8 CrossCoreCamera_ControlCoreFetchFrame(void)
{
    static uint32 last_seq = 0U;

    invalidate_shared_camera_cache();
    if(g_shared_cam->magic != SHARED_CAMERA_MAGIC)
    {
        return 0U;
    }

    if(g_shared_cam->frame_seq == last_seq)
    {
        return 0U;
    }

    last_seq = g_shared_cam->frame_seq;
    memcpy(mt9v03x_image[0], (const void *)g_shared_cam->image[0], MT9V03X_IMAGE_SIZE);
    mt9v03x_finish_flag = 1U;
    return 1U;
}

#if CY_CORE_CM7_0
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
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_CUSTOM);
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X,
                                                 mt9v03x_image[0],
                                                 MT9V03X_W,
                                                 MT9V03X_H);
}

uint8 CrossCoreCamera_ControlCoreSendAssistantFrame(void)
{
    if(!CommMux_IsAssistantImageMode())
    {
        return 0U;
    }

    seekfree_assistant_camera_send();
    return 1U;
}
#endif
