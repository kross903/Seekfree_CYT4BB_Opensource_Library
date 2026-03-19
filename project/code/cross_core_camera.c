#include "cross_core_camera.h"
#include <string.h>

#define SHARED_CAMERA_BASE_ADDR   0x28001000u
#define SHARED_CAMERA_MAGIC       0x43414D31u

typedef struct
{
    uint32 magic;
    uint32 frame_seq;
    uint16 width;
    uint16 height;
    uint8 reserved[8];
    uint8 image[MT9V03X_H][MT9V03X_W];
} SharedCameraFrame_t;

static volatile SharedCameraFrame_t *const g_shared_cam = (volatile SharedCameraFrame_t *)SHARED_CAMERA_BASE_ADDR;

void CrossCoreCamera_CameraCoreInit(void)
{
    (void)mt9v03x_init();
    g_shared_cam->magic = SHARED_CAMERA_MAGIC;
    g_shared_cam->frame_seq = 0U;
    g_shared_cam->width = (uint16)MT9V03X_W;
    g_shared_cam->height = (uint16)MT9V03X_H;
    SCB_CleanInvalidateDCache_by_Addr((void *)g_shared_cam, sizeof(SharedCameraFrame_t));
}

void CrossCoreCamera_CameraCoreTask(void)
{
    if(!mt9v03x_finish_flag)
    {
        return;
    }

    mt9v03x_finish_flag = 0;
    memcpy((void *)g_shared_cam->image[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    g_shared_cam->frame_seq++;
    SCB_CleanInvalidateDCache_by_Addr((void *)g_shared_cam, sizeof(SharedCameraFrame_t));
}

uint8 CrossCoreCamera_ControlCoreFetchFrame(void)
{
    static uint32 last_seq = 0U;

    SCB_CleanInvalidateDCache_by_Addr((void *)g_shared_cam, sizeof(SharedCameraFrame_t));
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
