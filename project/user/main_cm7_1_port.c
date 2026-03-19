#include "zf_common_headfile.h"
#include "../code/cross_core_camera.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_info_init();
    CrossCoreCamera_CameraCoreInit();

    while(true)
    {
        CrossCoreCamera_CameraCoreTask();
    }
}
