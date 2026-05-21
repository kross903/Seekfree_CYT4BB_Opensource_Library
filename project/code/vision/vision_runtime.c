#include "vision/vision_runtime.h"

#include "vision/ipm_interface.h"
#include "vision/ir_localization.h"
#include "zf_device_mt9v03x.h"

void VisionRuntime_ProcessFrame(float height_cm)
{
    IR_Localization_ServiceLostProtection();

    if(!mt9v03x_finish_flag)
    {
        return;
    }

    mt9v03x_finish_flag = 0;
    IPM_Interface_UpdateFromIMU(height_cm);
    IR_Localization_Process(mt9v03x_image, &ir_pose_estimate);
}
