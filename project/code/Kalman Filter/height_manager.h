#ifndef HEIGHT_MANAGER_H_
#define HEIGHT_MANAGER_H_

#include "zf_common_typedef.h"
#include "pid_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

// 高度融合对外结果快照
typedef struct
{
    float height_cm;
    float velocity_cms;
    float height_std;
    uint8 confidence;
} HeightFusionResult_t;

// 高度模块对外共享的结果与控制对象
extern HeightFusionResult_t g_height_result;

extern PID_Controller HeightPID;
extern PID_Controller HeightSpeedPID;

extern float HeightSpeedPID_output;
extern float tof_height_true_filt;

// 高度模块初始化与周期更新入口
// initial_height_cm: 高度融合器的初始高度，单位 cm，用于首个有效测量到来前的状态初值。
void HeightManager_Init(float initial_height_cm);
void HeightManager_Update(void);

#ifdef __cplusplus
}
#endif

#endif
