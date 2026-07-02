/**
 * @file ipm_demo.c
 * @brief 逆透视变换使用示例 - TC387无人机项目
 * 
 * 本文件演示如何在飞控代码中使用逆透视变换
 * 
 * 使用场景：
 * 1. 将图像中检测到的目标点转换为地面物理坐标
 * 2. 生成俯视图用于可视化或路径规划
 * 3. 补偿无人机姿态引起的图像偏移
 *
 * 说明：
 * - 本文件是“怎么调用 vision/IPM 模块”的示例集合
 * - 不承担主流程运行职责，主要用于理解接口和集成方式
 * - 本文件位于 reference 目录，不参与主工程编译
 */

#include "common/project_headfile.h"

/* ====================== 使用示例1: 目标点坐标转换 ====================== */

/**
 * @brief 将图像中检测到的目标转换为地面坐标
 * 
 * 假设你在图像中检测到了一个目标(如红外点、AprilTag等)
 * 需要知道它在地面上的实际位置(相对于无人机正下方)
 * 
 * @param target_img_x 目标在图像中的X坐标(列)
 * @param target_img_y 目标在图像中的Y坐标(行)
 * @param tof_height   ToF测量的高度(cm)
 * @param target_world_x 输出:目标地面X坐标(向前为正,cm)
 * @param target_world_y 输出:目标地面Y坐标(向左为正,cm)
 */
void Demo_ConvertTargetToWorld(int target_img_x, int target_img_y,
                                float tof_height,
                                float *target_world_x, float *target_world_y)
{
    /* 1. 使用当前IMU姿态更新逆透视变换 */
    IPM_Interface_UpdateFromIMU(tof_height);
    
    /* 2. 将图像坐标转换为地面坐标 */
    IPM_Image2World(target_img_x, target_img_y, 
                    target_world_x, target_world_y);
    
    /* 
     * 输出说明：
     * - target_world_x > 0 表示目标在无人机前方
     * - target_world_x < 0 表示目标在无人机后方
     * - target_world_y > 0 表示目标在无人机左侧
     * - target_world_y < 0 表示目标在无人机右侧
     */
}

/* ====================== 使用示例2: 俯视图显示 ====================== */

/**
 * @brief 在屏幕上显示实时俯视图
 * 
 * 调用此函数可以在IPS200屏幕上显示消除透视畸变后的俯视图
 * 正下方地面点始终在俯视图中心
 */
void Demo_ShowBirdView(void)
{
    /* 获取ToF高度(假设dl1b返回mm,需要转换为cm) */
    float height_cm = dl1b_distance_mm / 10.0f;
    
    /* 更新IPM */
    IPM_Interface_UpdateFromIMU(height_cm);
    
    /* 显示俯视图(左上角位置100,50) */
    IPM_DisplayBirdView(mt9v03x_image, 100, 50, IPM_SCALE);
}

/* ====================== 使用示例3: 定点悬停辅助 ====================== */

/**
 * @brief 计算无人机相对于目标的位置偏差
 * 
 * 用于定点悬停：检测地面标记，计算需要移动的方向和距离
 * 
 * @param marker_img_x 标记在图像中的X坐标
 * @param marker_img_y 标记在图像中的Y坐标
 * @param tof_height   ToF高度(cm)
 * @param error_x      输出:X方向偏差(向前为正,cm)
 * @param error_y      输出:Y方向偏差(向左为正,cm)
 */
void Demo_CalculateHoverError(int marker_img_x, int marker_img_y,
                               float tof_height,
                               float *error_x, float *error_y)
{
    /* 更新IPM */
    IPM_Interface_UpdateFromIMU(tof_height);
    
    /* 
     * 标记的地面坐标就是无人机需要移动的偏差
     * 因为正下方是原点(0,0)
     */
    IPM_Image2World(marker_img_x, marker_img_y, error_x, error_y);
    
    /* 
     * 返回值说明：
     * - 如果error_x > 0，标记在前方，无人机应该前进
     * - 如果error_x < 0，标记在后方，无人机应该后退
     * - 如果error_y > 0，标记在左侧，无人机应该左移
     * - 如果error_y < 0，标记在右侧，无人机应该右移
     */
}

/* ====================== 使用示例4: 检测正下方点偏移 ====================== */

/**
 * @brief 检测正下方点在图像中的偏移
 * 
 * 当无人机有roll或pitch时，正下方地面点不再在图像中心
 * 此函数返回偏移量，可用于可视化或调试
 * 
 * @param offset_x 输出:X方向偏移(像素)
 * @param offset_y 输出:Y方向偏移(像素)
 * @return true正下方点在图像内, false在图像外
 */
bool Demo_GetNadirOffset(int *offset_x, int *offset_y)
{
    int nadir_x, nadir_y;
    
    if (IPM_GetNadirPoint(&nadir_x, &nadir_y)) {
        *offset_x = nadir_x - MT9V03X_W / 2;
        *offset_y = nadir_y - MT9V03X_H / 2;
        return true;
    }
    
    return false;
}

/* ====================== 完整使用流程示例 ====================== */

/**
 * @brief 在主循环中使用IPM的完整示例
 * 
 * 假设在飞控主循环中检测红外信标并计算位置
 */
void Demo_MainLoopExample(void)
{
    /* ========== 初始化(只执行一次) ========== */
    static bool ipm_initialized = false;
    if (!ipm_initialized) {
        IPM_Interface_Init();
        ipm_initialized = true;
    }
    
    /* ========== 每帧执行 ========== */
    
    /* 1. 获取当前高度 */
    float height_cm = dl1b_distance_mm / 10.0f;
    if (height_cm < 10.0f) height_cm = 10.0f;    /* 最小高度限制 */
    if (height_cm > 500.0f) height_cm = 500.0f;  /* 最大高度限制 */
    
    /* 2. 更新IPM(使用IMU姿态) */
    IPM_Interface_UpdateFromIMU(height_cm);
    
    /* 3. 假设检测到目标在图像位置(100, 60) */
    int target_img_x = 100;
    int target_img_y = 60;
    
    /* 4. 转换为地面坐标 */
    float target_world_x, target_world_y;
    IPM_Image2World(target_img_x, target_img_y, 
                    &target_world_x, &target_world_y);
    
    /* 5. 计算距离 */
    float distance = sqrtf(target_world_x * target_world_x + 
                           target_world_y * target_world_y);
    
    /* 6. 计算方向角(相对于正前方) */
    float angle_rad = atan2f(target_world_y, target_world_x);
    float angle_deg = angle_rad * RAD2DEG_F;
    
    /* 7. 显示结果 */
    ips200_show_string(0, 0, "IPM Demo");
    ips200_show_float(0, 20, Roll_a, 2, 2);        /* Roll角 */
    ips200_show_float(60, 20, Pitch_a, 2, 2);      /* Pitch角 */
    ips200_show_float(0, 40, height_cm, 3, 1);     /* 高度 */
    ips200_show_float(0, 60, target_world_x, 3, 1); /* 目标X */
    ips200_show_float(60, 60, target_world_y, 3, 1);/* 目标Y */
    ips200_show_float(0, 80, distance, 3, 1);       /* 距离 */
    ips200_show_float(60, 80, angle_deg, 3, 1);     /* 角度 */
}

/* ====================== 集成到All_Init的示例 ====================== */

/*
 * 在 All_Init.c 的 Device_Init() 函数中添加:
 * 
 * #include "vision/projection/ground_projection.h"
 * 
 * void Device_Init(void)
 * {
 *     // ... 其他初始化 ...
 *     
 *     // 初始化逆透视模块
 *     IPM_Interface_Init();
 *     
 *     // ... 其他初始化 ...
 * }
 */

/* ====================== 集成到PIT中断的示例 ====================== */

/*
 * 在 PIT_Demo.c 的图像处理中断中添加:
 * 
 * #include "vision/projection/ground_projection.h"
 * 
 * void Image_Process_IRQ(void)
 * {
 *     // 获取高度
 *     float h = dl1b_distance_mm / 10.0f;
 *     
 *     // 更新IPM
 *     IPM_Interface_UpdateFromIMU(h);
 *     
 *     // 处理图像...
 *     // 使用 IPM_Image2World() 转换检测到的目标坐标
 * }
 */

