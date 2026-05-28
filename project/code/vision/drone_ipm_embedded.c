/**
 * @file drone_ipm_embedded.c
 * @brief 无人机正朝下摄像头逆透视变换 - 嵌入式优化版本实现
 *
 * 针对嵌入式平台优化的IPM算法，特点：
 * 1. 预计算所有可能的系数，减少实时计算
 * 2. 使用float而非double，适合没有FPU双精度的MCU
 * 3. 针对正朝下摄像头优化的坐标变换
 *
 * 坐标系说明：
 * - 世界坐标系(NED): X向前, Y向右, Z向下(地面为Z=0)
 * - 图像坐标系: u向右, v向下, 原点左上角
 * - 摄像头正朝下时: 图像中心对应正下方地面点
 */

#include "drone_ipm_embedded.h"
#include "common/project_headfile.h"
#include <string.h>

/*
 * 模块定位:
 * - 本文件是工程运行时实际使用的 IPM 数学核心
 * - 上层只应通过 ipm_interface 访问，避免把 IMU/显示/数学逻辑耦合在一起
 */

/* 全局IPM系数 */
DroneIPM_Coeff_t g_ipm;

/* 图像尺寸常量 */
static const float W_half = (float)IMAGE_WIDTH / 2.0f;
static const float H_half = (float)IMAGE_HEIGHT / 2.0f;

static void DroneIPM_Embedded_PixelToRayBody(float img_x, float img_y, float ray_body[3])
{
#if DRONE_IPM_CAMERA_PROJECTION == DRONE_IPM_CAMERA_PROJECTION_FISHEYE_EQUIDISTANT
    float du = img_x - W_half;
    float dv = img_y - H_half;
    float radius = sqrtf(du * du + dv * dv);

    if(radius < 1e-6f)
    {
        ray_body[0] = 0.0f;
        ray_body[1] = 0.0f;
        ray_body[2] = 1.0f;
        return;
    }

    {
        float theta = radius * g_ipm.K_fov_x;
        float sin_theta = sinf(theta);
        float cos_theta = cosf(theta);
        float radial_scale = sin_theta / radius;

        ray_body[0] = dv * radial_scale;
        ray_body[1] = du * radial_scale;
        ray_body[2] = cos_theta;
    }
#else
    float u_n = (img_x - W_half) * g_ipm.K_fov_x;
    float v_n = (img_y - H_half) * g_ipm.K_fov_y;

    ray_body[0] = v_n;
    ray_body[1] = u_n;
    ray_body[2] = 1.0f;
#endif
}

static bool DroneIPM_Embedded_RayBodyToPixel(const float ray_body[3], int *result)
{
    float img_x_f = 0.0f;
    float img_y_f = 0.0f;

    if(ray_body[2] <= 0.0f)
    {
        return false;
    }

#if DRONE_IPM_CAMERA_PROJECTION == DRONE_IPM_CAMERA_PROJECTION_FISHEYE_EQUIDISTANT
    {
        float lateral = sqrtf(ray_body[0] * ray_body[0] + ray_body[1] * ray_body[1]);

        if(lateral < 1e-6f)
        {
            img_x_f = W_half;
            img_y_f = H_half;
        }
        else
        {
            float theta = atan2f(lateral, ray_body[2]);
            float radius = theta * g_ipm.K_inv;
            float pixel_scale = radius / lateral;

            img_x_f = ray_body[1] * pixel_scale + W_half;
            img_y_f = ray_body[0] * pixel_scale + H_half;
        }
    }
#else
    {
        float v_n = ray_body[0] / ray_body[2];
        float u_n = ray_body[1] / ray_body[2];

        img_x_f = u_n / g_ipm.K_fov_x + W_half;
        img_y_f = v_n / g_ipm.K_fov_y + H_half;
    }
#endif

    result[0] = (int)(img_x_f + 0.5f);
    result[1] = (int)(img_y_f + 0.5f);

    if(result[0] < 0 || result[0] >= IMAGE_WIDTH ||
       result[1] < 0 || result[1] >= IMAGE_HEIGHT)
    {
        return false;
    }

    return true;
}

/**
 * @brief 初始化IPM系数
 *
 * 建立与图像尺寸/FOV有关的固定系数，并给出一组默认姿态，
 * 方便系统上电后即可获得可用的逆透视结果。
 */
void DroneIPM_Embedded_Init(void)
{
    memset(&g_ipm, 0, sizeof(g_ipm));

#if DRONE_IPM_CAMERA_PROJECTION == DRONE_IPM_CAMERA_PROJECTION_FISHEYE_EQUIDISTANT
    {
        float half_diagonal_fov_rad = 0.5f * CAM_FOV_DIAGONAL_DEG * DEG2RAD_F;
        float img_radius_half = sqrtf(W_half * W_half + H_half * H_half);

        g_ipm.K_fov_x = half_diagonal_fov_rad / img_radius_half;
        g_ipm.K_fov_y = g_ipm.K_fov_x;
        g_ipm.K_inv = img_radius_half / half_diagonal_fov_rad;
    }
#else
    {
        float tan_fov_x = tanf(CAM_FOV_X * DEG2RAD_F);
        float tan_fov_y = tanf(CAM_FOV_Y * DEG2RAD_F);

        g_ipm.K_fov_x = tan_fov_x / W_half;
        g_ipm.K_fov_y = tan_fov_y / H_half;
        g_ipm.K_inv = W_half / tan_fov_x;
    }
#endif

    /* 初始化为水平状态 */
    DroneIPM_Embedded_Update(0.0f, 0.0f, 100.0f);
}

/**
 * @brief 更新姿态并重新计算变换系数
 *
 * 旋转矩阵使用ZYX欧拉角顺序(先yaw，再pitch，最后roll)
 * 对于正朝下摄像头，只考虑roll和pitch
 *
 * R = Rx(roll) * Ry(pitch)
 *
 * 其中:
 * Rx(φ) = [1,    0,      0   ]   (roll)
 *         [0,  cos(φ), -sin(φ)]
 *         [0,  sin(φ),  cos(φ)]
 *
 * Ry(θ) = [ cos(θ), 0, sin(θ)]   (pitch)
 *         [   0,    1,   0   ]
 *         [-sin(θ), 0, cos(θ)]
 *
 * 这个函数是底层 IPM 的“状态刷新入口”，每当 roll/pitch/height 变化，
 * 上层都应先调用它，再做任意像素/世界坐标换算。
 */
void DroneIPM_Embedded_Update(float roll_deg, float pitch_deg, float height)
{
    /* 存储当前状态 */
    float roll_rad = roll_deg * DEG2RAD_F;
    float pitch_rad = pitch_deg * DEG2RAD_F;

    g_ipm.roll = roll_rad;
    g_ipm.pitch = pitch_rad;
    g_ipm.height = height;

    /* 计算三角函数 */
    float sr, cr, sp, cp;
    fast_sincos(roll_rad, &sr, &cr);
    fast_sincos(pitch_rad, &sp, &cp);

    /*
     * 构建旋转矩阵 R = Rx(roll) * Ry(pitch)
     *
     * R = [ cp,     0,    sp   ]
     *     [ sr*sp,  cr,  -sr*cp]
     *     [-cr*sp,  sr,   cr*cp]
     */
    g_ipm.R[0][0] = cp;
    g_ipm.R[0][1] = 0.0f;
    g_ipm.R[0][2] = sp;

    g_ipm.R[1][0] = sr * sp;
    g_ipm.R[1][1] = cr;
    g_ipm.R[1][2] = -sr * cp;

    g_ipm.R[2][0] = -cr * sp;
    g_ipm.R[2][1] = sr;
    g_ipm.R[2][2] = cr * cp;

    /*
     * 计算摄像头在世界坐标系中的位置
     * 假设摄像头固定在机体原点，高度为height
     * 世界坐标系地面为Z=0，摄像头在(0, 0, -height)
     */
    g_ipm.X0 = 0.0f;
    g_ipm.Y0 = 0.0f;
    g_ipm.Z0 = -height;  /* 负值，因为向上是负Z */
}

/**
 * @brief 图像坐标到地面坐标(逆透视变换)
 *
 * 算法步骤:
 * 1. 将像素(img_x, img_y)转为归一化坐标(u_n, v_n)
 * 2. 构建机体坐标系中的光线方向向量
 * 3. 用旋转矩阵R将光线变换到世界坐标系
 * 4. 计算光线与地面(Z=0)的交点
 *
 * @param img_x 图像列坐标(0到IMAGE_WIDTH-1)
 * @param img_y 图像行坐标(0到IMAGE_HEIGHT-1)
 * @param result 输出数组[world_x, world_y]
 *               world_x: 向前为正
 *               world_y: 向右为正
 */
void DroneIPM_Embedded_Image2World(sint16 img_x, sint16 img_y, float *result)
{
    float ray_body[3];
    DroneIPM_Embedded_PixelToRayBody((float)img_x, (float)img_y, ray_body);

    /* 步骤3: 将光线变换到世界坐标系
     * ray_world = R * ray_body
     */
    float ray_world[3];
    ray_world[0] = g_ipm.R[0][0] * ray_body[0] +
                   g_ipm.R[0][1] * ray_body[1] +
                   g_ipm.R[0][2] * ray_body[2];
    ray_world[1] = g_ipm.R[1][0] * ray_body[0] +
                   g_ipm.R[1][1] * ray_body[1] +
                   g_ipm.R[1][2] * ray_body[2];
    ray_world[2] = g_ipm.R[2][0] * ray_body[0] +
                   g_ipm.R[2][1] * ray_body[1] +
                   g_ipm.R[2][2] * ray_body[2];

    /* 步骤4: 求光线与地面(Z=0)的交点
     * 光线方程: P = (X0, Y0, Z0) + t * ray_world
     * 地面方程: Z = 0
     * 解得: t = -Z0 / ray_world[2]
     */
    if (fabsf(ray_world[2]) < 1e-6f) {
        /* 光线与地面平行，返回很大的值表示无穷远 */
        result[0] = 9999.0f;
        result[1] = 9999.0f;
        return;
    }

    float t = -g_ipm.Z0 / ray_world[2];

    if (t < 0.0f) {
        /* 交点在摄像头后方，无效 */
        result[0] = 9999.0f;
        result[1] = 9999.0f;
        return;
    }

    /* 计算地面交点 */
    result[0] = g_ipm.X0 + t * ray_world[0];  /* world_x: 向前为正 */
    result[1] = g_ipm.Y0 + t * ray_world[1];  /* world_y: 向右为正 */
}

/**
 * @brief 地面坐标到图像坐标(正透视变换)
 *
 * 这是逆透视的反向操作，用于将地面物理坐标投影到图像
 *
 * @param world_x 地面X坐标(向前为正)
 * @param world_y 地面Y坐标(向右为正)
 * @param result 输出数组[img_x, img_y]
 * @return true如果点在图像范围内，false否则
 *
 * 主要被俯视图生成和调试功能复用。
 */
bool DroneIPM_Embedded_World2Image(float world_x, float world_y, int *result)
{
    /* 步骤1: 计算地面点相对于摄像头的向量(世界坐标系) */
    float vec_world[3];
    vec_world[0] = world_x - g_ipm.X0;
    vec_world[1] = world_y - g_ipm.Y0;
    vec_world[2] = 0.0f - g_ipm.Z0;  /* 地面Z=0 */

    /* 步骤2: 将向量从世界坐标系转换到机体坐标系
     * vec_body = R^T * vec_world (R的转置)
     */
    float vec_body[3];
    vec_body[0] = g_ipm.R[0][0] * vec_world[0] +
                  g_ipm.R[1][0] * vec_world[1] +
                  g_ipm.R[2][0] * vec_world[2];
    vec_body[1] = g_ipm.R[0][1] * vec_world[0] +
                  g_ipm.R[1][1] * vec_world[1] +
                  g_ipm.R[2][1] * vec_world[2];
    vec_body[2] = g_ipm.R[0][2] * vec_world[0] +
                  g_ipm.R[1][2] * vec_world[1] +
                  g_ipm.R[2][2] * vec_world[2];

    return DroneIPM_Embedded_RayBodyToPixel(vec_body, result);
}

/**
 * @brief 获取正下方点在原始图像中的位置
 *
 * 正下方点的世界坐标是(0, 0)，这个函数返回它在原始图像中的像素位置
 * 当无人机有roll或pitch时，这个点会偏离图像中心
 */
bool DroneIPM_Embedded_GetNadir(int *result)
{
    return DroneIPM_Embedded_World2Image(0.0f, 0.0f, result);
}

/**
 * @brief 生成俯视图(逆透视变换)
 *
 * 对于输出图像的每个像素，计算其对应的地面物理坐标，
 * 然后投影回原始图像获取灰度值。
 *
 * 输出图像特点:
 * - 中心对应正下方地面点(world_x=0, world_y=0)
 * - 上方对应前方(+X)，下方对应后方(-X)
 * - 右方对应右侧(+Y)，左方对应左侧(-Y)
 * - 无论无人机姿态如何，正下方始终在输出图像中心
 *
 * @param src 源图像(灰度)
 * @param dst 目标图像(俯视图)
 * @param dst_w 目标图像宽度
 * @param dst_h 目标图像高度
 * @param scale 缩放系数(物理单位/像素)
 *
 * 这是“显示或调试俯视图”时的主函数；运行时目标检测本身并不依赖它。
 */
void DroneIPM_Embedded_BirdView(const uint8_t *src, uint8_t *dst,
                                 int dst_w, int dst_h, float scale)
{
    float center_px = (float)dst_w / 2.0f;
    float center_py = (float)dst_h / 2.0f;

    for (int dst_y = 0; dst_y < dst_h; dst_y++) {
        for (int dst_x = 0; dst_x < dst_w; dst_x++) {
            /* 计算该输出像素对应的地面物理坐标 */
            /* 输出图像Y轴向下对应物理X轴向后 */
            float world_x = (center_py - (float)dst_y) * scale;  /* 图像上方=前方 */
            float world_y = ((float)dst_x - center_px) * scale;  /* 图像右方=右方 */

            /* 投影到源图像 */
            int src_pos[2];
            if (DroneIPM_Embedded_World2Image(world_x, world_y, src_pos)) {
                /* 简单最近邻采样 */
                dst[dst_y * dst_w + dst_x] =
                    src[src_pos[1] * IMAGE_WIDTH + src_pos[0]];
            } else {
                /* 超出范围，填充背景 */
                dst[dst_y * dst_w + dst_x] = 0;
            }
        }
    }
}

/* ====================== 快速版本(预计算查找表) ====================== */

/* 可选：预计算查找表(用于固定姿态场景) */
#if DRONE_IPM_EMBEDDED_LUT_ENABLE
static int16_t g_lut_x[IMAGE_HEIGHT][IMAGE_WIDTH];
static int16_t g_lut_y[IMAGE_HEIGHT][IMAGE_WIDTH];
static bool g_lut_valid = false;
#endif

/**
 * @brief 生成查找表(姿态固定时使用)
 *
 * 当姿态不变时，可以预先计算每个输出像素对应的源像素位置
 * 后续只需查表，大幅提高速度
 *
 * @param out_w 输出图像宽度
 * @param out_h 输出图像高度
 * @param scale 缩放系数
 */
void DroneIPM_Embedded_GenerateLUT(int out_w, int out_h, float scale)
{
#if DRONE_IPM_EMBEDDED_LUT_ENABLE
    float center_px = (float)out_w / 2.0f;
    float center_py = (float)out_h / 2.0f;

    for (int y = 0; y < out_h && y < IMAGE_HEIGHT; y++) {
        for (int x = 0; x < out_w && x < IMAGE_WIDTH; x++) {
            float world_x = (center_py - (float)y) * scale;
            float world_y = ((float)x - center_px) * scale;

            int src_pos[2];
            if (DroneIPM_Embedded_World2Image(world_x, world_y, src_pos)) {
                g_lut_x[y][x] = (int16_t)src_pos[0];
                g_lut_y[y][x] = (int16_t)src_pos[1];
            } else {
                g_lut_x[y][x] = -1;  /* 标记为无效 */
                g_lut_y[y][x] = -1;
            }
        }
    }

    g_lut_valid = true;
#else
    (void)out_w;
    (void)out_h;
    (void)scale;
#endif
}

/**
 * @brief 使用查找表快速生成俯视图
 */
void DroneIPM_Embedded_BirdViewFast(const uint8_t *src, uint8_t *dst,
                                     int dst_w, int dst_h)
{
#if DRONE_IPM_EMBEDDED_LUT_ENABLE
    if (!g_lut_valid) {
        /* 查找表无效，回退到普通方法 */
        DroneIPM_Embedded_BirdView(src, dst, dst_w, dst_h, 0.5f);
        return;
    }

    for (int y = 0; y < dst_h && y < IMAGE_HEIGHT; y++) {
        for (int x = 0; x < dst_w && x < IMAGE_WIDTH; x++) {
            int16_t sx = g_lut_x[y][x];
            int16_t sy = g_lut_y[y][x];

            if (sx >= 0 && sy >= 0) {
                dst[y * dst_w + x] = src[sy * IMAGE_WIDTH + sx];
            } else {
                dst[y * dst_w + x] = 0;
            }
        }
    }
#else
    DroneIPM_Embedded_BirdView(src, dst, dst_w, dst_h, 0.5f);
#endif
}

