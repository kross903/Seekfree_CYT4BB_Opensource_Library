/**
 * @file drone_ipm.c
 * @brief 无人机正朝下摄像头逆透视变换实现
 * 
 * 数学原理：
 * 1. 摄像头正朝下时，图像中心对应正下方地面点
 * 2. 当有roll和pitch角时，需要通过旋转矩阵将像素光线变换到世界坐标系
 * 3. 求解光线与地面(z=0平面)的交点，即为像素对应的地面物理坐标
 * 
 * 坐标系约定：
 * - 世界坐标系: X向前, Y向左, Z向上
 * - 机体坐标系: 初始与世界对齐, roll绕X轴, pitch绕Y轴
 * - 摄像头坐标系: 正朝下时Z轴指向地面(即机体-Z方向)
 * - 图像坐标系: u向右(对应Y), v向下(对应X)
 *
 * 注意:
 * - 本文件是通用/双精度参考实现，便于离线推导和桌面验证。
 * - 当前工程运行时主流程使用 `projection/ground_projection.*` 调用
 *   `projection/ground_projection_ipm_core.*`。
 * - 视觉业务代码不应直接依赖本文件作为项目级 IPM 入口。
 * - 本文件位于 reference 目录，不参与主工程编译。
 */

#include "drone_ipm.h"
#include "common/project_headfile.h"
#include <string.h>
#include <stdlib.h>

/* ====================== 辅助函数实现 ====================== */

/**
 * @brief 从欧拉角创建旋转矩阵
 * 
 * 旋转顺序: ZYX (先yaw, 再pitch, 最后roll)
 * 这是常用的航空坐标系旋转顺序
 * 
 * R = Rx(roll) * Ry(pitch) * Rz(yaw)
 * 
 * 对于正朝下摄像头，通常yaw=0，只考虑roll和pitch
 */
void DroneIPM_EulerToRotationMatrix(double roll, double pitch, double yaw, 
                                     double R[3][3])
{
    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);
    
    /* ZYX旋转矩阵 */
    R[0][0] = cp * cy;
    R[0][1] = cp * sy;
    R[0][2] = -sp;
    
    R[1][0] = sr * sp * cy - cr * sy;
    R[1][1] = sr * sp * sy + cr * cy;
    R[1][2] = sr * cp;
    
    R[2][0] = cr * sp * cy + sr * sy;
    R[2][1] = cr * sp * sy - sr * cy;
    R[2][2] = cr * cp;
}

/**
 * @brief 矩阵向量乘法: result = R * v
 */
void DroneIPM_MatVecMul(const double R[3][3], const double v[3], double result[3])
{
    result[0] = R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2];
    result[1] = R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2];
    result[2] = R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2];
}

/**
 * @brief 矩阵转置
 */
void DroneIPM_MatTranspose(const double R[3][3], double Rt[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Rt[i][j] = R[j][i];
        }
    }
}

/* ====================== 主要函数实现 ====================== */

/**
 * @brief 初始化摄像头内参
 */
void DroneIPM_InitCamera(CameraIntrinsics_t *cam, int width, int height, 
                          double fov_x, double fov_y)
{
    cam->image_width = width;
    cam->image_height = height;
    cam->fov_x = fov_x;
    cam->fov_y = fov_y;
    
    /* 从FOV计算焦距 */
    /* fx = width / (2 * tan(fov_x/2)) */
    cam->fx = (double)width / (2.0 * tan(fov_x * DEG2RAD / 2.0));
    cam->fy = (double)height / (2.0 * tan(fov_y * DEG2RAD / 2.0));
    
    /* 主点在图像中心 */
    cam->cx = (double)width / 2.0;
    cam->cy = (double)height / 2.0;
}

/**
 * @brief 初始化IPM变换器
 */
void DroneIPM_Init(DroneIPM_t *ipm, const CameraIntrinsics_t *cam)
{
    memset(ipm, 0, sizeof(DroneIPM_t));
    
    if (cam != NULL) {
        memcpy(&ipm->camera, cam, sizeof(CameraIntrinsics_t));
    } else {
        /* 使用默认参数 */
        DroneIPM_InitCamera(&ipm->camera, 
                            DEFAULT_IMAGE_WIDTH, DEFAULT_IMAGE_HEIGHT,
                            DEFAULT_FOV_X, DEFAULT_FOV_Y);
    }
    
    /* 预计算FOV的tan值 */
    ipm->tan_half_fov_x = tan(ipm->camera.fov_x * DEG2RAD / 2.0);
    ipm->tan_half_fov_y = tan(ipm->camera.fov_y * DEG2RAD / 2.0);
    
    /* 默认输出参数 */
    ipm->output_scale = 1.0;
    ipm->output_center_x = 0.0;
    ipm->output_center_y = 0.0;
    
    /* 初始化为零状态(水平朝下) */
    DroneIPM_Update(ipm, 0.0, 0.0, 100.0);
}

/**
 * @brief 更新无人机状态并重新计算变换矩阵
 * 
 * 核心算法：
 * 1. 根据roll和pitch计算机体到世界的旋转矩阵
 * 2. 摄像头正朝下，所以摄像头坐标系相对机体有-90度旋转
 * 3. 计算摄像头在世界坐标系中的位置
 */
void DroneIPM_Update(DroneIPM_t *ipm, double roll, double pitch, double height)
{
    ipm->state.roll = roll;
    ipm->state.pitch = pitch;
    ipm->state.yaw = 0.0;  /* 逆透视不考虑yaw */
    ipm->state.height = height;
    
    /* 计算机体到世界的旋转矩阵 */
    /* 注意：这里yaw=0，因为逆透视只关心相对地面的姿态 */
    double roll_rad = roll * DEG2RAD;
    double pitch_rad = pitch * DEG2RAD;
    
    DroneIPM_EulerToRotationMatrix(roll_rad, pitch_rad, 0.0, ipm->R);
    
    /* 摄像头在世界坐标系中的位置 */
    /* 假设摄像头在无人机重心，高度为height */
    ipm->cam_pos_world[0] = 0.0;  /* X: 向前 */
    ipm->cam_pos_world[1] = 0.0;  /* Y: 向右 */
    ipm->cam_pos_world[2] = -height; /* Z: 向下(高度为负的-Z) */
}

/**
 * @brief 设置输出缩放参数
 */
void DroneIPM_SetOutputScale(DroneIPM_t *ipm, double scale, 
                              double center_x, double center_y)
{
    ipm->output_scale = scale;
    ipm->output_center_x = center_x;
    ipm->output_center_y = center_y;
}

/**
 * @brief 逆透视变换：图像坐标 -> 地面物理坐标
 * 
 * 算法步骤：
 * 1. 将像素坐标转换为归一化摄像头坐标
 * 2. 构建摄像头坐标系中的光线方向
 * 3. 将光线方向转换到世界坐标系
 * 4. 求解光线与地面(Z=0)的交点
 * 
 * 关键点：
 * - 摄像头正朝下时，图像中心的光线指向机体-Z方向
 * - 像素(u,v)对应的归一化坐标: 
 *   x_cam = (u - cx) / fx
 *   y_cam = (v - cy) / fy
 * - 摄像头坐标系光线方向: [x_cam, y_cam, 1]^T (指向+z_cam)
 * - 由于摄像头朝下，需要特殊处理坐标变换
 */
bool DroneIPM_ImageToWorld(const DroneIPM_t *ipm, int img_x, int img_y,
                            double *world_x, double *world_y)
{
    const CameraIntrinsics_t *cam = &ipm->camera;
    
    /* 步骤1: 像素坐标转归一化摄像头坐标 */
    /* 图像中心对应正下方 */
    double u_norm = ((double)img_x - cam->cx) / cam->fx;
    double v_norm = ((double)img_y - cam->cy) / cam->fy;
    
    /* 步骤2: 构建摄像头坐标系中的光线方向 */
    /* 摄像头正朝下时：
     * - 图像u方向(向右) 对应 机体Y方向(向右)
     * - 图像v方向(向下) 对应 机体X方向(向前)
     * - 光线主方向 对应 机体Z方向(向下)
     * 
     * 摄像头光线方向(在机体坐标系中):
     * ray_body = [v_norm, u_norm, 1]^T (归一化，指向地面)
     */
    double ray_body[3];
    ray_body[0] = v_norm;   /* 图像v对应机体X(向前) */
    ray_body[1] = u_norm;   /* 图像u对应机体Y(向右) */
    ray_body[2] = 1.0;      /* 主光轴指向机体+Z(向下) */
    
    /* 归一化光线方向 */
    double ray_len = sqrt(ray_body[0]*ray_body[0] + 
                          ray_body[1]*ray_body[1] + 
                          ray_body[2]*ray_body[2]);
    ray_body[0] /= ray_len;
    ray_body[1] /= ray_len;
    ray_body[2] /= ray_len;
    
    /* 步骤3: 将光线方向从机体坐标系转换到世界坐标系 */
    /* ray_world = R * ray_body */
    double ray_world[3];
    DroneIPM_MatVecMul(ipm->R, ray_body, ray_world);
    
    /* 步骤4: 求解光线与地面(Z=0)的交点 */
    /* 光线方程: P = cam_pos + t * ray_world */
    /* 地面方程: Z = 0 */
    /* 解: t = -cam_pos_z / ray_world_z */
    
    /* 检查光线是否与地面相交 */
    if (fabs(ray_world[2]) < 1e-10) {
        /* 光线与地面平行，无交点 */
        return false;
    }
    
    /* 这里 height 继续按到地面的正值距离使用，内部仍以地面交点约束做计算。 */
    /* cam_pos_world[2] = -height (负值，因为高度向上) */
    double t = -ipm->cam_pos_world[2] / ray_world[2];
    
    if (t < 0) {
        /* 交点在摄像头后方，无效 */
        return false;
    }
    
    /* 计算地面交点 */
    *world_x = ipm->cam_pos_world[0] + t * ray_world[0];
    *world_y = ipm->cam_pos_world[1] + t * ray_world[1];
    
    return true;
}

/**
 * @brief 正透视变换：地面物理坐标 -> 图像坐标
 * 
 * 这是逆透视的逆过程，用于将地面点投影回图像
 */
bool DroneIPM_WorldToImage(const DroneIPM_t *ipm, double world_x, double world_y,
                            int *img_x, int *img_y)
{
    const CameraIntrinsics_t *cam = &ipm->camera;
    
    /* 步骤1: 计算地面点相对于摄像头的向量(世界坐标系) */
    double vec_world[3];
    vec_world[0] = world_x - ipm->cam_pos_world[0];
    vec_world[1] = world_y - ipm->cam_pos_world[1];
    vec_world[2] = 0.0 - ipm->cam_pos_world[2];  /* 地面Z=0 */
    
    /* 步骤2: 将向量从世界坐标系转换到机体坐标系 */
    /* vec_body = R^T * vec_world */
    double Rt[3][3];
    DroneIPM_MatTranspose(ipm->R, Rt);
    
    double vec_body[3];
    DroneIPM_MatVecMul(Rt, vec_world, vec_body);
    
    /* 步骤3: 检查点是否在摄像头前方 */
    if (vec_body[2] <= 0) {
        /* 点在摄像头后方或平面上 */
        return false;
    }
    
    /* 步骤4: 计算归一化摄像头坐标 */
    /* 摄像头正朝下时的坐标映射 */
    double v_norm = vec_body[0] / vec_body[2];  /* X/Z -> v */
    double u_norm = vec_body[1] / vec_body[2];  /* Y/Z -> u */
    
    /* 步骤5: 转换为像素坐标 */
    double u = u_norm * cam->fx + cam->cx;
    double v = v_norm * cam->fy + cam->cy;
    
    /* 四舍五入到整数 */
    *img_x = (int)round(u);
    *img_y = (int)round(v);
    
    /* 检查是否在图像范围内 */
    if (*img_x < 0 || *img_x >= cam->image_width ||
        *img_y < 0 || *img_y >= cam->image_height) {
        return false;
    }
    
    return true;
}

/**
 * @brief 获取正下方点在图像中的位置
 * 
 * 正下方点是地面上直接位于无人机下方的点(world_x=0, world_y=0)
 * 当roll和pitch不为0时，这个点在图像中会偏离中心
 * 但经过逆透视变换后，它应该始终映射到输出图像的中心
 */
bool DroneIPM_GetNadirPoint(const DroneIPM_t *ipm, int *img_x, int *img_y)
{
    /* 正下方点的世界坐标是(0, 0, 0) */
    return DroneIPM_WorldToImage(ipm, 0.0, 0.0, img_x, img_y);
}

/**
 * @brief 生成逆透视变换后的俯视图
 * 
 * 算法：
 * 对于输出图像的每个像素，计算其对应的地面物理坐标，
 * 然后找到该物理坐标在原始图像中的位置，进行采样。
 * 
 * 这样可以保证：
 * 1. 正下方的地面点始终在输出图像的中心
 * 2. 输出图像是正交俯视图，消除了透视畸变
 * 
 * @param scale 物理单位/像素，例如1.0表示1cm/像素
 */
void DroneIPM_GenerateBirdView(const DroneIPM_t *ipm,
                                const uint8_t *src_img,
                                uint8_t *dst_img,
                                int dst_width, int dst_height,
                                double scale)
{
    const CameraIntrinsics_t *cam = &ipm->camera;
    
    /* 输出图像中心对应的物理坐标(0,0) - 正下方 */
    double center_px = dst_width / 2.0;
    double center_py = dst_height / 2.0;
    
    /* 遍历输出图像的每个像素 */
    for (int dst_y = 0; dst_y < dst_height; dst_y++) {
        for (int dst_x = 0; dst_x < dst_width; dst_x++) {
            /* 计算该输出像素对应的地面物理坐标 */
            /* 注意：输出图像Y轴向下对应物理X轴向前 */
            double world_x = (center_py - dst_y) * scale;  /* 图像上方对应前方 */
            double world_y = (dst_x - center_px) * scale;  /* 图像右方对应右方 */
            
            /* 将地面坐标投影回原始图像 */
            int src_x, src_y;
            if (DroneIPM_WorldToImage(ipm, world_x, world_y, &src_x, &src_y)) {
                /* 在源图像范围内，直接采样 */
                dst_img[dst_y * dst_width + dst_x] = 
                    src_img[src_y * cam->image_width + src_x];
            } else {
                /* 超出源图像范围，填充背景色 */
                dst_img[dst_y * dst_width + dst_x] = 0;  /* 黑色 */
            }
        }
    }
}

/* ====================== 扩展函数：双线性插值版本 ====================== */

/**
 * @brief 双线性插值采样
 */
static uint8_t bilinear_sample(const uint8_t *img, int width, int height,
                               double x, double y)
{
    if (x < 0 || x >= width - 1 || y < 0 || y >= height - 1) {
        return 0;
    }
    
    int x0 = (int)x;
    int y0 = (int)y;
    int x1 = x0 + 1;
    int y1 = y0 + 1;
    
    double fx = x - x0;
    double fy = y - y0;
    
    double v00 = img[y0 * width + x0];
    double v10 = img[y0 * width + x1];
    double v01 = img[y1 * width + x0];
    double v11 = img[y1 * width + x1];
    
    double v = v00 * (1-fx) * (1-fy) + 
               v10 * fx * (1-fy) +
               v01 * (1-fx) * fy +
               v11 * fx * fy;
    
    return (uint8_t)(v + 0.5);
}

/**
 * @brief 带双线性插值的逆透视变换(浮点精度)
 */
bool DroneIPM_ImageToWorldFloat(const DroneIPM_t *ipm, double img_x, double img_y,
                                 double *world_x, double *world_y)
{
    const CameraIntrinsics_t *cam = &ipm->camera;
    
    /* 归一化坐标 */
    double u_norm = (img_x - cam->cx) / cam->fx;
    double v_norm = (img_y - cam->cy) / cam->fy;
    
    /* 机体坐标系中的光线方向 */
    double ray_body[3];
    ray_body[0] = v_norm;
    ray_body[1] = u_norm;
    ray_body[2] = 1.0;
    
    /* 归一化 */
    double ray_len = sqrt(ray_body[0]*ray_body[0] + 
                          ray_body[1]*ray_body[1] + 
                          ray_body[2]*ray_body[2]);
    ray_body[0] /= ray_len;
    ray_body[1] /= ray_len;
    ray_body[2] /= ray_len;
    
    /* 转换到世界坐标系 */
    double ray_world[3];
    DroneIPM_MatVecMul(ipm->R, ray_body, ray_world);
    
    /* 求与地面交点 */
    if (fabs(ray_world[2]) < 1e-10) {
        return false;
    }
    
    double t = -ipm->cam_pos_world[2] / ray_world[2];
    
    if (t < 0) {
        return false;
    }
    
    *world_x = ipm->cam_pos_world[0] + t * ray_world[0];
    *world_y = ipm->cam_pos_world[1] + t * ray_world[1];
    
    return true;
}

/**
 * @brief 带双线性插值的正透视变换(浮点精度)
 */
bool DroneIPM_WorldToImageFloat(const DroneIPM_t *ipm, double world_x, double world_y,
                                 double *img_x, double *img_y)
{
    const CameraIntrinsics_t *cam = &ipm->camera;
    
    /* 计算地面点相对于摄像头的向量 */
    double vec_world[3];
    vec_world[0] = world_x - ipm->cam_pos_world[0];
    vec_world[1] = world_y - ipm->cam_pos_world[1];
    vec_world[2] = 0.0 - ipm->cam_pos_world[2];
    
    /* 转换到机体坐标系 */
    double Rt[3][3];
    DroneIPM_MatTranspose(ipm->R, Rt);
    
    double vec_body[3];
    DroneIPM_MatVecMul(Rt, vec_world, vec_body);
    
    /* 检查点是否在摄像头前方 */
    if (vec_body[2] <= 0) {
        return false;
    }
    
    /* 归一化坐标 */
    double v_norm = vec_body[0] / vec_body[2];
    double u_norm = vec_body[1] / vec_body[2];
    
    /* 像素坐标 */
    *img_x = u_norm * cam->fx + cam->cx;
    *img_y = v_norm * cam->fy + cam->cy;
    
    /* 检查范围 */
    if (*img_x < 0 || *img_x >= cam->image_width ||
        *img_y < 0 || *img_y >= cam->image_height) {
        return false;
    }
    
    return true;
}

/**
 * @brief 生成高质量俯视图(带双线性插值)
 */
void DroneIPM_GenerateBirdViewSmooth(const DroneIPM_t *ipm,
                                      const uint8_t *src_img,
                                      uint8_t *dst_img,
                                      int dst_width, int dst_height,
                                      double scale)
{
    const CameraIntrinsics_t *cam = &ipm->camera;
    
    double center_px = dst_width / 2.0;
    double center_py = dst_height / 2.0;
    
    for (int dst_y = 0; dst_y < dst_height; dst_y++) {
        for (int dst_x = 0; dst_x < dst_width; dst_x++) {
            /* 物理坐标 */
            double world_x = (center_py - dst_y) * scale;
            double world_y = (dst_x - center_px) * scale;
            
            /* 投影到源图像(浮点精度) */
            double src_x, src_y;
            if (DroneIPM_WorldToImageFloat(ipm, world_x, world_y, &src_x, &src_y)) {
                /* 双线性插值采样 */
                dst_img[dst_y * dst_width + dst_x] = 
                    bilinear_sample(src_img, cam->image_width, cam->image_height,
                                    src_x, src_y);
            } else {
                dst_img[dst_y * dst_width + dst_x] = 0;
            }
        }
    }
}

