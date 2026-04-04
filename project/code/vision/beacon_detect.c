/**
 * @file beacon_detect.c
 * @brief 红外信标检测实现 — 移植自 case02 Unicycle_Beacon_2025/dual_camera.c
 *
 * 保留的核心算法:
 *   1. 全局阈值二值化  (convertToBitStorage)
 *   2. 八邻域BFS连通域检测  (bfs / findConnectedComponents)
 *   3. 面积 / 宽高比 / 占空比 几何过滤
 *   4. 逆透视变换获取地面坐标
 *
 * 适配本项目的变更:
 *   - IPM: 使用 drone_ipm_embedded (俯视摄像头), 通过 IPM_Image2World()
 *   - 去除去畸变(mapping_table)  — 无人机俯视不需要
 *   - 去除地平线过滤             — 俯视摄像头无地平线
 *   - 去除双摄融合               — 本项目单摄像头
 */

#include "beacon_detect.h"
#include "common/project_headfile.h"
#include <math.h>
#include <string.h>

/*
 * 模块定位:
 * - 输入: 一帧 MT9V03X 灰度图
 * - 输出: 每个信标候选的图像质心、几何特征和地面坐标
 * - 依赖: IPM_Interface_Update*() 负责提前写入当前姿态/高度
 */

/* ====================== 默认参数 ====================== */

BeaconConfig_t g_beacon_config = {
    .binary_threshold = 170,
    .area_threshold   = 30,
    .min_aspect       = 0.3f,
    .max_aspect       = 6.0f,
    .min_occupancy    = 0.15f
};

/* ====================== 静态缓冲区 ====================== */

static uint8_t g_binary[BEACON_IMG_ROWS][BEACON_IMG_COLS];
static uint8_t g_visited[BEACON_IMG_ROWS][BEACON_IMG_COLS];

/* BFS 队列 — 用 uint16 存 row*cols+col (最大 120*188=22560) */
static uint16_t g_bfs_queue[BEACON_IMG_ROWS * BEACON_IMG_COLS];

/* ==================== 内部: 二值化 ==================== */

/**
 * @brief 全局阈值二值化  (对应 case02 convertToBitStorage)
 */
static void binarize(const uint8 src[BEACON_IMG_ROWS][BEACON_IMG_COLS],
                     uint8_t dst[BEACON_IMG_ROWS][BEACON_IMG_COLS],
                     int thresh)
{
    memset(dst, 0, BEACON_IMG_ROWS * BEACON_IMG_COLS);
    for (int r = 0; r < BEACON_IMG_ROWS; r++) {
        for (int c = 0; c < BEACON_IMG_COLS; c++) {
            if (src[r][c] > thresh) {
                dst[r][c] = 1;
            }
        }
    }
}

/* ==================== 内部: 八邻域 BFS ==================== */

/**
 * @brief 八邻域BFS搜索单个连通域  (与 case02 bfs() 完全一致)
 *
 * 从 (start_row, start_col) 出发, 搜集连通的白色像素,
 * 返回面积/重心/亮度/宽高比/占空比.
 */
static void bfs_region(int start_row, int start_col,
                       const uint8 original[BEACON_IMG_ROWS][BEACON_IMG_COLS],
                       int *out_area,
                       float *out_centroid_row, float *out_centroid_col,
                       float *out_brightness,
                       float *out_aspect, float *out_occupancy)
{
    static const int dirs[8][2] = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1},
        {-1,-1}, {-1, 1}, {1,-1}, {1, 1}
    };

    uint16_t front = 0, rear = 0;
    g_bfs_queue[rear++] = (uint16_t)(start_row * BEACON_IMG_COLS + start_col);
    g_visited[start_row][start_col] = 1;

    int pixel_count = 0;
    int row_sum = 0, col_sum = 0;
    int brightness_sum = 0;
    int min_r = start_row, max_r = start_row;
    int min_c = start_col, max_c = start_col;

    while (front < rear) {
        uint16_t idx = g_bfs_queue[front++];
        int r = idx / BEACON_IMG_COLS;
        int c = idx % BEACON_IMG_COLS;

        pixel_count++;
        row_sum += r;
        col_sum += c;
        brightness_sum += original[r][c];

        if (r < min_r) min_r = r;
        if (r > max_r) max_r = r;
        if (c < min_c) min_c = c;
        if (c > max_c) max_c = c;

        for (int i = 0; i < 8; i++) {
            int nr = r + dirs[i][0];
            int nc = c + dirs[i][1];
            if (nr >= 0 && nr < BEACON_IMG_ROWS &&
                nc >= 0 && nc < BEACON_IMG_COLS &&
                !g_visited[nr][nc] &&
                g_binary[nr][nc] == 1)
            {
                if (rear < (BEACON_IMG_ROWS * BEACON_IMG_COLS)) {
                    g_visited[nr][nc] = 1;
                    g_bfs_queue[rear++] = (uint16_t)(nr * BEACON_IMG_COLS + nc);
                } else {
                    goto BFS_END;   /* 队列满, 安全退出 */
                }
            }
        }
    }

BFS_END:
    *out_area = pixel_count;
    if (pixel_count > 0) {
        *out_centroid_row = (float)row_sum / pixel_count;
        *out_centroid_col = (float)col_sum / pixel_count;
        *out_brightness   = (float)brightness_sum / pixel_count;

        int w = max_c - min_c + 1;
        int h = max_r - min_r + 1;
        *out_aspect    = (h > 0) ? ((float)w / (float)h) : 1.0f;
        *out_occupancy = (w > 0 && h > 0) ? ((float)pixel_count / (float)(w * h)) : 0.0f;
    } else {
        *out_centroid_row = 0.0f;
        *out_centroid_col = 0.0f;
        *out_brightness   = 0.0f;
        *out_aspect       = 1.0f;
        *out_occupancy    = 0.0f;
    }
}

/* ==================== 对外接口 ==================== */

/**
 * @brief 初始化信标检测模块
 *
 * 当前实现没有动态资源需要申请，保留这个入口是为了让 vision
 * 目录里的各个模块拥有统一的初始化习惯，后续如果接菜单/Flash 参数
 * 也可以直接扩展在这里。
 */
void BeaconDetect_Init(void)
{
    /* 参数已有默认初始值; 后续可从菜单/Flash加载 */
}

/**
 * @brief 处理一帧图像, 检测红外信标
 *
 * 流程与 case02 processCameraImage() + convertToGlobalCoordinates() 等效:
 *   binarize → BFS findConnectedComponents → 几何过滤 → IPM_Image2World
 */
int BeaconDetect_Process(const uint8 image[MT9V03X_H][MT9V03X_W],
                         BeaconResult_t *results, int max_results)
{
    int count = 0;

    /* 1. 二值化 */
    binarize(image, g_binary, g_beacon_config.binary_threshold);

    /* 2. 清访问标记 */
    memset(g_visited, 0, sizeof(g_visited));

    /* 3. 扫描连通域 (对应 case02 findConnectedComponents) */
    for (int r = 0; r < BEACON_IMG_ROWS; r++) {
        for (int c = 0; c < BEACON_IMG_COLS; c++) {
            if (g_binary[r][c] != 1 || g_visited[r][c]) continue;

            int   area;
            float crow, ccol, brightness, aspect, occupancy;
            bfs_region(r, c, image,
                       &area, &crow, &ccol,
                       &brightness, &aspect, &occupancy);

            /* 4. 几何过滤 (与 case02 完全一致) */
            if (area < g_beacon_config.area_threshold) continue;
            if (aspect < g_beacon_config.min_aspect ||
                aspect > g_beacon_config.max_aspect)  continue;
            if (occupancy < g_beacon_config.min_occupancy) continue;
            /* 原 case02: 近处面积过小则跳过 */
            if (crow > (BEACON_IMG_ROWS / 2) && area < 10) continue;

            if (count >= max_results) goto DONE;

            /* 5. 逆透视: 图像 → 地面(cm) */
            float wx, wy;
            IPM_Image2World((int)ccol, (int)crow, &wx, &wy);

            float dist  = sqrtf(wx * wx + wy * wy);
            float angle = atan2f(wy, wx) * (180.0f / 3.14159265f);

            BeaconResult_t *b = &results[count];
            b->centroid_row      = crow;
            b->centroid_col      = ccol;
            b->area              = area;
            b->average_brightness = brightness;
            b->aspect_ratio      = aspect;
            b->occupancy_ratio   = occupancy;
            b->world_x           = wx;
            b->world_y           = wy;
            b->distance_cm       = dist;
            b->angle_deg         = angle;
            count++;
        }
    }

DONE:
    return count;
}

/**
 * @brief 从结果数组中挑选距离最近的信标
 *
 * 这个函数不重新做筛选，只在 BeaconDetect_Process() 已经输出的
 * 候选结果里按照 `distance_cm` 取最小值，方便主循环直接拿最近目标。
 */
int BeaconDetect_GetNearestIndex(const BeaconResult_t *results, int count)
{
    if (count <= 0) return -1;

    int   best = 0;
    float best_dist = results[0].distance_cm;
    for (int i = 1; i < count; i++) {
        if (results[i].distance_cm < best_dist) {
            best_dist = results[i].distance_cm;
            best = i;
        }
    }
    return best;
}

