/**
 * @file camera_undistort_rect_map.h
 * @brief 整图调试去畸变映射占位文件，由电脑端标定工具生成后覆盖。
 */

#ifndef CAMERA_UNDISTORT_RECT_MAP_H_
#define CAMERA_UNDISTORT_RECT_MAP_H_

#define CAMERA_UNDISTORT_RECT_MAP_HAS_DATA    (0)

#if CAMERA_UNDISTORT_RECT_MAP_HAS_DATA
extern const int16 camera_undistort_rect_src_x[MT9V03X_H][MT9V03X_W];
extern const int16 camera_undistort_rect_src_y[MT9V03X_H][MT9V03X_W];
extern const uint8 camera_undistort_rect_valid[MT9V03X_H][MT9V03X_W];
#endif

#endif /* CAMERA_UNDISTORT_RECT_MAP_H_ */
