/**
 * @file camera_ray_table.h
 * @brief 原始像素到机体系射线查表占位文件，由电脑端标定工具生成后覆盖。
 */

#ifndef CAMERA_RAY_TABLE_H_
#define CAMERA_RAY_TABLE_H_

#define CAMERA_RAY_TABLE_HAS_DATA    (0)
#define CAMERA_RAY_TABLE_Q           (14)

#if CAMERA_RAY_TABLE_HAS_DATA
extern const int16 camera_ray_table_x[MT9V03X_H][MT9V03X_W];
extern const int16 camera_ray_table_y[MT9V03X_H][MT9V03X_W];
extern const int16 camera_ray_table_z[MT9V03X_H][MT9V03X_W];
extern const uint8 camera_ray_table_valid[MT9V03X_H][MT9V03X_W];
#endif

#endif /* CAMERA_RAY_TABLE_H_ */
