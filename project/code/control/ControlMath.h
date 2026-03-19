#ifndef CONTROL_MATH_H
#define CONTROL_MATH_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <float.h>

// ========== 数学常量定义 ==========
#ifndef M_PI
#define M_PI        3.14159265358979323846f
#endif

#ifndef M_PI_2
#define M_PI_2      1.57079632679489661923f
#endif

// ========== 角度弧度转换宏 ==========
#ifndef DEG2RAD
#define DEG2RAD     0.01745329252f    // π/180
#endif

#ifndef RAD2DEG
#define RAD2DEG     57.295779513f     // 180/π
#endif

// 别名定义（兼容旧代码）
#ifndef Ang2Rad
#define Ang2Rad     DEG2RAD
#endif

#ifndef Rad2Ang
#define Rad2Ang     RAD2DEG
#endif

// ========== 基础类型定义 ==========

typedef struct {
    float x;
    float y;
} Vector2f;

typedef struct {
    float x;
    float y;
    float z;
} Vector3f;

typedef struct {
    float w;  // 实部
    float x;
    float y;
    float z;
} Quaternion;

// 通用四元数类型（为兼容不同模块）
#ifndef QUATERNION_T_DEFINED
#define QUATERNION_T_DEFINED
typedef struct {
    float w;  // 实部
    float x;  // 虚部i
    float y;  // 虚部j
    float z;  // 虚部k
} Quaternion_t;
#endif

// 3x3旋转矩阵 (Direction Cosine Matrix)
typedef struct {
    float m[3][3];
} Dcmf;

// 欧拉角
typedef struct {
    float phi;    // roll
    float theta;  // pitch
    float psi;    // yaw
} Eulerf;

// 姿态设定点
typedef struct {
    uint64_t timestamp;
    float q_d[4];           // 期望四元数 [w, x, y, z]
    float yaw_sp_move_rate; // 偏航速率
    float thrust_body[3];   // 机体坐标系推力
    float roll_body;        // 仅用于日志
    float pitch_body;       // 仅用于日志
    float yaw_body;         // 仅用于日志
} vehicle_attitude_setpoint_s;

// ========== 常量定义 ==========
#define PX4_ISFINITE(x) isfinite(x)
#ifndef FLT_EPSILON
#define FLT_EPSILON 1.19209290e-07F
#endif

// ========== 向量运算辅助函数 ==========

// Vector3f 操作
float Vector3f_dot(const Vector3f *v1, const Vector3f *v2);
Vector3f Vector3f_cross(const Vector3f *v1, const Vector3f *v2);
float Vector3f_length(const Vector3f *v);
float Vector3f_norm_squared(const Vector3f *v);
void Vector3f_normalize(Vector3f *v);
Vector3f Vector3f_normalized(const Vector3f *v);
Vector3f Vector3f_unit(const Vector3f *v);  // 同 normalized
void Vector3f_zero(Vector3f *v);
Vector3f Vector3f_add(const Vector3f *v1, const Vector3f *v2);
Vector3f Vector3f_sub(const Vector3f *v1, const Vector3f *v2);
Vector3f Vector3f_scale(const Vector3f *v, float scalar);

// Vector2f 操作
float Vector2f_length(const Vector2f *v);
float Vector2f_norm(const Vector2f *v);
float Vector2f_dot(const Vector2f *v1, const Vector2f *v2);
void Vector2f_normalize(Vector2f *v);
Vector2f Vector2f_normalized(const Vector2f *v);

// ========== 四元数操作 ==========
Quaternion Quaternion_from_dcm(const Dcmf *dcm);
void Quaternion_normalize(Quaternion *q);
void Quaternion_copyTo(const Quaternion *q, float dest[4]);

// ========== 旋转矩阵操作 ==========
Dcmf Dcmf_identity(void);
Eulerf Eulerf_from_dcm(const Dcmf *dcm);

// ========== ControlMath 核心函数 ==========

/**
 * 将推力向量和偏航设定点转换为期望姿态
 * @param thr_sp 期望的3D推力向量
 * @param yaw_sp 期望的偏航角
 * @param att_sp 要填充的姿态设定点
 */
void ControlMath_thrustToAttitude(const Vector3f *thr_sp, float yaw_sp, 
                                  vehicle_attitude_setpoint_s *att_sp);

/**
 * 限制两个单位向量之间的倾斜角
 * @param body_unit 如果角度太大将被调整的单位向量
 * @param world_unit 用于测量角度的固定向量
 * @param max_angle 向量之间的最大倾斜角（弧度）
 */
void ControlMath_limitTilt(Vector3f *body_unit, const Vector3f *world_unit, 
                           float max_angle);

/**
 * 将机体z向量和偏航设定点转换为期望姿态
 * @param body_z 期望机体z轴方向的世界坐标系3D向量
 * @param yaw_sp 期望的偏航设定点
 * @param att_sp 要填充的姿态设定点
 */
void ControlMath_bodyzToAttitude(Vector3f body_z, float yaw_sp, 
                                 vehicle_attitude_setpoint_s *att_sp);

/**
 * 输出两个向量的和，但要尊重限制和优先级
 * 两个向量的和被约束，使得v0优先于v1
 * @param v0 在给定最大可用幅度下具有优先级的2D向量
 * @param v1 在给定最大可用幅度下优先级较低的2D向量
 * @param max 最大幅度
 * @return 2D向量
 */
Vector2f ControlMath_constrainXY(const Vector2f *v0, const Vector2f *v1, float max);

/**
 * 球体与线段相交
 * @param sphere_c 球心
 * @param sphere_r 球半径
 * @param line_a 线段起点
 * @param line_b 线段终点
 * @param res 结果点
 * @return 是否相交
 */
bool ControlMath_cross_sphere_line(const Vector3f *sphere_c, float sphere_r,
                                   const Vector3f *line_a, const Vector3f *line_b,
                                   Vector3f *res);

/**
 * 添加前馈到设定点，确保现有或添加的NAN不影响控制
 * @param setpoint 现有的可能为NAN的设定点
 * @param addition 要添加到设定点的值/NAN
 */
void ControlMath_addIfNotNan(float *setpoint, float addition);

/**
 * Vector3f版本的addIfNotNan，单独处理每个元素
 */
void ControlMath_addIfNotNanVector3f(Vector3f *setpoint, const Vector3f *addition);

/**
 * 用零覆盖Vector3f中为NAN的元素
 * @param vector 可能包含NAN元素的向量
 */
void ControlMath_setZeroIfNanVector3f(Vector3f *vector);

#endif // CONTROL_MATH_H
