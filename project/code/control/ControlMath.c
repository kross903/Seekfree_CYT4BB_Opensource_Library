#include "ControlMath.h"
#include <string.h>
#include <float.h>

// ========== Vector3f 基础操作实现 ==========

float Vector3f_dot(const Vector3f *v1, const Vector3f *v2)
{
    return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

Vector3f Vector3f_cross(const Vector3f *v1, const Vector3f *v2)
{
    Vector3f result;
    result.x = v1->y * v2->z - v1->z * v2->y;
    result.y = v1->z * v2->x - v1->x * v2->z;
    result.z = v1->x * v2->y - v1->y * v2->x;
    return result;
}

float Vector3f_length(const Vector3f *v)
{
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

float Vector3f_norm_squared(const Vector3f *v)
{
    return v->x * v->x + v->y * v->y + v->z * v->z;
}

void Vector3f_normalize(Vector3f *v)
{
    float len = Vector3f_length(v);
    if (len > FLT_EPSILON) {
        v->x /= len;
        v->y /= len;
        v->z /= len;
    }
}

Vector3f Vector3f_normalized(const Vector3f *v)
{
    Vector3f result = *v;
    Vector3f_normalize(&result);
    return result;
}

Vector3f Vector3f_unit(const Vector3f *v)
{
    return Vector3f_normalized(v);
}

void Vector3f_zero(Vector3f *v)
{
    v->x = 0.0f;
    v->y = 0.0f;
    v->z = 0.0f;
}

Vector3f Vector3f_add(const Vector3f *v1, const Vector3f *v2)
{
    Vector3f result;
    result.x = v1->x + v2->x;
    result.y = v1->y + v2->y;
    result.z = v1->z + v2->z;
    return result;
}

Vector3f Vector3f_sub(const Vector3f *v1, const Vector3f *v2)
{
    Vector3f result;
    result.x = v1->x - v2->x;
    result.y = v1->y - v2->y;
    result.z = v1->z - v2->z;
    return result;
}

Vector3f Vector3f_scale(const Vector3f *v, float scalar)
{
    Vector3f result;
    result.x = v->x * scalar;
    result.y = v->y * scalar;
    result.z = v->z * scalar;
    return result;
}

// ========== Vector2f 基础操作实现 ==========

float Vector2f_length(const Vector2f *v)
{
    return sqrtf(v->x * v->x + v->y * v->y);
}

float Vector2f_norm(const Vector2f *v)
{
    return Vector2f_length(v);
}

float Vector2f_dot(const Vector2f *v1, const Vector2f *v2)
{
    return v1->x * v2->x + v1->y * v2->y;
}

void Vector2f_normalize(Vector2f *v)
{
    float len = Vector2f_length(v);
    if (len > FLT_EPSILON) {
        v->x /= len;
        v->y /= len;
    }
}

Vector2f Vector2f_normalized(const Vector2f *v)
{
    Vector2f result = *v;
    Vector2f_normalize(&result);
    return result;
}

// ========== 四元数操作实现 ==========

void Quaternion_normalize(Quaternion *q)
{
    float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm > FLT_EPSILON) {
        q->w /= norm;
        q->x /= norm;
        q->y /= norm;
        q->z /= norm;
    } else {
        q->w = 1.0f;
        q->x = 0.0f;
        q->y = 0.0f;
        q->z = 0.0f;
    }
}

Quaternion Quaternion_from_dcm(const Dcmf *dcm)
{
    Quaternion q;
    float trace = dcm->m[0][0] + dcm->m[1][1] + dcm->m[2][2];
    
    if (trace > 0.0f) {
        float s = sqrtf(trace + 1.0f) * 2.0f;
        q.w = 0.25f * s;
        q.x = (dcm->m[2][1] - dcm->m[1][2]) / s;
        q.y = (dcm->m[0][2] - dcm->m[2][0]) / s;
        q.z = (dcm->m[1][0] - dcm->m[0][1]) / s;
    } else if ((dcm->m[0][0] > dcm->m[1][1]) && (dcm->m[0][0] > dcm->m[2][2])) {
        float s = sqrtf(1.0f + dcm->m[0][0] - dcm->m[1][1] - dcm->m[2][2]) * 2.0f;
        q.w = (dcm->m[2][1] - dcm->m[1][2]) / s;
        q.x = 0.25f * s;
        q.y = (dcm->m[0][1] + dcm->m[1][0]) / s;
        q.z = (dcm->m[0][2] + dcm->m[2][0]) / s;
    } else if (dcm->m[1][1] > dcm->m[2][2]) {
        float s = sqrtf(1.0f + dcm->m[1][1] - dcm->m[0][0] - dcm->m[2][2]) * 2.0f;
        q.w = (dcm->m[0][2] - dcm->m[2][0]) / s;
        q.x = (dcm->m[0][1] + dcm->m[1][0]) / s;
        q.y = 0.25f * s;
        q.z = (dcm->m[1][2] + dcm->m[2][1]) / s;
    } else {
        float s = sqrtf(1.0f + dcm->m[2][2] - dcm->m[0][0] - dcm->m[1][1]) * 2.0f;
        q.w = (dcm->m[1][0] - dcm->m[0][1]) / s;
        q.x = (dcm->m[0][2] + dcm->m[2][0]) / s;
        q.y = (dcm->m[1][2] + dcm->m[2][1]) / s;
        q.z = 0.25f * s;
    }
    
    Quaternion_normalize(&q);
    return q;
}

void Quaternion_copyTo(const Quaternion *q, float dest[4])
{
    dest[0] = q->w;
    dest[1] = q->x;
    dest[2] = q->y;
    dest[3] = q->z;
}

// ========== 旋转矩阵操作实现 ==========

Dcmf Dcmf_identity(void)
{
    Dcmf dcm;
    memset(&dcm, 0, sizeof(Dcmf));
    dcm.m[0][0] = 1.0f;
    dcm.m[1][1] = 1.0f;
    dcm.m[2][2] = 1.0f;
    return dcm;
}

Eulerf Eulerf_from_dcm(const Dcmf *dcm)
{
    Eulerf euler;
    
    // Roll (phi)
    euler.phi = atan2f(dcm->m[2][1], dcm->m[2][2]);
    
    // Pitch (theta)
    euler.theta = asinf(-dcm->m[2][0]);
    
    // Yaw (psi)
    euler.psi = atan2f(dcm->m[1][0], dcm->m[0][0]);
    
    return euler;
}

// ========== ControlMath 核心函数实现 ==========

// 将推力向量转为姿态设定
// 原理：推力方向即机体 -Z 轴方向，结合期望偏航构造姿态
// 调用者：PositionControl_getAttitudeSetpoint()
void ControlMath_thrustToAttitude(const Vector3f *thr_sp, float yaw_sp, 
                                  vehicle_attitude_setpoint_s *att_sp)
{
    // 将推力取反作为body_z
    Vector3f body_z;
    body_z.x = -thr_sp->x;
    body_z.y = -thr_sp->y;
    body_z.z = -thr_sp->z;
    
    ControlMath_bodyzToAttitude(body_z, yaw_sp, att_sp);
    
    // 推力幅度
    att_sp->thrust_body[2] = -Vector3f_length(thr_sp);
}

// 限制机体倾角，避免姿态过大
// 调用者：PositionControl._accelerationControl()
void ControlMath_limitTilt(Vector3f *body_unit, const Vector3f *world_unit, 
                           float max_angle)
{
    // 确定倾斜角
    float dot_product_unit = Vector3f_dot(body_unit, world_unit);
    float angle = acosf(dot_product_unit);
    
    // 限制倾斜角
    if (angle > max_angle) {
        angle = max_angle;
    } else {
        return;  // 不需要限制
    }
    
    // rejection = body_unit - (dot_product_unit * world_unit)
    Vector3f world_scaled = Vector3f_scale(world_unit, dot_product_unit);
    Vector3f rejection = Vector3f_sub(body_unit, &world_scaled);
    
    // 处理完全平行向量的特殊情况
    if (Vector3f_norm_squared(&rejection) < FLT_EPSILON) {
        rejection.x = 1.0f;
        rejection.y = 0.0f;
        rejection.z = 0.0f;
    }
    
    // body_unit = cos(angle) * world_unit + sin(angle) * rejection.unit()
    Vector3f rejection_unit = Vector3f_unit(&rejection);
    Vector3f term1 = Vector3f_scale(world_unit, cosf(angle));
    Vector3f term2 = Vector3f_scale(&rejection_unit, sinf(angle));
    *body_unit = Vector3f_add(&term1, &term2);
}

// 由机体 Z 轴方向和偏航构造姿态
// 调用者：ControlMath_thrustToAttitude()
void ControlMath_bodyzToAttitude(Vector3f body_z, float yaw_sp, 
                                 vehicle_attitude_setpoint_s *att_sp)
{
    // 零向量，无方向，设置安全水平值
    if (Vector3f_norm_squared(&body_z) < FLT_EPSILON) {
        body_z.x = 0.0f;
        body_z.y = 0.0f;
        body_z.z = 1.0f;
    }
    
    Vector3f_normalize(&body_z);
    
    // XY平面上期望偏航方向的向量，旋转PI/2
    Vector3f y_C;
    y_C.x = -sinf(yaw_sp);
    y_C.y = cosf(yaw_sp);
    y_C.z = 0.0f;
    
    // 期望的body_x轴，与body_z正交
    // body_x = y_C % body_z (叉积)
    Vector3f body_x = Vector3f_cross(&y_C, &body_z);
    
    // 倒置时保持机头朝前
    if (body_z.z < 0.0f) {
        body_x.x = -body_x.x;
        body_x.y = -body_x.y;
        body_x.z = -body_x.z;
    }
    
    if (fabsf(body_z.z) < 0.000001f) {
        // 期望推力在XY平面，设置X向下以构造正确的矩阵
        // 但实际上不会使用偏航分量
        Vector3f_zero(&body_x);
        body_x.z = 1.0f;
    }
    
    Vector3f_normalize(&body_x);
    
    // 期望的body_y轴
    // body_y = body_z % body_x
    Vector3f body_y = Vector3f_cross(&body_z, &body_x);
    
    // 填充旋转矩阵
    Dcmf R_sp;
    for (int i = 0; i < 3; i++) {
        R_sp.m[i][0] = (i == 0) ? body_x.x : (i == 1) ? body_x.y : body_x.z;
        R_sp.m[i][1] = (i == 0) ? body_y.x : (i == 1) ? body_y.y : body_y.z;
        R_sp.m[i][2] = (i == 0) ? body_z.x : (i == 1) ? body_z.y : body_z.z;
    }
    
    // 将四元数设定点复制到姿态设定点主题
    Quaternion q_sp = Quaternion_from_dcm(&R_sp);
    Quaternion_copyTo(&q_sp, att_sp->q_d);
    
    // 计算欧拉角，仅用于日志记录，不得用于控制
    Eulerf euler = Eulerf_from_dcm(&R_sp);
    att_sp->roll_body = euler.phi;
    att_sp->pitch_body = euler.theta;
    att_sp->yaw_body = euler.psi;
}

Vector2f ControlMath_constrainXY(const Vector2f *v0, const Vector2f *v1, float max)
{
    // v0 + v1
    Vector2f sum;
    sum.x = v0->x + v1->x;
    sum.y = v0->y + v1->y;
    
    float sum_norm = Vector2f_norm(&sum);
    
    if (sum_norm <= max) {
        // 向量不超过最大幅度
        return sum;
    }
    
    float v0_len = Vector2f_length(v0);
    
    if (v0_len >= max) {
        // v0的幅度（具有优先级）已经超过最大值
        Vector2f result = Vector2f_normalized(v0);
        result.x *= max;
        result.y *= max;
        return result;
    }
    
    // v1 - v0
    Vector2f diff;
    diff.x = v1->x - v0->x;
    diff.y = v1->y - v0->y;
    float diff_norm = Vector2f_norm(&diff);
    
    if (fabsf(diff_norm) < 0.001f) {
        // 两个向量相等
        Vector2f result = Vector2f_normalized(v0);
        result.x *= max;
        result.y *= max;
        return result;
    }
    
    if (v0_len < 0.001f) {
        // 第一个向量为0
        Vector2f result = Vector2f_normalized(v1);
        result.x *= max;
        result.y *= max;
        return result;
    }
    
    // 求解缩放因子 s
    // vf = v0 + s * u1
    // ||vf|| = max
    //
    // 二次方程求解:
    // s^2*a + s*b + c = 0
    // a = 1 (因为u1是归一化的)
    // b = 2 * u1.dot(v0)
    // c = ||v0||^2 - max^2
    //
    // s = -m + sqrt(m^2 - c)
    // 其中 m = u1.dot(v0)
    
    Vector2f u1 = Vector2f_normalized(v1);
    float m = Vector2f_dot(&u1, v0);
    float c = Vector2f_dot(v0, v0) - max * max;
    float s = -m + sqrtf(m * m - c);
    
    Vector2f result;
    result.x = v0->x + u1.x * s;
    result.y = v0->y + u1.y * s;
    
    return result;
}

bool ControlMath_cross_sphere_line(const Vector3f *sphere_c, float sphere_r,
                                   const Vector3f *line_a, const Vector3f *line_b,
                                   Vector3f *res)
{
    // 将球心投影到线段AB上（归一化）
    Vector3f ab_norm = Vector3f_sub(line_b, line_a);
    
    if (Vector3f_length(&ab_norm) < 0.01f) {
        return true;
    }
    
    Vector3f_normalize(&ab_norm);
    
    // d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm)
    Vector3f ca = Vector3f_sub(sphere_c, line_a);
    float proj_len = Vector3f_dot(&ca, &ab_norm);
    Vector3f ab_scaled = Vector3f_scale(&ab_norm, proj_len);
    Vector3f d = Vector3f_add(line_a, &ab_scaled);
    
    Vector3f cd = Vector3f_sub(sphere_c, &d);
    float cd_len = Vector3f_length(&cd);
    
    if (sphere_r > cd_len) {
        // 有三角形CDX，已知CD和CX=R，求DX
        float dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);
        
        Vector3f cb = Vector3f_sub(sphere_c, line_b);
        float cb_dot_ab = Vector3f_dot(&cb, &ab_norm);
        
        if (cb_dot_ab > 0.0f) {
            // 目标航点已经在我们后面
            *res = *line_b;
        } else {
            // 目标在我们前面
            Vector3f dx_vec = Vector3f_scale(&ab_norm, dx_len);
            *res = Vector3f_add(&d, &dx_vec);
        }
        
        return true;
        
    } else {
        // 没有交点，返回D
        *res = d;
        
        // 前一个航点仍在我们前面
        Vector3f ca_check = Vector3f_sub(sphere_c, line_a);
        if (Vector3f_dot(&ca_check, &ab_norm) < 0.0f) {
            *res = *line_a;
        }
        
        // 目标航点已经在我们后面
        Vector3f cb_check = Vector3f_sub(sphere_c, line_b);
        if (Vector3f_dot(&cb_check, &ab_norm) > 0.0f) {
            *res = *line_b;
        }
        
        return false;
    }
}

void ControlMath_addIfNotNan(float *setpoint, float addition)
{
    if (PX4_ISFINITE(*setpoint) && PX4_ISFINITE(addition)) {
        // 都不是NAN，添加到设定点
        *setpoint += addition;
        
    } else if (!PX4_ISFINITE(*setpoint)) {
        // 设定点是NAN，取addition
        *setpoint = addition;
    }
    
    // addition是NAN或两者都是NAN，不做任何操作
}

void ControlMath_addIfNotNanVector3f(Vector3f *setpoint, const Vector3f *addition)
{
    ControlMath_addIfNotNan(&setpoint->x, addition->x);
    ControlMath_addIfNotNan(&setpoint->y, addition->y);
    ControlMath_addIfNotNan(&setpoint->z, addition->z);
}

void ControlMath_setZeroIfNanVector3f(Vector3f *vector)
{
    // 添加零向量会用零覆盖NAN元素
    Vector3f zero = {0.0f, 0.0f, 0.0f};
    ControlMath_addIfNotNanVector3f(vector, &zero);
}
