#include "alg_quaternion.h"
/**
 * @brief 四元数乘法
 */
static inline quat_t quat_mul(quat_t a, quat_t b)
{
    quat_t result;
    result.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    result.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    result.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    result.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return result;
}

/**
 * @brief 四元数共轭
 */
static inline quat_t quat_conj(quat_t q)
{
    return (quat_t){q.w, -q.x, -q.y, -q.z};
}

/**
 * @brief 四元数归一化
 */
static inline quat_t quat_normalize(quat_t q)
{
    float norm = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (norm > 0) {
        q.w /= norm;
        q.x /= norm;
        q.y /= norm;
        q.z /= norm;
    }
    return q;
}

// ===== 核心姿态解算 =====

/**
 * @brief 用四元数旋转向量 (v' = q * v * q⁻¹)
 */
static inline vec3_t quat_rotate_vector(quat_t q, vec3_t v)
{
    // 把向量转成纯四元数
    quat_t p = {0, v.x, v.y, v.z};
    
    // 计算 q * p * q⁻¹
    quat_t q_conj = quat_conj(q);
    quat_t temp = quat_mul(q, p);
    quat_t result = quat_mul(temp, q_conj);
    
    return (vec3_t){result.x, result.y, result.z};
}

/**
 * @brief 从加速度计估计重力方向（用于误差计算）
 */
static inline vec3_t quat_to_gravity(quat_t q)
{
    // 在机体坐标系中，重力方向应该是 (0, 0, 1) 旋转后的结果
    vec3_t gravity_world = {0, 0, 1};
    return quat_rotate_vector(q, gravity_world);
}

/**
 * @brief 四元数微分方程：用陀螺仪更新四元数
 */
static inline quat_t quat_integrate(quat_t q, vec3_t gyro, float dt)
{
    // dq/dt = 0.5 * q * ω
    // 其中 ω 是纯四元数 (0, gyro.x, gyro.y, gyro.z)
    
    quat_t omega = {0, gyro.x, gyro.y, gyro.z};
    quat_t dq = quat_mul(q, omega);
    
    // 一阶龙格库塔积分
    q.w += 0.5f * dt * dq.w;
    q.x += 0.5f * dt * dq.x;
    q.y += 0.5f * dt * dq.y;
    q.z += 0.5f * dt * dq.z;
    
    // 归一化后返回
    return quat_normalize(q);
}

/**
 * @brief 四元数转欧拉角（直接用四元数算，不用矩阵）
 * @param q 输入四元数
 * @return euler_t 输出欧拉角（角度）
 */
euler_t quat_to_euler(quat_t q)
{
    euler_t euler;
    
    // Roll (X轴)
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    euler.roll = atan2f(sinr_cosp, cosr_cosp);
    
    // Pitch (Y轴)
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1.0f)
        euler.pitch = copysignf(M_PI / 2.0f, sinp);  // 处理万向锁
    else
        euler.pitch = asinf(sinp);
    
    // Yaw (Z轴)
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    euler.yaw = atan2f(siny_cosp, cosy_cosp);
    
    // 转换为角度
    euler.roll *= 180.0f / M_PI;
    euler.pitch *= 180.0f / M_PI;
    euler.yaw *= 180.0f / M_PI;
  
    return euler;
}

// ===== Mahony互补滤波（纯四元数版）=====

/**
 * @brief Mahony姿态解算（一步更新）
 * @param q 当前四元数（会被更新）
 * @param imu IMU数据
 * @param kp 比例增益
 * @param ki 积分增益
 */
void mahony_update(quat_t *q, imu_data_t imu, float kp, float ki)
{
    static vec3_t integral_error = {0, 0, 0};  // 误差积分
    imu.Ki = ki;
    imu.Kp = kp;
    // 1. 归一化加速度计
    float acc_norm = sqrtf(imu.acc.x*imu.acc.x + 
                           imu.acc.y*imu.acc.y + 
                           imu.acc.z*imu.acc.z);
    if (acc_norm > 0) {
        imu.acc.x /= acc_norm;
        imu.acc.y /= acc_norm;
        imu.acc.z /= acc_norm;
    }
    
    // 2. 用当前四元数估计重力方向（在机体坐标系中）
    vec3_t gravity = quat_to_gravity(*q);
    
    // 3. 计算误差 = 加速度计测量的重力方向 × 估计的重力方向
    vec3_t error;
    error.x = imu.acc.y * gravity.z - imu.acc.z * gravity.y;
    error.y = imu.acc.z * gravity.x - imu.acc.x * gravity.z;
    error.z = imu.acc.x * gravity.y - imu.acc.y * gravity.x;
    
    // 4. 积分误差
    integral_error.x += error.x * ki * imu.dt;
    integral_error.y += error.y * ki * imu.dt;
    integral_error.z += error.z * ki * imu.dt;
    
    // 5. 修正陀螺仪
    vec3_t gyro_corrected;
    gyro_corrected.x = imu.gyro.x + kp * error.x + integral_error.x;
    gyro_corrected.y = imu.gyro.y + kp * error.y + integral_error.y;
    gyro_corrected.z = imu.gyro.z + kp * error.z + integral_error.z;
    
    // 6. 用修正后的陀螺仪更新四元数
    *q = quat_integrate(*q, gyro_corrected, imu.dt);
}
