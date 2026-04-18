/**
 * @file alg_quaternion.h
 * @brief 四元数与 Mahony 姿态解算接口定义
 *
 * @details
 * 该文件在保留纯数据结构的同时，引入一个真正持有滤波器积分状态的类对象，
 * 用来替代原来文件级
 * `static integralFBx / integralFBy / integralFBz` 的做法。
 */
#ifndef __ALG_QUATERNION_H__
#define __ALG_QUATERNION_H__

#include <math.h>
#include <stdint.h>

/**
 * @brief 四元数数据结构
 */
typedef struct
{
    float w; /**< 标量部 */
    float x; /**< 虚部 X */
    float y; /**< 虚部 Y */
    float z; /**< 虚部 Z */
} quat_t;

/**
 * @brief 三维向量
 */
typedef struct
{
    float x; /**< X 分量 */
    float y; /**< Y 分量 */
    float z; /**< Z 分量 */
} vec3_t;

/**
 * @brief 欧拉角
 *
 * @details
 * 当前工程中欧拉角对外统一使用角度制。
 */
typedef struct
{
    float roll;   /**< 绕 X 轴转角，单位 deg */
    float pitch;  /**< 绕 Y 轴转角，单位 deg */
    float yaw;    /**< 绕 Z 轴转角，单位 deg */
} euler_t;

/**
 * @brief IMU 输入数据结构
 */
typedef struct
{
    vec3_t gyro;  /**< 陀螺仪角速度，单位 deg/s */
    vec3_t acc;   /**< 加速度计值，单位 g */
    float temp;   /**< 温度，单位 ℃ */
    float dt;     /**< 采样周期，单位 s */

    float Kp;     /**< 比例增益，占位保留 */
    float Ki;     /**< 积分增益，占位保留 */
} imu_data_t;

/**
 * @brief Mahony 调试观测量
 */
typedef struct
{
    float acc_norm_raw;     /**< 本周期原始加速度模长 */
    float acc_weight;       /**< 本周期加速度纠偏参与权重 */
    float dt_used;          /**< 本周期实际使用的 dt */
    uint8_t gyro_only_mode; /**< 1 表示仅陀螺积分，0 表示融合加速度 */
} mahony_debug_t;

/**
 * @brief Mahony 四元数姿态滤波器对象
 *
 * @details
 * 该类真正持有 Mahony 算法跨周期积分项状态，因此是本文件中最适合类化的部分。
 * 纯数据结构 `quat_t` / `vec3_t` / `euler_t` 仍保持为简单 POD 结构，避免过度设计。
 */
class Class_Quaternion
{
public:
    /**
     * @brief 初始化滤波器内部积分状态
     */
    void Init();

    /**
     * @brief 四元数转欧拉角
     * @param q 输入四元数
     * @return 角度制欧拉角
     */
    euler_t QuatToEuler(quat_t q);

    /**
     * @brief 执行一次 Mahony 姿态更新
     * @param q 当前四元数，输入输出参数
     * @param imu IMU 输入数据
     * @param kp 比例增益
     * @param ki 积分增益
     */
    void MahonyUpdate(quat_t *q, imu_data_t imu, float kp, float ki);

private:
    float integralFBx; /**< X 轴积分反馈项 */
    float integralFBy; /**< Y 轴积分反馈项 */
    float integralFBz; /**< Z 轴积分反馈项 */

    /**
     * @brief 计算倒平方根
     * @param x 输入值
     * @return 倒平方根结果；当输入非法时返回 0
     */
    static float InvSqrt(float x);

    /**
     * @brief 对 dt 做合法化处理
     * @param dt_s 原始 dt
     * @return 修正后的 dt
     */
    static float SanitizeDt(float dt_s);

    /**
     * @brief 将四元数重置为单位四元数
     * @param q 四元数指针
     */
    static void ResetQuat(quat_t *q);

    /**
     * @brief 清零积分反馈项
     */
    void ResetIntegral();
};

/**
 * @brief 全局 Mahony 滤波器对象
 */
extern Class_Quaternion Quaternion_Filter;

/**
 * @brief 全局 Mahony 调试观测量
 */
extern volatile mahony_debug_t g_mahony_debug;

#endif /* __ALG_QUATERNION_H__ */
