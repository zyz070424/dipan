/**
 * @file alg_pid.h
 * @brief PID 控制器数据结构与接口定义
 *
 * @details
 * 本文件提供 PID 控制器的类定义以及兼容旧工程调用方式的 C 风格桥接接口。
 * 当前实现支持：
 * - 基本的 P/I/D 控制
 * - 前馈项
 * - 输出限幅
 * - 积分限幅
 * - 目标限幅
 * - 变速积分
 * - 微分先行
 * - 死区处理
 */
#ifndef __ALG_PID_H__
#define __ALG_PID_H__

#include "main.h"
#include <stdbool.h>
#include <math.h>
#include <sys/types.h>

/**
 * @brief PID 控制器对象
 *
 * @details
 * 该对象保存一套 PID 控制器的全部配置参数和运行时状态。
 */
class Class_PID {
public:
    float Kp;          /**< 比例系数 */
    float Ki;          /**< 积分系数 */
    float Kd;          /**< 微分系数 */
    float FeedForward; /**< 前馈系数 */

    float P_out;           /**< 本次计算得到的比例输出 */
    float I_out;           /**< 本次计算得到的积分输出 */
    float D_out;           /**< 本次计算得到的微分输出 */
    float FeedForward_out; /**< 本次计算得到的前馈输出 */

    float target;      /**< 当前控制目标值 */
    float prev_target; /**< 上一次控制目标值 */
    float output;      /**< 当前总输出 */
    float Input;       /**< 当前输入值 */

    float error;      /**< 当前误差 */
    float prev_error; /**< 上一次误差 */
    float integral;   /**< 当前积分累计值 */

    float out_min; /**< 输出最小限幅 */
    float out_max; /**< 输出最大限幅 */

    float integral_min; /**< 积分项最小限幅 */
    float integral_max; /**< 积分项最大限幅 */

    bool target_limit_enable; /**< 是否启用目标值限幅 */
    float target_limit_min;   /**< 目标值下限 */
    float target_limit_max;   /**< 目标值上限 */

    bool integral_separation_enable;       /**< 是否启用变速积分 */
    float integral_separation_threshold_A; /**< 变速积分强抑制阈值 */
    float integral_separation_threshold_B; /**< 变速积分弱抑制阈值 */

    bool differential_enable; /**< 是否启用微分先行 */

    bool deadband_enable; /**< 是否启用死区 */
    float deadband;       /**< 死区阈值 */

    float dt; /**< 本次控制计算使用的时间间隔，单位秒 */

    /**
     * @brief 初始化 PID 对象
     *
     * @details
     * 该函数会清空全部参数与运行时状态，使对象恢复到未配置的初始状态。
     */
    void Init();

    /**
     * @brief 使能或关闭微分先行
     * @param enable `true` 表示启用，`false` 表示关闭
     */
    void DifferentialEnable(bool enable);

    /**
     * @brief 配置死区功能
     * @param enable `true` 表示启用死区
     * @param deadband_value 死区阈值
     */
    void DeadbandEnable(bool enable, float deadband_value);

    /**
     * @brief 配置变速积分功能
     * @param enable `true` 表示启用变速积分
     * @param threshold_A 误差大于等于该值时积分完全关闭
     * @param threshold_B 误差小于等于该值时积分完全开启
     */
    void IntegralSeparationEnable(bool enable, float threshold_A, float threshold_B);

    /**
     * @brief 配置目标值限幅
     * @param enable `true` 表示启用目标值限幅
     * @param min 目标值下限
     * @param max 目标值上限
     */
    void TargetLimitEnable(bool enable, float min, float max);

    /**
     * @brief 设置 PID 参数和限幅参数
     * @param P 比例系数
     * @param I 积分系数
     * @param D 微分系数
     * @param feed_forward 前馈系数
     * @param integral_min_value 积分项下限
     * @param integral_max_value 积分项上限
     * @param out_min_value 输出下限
     * @param out_max_value 输出上限
     */
    void SetParameters(float P, float I, float D, float feed_forward,
                       float integral_min_value, float integral_max_value,
                       float out_min_value, float out_max_value);

    /**
     * @brief 执行一次 PID 控制计算
     * @param input_value 当前输入值
     * @param target_value 当前目标值
     * @param dt_value 本次控制周期，单位秒
     * @return 当前时刻的控制输出
     *
     * @details
     * 该函数会按顺序执行目标值限幅、误差计算、死区判断、变速积分、
     * 微分计算、前馈计算以及总输出限幅。
     */
    float Calculate(float input_value, float target_value, float dt_value);
};

#endif /* __ALG_PID_H__ */
