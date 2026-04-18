/**
 * @file alg_pid.cpp
 * @brief PID 控制器实现
 */
#include "alg_pid.h"

/**
 * @brief 初始化 PID 对象
 */
void Class_PID::Init()
{
    Kp = 0.0f;
    Ki = 0.0f;
    Kd = 0.0f;
    FeedForward = 0.0f;

    P_out = 0.0f;
    I_out = 0.0f;
    D_out = 0.0f;
    FeedForward_out = 0.0f;

    target = 0.0f;
    prev_target = 0.0f;
    output = 0.0f;
    Input = 0.0f;

    error = 0.0f;
    prev_error = 0.0f;
    integral = 0.0f;

    out_min = 0.0f;
    out_max = 0.0f;

    integral_min = 0.0f;
    integral_max = 0.0f;

    target_limit_enable = false;
    target_limit_min = 0.0f;
    target_limit_max = 0.0f;

    integral_separation_enable = false;
    integral_separation_threshold_A = 0.0f;
    integral_separation_threshold_B = 0.0f;

    differential_enable = false;

    deadband_enable = false;
    deadband = 0.0f;

    dt = 0.0f;
}

/**
 * @brief 配置微分先行开关
 * @param enable `true` 表示启用
 */
void Class_PID::DifferentialEnable(bool enable)
{
    differential_enable = enable;
}

/**
 * @brief 配置死区功能
 * @param enable `true` 表示启用
 * @param deadband_value 死区阈值
 */
void Class_PID::DeadbandEnable(bool enable, float deadband_value)
{
    deadband_enable = enable;
    deadband = deadband_value;
}

/**
 * @brief 配置变速积分
 * @param enable `true` 表示启用
 * @param threshold_A 强抑制阈值
 * @param threshold_B 弱抑制阈值
 */
void Class_PID::IntegralSeparationEnable(bool enable, float threshold_A, float threshold_B)
{
    integral_separation_enable = enable;
    integral_separation_threshold_A = threshold_A;
    integral_separation_threshold_B = threshold_B;
}

/**
 * @brief 配置目标值限幅
 * @param enable `true` 表示启用
 * @param min 目标下限
 * @param max 目标上限
 */
void Class_PID::TargetLimitEnable(bool enable, float min, float max)
{
    target_limit_enable = enable;
    target_limit_min = min;
    target_limit_max = max;
}

/**
 * @brief 设置 PID 参数与限幅参数
 * @param P 比例系数
 * @param I 积分系数
 * @param D 微分系数
 * @param feed_forward 前馈系数
 * @param integral_min_value 积分项下限
 * @param integral_max_value 积分项上限
 * @param out_min_value 输出下限
 * @param out_max_value 输出上限
 */
void Class_PID::SetParameters(float P, float I, float D, float feed_forward,
                              float integral_min_value, float integral_max_value,
                              float out_min_value, float out_max_value)
{
    Kp = P;
    Ki = I;
    Kd = D;
    FeedForward = feed_forward;
    integral_min = integral_min_value;
    integral_max = integral_max_value;
    out_min = out_min_value;
    out_max = out_max_value;
}

/**
 * @brief 执行一次 PID 控制计算
 * @param input_value 当前输入值
 * @param target_value 当前目标值
 * @param dt_value 控制周期，单位秒
 * @return 本次计算得到的控制输出
 *
 * @details
 * 算法流程如下：
 * - 对 dt 做最小值保护，避免除零
 * - 可选地对目标值做限幅
 * - 计算误差并进行死区判断
 * - 计算比例项
 * - 按误差大小执行变速积分并做积分限幅
 * - 计算微分项，可选择误差微分或微分先行
 * - 计算前馈项
 * - 汇总总输出并执行输出限幅
 */
float Class_PID::Calculate(float input_value, float target_value, float dt_value)
{
    const float dt_min = 1e-6f;
    float integral_coef = 1.0f;

    if (dt_value < dt_min)
    {
        dt_value = dt_min;
    }

    dt = dt_value;
    Input = input_value;
    target = target_value;

    if (target_limit_enable)
    {
        if (target < target_limit_min)
        {
            target = target_limit_min;
        }
        else if (target > target_limit_max)
        {
            target = target_limit_max;
        }
    }

    error = target - Input;

    if (deadband_enable && fabsf(error) <= deadband)
    {
        output = 0.0f;
        P_out = 0.0f;
        I_out = 0.0f;
        D_out = 0.0f;
        FeedForward_out = 0.0f;
        prev_error = error;
        prev_target = target;
        return 0.0f;
    }

    P_out = Kp * error;

    if (integral_separation_enable)
    {
        float A = integral_separation_threshold_A;
        float B = integral_separation_threshold_B;
        float abs_err = fabsf(error);

        if (A <= B)
        {
            A = B + 1e-6f;
        }

        if (abs_err >= A)
        {
            integral_coef = 0.0f;
        }
        else if (abs_err > B)
        {
            integral_coef = (A - abs_err) / (A - B);
        }
    }

    integral += error * dt * integral_coef;

    if (integral < integral_min)
    {
        integral = integral_min;
    }
    else if (integral > integral_max)
    {
        integral = integral_max;
    }

    I_out = Ki * integral;

    if (differential_enable)
    {
        D_out = Kd * (-(target - prev_target) / dt);
    }
    else
    {
        D_out = Kd * ((error - prev_error) / dt);
    }

    FeedForward_out = FeedForward * ((target - prev_target) / dt);
    output = P_out + I_out + D_out + FeedForward_out;

    if (output < out_min)
    {
        output = out_min;
    }
    else if (output > out_max)
    {
        output = out_max;
    }

    prev_error = error;
    prev_target = target;

    return output;
}
