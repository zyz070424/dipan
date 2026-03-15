#include "alg_pid.h"
/** 
 * @brief 初始化 PID 控制器
 * @param pid: PID 控制器指针
 * @param integral_min: 积分项限幅最小值
 * @param integral_max: 积分项限幅最大值
 * @param out_min: 输出限幅最小值
 * @param out_max: 输出最大值
 */
void PID_Init(PID_TypeDef *pid)
{
    // ===== 1. PID参数 =====
    pid->Kp = 0.0f;
    pid->Ki = 0.0f;
    pid->Kd = 0.0f;
    pid->FeedForward = 0.0f;

    pid->P_out = 0.0f;
    pid->I_out = 0.0f;
    pid->D_out = 0.0f;
    pid->FeedForward_out = 0.0f;

    // ===== 2. 运行状态 =====
    pid->target = 0.0f;
    pid->prev_target = 0.0f;
    pid->output = 0.0f;
    pid->Input = 0.0f;

    // ===== 3. 误差相关 =====
    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
     
    // ===== 4. 输出限幅 =====
    pid->out_min = 0.0f;
    pid->out_max = 0.0f;

    // ===== 5. 积分限幅（抗积分饱和）=====
    pid->integral_min = 0.0f; // 积分项最小值
    pid->integral_max = 0.0f; // 积分项最大值
    
    // ===== 6. 目标限幅 =====
    pid->target_limit_enable = false;
    pid->target_limit_min = 0.0f;
    pid->target_limit_max = 0.0f;

    // ===== 7. 变速积分 =====
    pid->integral_separation_enable = false;
    pid->integral_separation_threshold_A = 0.0f;
    pid->integral_separation_threshold_B = 0.0f;
    
    // ===== 8. 微分先行 =====
    pid->differential_enable = false;

    // ===== 9. 死区 =====
    pid->deadband_enable = false;
    pid->deadband= 0.0f;

     // ===== 10. 时间间隔 =====
    pid->dt = 0.0f;
}
/** 
 * @brief 使能 PID 目标限幅
 * @param pid: PID 控制器指针
 * @param enable: 是否使能目标限幅
 * @param min: 目标限幅最小值
 * @param max: 目标限幅最大值
 */
void PID_Target_Limit_Enable(PID_TypeDef *pid,bool enable,float min,float max)
{
    pid->target_limit_enable = enable;
    pid->target_limit_min = min;
    pid->target_limit_max = max;
}
/** 
 * @brief 使能 PID 积分分离
 * @param pid: PID 控制器指针
 * @param enable: 是否使能积分分离
 * @param threshold_A: 积分分离阈值A
 * @param threshold_B: 积分分离阈值B
 */ 
void PID_Integral_Separation_Enable(PID_TypeDef *pid,bool enable,float threshold_A,float threshold_B)
{
    pid->integral_separation_enable = enable;
    pid->integral_separation_threshold_A = threshold_A;
    pid->integral_separation_threshold_B = threshold_B;
}
/** 
 * @brief 使能 PID 微分先行
 * @param pid: PID 控制器指针
 * @param enable: 是否使能微分先行
 */ 
void PID_Differential_Enable(PID_TypeDef *pid,bool enable)
{
    pid->differential_enable = enable;
}
/** 
 * @brief 使能 PID 死区
 * @param pid: PID 控制器指针
 * @param enable: 是否使能死区
 * @param deadband: 死区值
 */     
void PID_Deadband_Enable(PID_TypeDef *pid,bool enable,float deadband)
{
    pid->deadband_enable = enable;
    pid->deadband = deadband;
}
/** 
 * @brief 设置 PID 参数
 * @param pid: PID 控制器指针
 * @param P: 比例系数
 * @param I: 积分系数
 * @param D: 微分系数
 * @param FeedForward: 前馈系数
 */
void PID_Set_Parameters(PID_TypeDef *pid,float P,float I,float D,float FeedForward,float integral_min,float integral_max,float out_min,float out_max)
{
    pid->Kp = P;
    pid->Ki = I;
    pid->Kd = D;
    pid->FeedForward = FeedForward;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    pid->out_min = out_min;
    pid->out_max = out_max;
}
/** 
 * @brief 计算 PID 输出
 * @param pid: PID 控制器指针
 * @param Input: 输入值
 * @param Target: 目标值
 * @param dt: 时间间隔
 * @retval PID 输出值
 */
int16_t PID_Calculate(PID_TypeDef *pid,float Input,float Target,float dt)
{
    // 时间间隔
    pid->dt = dt;
    
    pid->Input = Input;
    pid->target = Target;
    //目标限幅
    if(pid->target_limit_enable)
    {
        if(pid->target < pid->target_limit_min)
        {
            pid->target = pid->target_limit_min;
        }
        else if(pid->target > pid->target_limit_max)
        {
            pid->target = pid->target_limit_max;
        }
    }

    //计算误差
    pid->error = pid->target - pid->Input;
   

    // 应用死区
    if(pid->deadband_enable && fabs(pid->error) <= pid->deadband)
    {
        pid->output = 0.0f;
        pid->P_out = pid->I_out = pid->D_out = pid->FeedForward_out = 0.0f;
   
        pid->prev_error = pid->error;
        pid->prev_target = pid->target;
        return 0;
    }
    
    // 计算比例项
    pid->P_out = pid->Kp * pid->error;

    float integral_coef = 1.0f;
    if(pid->integral_separation_enable)
    {
        float A = pid->integral_separation_threshold_A;
        float B = pid->integral_separation_threshold_B;
        
        if(fabs(pid->error) >= A)
        {
                integral_coef = 0.0f;
        }
            
        else if(fabs(pid->error) > B)
        {
            integral_coef = (A - fabs(pid->error)) / (A - B);
        }
        else
        {
            // 其他情况 integral_coef = 1.0f
            integral_coef = 1.0f;
        }                 
    }
    
    //计算积分项
    pid->integral += pid->error * pid->dt;

    // 积分限幅
    if(pid->integral < pid->integral_min)
    {
        pid->integral = pid->integral_min;
    }
    else if(pid->integral > pid->integral_max)
    {
        pid->integral = pid->integral_max;
    }
    // 计算积分项
    pid->I_out = pid->Ki * pid->integral * integral_coef;
    


    //这里可以滤波，例如低通滤波
    // 计算微分项
    if(pid->differential_enable)
    {
        // 微分先行
            pid->D_out = pid->Kd * - (pid->target - pid->prev_target) / pid->dt;
    }
    else
    {
        // 普通微分
        pid->D_out = pid->Kd * (pid->error - pid->prev_error) / pid->dt;
    }
    //计算前馈项
    pid->FeedForward_out = pid->FeedForward * pid->target;

    // 计算输出
    pid->output = pid->P_out + pid->I_out + pid->D_out + pid->FeedForward_out;
        
    
    // 输出限幅
    if(pid->output < pid->out_min)
    {
            pid->output = pid->out_min;
    }
    else if(pid->output > pid->out_max)
    {
        pid->output = pid->out_max;
    }
    // 更新上一次误差
    pid->prev_error = pid->error;
    // 更新上一次目标值
    pid->prev_target = pid->target;

    return pid->output;
    
}
    
