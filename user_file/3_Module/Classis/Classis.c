#include "Classis.h"

#include "can.h"
#include "tim.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

// 全局底盘对象：4个电机 + 1份遥控数据
static DR16_DataTypeDef g_dr16_data;
static Motor_TypeDef g_chassis_motors[CHASSIS_WHEEL_COUNT];
static float g_auto_spin_speed_radps = CHASSIS_AUTO_SPIN_SPEED_RADPS;

// 轮子顺序固定为：前左、前右、后左、后右
static const int8_t g_wheel_dir[CHASSIS_WHEEL_COUNT] = {
    CHASSIS_DIR_FRONT_LEFT,
    CHASSIS_DIR_FRONT_RIGHT,
    CHASSIS_DIR_BACK_LEFT,
    CHASSIS_DIR_BACK_RIGHT,
};

// 电机输出限幅（M3508 常见电流命令范围）
#define CHASSIS_MOTOR_CMD_LIMIT 16384.0f

typedef enum
{
    CHASSIS_ROTATE_MODE_MANUAL = 0,
    CHASSIS_ROTATE_MODE_AUTO,
    CHASSIS_ROTATE_MODE_STOP,
} Chassis_Rotate_ModeTypeDef;
/**
 * @brief 限幅函数
 *
 * @param value 输入值
 * @param min_val 最小值
 * @param max_val 最大值
 * @return float 限幅后的结果
 */
static float Classis_Clamp(float value, float min_val, float max_val)
{
    if (value < min_val)
    {
        return min_val;
    }

    if (value > max_val)
    {
        return max_val;
    }

    return value;
}
/**
 * @brief 对遥控器开关输入进行处理，确保只返回有效状态
 *
 * @param sw 原始开关状态
 * @return uint8_t 处理后的开关状态（UP、MIDDLE、DOWN）
 */
static uint8_t Classis_SanitizeSwitch(uint8_t sw)
{
    if ((sw == DR16_SWITCH_UP) || (sw == DR16_SWITCH_MIDDLE) || (sw == DR16_SWITCH_DOWN))
    {
        return sw;
    }

    return DR16_SWITCH_MIDDLE;
}
/**
 * @brief 获取底盘旋转模式
 *
 * @param dr16 遥控器数据指针
 * @return Chassis_Rotate_ModeTypeDef 底盘旋转模式（手动、自动、停止）
 */
static Chassis_Rotate_ModeTypeDef Classis_GetRotateMode(const DR16_DataTypeDef *dr16)
{
    uint8_t left_sw;

    if (dr16 == NULL)
    {
        return CHASSIS_ROTATE_MODE_STOP;
    }

    left_sw = Classis_SanitizeSwitch(dr16->raw_s1);

    if (left_sw == DR16_SWITCH_UP)
    {
        return CHASSIS_ROTATE_MODE_MANUAL;
    }

    if (left_sw == DR16_SWITCH_DOWN)
    {
        return CHASSIS_ROTATE_MODE_AUTO;
    }

    return CHASSIS_ROTATE_MODE_STOP;
}
/**
 * @brief 获取底盘旋转角速度指令
 *
 * @param dr16 遥控器数据指针
 * @param max_angular_velocity 最大角速度（rad/s）
 * @return float 旋转角速度指令（rad/s）
 */
static float Classis_GetOmegaCommand(const DR16_DataTypeDef *dr16, float max_angular_velocity)
{
    float speed_limit;
    float adjust_input;

    if (dr16 == NULL)
    {
        return 0.0f;
    }

    switch (Classis_GetRotateMode(dr16))
    {
    case CHASSIS_ROTATE_MODE_MANUAL:
        return dr16->right_x * max_angular_velocity;

    case CHASSIS_ROTATE_MODE_AUTO:
        // 自动旋转模式：持续自旋，右摇杆X用于调节自旋速度
        adjust_input = dr16->right_x;
        if ((adjust_input > -CHASSIS_AUTO_SPIN_ADJUST_DEADZONE) &&
            (adjust_input < CHASSIS_AUTO_SPIN_ADJUST_DEADZONE))
        {
            adjust_input = 0.0f;
        }
        g_auto_spin_speed_radps += adjust_input * CHASSIS_AUTO_SPIN_ADJUST_RADPS_PER_S * CHASSIS_CTRL_DT_S;
        g_auto_spin_speed_radps = Classis_Clamp(g_auto_spin_speed_radps,
                                           CHASSIS_AUTO_SPIN_SPEED_MIN_RADPS,
                                           CHASSIS_AUTO_SPIN_SPEED_MAX_RADPS);

        speed_limit = Classis_Clamp(max_angular_velocity, 0.0f, CHASSIS_AUTO_SPIN_SPEED_MAX_RADPS);
        return Classis_Clamp(g_auto_spin_speed_radps, 0.0f, speed_limit);

    case CHASSIS_ROTATE_MODE_STOP:
    default:
        return 0.0f;
    }
}
/**
 * @brief 将浮点数转换为16位有符号整数，同时进行限幅
 *
 * @param value 输入浮点数
 * @return int16_t 限幅后的16位有符号整数
 */
static int16_t Classis_FloatToInt16_Sat(float value)
{
    if (value > 32767.0f)
    {
        return 32767;
    }

    if (value < -32768.0f)
    {
        return -32768;
    }

    return (int16_t)value;
}
/**
 * @brief 设置底盘电机的默认 PID 参数
 *
 * @param motor 电机指针
 */
static void Classis_Set_DefaultPID(Motor_TypeDef *motor)
{
    // 这是“可跑起来”的初始参数，不是最终参数。
    // 单位：目标/反馈都是轮子角速度(rad/s)，输出是电机命令。
    Motor_Set_PID_Params(motor,
                         0,
                         450.0f,                      // Kp
                         20.0f,                       // Ki
                         0.0f,                        // Kd
                         0.0f,                        // FeedForward
                         -CHASSIS_MOTOR_CMD_LIMIT,    // out_min
                         CHASSIS_MOTOR_CMD_LIMIT,     // out_max
                         -3000.0f,                    // integral_min
                         3000.0f);                    // integral_max
}
/**
 * @brief 应用轮子方向补偿
 *
 * @param target_speed 目标速度数组（4个元素）
 * @param target_speed_with_dir 带方向补偿的目标速度数组（4个元素）
 */
static void Classis_ApplyWheelDirection(const float *target_speed, float *target_speed_with_dir)
{
    uint8_t i;

    // 方向补偿放在 PID 前：这样调参和看日志都更直观。
    for (i = 0; i < CHASSIS_WHEEL_COUNT; i++)
    {
        target_speed_with_dir[i] = target_speed[i] * (float)g_wheel_dir[i];
    }
}
/**
 * @brief 初始化底盘
 *
 * @param params 初始化参数指针（未使用）
 */
void Classis_Init(void *params)
{
    (void)params;

    // 遥控接收初始化（USART1 + 双缓冲）
    DR16_Init(&huart1);

    // CAN 必须显式启动；新版 dvc_motor 不会在 Motor_Init 里帮你启动。
    CAN_Start(&hcan1);

    // 启动 TIM2 1ms 中断，用于 DR16 按键/开关边沿状态更新。
    HAL_TIM_Base_Start_IT(&htim2);

    // 电机 ID 对应关系：1前左、2前右、3后左、4后右
    Motor_Init(&g_chassis_motors[0], 1, M3508, &hcan1, DJI_Control_Method_Speed);
    Motor_Init(&g_chassis_motors[1], 2, M3508, &hcan1, DJI_Control_Method_Speed);
    Motor_Init(&g_chassis_motors[2], 3, M3508, &hcan1, DJI_Control_Method_Speed);
    Motor_Init(&g_chassis_motors[3], 4, M3508, &hcan1, DJI_Control_Method_Speed);

    Classis_Set_DefaultPID(&g_chassis_motors[0]);
    Classis_Set_DefaultPID(&g_chassis_motors[1]);
    Classis_Set_DefaultPID(&g_chassis_motors[2]);
    Classis_Set_DefaultPID(&g_chassis_motors[3]);
}
/**
 * @brief 底盘控制函数
 *
 * @param dr16 遥控器数据指针
 * @param target_speed 目标速度数组（4个元素）
 * @param max_speed 最大线速度（m/s）
 * @param max_angular_velocity 最大角速度（rad/s）
 */
void Classis_Control(const DR16_DataTypeDef *dr16,
                     float *target_speed,
                     float max_speed,
                     float max_angular_velocity)
{
    float vx;
    float vy;
    float omega;

    if ((dr16 == NULL) || (target_speed == NULL))
    {
        return;
    }

    // 摇杆归一化值(-1~1) -> 期望底盘速度
    vx = dr16->left_x * max_speed;
    vy = dr16->left_y * max_speed;
    omega = Classis_GetOmegaCommand(dr16, max_angular_velocity);

    // 全向轮运动学解算
    target_speed[0] = (-SQRT2_2 * vx + SQRT2_2 * vy + omega * ROBOT_RADIUS) / WHEEL_RADIUS;
    target_speed[1] = (-SQRT2_2 * vx - SQRT2_2 * vy + omega * ROBOT_RADIUS) / WHEEL_RADIUS;
    target_speed[2] = ( SQRT2_2 * vx - SQRT2_2 * vy + omega * ROBOT_RADIUS) / WHEEL_RADIUS;
    target_speed[3] = ( SQRT2_2 * vx + SQRT2_2 * vy + omega * ROBOT_RADIUS) / WHEEL_RADIUS;
}
/**
 * @brief 1ms 定时器回调函数，用于处理遥控输入
 *
 * @param params 未使用参数指针
 */
void Classis_Timer1msCallback(void)
{
    DR16_Timer1msCallback(&g_dr16_data);
}
/**
 * @brief 底盘主循环任务函数
 * @note  电机任务1ms的中断会不会有点吃紧，但是考虑到底盘控制循环为2ms，can通信会不会有问题，can通信的话最好只发一节数据帧
 * @param params 未使用参数指针
 */
void Classis_RunTask(void *params)
{
    float target_speed[CHASSIS_WHEEL_COUNT] = {0.0f};
    float target_speed_with_dir[CHASSIS_WHEEL_COUNT] = {0.0f};
    float output_cmd[CHASSIS_WHEEL_COUNT] = {0.0f};
    TickType_t last_wake_time;
    const TickType_t loop_ticks = pdMS_TO_TICKS(1);
    uint8_t i;

    (void)params;

    last_wake_time = xTaskGetTickCount();

    while (1)
    {
        // 1) 更新遥控输入（无新帧时会保持上次值，不阻塞）
        DR16_Process(&g_dr16_data);

        // 2) 读取四个电机反馈
        for (i = 0; i < CHASSIS_WHEEL_COUNT; i++)
        {
            Motor_CAN_Data_Receive(&g_chassis_motors[i]);
        }

        // 3) 根据 vx/vy/w 做全向轮解算
        Classis_Control(&g_dr16_data,
                        target_speed,
                        CHASSIS_MAX_LINEAR_SPEED_MPS,
                        CHASSIS_MAX_ANGULAR_SPEED_RADPS);

        // 4) 每轮方向补偿（解决电机安装镜像导致的正负号问题）
        Classis_ApplyWheelDirection(target_speed, target_speed_with_dir);

        // 5) 每个轮子做速度环 PID 并下发 CAN
        for (i = 0; i < CHASSIS_WHEEL_COUNT; i++)
        {
            output_cmd[i] = Motor_PID_Calculate(&g_chassis_motors[i],
                                                target_speed_with_dir[i],
                                                0.0f,
                                                CHASSIS_CTRL_DT_S);

            Motor_Send_CAN_Data(&g_chassis_motors[i],
                                Classis_FloatToInt16_Sat(output_cmd[i]));
        }

        // 6) 固定周期运行，保持控制频率稳定
        vTaskDelayUntil(&last_wake_time, loop_ticks);
    }
}
