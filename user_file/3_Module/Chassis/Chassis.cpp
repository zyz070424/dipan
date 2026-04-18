/**
 * @file Chassis.cpp
 * @brief 底盘模块对象实现
 *
 * @details
 * 本文件在保留现有底盘控制流程的前提下，将底盘自身状态收进
 * `Class_Chassis`。
 */
#include "Chassis.h"

#include "can.h"
#include "tim.h"
#include "usart.h"
#include "spi.h"
#include "drv_spi.h"
#include "drv_usb.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stdint.h>

/**
 * @brief 电机输出限幅值
 */
#define CHASSIS_MOTOR_CMD_LIMIT 10000.0f
#define CHASSIS_PID_TEST_WHEEL_INDEX 3u
#define CHASSIS_PID_TEST_PERIOD_TICK pdMS_TO_TICKS(1)
#define CHASSIS_DR16_TEST_PERIOD_TICK pdMS_TO_TICKS(1)
#define CHASSIS_BMI088_PERIOD_TICK pdMS_TO_TICKS(1)
#define CHASSIS_BMI088_RETRY_DELAY_TICK pdMS_TO_TICKS(100)
#define CHASSIS_DEG_TO_RAD 0.01745329252f
#define CHASSIS_BMI088_MAHONY_KP 0.5f
#define CHASSIS_BMI088_MAHONY_KI 0.001f

#if (CHASSIS_PID_TEST_WHEEL_INDEX >= CHASSIS_WHEEL_COUNT)
#error "CHASSIS_PID_TEST_WHEEL_INDEX must be 0~3"
#endif

Class_Chassis Chassis;

const int8_t Class_Chassis::kWheelDir[CHASSIS_WHEEL_COUNT] = {
    CHASSIS_DIR_FRONT_LEFT,
    CHASSIS_DIR_FRONT_RIGHT,
    CHASSIS_DIR_BACK_LEFT,
    CHASSIS_DIR_BACK_RIGHT,
};

/**
 * @brief 对输入值进行限幅
 * @param value 输入值
 * @param min_val 最小值
 * @param max_val 最大值
 * @return 限幅后的结果
 */
float Class_Chassis::Clamp(float value, float min_val, float max_val)
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
 * @brief 对拨码开关值进行合法化处理
 * @param sw 原始拨码值
 * @return 合法拨码值
 */
uint8_t Class_Chassis::SanitizeSwitch(uint8_t sw)
{
    if ((sw == DR16_SWITCH_UP) || (sw == DR16_SWITCH_MIDDLE) || (sw == DR16_SWITCH_DOWN))
    {
        return sw;
    }

    return DR16_SWITCH_MIDDLE;
}

/**
 * @brief 根据 DR16 状态判断当前底盘旋转模式
 * @param dr16 遥控器数据
 * @return 当前底盘旋转模式
 */
Chassis_Rotate_ModeTypeDef Class_Chassis::GetRotateMode(const DR16_DataTypeDef *dr16)
{
    uint8_t left_sw;

    if (dr16 == NULL)
    {
        return CHASSIS_ROTATE_MODE_STOP;
    }

    left_sw = SanitizeSwitch(dr16->raw_s1);

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
 * @brief 计算当前旋转角速度指令
 * @param dr16 遥控器数据
 * @param max_angular_velocity 最大角速度
 * @return 旋转角速度指令
 */
float Class_Chassis::GetOmegaCommand(const DR16_DataTypeDef *dr16, float max_angular_velocity)
{
    float speed_limit;
    float adjust_input;

    if (dr16 == NULL)
    {
        return 0.0f;
    }

    switch (GetRotateMode(dr16))
    {
    case CHASSIS_ROTATE_MODE_MANUAL:
        return -(dr16->right_x * max_angular_velocity);

    case CHASSIS_ROTATE_MODE_AUTO:
        adjust_input = dr16->right_x;
        if ((adjust_input > -CHASSIS_AUTO_SPIN_ADJUST_DEADZONE) &&
            (adjust_input < CHASSIS_AUTO_SPIN_ADJUST_DEADZONE))
        {
            adjust_input = 0.0f;
        }

        auto_spin_speed_radps_ += adjust_input * CHASSIS_AUTO_SPIN_ADJUST_RADPS_PER_S * CHASSIS_CTRL_DT_S;
        auto_spin_speed_radps_ = Clamp(auto_spin_speed_radps_,
                                       CHASSIS_AUTO_SPIN_SPEED_MIN_RADPS,
                                       CHASSIS_AUTO_SPIN_SPEED_MAX_RADPS);

        speed_limit = Clamp(max_angular_velocity, 0.0f, CHASSIS_AUTO_SPIN_SPEED_MAX_RADPS);
        return Clamp(auto_spin_speed_radps_, 0.0f, speed_limit);

    case CHASSIS_ROTATE_MODE_STOP:
    default:
        return 0.0f;
    }
}

/**
 * @brief 将浮点数转换为 int16_t
 * @param value 输入值
 * @return 限幅后的 int16_t
 */
int16_t Class_Chassis::FloatToInt16Sat(float value)
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
 * @brief 应用轮子方向补偿
 * @param target_speed 原始目标速度
 * @param target_speed_with_dir 带方向补偿后的目标速度
 */
void Class_Chassis::ApplyWheelDirection(const float *target_speed, float *target_speed_with_dir)
{
    uint8_t i;

    for (i = 0u; i < CHASSIS_WHEEL_COUNT; i++)
    {
        target_speed_with_dir[i] = target_speed[i] * (float)kWheelDir[i];
    }
}

/**
 * @brief CAN 离线保护
 */
void Class_Chassis::CANOfflineProtect(void)
{
    uint8_t i;

    for (i = 0u; i < CHASSIS_WHEEL_COUNT; i++)
    {
        Motor[i].PID[0].integral = 0.0f;
        Motor[i].PID[1].integral = 0.0f;
    }
}

/**
 * @brief CAN 在线恢复
 */
void Class_Chassis::CANOnlineProtect(void)
{
}

/**
 * @brief SPI 离线保护
 */
void Class_Chassis::SPIOfflineProtect(void)
{
}

/**
 * @brief SPI 在线恢复
 */
void Class_Chassis::SPIOnlineProtect(void)
{
}

/**
 * @brief USB 离线保护
 */
void Class_Chassis::USBOfflineProtect(void)
{
}

/**
 * @brief USB 在线恢复
 */
void Class_Chassis::USBOnlineProtect(void)
{
}

/**
 * @brief 初始化底盘模块
 * @param params 初始化参数，当前未使用
 */
void Class_Chassis::Init(void *params)
{
    (void)params;

    DR16_Manage_Object.Init(&huart3);
    CAN1_Manage_Object.Init(&hcan1);
    CAN1_Manage_Object.Start();
    HAL_TIM_Base_Start_IT(&htim2);

    Motor[0].Init(1, M3508, &hcan1, DJI_Control_Method_Speed);
    Motor[1].Init(2, M3508, &hcan1, DJI_Control_Method_Speed);
    Motor[2].Init(3, M3508, &hcan1, DJI_Control_Method_Speed);
    Motor[3].Init(4, M3508, &hcan1, DJI_Control_Method_Speed);

    Motor[0].SetPIDParams(0, 500, 100, 0, 0.0f,
                          -CHASSIS_MOTOR_CMD_LIMIT, CHASSIS_MOTOR_CMD_LIMIT,
                          -1000.0f, 1000.0f);
    Motor[1].SetPIDParams(0, 500, 100, 0, 0.0f,
                          -CHASSIS_MOTOR_CMD_LIMIT, CHASSIS_MOTOR_CMD_LIMIT,
                          -1000.0f, 1000.0f);
    Motor[2].SetPIDParams(0, 500, 100, 0, 0.0f,
                          -CHASSIS_MOTOR_CMD_LIMIT, CHASSIS_MOTOR_CMD_LIMIT,
                          -1000.0f, 1000.0f);
    Motor[3].SetPIDParams(0, 500, 100, 0, 0.0f,
                          -CHASSIS_MOTOR_CMD_LIMIT, CHASSIS_MOTOR_CMD_LIMIT,
                          -1000.0f, 1000.0f);
}

/**
 * @brief BMI088 周期读取任务
 * @param params 任务参数，当前未使用
 */
void Class_Chassis::BMI088Task(void *params)
{
    TickType_t last_wake_time;
    imu_data_t imu_data = {0};
    euler_t euler_deg = {0};

    (void)params;

    last_wake_time = xTaskGetTickCount();

    while (1)
    {
        if (bmi088_ready_ == 0u)
        {
            bmi088_init_status_ = BMI088_Manage_Object.Init(&hspi1);
            if (bmi088_init_status_ != HAL_OK)
            {
                vTaskDelay(CHASSIS_BMI088_RETRY_DELAY_TICK);
                continue;
            }

            taskENTER_CRITICAL();
            bmi088_imu_data_ = imu_data;
            bmi088_euler_deg_ = euler_deg;
            taskEXIT_CRITICAL();

            bmi088_ready_ = 1u;
            last_wake_time = xTaskGetTickCount();
        }

        BMI088_Manage_Object.ReadGyro(&imu_data);
        BMI088_Manage_Object.ReadAccel(&imu_data);
        BMI088_Manage_Object.ReadTemp(&imu_data);
        euler_deg = BMI088_Manage_Object.ComplementaryFilter(&imu_data,
                                                             CHASSIS_CTRL_DT_S,
                                                             CHASSIS_BMI088_MAHONY_KP,
                                                             CHASSIS_BMI088_MAHONY_KI);

        taskENTER_CRITICAL();
        bmi088_imu_data_ = imu_data;
        bmi088_euler_deg_ = euler_deg;
        taskEXIT_CRITICAL();

        vTaskDelayUntil(&last_wake_time, CHASSIS_BMI088_PERIOD_TICK);
    }
}

/**
 * @brief 单轮 PID 调试任务
 * @param params 任务参数，当前未使用
 */
void Class_Chassis::PIDTestTask(void *params)
{
    TickType_t last_wake_time;
    float output_cmd;
    uint16_t alive_check_div = 0u;
    uint8_t can_online;
    uint8_t i;
    static uint32_t cnt = 0u;

    (void)params;

    last_wake_time = xTaskGetTickCount();

    while (1)
    {
        if (++alive_check_div >= 100u)
        {
            alive_check_div = 0u;
            CAN1_Manage_Object.AliveCheck100ms();
        }

        can_online = CAN1_Manage_Object.AliveIsOnline();

        for (i = 0u; i < CHASSIS_WHEEL_COUNT; i++)
        {
            Motor[i].CANDataReceive();
        }

        if (cnt++ % 10000u == 0u)
        {
            PID_Test_Target_Radps = -PID_Test_Target_Radps;
        }

        output_cmd = Motor[CHASSIS_PID_TEST_WHEEL_INDEX].PIDCalculate(PID_Test_Target_Radps,
                                                                      0.0f,
                                                                      CHASSIS_CTRL_DT_S);
        output_cmd = Clamp(output_cmd, -CHASSIS_MOTOR_CMD_LIMIT, CHASSIS_MOTOR_CMD_LIMIT);

        for (i = 0u; i < CHASSIS_WHEEL_COUNT; i++)
        {
            if (i == CHASSIS_PID_TEST_WHEEL_INDEX)
            {
                Motor[i].UpdateCANCache(FloatToInt16Sat(output_cmd));
            }
            else
            {
                Motor[i].UpdateCANCache(0);
            }
        }

        if (can_online != 0u)
        {
            Motor[CHASSIS_PID_TEST_WHEEL_INDEX].SendCANData(FloatToInt16Sat(output_cmd));
        }
        else
        {
            Motor[CHASSIS_PID_TEST_WHEEL_INDEX].SendCANData(0);
        }

        vTaskDelayUntil(&last_wake_time, CHASSIS_PID_TEST_PERIOD_TICK);
    }
}

/**
 * @brief DR16 调试任务
 * @param params 任务参数，当前未使用
 */
void Class_Chassis::DR16TestTask(void *params)
{
    TickType_t last_wake_time;

    (void)params;

    last_wake_time = xTaskGetTickCount();

    while (1)
    {
        DR16_Manage_Object.Process(&DR16_Data);
        vTaskDelayUntil(&last_wake_time, CHASSIS_DR16_TEST_PERIOD_TICK);
    }
}

/**
 * @brief 获取 BMI088 IMU 数据
 * @param imu_data 输出结构体
 */
void Class_Chassis::GetBMI088ImuData(imu_data_t *imu_data)
{
    if (imu_data == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *imu_data = bmi088_imu_data_;
    taskEXIT_CRITICAL();
}

/**
 * @brief 获取 BMI088 欧拉角数据
 * @param euler_deg 输出结构体
 */
void Class_Chassis::GetBMI088EulerData(euler_t *euler_deg)
{
    if (euler_deg == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *euler_deg = bmi088_euler_deg_;
    taskEXIT_CRITICAL();
}

/**
 * @brief 查询 BMI088 是否已就绪
 * @return 1 表示已就绪，0 表示未就绪
 */
uint8_t Class_Chassis::IsBMI088Ready(void)
{
    return bmi088_ready_;
}

/**
 * @brief 获取 BMI088 初始化状态
 * @return 初始化状态码
 */
HAL_StatusTypeDef Class_Chassis::GetBMI088InitStatus(void)
{
    return bmi088_init_status_;
}

/**
 * @brief 根据遥控输入解算四轮目标速度
 * @param dr16 遥控器数据
 * @param yaw_deg 当前车体朝向，单位 deg
 * @param yaw_valid 当前 Yaw 是否有效
 * @param target_speed 输出的四轮目标速度数组
 * @param max_speed 最大线速度
 * @param max_angular_velocity 最大角速度
 */
void Class_Chassis::Control(const DR16_DataTypeDef *dr16,
                            float yaw_deg,
                            uint8_t yaw_valid,
                            float *target_speed,
                            float max_speed,
                            float max_angular_velocity)
{
    float vx_ref;
    float vy_ref;
    float vx_body;
    float vy_body;
    float omega;
    float theta_rad;
    Chassis_Rotate_ModeTypeDef rotate_mode;

    if ((dr16 == NULL) || (target_speed == NULL))
    {
        return;
    }

    vx_ref = dr16->left_x * max_speed;
    vy_ref = dr16->left_y * max_speed;
    vx_body = vx_ref;
    vy_body = vy_ref;
    rotate_mode = GetRotateMode(dr16);

    if ((rotate_mode == CHASSIS_ROTATE_MODE_AUTO) && (yaw_valid != 0u))
    {
        if ((prev_rotate_mode_ != CHASSIS_ROTATE_MODE_AUTO) || (spin_lock_valid_ == 0u))
        {
            spin_lock_yaw_deg_ = yaw_deg;
            spin_lock_valid_ = 1u;
        }

        theta_rad = (yaw_deg - spin_lock_yaw_deg_) * CHASSIS_DEG_TO_RAD;
        vx_body = cosf(theta_rad) * vx_ref + sinf(theta_rad) * vy_ref;
        vy_body = -sinf(theta_rad) * vx_ref + cosf(theta_rad) * vy_ref;
    }
    else if (rotate_mode != CHASSIS_ROTATE_MODE_AUTO)
    {
        spin_lock_valid_ = 0u;
    }

    prev_rotate_mode_ = rotate_mode;
    omega = GetOmegaCommand(dr16, max_angular_velocity);

    target_speed[0] = (-SQRT2_2 * vx_body + SQRT2_2 * vy_body + omega * ROBOT_RADIUS) / WHEEL_RADIUS;
    target_speed[1] = (-SQRT2_2 * vx_body - SQRT2_2 * vy_body + omega * ROBOT_RADIUS) / WHEEL_RADIUS;
    target_speed[2] = ( SQRT2_2 * vx_body - SQRT2_2 * vy_body + omega * ROBOT_RADIUS) / WHEEL_RADIUS;
    target_speed[3] = ( SQRT2_2 * vx_body + SQRT2_2 * vy_body + omega * ROBOT_RADIUS) / WHEEL_RADIUS;
}

/**
 * @brief 1ms 定时回调
 */
void Class_Chassis::Timer1msCallback(void)
{
    DR16_Manage_Object.Timer1msCallback(&DR16_Data);
}

/**
 * @brief 底盘主循环任务
 * @param params 任务参数，当前未使用
 */
void Class_Chassis::RunTask(void *params)
{
    float target_speed[CHASSIS_WHEEL_COUNT] = {0.0f};
    float target_speed_with_dir[CHASSIS_WHEEL_COUNT] = {0.0f};
    float output_cmd[CHASSIS_WHEEL_COUNT] = {0.0f};
    euler_t imu_euler = {0};
    TickType_t last_wake_time;
    const TickType_t loop_ticks = pdMS_TO_TICKS(1);
    uint8_t i;
    uint8_t can_online;
    uint8_t can_online_changed;
    uint8_t imu_ready;
    uint8_t spi_online_changed;
    uint8_t usb_online_changed;
    uint16_t alive_check_div = 0u;

    (void)params;

    last_wake_time = xTaskGetTickCount();

    while (1)
    {
        if (++alive_check_div >= 100u)
        {
            alive_check_div = 0u;
            CAN1_Manage_Object.AliveCheck100ms();
            SPI1_Manage_Object.AliveCheck100ms();
            USB_Manage_Object.AliveCheck100ms();
        }

        if (CAN1_Manage_Object.AliveTryConsumeChanged(&can_online_changed) != 0)
        {
            if (can_online_changed == 0u)
            {
                CANOfflineProtect();
            }
            else
            {
                CANOnlineProtect();
            }
        }

        if (SPI1_Manage_Object.AliveTryConsumeChanged(&spi_online_changed) != 0)
        {
            if (spi_online_changed == 0u)
            {
                SPIOfflineProtect();
            }
            else
            {
                SPIOnlineProtect();
            }
        }

        if (USB_Manage_Object.AliveTryConsumeChanged(&usb_online_changed) != 0)
        {
            if (usb_online_changed == 0u)
            {
                USBOfflineProtect();
            }
            else
            {
                USBOnlineProtect();
            }
        }

        can_online = CAN1_Manage_Object.AliveIsOnline();

        DR16_Manage_Object.Process(&DR16_Data);

        imu_ready = IsBMI088Ready();
        if (imu_ready != 0u)
        {
            GetBMI088EulerData(&imu_euler);
        }

        for (i = 0u; i < CHASSIS_WHEEL_COUNT; i++)
        {
            Motor[i].CANDataReceive();
        }

        Control(&DR16_Data,
                imu_euler.yaw,
                imu_ready,
                target_speed,
                CHASSIS_MAX_LINEAR_SPEED_MPS,
                CHASSIS_MAX_ANGULAR_SPEED_RADPS);

        ApplyWheelDirection(target_speed, target_speed_with_dir);

        for (i = 0u; i < CHASSIS_WHEEL_COUNT; i++)
        {
            output_cmd[i] = Motor[i].PIDCalculate(target_speed_with_dir[i],
                                                  0.0f,
                                                  CHASSIS_CTRL_DT_S);

            if (can_online != 0u)
            {
                Motor[i].SendCANData(FloatToInt16Sat(output_cmd[i]));
            }
            else
            {
                Motor[i].SendCANData(0);
            }
        }

        vTaskDelayUntil(&last_wake_time, loop_ticks);
    }
}

extern "C" {

/**
 * @brief 1ms 定时回调桥接入口
 */
void Chassis_Timer1msCallback(void)
{
    Chassis.Timer1msCallback();
}

}
