#include "Classis.h"

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

// 全局底盘对象：4个电机 + 1份遥控数据
DR16_DataTypeDef g_dr16_data;
Motor_TypeDef g_chassis_motors[CHASSIS_WHEEL_COUNT];
static float g_auto_spin_speed_radps = CHASSIS_AUTO_SPIN_SPEED_RADPS;

static void Classis_CAN_Offline_Protect(void)
{
    uint8_t i;

    for (i = 0; i < CHASSIS_WHEEL_COUNT; i++)
    {
        g_chassis_motors[i].PID[0].integral = 0.0f;
        g_chassis_motors[i].PID[1].integral = 0.0f;
    }
}

static void Classis_CAN_Online_Protect(void)
{
}

static void Classis_SPI_Offline_Protect(void)
{
}

static void Classis_SPI_Online_Protect(void)
{
}

static void Classis_USB_Offline_Protect(void)
{
}

static void Classis_USB_Online_Protect(void)
{
}

// 轮子顺序固定为：前左、前右、后左、后右
/**
 * @brief 底盘电机方向数组
 *
 * 定义了底盘4个电机的旋转方向，用于计算电机输出。
 * 顺序为：前左、前右、后左、后右。
 */
static const int8_t g_wheel_dir[CHASSIS_WHEEL_COUNT] = {
    CHASSIS_DIR_FRONT_LEFT,
    CHASSIS_DIR_FRONT_RIGHT,
    CHASSIS_DIR_BACK_LEFT,
    CHASSIS_DIR_BACK_RIGHT,
};

// 电机输出限幅（M3508 常见电流命令范围）
/**
 * @brief 底盘电机输出限幅值
 *
 * 定义了底盘4个电机的输出电流限幅，用于防止电机过载。
 * 限幅范围为 [-10000.0, 10000.0]。
 */
#define CHASSIS_MOTOR_CMD_LIMIT 10000.0f
#define CHASSIS_PID_TEST_WHEEL_INDEX 3u      // 0=1号轮, 1=2号轮, 2=3号轮, 3=4号轮
#define CHASSIS_PID_TEST_PERIOD_TICK  pdMS_TO_TICKS(1)
#define CHASSIS_DR16_TEST_PERIOD_TICK pdMS_TO_TICKS(1)
#define CHASSIS_BMI088_PERIOD_TICK    pdMS_TO_TICKS(1)
#define CHASSIS_BMI088_RETRY_DELAY_TICK pdMS_TO_TICKS(100)
#define CHASSIS_DEG_TO_RAD            0.01745329252f
#define CHASSIS_BMI088_MAHONY_KP      0.5f
#define CHASSIS_BMI088_MAHONY_KI      0.001f

#if (CHASSIS_PID_TEST_WHEEL_INDEX >= CHASSIS_WHEEL_COUNT)
#error "CHASSIS_PID_TEST_WHEEL_INDEX must be 0~3"
#endif

float CHASSIS_PID_TEST_TARGET_RADPS = 0;   // 目标轮速(rad/s)
typedef enum
{
    CHASSIS_ROTATE_MODE_MANUAL = 0,
    CHASSIS_ROTATE_MODE_AUTO,
    CHASSIS_ROTATE_MODE_STOP,
} Chassis_Rotate_ModeTypeDef;

Chassis_Rotate_ModeTypeDef g_prev_rotate_mode = CHASSIS_ROTATE_MODE_STOP;
float g_spin_lock_yaw_deg = 0.0f;
uint8_t g_spin_lock_valid = 0u;
imu_data_t g_bmi088_imu_data = {0};
euler_t g_bmi088_euler_deg = {0};
volatile uint8_t g_bmi088_ready = 0u;
volatile HAL_StatusTypeDef g_bmi088_init_status = HAL_ERROR;
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
        return -(dr16->right_x * max_angular_velocity);

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

    // 遥控接收初始化（USART3 + 双缓冲）
    DR16_Init(&huart3);

    // CAN 必须显式启动；新版 dvc_motor 不会在 Motor_Init 里帮你启动。
    CAN_Start(&hcan1);

    // 启动 TIM2 1ms 中断，用于 DR16 按键/开关边沿状态更新。
    HAL_TIM_Base_Start_IT(&htim2);

    // 电机 ID 对应关系：1前左、2前右、3后左、4后右
    Motor_Init(&g_chassis_motors[0], 1, M3508, &hcan1, DJI_Control_Method_Speed);
    Motor_Init(&g_chassis_motors[1], 2, M3508, &hcan1, DJI_Control_Method_Speed);
    Motor_Init(&g_chassis_motors[2], 3, M3508, &hcan1, DJI_Control_Method_Speed);
    Motor_Init(&g_chassis_motors[3], 4, M3508, &hcan1, DJI_Control_Method_Speed);

   Motor_Set_PID_Params(&g_chassis_motors[0], 0, 500, 100, 0, 0.0f, -CHASSIS_MOTOR_CMD_LIMIT, CHASSIS_MOTOR_CMD_LIMIT, -1000.0f, 1000.0f);
   Motor_Set_PID_Params(&g_chassis_motors[1], 0, 500, 100, 0, 0.0f, -CHASSIS_MOTOR_CMD_LIMIT, CHASSIS_MOTOR_CMD_LIMIT, -1000.0f, 1000.0f);
    Motor_Set_PID_Params(&g_chassis_motors[2], 0, 500, 100, 0, 0.0f, -CHASSIS_MOTOR_CMD_LIMIT, CHASSIS_MOTOR_CMD_LIMIT, -1000.0f, 1000.0f);
    Motor_Set_PID_Params(&g_chassis_motors[3], 0, 500, 100, 0, 0.0f, -CHASSIS_MOTOR_CMD_LIMIT, CHASSIS_MOTOR_CMD_LIMIT, -1000.0f, 1000.0f);
}
/**
 * @brief BMI088 任务
 *
 * @param params 任务参数指针（未使用）
 */
void Classis_BMI088_Task(void *params)
{
    TickType_t last_wake_time;
    imu_data_t imu_data = {0};
    euler_t euler_deg = {0};

    (void)params;

    last_wake_time = xTaskGetTickCount();

    while (1)
    {
        if (g_bmi088_ready == 0u)
        {
            g_bmi088_init_status = BMI088_Init(&hspi1);
            if (g_bmi088_init_status != HAL_OK)
            {
                vTaskDelay(CHASSIS_BMI088_RETRY_DELAY_TICK);
                continue;
            }

            taskENTER_CRITICAL();
            g_bmi088_imu_data = imu_data;
            g_bmi088_euler_deg = euler_deg;
            taskEXIT_CRITICAL();

            g_bmi088_ready = 1u;
            last_wake_time = xTaskGetTickCount();
        }

        BMI088_ReadGyro(&hspi1, &imu_data);
        BMI088_ReadAccel(&hspi1, &imu_data);
        BMI088_ReadTemp(&hspi1, &imu_data);
        euler_deg = BMI088_Complementary_Filter(&imu_data,
                                                CHASSIS_CTRL_DT_S,
                                                CHASSIS_BMI088_MAHONY_KP,
                                                CHASSIS_BMI088_MAHONY_KI);

        taskENTER_CRITICAL();
        g_bmi088_imu_data = imu_data;
        g_bmi088_euler_deg = euler_deg;
        taskEXIT_CRITICAL();

        vTaskDelayUntil(&last_wake_time, CHASSIS_BMI088_PERIOD_TICK);
    }
}

/**
 * @brief 最简单的单轮 PID 调参任务
 *
 * 只让 CHASSIS_PID_TEST_WHEEL_INDEX 指定的轮子跑速度环，
 * 其他三个轮子固定发0，方便单独观察和调 PID 参数。
 */
void Classis_PID_TestTask(void *params)
{
    TickType_t last_wake_time;
    float output_cmd;
    uint32_t send_id;
    uint16_t alive_check_div = 0;
    uint8_t can_online;
    uint8_t i;
    static uint32_t cnt = 0;
    (void)params;

    last_wake_time = xTaskGetTickCount();
    send_id = Motor_Get_CAN_Send_Id(&g_chassis_motors[CHASSIS_PID_TEST_WHEEL_INDEX]);

    while (1)
    {
        if (++alive_check_div >= 100)
        {
            alive_check_div = 0;
            CAN_Alive_Check_100ms(&hcan1);
        }

        can_online = CAN_Alive_IsOnline(&hcan1);

        for (i = 0; i < CHASSIS_WHEEL_COUNT; i++)
        {
            Motor_CAN_Data_Receive(&g_chassis_motors[i]);
        }
        if(cnt++ % 10000 == 0)
        {
            // 每10000次循环（约10秒）打印一次当前速度，观察PID响应
            CHASSIS_PID_TEST_TARGET_RADPS = -CHASSIS_PID_TEST_TARGET_RADPS; // 每10秒切换一次正负目标速度，观察电机反转情况
        }
        output_cmd = Motor_PID_Calculate(&g_chassis_motors[CHASSIS_PID_TEST_WHEEL_INDEX],
                                         CHASSIS_PID_TEST_TARGET_RADPS,
                                         0.0f,
                                         CHASSIS_CTRL_DT_S);
        output_cmd = Classis_Clamp(output_cmd,
                                   -CHASSIS_MOTOR_CMD_LIMIT,
                                   CHASSIS_MOTOR_CMD_LIMIT);

        for (i = 0; i < CHASSIS_WHEEL_COUNT; i++)
        {
            if (i == CHASSIS_PID_TEST_WHEEL_INDEX)
            {
                Motor_Update_CAN_Cache(&g_chassis_motors[i],
                                       Classis_FloatToInt16_Sat(output_cmd));
            }
            else
            {
                Motor_Update_CAN_Cache(&g_chassis_motors[i], 0);
            }
        }

        if ((can_online != 0u) && (send_id != 0u))
        {
            Motor_Send_CAN_Frame_By_Id(&hcan1, send_id);
        }

        vTaskDelayUntil(&last_wake_time, CHASSIS_PID_TEST_PERIOD_TICK);
    }
}

/**
 * @brief 遥控器测试任务
 *
 * 只解析 DR16 数据并刷新到 g_dr16_data，方便在调试器 Watch 窗口观察。
 */
void Classis_DR16_TestTask(void *params)
{
    TickType_t last_wake_time;

    (void)params;

    last_wake_time = xTaskGetTickCount();

    while (1)
    {
        DR16_Process(&g_dr16_data);
        vTaskDelayUntil(&last_wake_time, CHASSIS_DR16_TEST_PERIOD_TICK);
    }
}

void Classis_GetBMI088ImuData(imu_data_t *imu_data)
{
    if (imu_data == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *imu_data = g_bmi088_imu_data;
    taskEXIT_CRITICAL();
}

void Classis_GetBMI088EulerData(euler_t *euler_deg)
{
    if (euler_deg == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *euler_deg = g_bmi088_euler_deg;
    taskEXIT_CRITICAL();
}

uint8_t Classis_IsBMI088Ready(void)
{
    return g_bmi088_ready;
}

HAL_StatusTypeDef Classis_GetBMI088InitStatus(void)
{
    return g_bmi088_init_status;
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

    // DR16 左杆: ch3=上下, ch2=左右。
    // 这里统一到底盘坐标: x=前进后退, y=左右平移。
    vx_ref = dr16->left_x * max_speed;
    vy_ref = dr16->left_y * max_speed;
    vx_body = vx_ref;
    vy_body = vy_ref;
    rotate_mode = Classis_GetRotateMode(dr16);

    if ((rotate_mode == CHASSIS_ROTATE_MODE_AUTO) && (yaw_valid != 0u))
    {
        if ((g_prev_rotate_mode != CHASSIS_ROTATE_MODE_AUTO) || (g_spin_lock_valid == 0u))
        {
            // 锁定切入小陀螺瞬间的车头朝向，后续平移始终相对这个方向保持不变。
            g_spin_lock_yaw_deg = yaw_deg;
            g_spin_lock_valid = 1u;
        }

        theta_rad = (yaw_deg - g_spin_lock_yaw_deg) * CHASSIS_DEG_TO_RAD;
        vx_body = cosf(theta_rad) * vx_ref + sinf(theta_rad) * vy_ref;
        vy_body = -sinf(theta_rad) * vx_ref + cosf(theta_rad) * vy_ref;
    }
    else if (rotate_mode != CHASSIS_ROTATE_MODE_AUTO)
    {
        g_spin_lock_valid = 0u;
    }

    g_prev_rotate_mode = rotate_mode;
    omega = Classis_GetOmegaCommand(dr16, max_angular_velocity);

    // 全向轮运动学解算
    target_speed[0] = (-SQRT2_2 * vx_body + SQRT2_2 * vy_body + omega * ROBOT_RADIUS) / WHEEL_RADIUS;
    target_speed[1] = (-SQRT2_2 * vx_body - SQRT2_2 * vy_body + omega * ROBOT_RADIUS) / WHEEL_RADIUS;
    target_speed[2] = ( SQRT2_2 * vx_body - SQRT2_2 * vy_body + omega * ROBOT_RADIUS) / WHEEL_RADIUS;
    target_speed[3] = ( SQRT2_2 * vx_body + SQRT2_2 * vy_body + omega * ROBOT_RADIUS) / WHEEL_RADIUS;
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
    euler_t imu_euler = {0};
    TickType_t last_wake_time;
    const TickType_t loop_ticks = pdMS_TO_TICKS(1);
    uint8_t i;
    uint8_t can_online;
    uint8_t can_online_changed;
    uint8_t imu_ready;
    uint8_t spi_online_changed;
    uint8_t usb_online_changed;
    uint16_t alive_check_div = 0;

    static void (* const can_link_action[2])(void) =
    {
        Classis_CAN_Offline_Protect,
        Classis_CAN_Online_Protect
    };
    static void (* const spi_link_action[2])(void) =
    {
        Classis_SPI_Offline_Protect,
        Classis_SPI_Online_Protect
    };
    static void (* const usb_link_action[2])(void) =
    {
        Classis_USB_Offline_Protect,
        Classis_USB_Online_Protect
    };

    (void)params;

    last_wake_time = xTaskGetTickCount();

    while (1)
    {
        if (++alive_check_div >= 100)
        {
            alive_check_div = 0;
            CAN_Alive_Check_100ms(&hcan1);
            SPI_Alive_Check_100ms();
            USB_Alive_Check_100ms();
        }

        if (CAN_Alive_TryConsumeChanged(&hcan1, &can_online_changed) != 0)
        {
            can_link_action[can_online_changed]();
        }

        if (SPI_Alive_TryConsumeChanged(&spi_online_changed) != 0)
        {
            spi_link_action[spi_online_changed]();
        }

        if (USB_Alive_TryConsumeChanged(&usb_online_changed) != 0)
        {
            usb_link_action[usb_online_changed]();
        }

        can_online = CAN_Alive_IsOnline(&hcan1);

        // 1) 更新遥控输入（无新帧时会保持上次值，不阻塞）
        DR16_Process(&g_dr16_data);

        imu_ready = Classis_IsBMI088Ready();
        if (imu_ready != 0u)
        {
            Classis_GetBMI088EulerData(&imu_euler);
        }

        // 2) 读取四个电机反馈
        for (i = 0; i < CHASSIS_WHEEL_COUNT; i++)
        {
            Motor_CAN_Data_Receive(&g_chassis_motors[i]);
        }

        // 3) 根据 vx/vy/w 做全向轮解算
        Classis_Control(&g_dr16_data,
                        imu_euler.yaw,
                        imu_ready,
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

            if (can_online != 0)
            {
                Motor_Send_CAN_Data(&g_chassis_motors[i],
                                    Classis_FloatToInt16_Sat(output_cmd[i]));
            }
            else 
            {
                Motor_Send_CAN_Data(&g_chassis_motors[i], 0);
            }
        }

        // 6) 固定周期运行，保持控制频率稳定
        vTaskDelayUntil(&last_wake_time, loop_ticks);
    }
}
