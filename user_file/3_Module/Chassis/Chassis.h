/**
 * @file Chassis.h
 * @brief 底盘模块对象定义与对外接口
 *
 * @details
 * 本文件定义当前工程的 `Chassis` 底盘模块，并在 C++ 下引入
 * `Class_Chassis` 对象来收纳底盘自身状态。
 * 当前仅保留 `main.c` 所需的 1ms 定时回调桥接入口。
 */
#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "main.h"

#include <stdint.h>

/**
 * @brief 底盘轮子数量
 */
#define CHASSIS_WHEEL_COUNT 4

/**
 * @brief 45 度对应的 sin/cos 值
 */
#define SQRT2_2 0.707106781f
/**
 * @brief 轮组半径，单位 m
 */
#define WHEEL_RADIUS 0.0815f
/**
 * @brief 轮组到车体中心的等效半径，单位 m
 */
#define ROBOT_RADIUS 0.2125f

/**
 * @brief 各轮方向补偿系数
 */
#define CHASSIS_DIR_FRONT_LEFT  (1)
#define CHASSIS_DIR_FRONT_RIGHT (1)
#define CHASSIS_DIR_BACK_LEFT   (1)
#define CHASSIS_DIR_BACK_RIGHT  (1)

/**
 * @brief 控制循环周期，单位 s
 */
#define CHASSIS_CTRL_DT_S 0.001f
/**
 * @brief 最大线速度，单位 m/s
 */
#define CHASSIS_MAX_LINEAR_SPEED_MPS 3.0f
/**
 * @brief 最大角速度，单位 rad/s
 */
#define CHASSIS_MAX_ANGULAR_SPEED_RADPS 4.0f
/**
 * @brief 默认自动自旋角速度，单位 rad/s
 */
#define CHASSIS_AUTO_SPIN_SPEED_RADPS 6.0f
#define CHASSIS_AUTO_SPIN_SPEED_MIN_RADPS 0.5f
#define CHASSIS_AUTO_SPIN_SPEED_MAX_RADPS 6.0f
#define CHASSIS_AUTO_SPIN_ADJUST_RADPS_PER_S 3.0f
#define CHASSIS_AUTO_SPIN_ADJUST_DEADZONE 0.05f

#ifdef __cplusplus
#include "dvc_dr16.h"
#include "dvc_bmi088.h"
#include "dvc_motor.h"

/**
 * @brief 底盘旋转模式
 */
enum Chassis_Rotate_ModeTypeDef
{
    CHASSIS_ROTATE_MODE_MANUAL = 0, /**< 手动旋转 */
    CHASSIS_ROTATE_MODE_AUTO,       /**< 自动自旋 */
    CHASSIS_ROTATE_MODE_STOP,       /**< 停止旋转 */
};

/**
 * @brief 底盘模块对象
 *
 * @details
 * 当前版本的 `Class_Chassis` 负责管理：
 * - DR16 遥控输入数据
 * - 4 个底盘轮电机对象
 * - BMI088 姿态数据缓存
 * - 底盘控制流程与测试任务
 */
class Class_Chassis
{
public:
    DR16_DataTypeDef DR16_Data;                /**< 当前遥控数据 */
    Class_Motor Motor[CHASSIS_WHEEL_COUNT];    /**< 4 个底盘轮电机对象 */
    float PID_Test_Target_Radps = 0.0f;        /**< PID 单轮测试目标轮速 */

    /**
     * @brief 初始化底盘模块
     * @param params 初始化参数，当前未使用
     */
    void Init(void *params);

    /**
     * @brief 根据遥控输入解算四轮目标角速度
     * @param dr16 遥控器数据
     * @param yaw_deg 当前车体朝向，单位 deg
     * @param yaw_valid 当前 Yaw 是否有效
     * @param target_speed 输出的四轮目标速度数组
     * @param max_speed 最大线速度
     * @param max_angular_velocity 最大角速度
     */
    void Control(const DR16_DataTypeDef *dr16,
                 float yaw_deg,
                 uint8_t yaw_valid,
                 float *target_speed,
                 float max_speed,
                 float max_angular_velocity);

    /**
     * @brief 底盘主循环任务
     * @param params 任务参数，当前未使用
     */
    void RunTask(void *params);

    /**
     * @brief BMI088 周期读取任务
     * @param params 任务参数，当前未使用
     */
    void BMI088Task(void *params);

    /**
     * @brief 单轮 PID 调试任务
     * @param params 任务参数，当前未使用
     */
    void PIDTestTask(void *params);

    /**
     * @brief DR16 调试任务
     * @param params 任务参数，当前未使用
     */
    void DR16TestTask(void *params);

    /**
     * @brief 获取最近一次 BMI088 IMU 数据
     * @param imu_data 输出结构体
     */
    void GetBMI088ImuData(imu_data_t *imu_data);

    /**
     * @brief 获取最近一次 BMI088 欧拉角数据
     * @param euler_deg 输出结构体
     */
    void GetBMI088EulerData(euler_t *euler_deg);

    /**
     * @brief 查询 BMI088 是否已经初始化完成
     * @return 1 表示已就绪，0 表示未就绪
     */
    uint8_t IsBMI088Ready(void);

    /**
     * @brief 获取最近一次 BMI088 初始化结果
     * @return 初始化状态码
     */
    HAL_StatusTypeDef GetBMI088InitStatus(void);

    /**
     * @brief 1ms 定时回调，用于 DR16 边沿状态更新
     */
    void Timer1msCallback(void);

private:
    float auto_spin_speed_radps_ = CHASSIS_AUTO_SPIN_SPEED_RADPS;  /**< 自动自旋当前角速度 */
    Chassis_Rotate_ModeTypeDef prev_rotate_mode_ = CHASSIS_ROTATE_MODE_STOP; /**< 上一周期旋转模式 */
    float spin_lock_yaw_deg_ = 0.0f;                               /**< 自动自旋模式下锁定的参考车头角 */
    uint8_t spin_lock_valid_ = 0u;                                 /**< 车头锁定角是否有效 */

    imu_data_t bmi088_imu_data_ = {0};                             /**< 最近一次 IMU 数据 */
    euler_t bmi088_euler_deg_ = {0};                               /**< 最近一次欧拉角数据 */
    volatile uint8_t bmi088_ready_ = 0u;                           /**< BMI088 是否已就绪 */
    volatile HAL_StatusTypeDef bmi088_init_status_ = HAL_ERROR;    /**< BMI088 初始化状态 */

    static const int8_t kWheelDir[CHASSIS_WHEEL_COUNT];            /**< 轮向方向补偿数组 */

    /**
     * @brief 对输入值进行限幅
     * @param value 输入值
     * @param min_val 最小值
     * @param max_val 最大值
     * @return 限幅后的结果
     */
    static float Clamp(float value, float min_val, float max_val);

    /**
     * @brief 对拨码开关值进行合法化处理
     * @param sw 原始拨码值
     * @return 合法拨码值
     */
    static uint8_t SanitizeSwitch(uint8_t sw);

    /**
     * @brief 根据 DR16 状态判断当前旋转模式
     * @param dr16 遥控器数据
     * @return 当前底盘旋转模式
     */
    Chassis_Rotate_ModeTypeDef GetRotateMode(const DR16_DataTypeDef *dr16);

    /**
     * @brief 计算当前角速度指令
     * @param dr16 遥控器数据
     * @param max_angular_velocity 最大角速度
     * @return 旋转角速度指令
     */
    float GetOmegaCommand(const DR16_DataTypeDef *dr16, float max_angular_velocity);

    /**
     * @brief 将浮点数限幅后转换为 int16_t
     * @param value 输入浮点值
     * @return 限幅后的 int16_t
     */
    static int16_t FloatToInt16Sat(float value);

    /**
     * @brief 应用轮子方向补偿
     * @param target_speed 原始目标速度
     * @param target_speed_with_dir 带方向补偿后的目标速度
     */
    void ApplyWheelDirection(const float *target_speed, float *target_speed_with_dir);

    /**
     * @brief CAN 离线保护
     */
    void CANOfflineProtect(void);

    /**
     * @brief CAN 在线恢复
     */
    void CANOnlineProtect(void);

    /**
     * @brief SPI 离线保护
     */
    void SPIOfflineProtect(void);

    /**
     * @brief SPI 在线恢复
     */
    void SPIOnlineProtect(void);

    /**
     * @brief USB 离线保护
     */
    void USBOfflineProtect(void);

    /**
     * @brief USB 在线恢复
     */
    void USBOnlineProtect(void);
};

/**
 * @brief 全局底盘对象
 */
extern Class_Chassis Chassis;
#endif

/**
 * @brief 1ms 定时回调桥接入口
 *
 * @details
 * 该入口仍保留给 `main.c` 使用，用于在 TIM2 1ms 节拍中转入当前底盘对象。
 */
#ifdef __cplusplus
extern "C" {
#endif
void Chassis_Timer1msCallback(void);
#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_H__ */
