/**
 * @file dvc_motor.h
 * @brief DJI 电机设备对象定义与对外接口
 *
 * @details
 * 本文件封装了当前工程中使用的 DJI 电机控制对象。
 * 当前版本保持原有文件名，同时把电机控制逻辑收进类作用域成员函数。
 * 支持的核心能力包括：
 * - M3508 / GM6020 反馈数据解算
 * - 速度环或角度串级环 PID 控制
 * - 多电机共享 CAN 发送缓存
 */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "drv_can.h"
#include "alg_pid.h"
#include "stm32f4xx_hal_can.h"
#include <stdint.h>

/**
 * @brief 电机编码器单圈计数值
 */
#define Encoder_Num_Per_Round 8192
/**
 * @brief M3508 减速比
 */
#define M3508_Gearbox_Rate   (268.0f / 17.0f)
/**
 * @brief RPM 转换为 rad/s 的系数
 */
#define RPM_TO_RADS 0.104719755f

/**
 * @brief 电机控制方式
 */
enum Motor_DJI_Control_Method
{
    DJI_Control_Method_Angle = 0, /**< 角度串级控制：外环角度，内环速度 */
    DJI_Control_Method_Speed = 1, /**< 速度控制：单级速度环 */
};

/**
 * @brief 电机类型
 */
enum Motor_DJI_type
{
    GM6020_Current, /**< GM6020 电流控制模式 */
    GM6020_Voltage, /**< GM6020 电压控制模式 */
    M3508,          /**< M3508 减速电机 */
};

/**
 * @brief 电机反馈数据结构
 *
 * @details
 * 该结构保存从电机反馈帧中解算出的当前状态，以及连续角度计算所需的编码器累积信息。
 */
typedef struct
{
    float Angle;                 /**< 连续角度，单位度 */
    float Speed;                 /**< 当前转速，单位 rad/s */
    int16_t Torque;              /**< 原始转矩或电流反馈值 */
    uint8_t Temperature;         /**< 电机温度 */

    int32_t Total_Encode;        /**< 累计编码器计数 */
    uint16_t Last_encoder_angle; /**< 上一次收到的单圈编码器值 */
    int32_t Total_Round;         /**< 累计圈数 */
    uint8_t Encoder_Initialized; /**< 编码器连续角度计算是否已完成首帧初始化 */
} Motor_DataTypeDef;

/**
 * @brief DJI 电机对象
 *
 * @details
 * 一个 `Class_Motor` 表示一台具体的 DJI 电机。
 * 该对象内部持有：
 * - 该电机使用的 PID 控制器
 * - 电机 ID、类型、控制方式
 * - 对应 CAN 句柄
 * - 最近一次反馈解析结果
 */
class Class_Motor
{
public:
    Class_PID PID[2];                 /**< 控制器数组，角度模式下使用两级 */
    uint8_t PID_Use_Count;            /**< 当前实际启用的 PID 级数 */
    uint8_t ID;                       /**< 电机编号，取值范围通常为 1~8 */
    Motor_DJI_type type;              /**< 电机类型 */
    CAN_HandleTypeDef *can;           /**< 对应 CAN 句柄 */
    Motor_DJI_Control_Method method;  /**< 控制方式 */
    Motor_DataTypeDef RxData;         /**< 最近一次反馈解析结果 */

    /**
     * @brief 初始化电机对象
     * @param id 电机 ID
     * @param motor_type 电机类型
     * @param can_handle 电机所在 CAN 总线句柄
     * @param control_method 控制方式
     */
    void Init(uint8_t id, Motor_DJI_type motor_type,
              CAN_HandleTypeDef *can_handle,
              Motor_DJI_Control_Method control_method);

    /**
     * @brief 设置指定 PID 级的参数
     * @param pid_index PID 序号
     * @param p 比例系数
     * @param i 积分系数
     * @param d 微分系数
     * @param feedforward 前馈系数
     * @param out_min 输出下限
     * @param out_max 输出上限
     * @param integral_min 积分项下限
     * @param integral_max 积分项上限
     */
    void SetPIDParams(uint8_t pid_index,
                      float p, float i, float d, float feedforward,
                      float out_min, float out_max,
                      float integral_min, float integral_max);

    /**
     * @brief 执行一次电机控制输出计算
     * @param target 目标值
     * @param feedback_angle 当前角度反馈
     * @param dt 控制周期，单位秒
     * @return 计算得到的控制输出
     *
     * @details
     * 在速度模式下直接使用一级速度 PID。
     * 在角度模式下先计算目标速度，再由速度环输出最终控制量。
     */
    float PIDCalculate(float target, float feedback_angle, float dt);

    /**
     * @brief 从 CAN 软件缓冲区读取并解析本电机反馈
     *
     * @details
     * 该函数会持续读取当前电机对应反馈 ID 的所有报文，直到缓冲区中没有匹配帧为止。
     */
    void CANDataReceive();

    /**
     * @brief 获取当前电机对应的发送帧 ID
     * @return 发送帧 ID；当对象状态无效时返回 0
     */
    uint32_t GetCANSendId();

    /**
     * @brief 仅更新共享 CAN 发送缓存中的本电机控制值
     * @param data 写入缓存的 16 位控制量
     */
    void UpdateCANCache(int16_t data);

    /**
     * @brief 更新共享发送缓存并立即发送对应帧
     * @param data 本电机控制量
     */
    void SendCANData(int16_t data);

    /**
     * @brief 对角度环目标值进行限幅
     * @param min 角度目标下限
     * @param max 角度目标上限
     */
    void AngleLimit(float min, float max);

private:
    /**
     * @brief 计算当前电机控制量在共享发送帧中的位置
     * @param send_id 输出发送帧 ID
     * @param byte_index 输出控制值所在的起始字节偏移
     * @return 1 表示成功，0 表示当前对象配置无效
     */
    uint8_t GetSendFrameInfo(uint32_t *send_id, uint8_t *byte_index);

    /**
     * @brief 解析 GM6020 反馈数据
     * @param data 指向 8 字节反馈帧数据的指针
     */
    void GM6020DataProcess(uint8_t *data);

    /**
     * @brief 解析 M3508 反馈数据
     * @param data 指向 8 字节反馈帧数据的指针
     */
    void M3508DataProcess(uint8_t *data);
};

#endif /* __MOTOR_H__ */
