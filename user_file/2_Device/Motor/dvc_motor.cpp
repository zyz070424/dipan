/**
 * @file dvc_motor.cpp
 * @brief DJI 电机对象实现
 */
#include "dvc_motor.h"
#include <stdint.h>
#include <string.h>

/**
 * @brief 单路 CAN 总线的共享发送缓存
 *
 * @details
 * DJI 电机通常以共享帧的形式发送多个电机的控制量。
 * 本结构体按发送 ID 维护一组 8 字节缓存，用于先更新局部控制值，再统一发送。
 */
typedef struct
{
    uint8_t frame_0x200[8];
    uint8_t frame_0x1FF[8];
    uint8_t frame_0x2FF[8];
    uint8_t frame_0x1FE[8];
    uint8_t frame_0x2FE[8];
} Motor_CAN_Tx_Cache_TypeDef;

/** @brief CAN1 的电机发送缓存 */
static Motor_CAN_Tx_Cache_TypeDef Motor_CAN1_Tx_Cache = {0};
/** @brief CAN2 的电机发送缓存 */
static Motor_CAN_Tx_Cache_TypeDef Motor_CAN2_Tx_Cache = {0};

/**
 * @brief 根据 CAN 句柄获取对应的软件管理对象
 * @param can CAN 句柄
 * @return 成功时返回管理对象指针，失败返回 NULL
 */
static Class_CAN_Manage_Object *Motor_Get_CAN_Manager(CAN_HandleTypeDef *can)
{
    if (can == NULL)
    {
        return NULL;
    }

    if (can->Instance == CAN1)
    {
        return &CAN1_Manage_Object;
    }

    if (can->Instance == CAN2)
    {
        return &CAN2_Manage_Object;
    }

    return NULL;
}

/**
 * @brief 根据 CAN 句柄获取对应发送缓存
 * @param can CAN 句柄
 * @return 成功时返回缓存指针，失败返回 NULL
 */
static Motor_CAN_Tx_Cache_TypeDef *Motor_Get_Tx_Cache(CAN_HandleTypeDef *can)
{
    if (can == NULL)
    {
        return NULL;
    }

    if (can->Instance == CAN1)
    {
        return &Motor_CAN1_Tx_Cache;
    }

    if (can->Instance == CAN2)
    {
        return &Motor_CAN2_Tx_Cache;
    }

    return NULL;
}

/**
 * @brief 根据发送帧 ID 获取对应的 8 字节缓存区
 * @param can CAN 句柄
 * @param send_id 发送帧 ID
 * @return 成功时返回缓存区首地址，失败返回 NULL
 */
static uint8_t *Motor_Get_Tx_Frame_Buffer(CAN_HandleTypeDef *can, uint32_t send_id)
{
    Motor_CAN_Tx_Cache_TypeDef *cache = Motor_Get_Tx_Cache(can);

    if (cache == NULL)
    {
        return NULL;
    }

    switch (send_id)
    {
        case 0x200:
            return cache->frame_0x200;
        case 0x1FF:
            return cache->frame_0x1FF;
        case 0x2FF:
            return cache->frame_0x2FF;
        case 0x1FE:
            return cache->frame_0x1FE;
        case 0x2FE:
            return cache->frame_0x2FE;
        default:
            return NULL;
    }
}

/**
 * @brief 仅更新共享缓存中的某台电机控制值
 * @param motor 电机对象指针
 * @param send_id 共享发送帧 ID
 * @param byte_index 当前电机控制量在帧中的起始偏移
 * @param data 当前电机控制值
 */
static void Motor_Update_Frame_Data(Class_Motor *motor, uint32_t send_id, uint8_t byte_index, int16_t data)
{
    uint8_t *tx_data;

    if (motor == NULL || motor->can == NULL)
    {
        return;
    }

    if (byte_index > 6u)
    {
        return;
    }

    tx_data = Motor_Get_Tx_Frame_Buffer(motor->can, send_id);
    if (tx_data == NULL)
    {
        return;
    }

    tx_data[byte_index] = (uint8_t)(data >> 8);
    tx_data[byte_index + 1] = (uint8_t)data;
}

/**
 * @brief 直接发送指定共享发送帧
 * @param can CAN 句柄
 * @param send_id 共享发送帧 ID
 */
static void Motor_Send_Frame_By_Id(CAN_HandleTypeDef *can, uint32_t send_id)
{
    uint8_t *tx_data;
    Class_CAN_Manage_Object *can_manage;

    if (can == NULL)
    {
        return;
    }

    tx_data = Motor_Get_Tx_Frame_Buffer(can, send_id);
    if (tx_data == NULL)
    {
        return;
    }

    can_manage = Motor_Get_CAN_Manager(can);
    if (can_manage == NULL)
    {
        return;
    }

    can_manage->Send(send_id, tx_data);
}

/**
 * @brief 更新共享缓存并立即发送对应帧
 * @param motor 电机对象指针
 * @param send_id 共享发送帧 ID
 * @param byte_index 当前电机控制量在帧中的起始偏移
 * @param data 当前电机控制值
 */
static void Motor_Update_Frame_And_Send(Class_Motor *motor, uint32_t send_id, uint8_t byte_index, int16_t data)
{
    if (motor == NULL || motor->can == NULL)
    {
        return;
    }

    Motor_Update_Frame_Data(motor, send_id, byte_index, data);
    Motor_Send_Frame_By_Id(motor->can, send_id);
}

/**
 * @brief 初始化电机对象
 * @param id 电机 ID
 * @param motor_type 电机类型
 * @param can_handle 所在 CAN 总线句柄
 * @param control_method 控制方式
 */
void Class_Motor::Init(uint8_t id, Motor_DJI_type motor_type,
                       CAN_HandleTypeDef *can_handle,
                       Motor_DJI_Control_Method control_method)
{
    can = can_handle;
    ID = id;
    type = motor_type;
    method = control_method;
    memset(&RxData, 0, sizeof(RxData));

    switch (method)
    {
        case DJI_Control_Method_Speed:
            PID_Use_Count = 1u;
            PID[0].Init();
            break;

        case DJI_Control_Method_Angle:
            PID_Use_Count = 2u;
            PID[0].Init();
            PID[1].Init();
            break;

        default:
            PID_Use_Count = 0u;
            break;
    }
}

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
void Class_Motor::SetPIDParams(uint8_t pid_index,
                               float p, float i, float d, float feedforward,
                               float out_min, float out_max,
                               float integral_min, float integral_max)
{
    if (pid_index >= 2u)
    {
        return;
    }

    PID[pid_index].SetParameters(p, i, d, feedforward,
                                 integral_min, integral_max,
                                 out_min, out_max);
}

/**
 * @brief 执行一次电机控制计算
 * @param target_value 目标值
 * @param feedback_angle 当前角度反馈
 * @param dt 控制周期
 * @return 控制输出
 */
float Class_Motor::PIDCalculate(float target_value, float feedback_angle, float dt)
{
    switch (method)
    {
        case DJI_Control_Method_Speed:
            return PID[0].Calculate((float)RxData.Speed, target_value, dt);

        case DJI_Control_Method_Angle:
        {
            float target_speed = PID[0].Calculate(feedback_angle, target_value, dt);
            return PID[1].Calculate((float)RxData.Speed, target_speed, dt);
        }

        default:
            return 0.0f;
    }
}

/**
 * @brief 计算当前电机控制值在共享发送帧中的位置
 * @param send_id 输出发送帧 ID
 * @param byte_index 输出字节偏移
 * @return 1 表示成功，0 表示失败
 */
uint8_t Class_Motor::GetSendFrameInfo(uint32_t *send_id, uint8_t *byte_index)
{
    if (can == NULL || send_id == NULL || byte_index == NULL)
    {
        return 0u;
    }

    if (ID < 1u || ID > 8u)
    {
        return 0u;
    }

    if (ID <= 4u)
    {
        *byte_index = (uint8_t)((ID - 1u) * 2u);

        switch (type)
        {
            case M3508:
                *send_id = 0x200;
                return 1u;
            case GM6020_Voltage:
                *send_id = 0x1FF;
                return 1u;
            case GM6020_Current:
                *send_id = 0x1FE;
                return 1u;
            default:
                return 0u;
        }
    }

    *byte_index = (uint8_t)((ID - 5u) * 2u);

    switch (type)
    {
        case M3508:
            *send_id = 0x1FF;
            return 1u;
        case GM6020_Voltage:
            *send_id = 0x2FF;
            return 1u;
        case GM6020_Current:
            *send_id = 0x2FE;
            return 1u;
        default:
            return 0u;
    }
}

/**
 * @brief 解析 GM6020 反馈帧
 * @param data 指向 8 字节反馈数据的指针
 *
 * @details
 * 该函数会维护编码器圈数和累计角度，从而把单圈编码器值转换为连续角度。
 */
void Class_Motor::GM6020DataProcess(uint8_t *data)
{
    int32_t delta_encoder;
    uint16_t encoder_angle = (uint16_t)data[0] << 8 | data[1];

    RxData.Speed = (int16_t)((uint16_t)data[2] << 8 | data[3]) * RPM_TO_RADS;
    RxData.Torque = (int16_t)((uint16_t)data[4] << 8 | data[5]);
    RxData.Temperature = data[6];

    if (RxData.Encoder_Initialized == 0u)
    {
        RxData.Last_encoder_angle = encoder_angle;
        RxData.Total_Round = 0;
        RxData.Total_Encode = encoder_angle;
        RxData.Angle = (float)RxData.Total_Encode * 360.0f / (float)Encoder_Num_Per_Round;
        RxData.Encoder_Initialized = 1u;
        return;
    }

    delta_encoder = encoder_angle - RxData.Last_encoder_angle;
    if (delta_encoder > Encoder_Num_Per_Round / 2)
    {
        RxData.Total_Round--;
    }
    else if (delta_encoder < -Encoder_Num_Per_Round / 2)
    {
        RxData.Total_Round++;
    }

    RxData.Last_encoder_angle = encoder_angle;
    RxData.Total_Encode = RxData.Total_Round * Encoder_Num_Per_Round + encoder_angle;
    RxData.Angle = (float)RxData.Total_Encode * 360.0f / (float)Encoder_Num_Per_Round;
}

/**
 * @brief 解析 M3508 反馈帧
 * @param data 指向 8 字节反馈数据的指针
 *
 * @details
 * 与 GM6020 类似，该函数会维护累计角度；不同点在于速度和角度都会额外除以减速比。
 */
void Class_Motor::M3508DataProcess(uint8_t *data)
{
    int32_t delta_encoder;
    uint16_t encoder_angle = (uint16_t)data[0] << 8 | data[1];

    RxData.Speed = (int16_t)((uint16_t)data[2] << 8 | data[3]) * RPM_TO_RADS / M3508_Gearbox_Rate;
    RxData.Torque = (int16_t)((uint16_t)data[4] << 8 | data[5]);
    RxData.Temperature = data[6];

    if (RxData.Encoder_Initialized == 0u)
    {
        RxData.Last_encoder_angle = encoder_angle;
        RxData.Total_Round = 0;
        RxData.Total_Encode = encoder_angle;
        RxData.Angle = (float)RxData.Total_Encode * 360.0f / (float)Encoder_Num_Per_Round / M3508_Gearbox_Rate;
        RxData.Encoder_Initialized = 1u;
        return;
    }

    delta_encoder = encoder_angle - RxData.Last_encoder_angle;
    if (delta_encoder > Encoder_Num_Per_Round / 2)
    {
        RxData.Total_Round--;
    }
    else if (delta_encoder < -Encoder_Num_Per_Round / 2)
    {
        RxData.Total_Round++;
    }

    RxData.Last_encoder_angle = encoder_angle;
    RxData.Total_Encode = RxData.Total_Round * Encoder_Num_Per_Round + encoder_angle;
    RxData.Angle = (float)RxData.Total_Encode * 360.0f / (float)Encoder_Num_Per_Round / M3508_Gearbox_Rate;
}

/**
 * @brief 从 CAN 软件缓冲区读取并解析当前电机反馈
 *
 * @details
 * 会根据电机类型计算反馈帧 ID，并持续读取所有匹配帧，直到缓存区中无匹配数据。
 */
void Class_Motor::CANDataReceive()
{
    CAN_RX_MESSAGE rx_buffer;
    uint32_t feedback_id;
    Class_CAN_Manage_Object *can_manage;

    if (can == NULL)
    {
        return;
    }

    can_manage = Motor_Get_CAN_Manager(can);
    if (can_manage == NULL)
    {
        return;
    }

    if (ID < 1u || ID > 8u)
    {
        return;
    }

    switch (type)
    {
        case M3508:
            feedback_id = 0x200 + ID;
            break;

        case GM6020_Voltage:
        case GM6020_Current:
            feedback_id = 0x204 + ID;
            break;

        default:
            return;
    }

    while (can_manage->ReadMessageByStdId(feedback_id, &rx_buffer) == HAL_OK)
    {
        switch (type)
        {
            case M3508:
                M3508DataProcess(rx_buffer.rx_data);
                break;

            case GM6020_Voltage:
            case GM6020_Current:
                GM6020DataProcess(rx_buffer.rx_data);
                break;

            default:
                return;
        }
    }
}

/**
 * @brief 获取当前电机对应的共享发送帧 ID
 * @return 发送帧 ID；失败返回 0
 */
uint32_t Class_Motor::GetCANSendId()
{
    uint32_t send_id;
    uint8_t byte_index;

    if (GetSendFrameInfo(&send_id, &byte_index) == 0u)
    {
        return 0u;
    }

    return send_id;
}

/**
 * @brief 仅更新共享发送缓存中的当前电机控制值
 * @param data 当前电机控制值
 */
void Class_Motor::UpdateCANCache(int16_t data)
{
    uint32_t send_id;
    uint8_t byte_index;

    if (GetSendFrameInfo(&send_id, &byte_index) == 0u)
    {
        return;
    }

    Motor_Update_Frame_Data(this, send_id, byte_index, data);
}

/**
 * @brief 更新共享缓存并立即发送
 * @param data 当前电机控制值
 */
void Class_Motor::SendCANData(int16_t data)
{
    uint32_t send_id;
    uint8_t byte_index;

    if (GetSendFrameInfo(&send_id, &byte_index) == 0u)
    {
        return;
    }

    Motor_Update_Frame_And_Send(this, send_id, byte_index, data);
}

/**
 * @brief 对角度环目标值进行限幅
 * @param min 目标下限
 * @param max 目标上限
 */
void Class_Motor::AngleLimit(float min, float max)
{
    PID[0].TargetLimitEnable(true, min, max);

    if (PID[0].target < min)
    {
        PID[0].target = min;
    }
    else if (PID[0].target > max)
    {
        PID[0].target = max;
    }
}
