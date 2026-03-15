#include "motor.h"
#include <stdint.h>

/**
 * @brief 初始化电机
 * @param motor 电机结构体指针
 * @param ID 电机ID
 * @param type 电机类型
 * @param can CAN句柄
 * @param method 控制方法
 */
void Motor_Init(Motor_TypeDef *motor, uint8_t ID, enum Motor_DJI_type type,
                CAN_HandleTypeDef *can, enum Motor_DJI_Control_Method method)
{
    // 初始化CAN
    CAN_Start(can);
    motor->can = can;
    motor->ID = ID;
    motor->type = type;
    motor->method = method;

    // 根据控制方法动态分配PID控制器
    switch (method) {
        case DJI_Control_Method_Speed:
            motor->PID_Use_Count = 1;
            // 初始限幅设为0，后续通过Motor_Set_PID_Params设置
            PID_Init(&motor->PID[0]);
            break;

        case DJI_Control_Method_Angle:
            motor->PID_Use_Count = 2;
            // 外环角度PID
            PID_Init(&motor->PID[0]);
            // 内环速度PID
            PID_Init(&motor->PID[1]);
            break;

        default:
            motor->PID_Use_Count = 0;
            break;
    }
}

/**
 * @brief 设置电机PID参数（系数和限幅）
 * @param motor 电机结构体指针
 * @param pid_index PID索引（0:外环/单级, 1:内环）
 * @param p 比例系数
 * @param i 积分系数
 * @param d 微分系数
 * @param feedforward 前馈系数
 * @param out_min 输出最小值（通常为负的电流限幅）
 * @param out_max 输出最大值
 * @param integral_min 积分最小值
 * @param integral_max 积分最大值
 */
void Motor_Set_PID_Params(Motor_TypeDef *motor, uint8_t pid_index,
                          float p, float i, float d, float feedforward,
                          float out_min, float out_max,
                          float integral_min, float integral_max)
{
   PID_Set_Parameters(&motor->PID[pid_index], p, i, d, feedforward, out_min, out_max, integral_min, integral_max);
}


/**
 * @brief 电机PID计算（自动选择单级或级联模式）
 * @param motor 电机结构体指针
 * @param target 目标值（角度或速度，取决于模式）
 * @param feedback_angle 当前角度反馈（用于角度模式的外环，速度模式下可传入任意值）
 * @param dt 时间间隔（秒）
 * @return 计算后的输出值（期望电流/转矩）
 */
float Motor_PID_Calculate(Motor_TypeDef *motor, float target, float feedback_angle, float dt)
{
    if (motor == NULL)
        return 0.0f;

    switch (motor->method) {
        case DJI_Control_Method_Speed:
            // 单级PID：直接速度控制（反馈为电机当前速度）
            return PID_Calculate(&motor->PID[0], (float)motor->RxData.Speed, target, dt);

        case DJI_Control_Method_Angle: {
            // 级联PID：外环角度PID -> 内环速度PID
            // 外环：目标角度 vs 当前角度，输出为目标速度
            float target_speed = PID_Calculate(&motor->PID[0], feedback_angle, target, dt);
            // 内环：目标速度 vs 当前速度，输出为最终电流
            return PID_Calculate(&motor->PID[1], (float)motor->RxData.Speed, target_speed, dt);
        }

        default:
            return 0.0f;
    }
}


/**
 * @brief 处理电机原始数据（字节解析）
 * @param motor 电机结构体指针
 * @param data 接收数据指针（8字节）
 */
static void Motor_GM6020_Data_Process(Motor_TypeDef *motor, uint8_t *data)
{
    int32_t delta_encoder;
    uint16_t Encoder_Angle = (uint16_t)data[0] << 8 | data[1];
    motor->RxData.Speed       = (int16_t)((uint16_t)data[2] << 8 | data[3]) * RPM_TO_RADS;
    motor->RxData.Torque      = (int16_t)((uint16_t)data[4] << 8 | data[5]);
    motor->RxData.Temperature = data[6];
    delta_encoder = Encoder_Angle - motor->RxData.Last_encoder_angle;
    if(delta_encoder > Encoder_Num_Per_Round / 2)
    {
        motor->RxData.Total_Round--;
    }
    else if(delta_encoder < -Encoder_Num_Per_Round / 2)
    {
        motor->RxData.Total_Round++;
    }
    motor->RxData.Last_encoder_angle = Encoder_Angle;
    motor->RxData.Total_Encode = motor->RxData.Total_Round * Encoder_Num_Per_Round + Encoder_Angle;
    motor->RxData.Angle = (float)motor->RxData.Total_Encode *360.f / (float)Encoder_Num_Per_Round ;
}

static void Motor_M3508_Data_Process(Motor_TypeDef *motor, uint8_t *data)
{
    int32_t delta_encoder;
    uint16_t Encoder_Angle = (uint16_t)data[0] << 8 | data[1];
    motor->RxData.Speed       = (int16_t)((uint16_t)data[2] << 8 | data[3]) * RPM_TO_RADS / M2508_Gearbox_Rate;
    motor->RxData.Torque      = (int16_t)((uint16_t)data[4] << 8 | data[5]);
    motor->RxData.Temperature = data[6];
    delta_encoder = Encoder_Angle - motor->RxData.Last_encoder_angle;
    if(delta_encoder > Encoder_Num_Per_Round / 2)
    {
        motor->RxData.Total_Round--;
    }
    else if(delta_encoder < -Encoder_Num_Per_Round / 2)
    {
        motor->RxData.Total_Round++;
    }
    motor->RxData.Last_encoder_angle = Encoder_Angle;
    motor->RxData.Total_Encode = motor->RxData.Total_Round * Encoder_Num_Per_Round + Encoder_Angle;
    motor->RxData.Angle = (float)motor->RxData.Total_Encode *360.f / (float)Encoder_Num_Per_Round / M2508_Gearbox_Rate;
}

/**
 * @brief 接收电机CAN数据（从队列中获取并匹配ID）
 * @param motor 电机结构体指针
 */
void Motor_CAN_Data_Receive(Motor_TypeDef *motor)
{
    CAN_RX_MESSAGE RxBuffer;
    int can_index = (motor->can->Instance == CAN1) ? 0 :
                    (motor->can->Instance == CAN2) ? 1 : -1;
    if (can_index < 0) return;

    // 从对应CAN的队列中接收一帧数据（非阻塞）
    if (xQueueReceive(xQueue_CAN[can_index], &RxBuffer, 0) != pdPASS)
        return;
    if(motor->type == M3508)
    {
        if(RxBuffer.rx_header.StdId == 0x200 + motor->ID)
        {
            Motor_M3508_Data_Process(motor, RxBuffer.rx_data);
        }
    }
    else if(motor->type == GM6020_Voltage||motor->type == GM6020_Current)
    {
        if(RxBuffer.rx_header.StdId == 0x204 + motor->ID)
        {
            Motor_GM6020_Data_Process(motor, RxBuffer.rx_data);
        }
    }
}
/**
 * @brief 发送电机CAN数据（根据电机类型和ID）
 * @param motor 电机结构体指针
 * @param data 要发送的数据（16位）
 */
void Motor_Send_CAN_Data(Motor_TypeDef *motor, int16_t data)
{
    uint8_t tx_data[8] = {0};
    uint32_t Send_id = 0;
    if(motor->type == M3508)
    {
        switch (motor->ID)
        {
        case 1:
            Send_id = 0x201;
            tx_data[0] = (uint8_t)(data >> 8);
            tx_data[1] = (uint8_t)data;
            break;  
        case 2:
            Send_id = 0x202;
            tx_data[2] = (uint8_t)(data >> 8);
            tx_data[3] = (uint8_t)data;
            break;  
        case 3:
            Send_id = 0x203;
            tx_data[4] = (uint8_t)(data >> 8);
            tx_data[5] = (uint8_t)data;
            break;  
        case 4:
            Send_id = 0x204;
            tx_data[6] = (uint8_t)(data >> 8);
            tx_data[7] = (uint8_t)data;
            break;  
        case 5:
            Send_id = 0x1FF;
            tx_data[0] = (uint8_t)(data >> 8);
            tx_data[1] = (uint8_t)data;
            break;  
        case 6:
            Send_id = 0x1FF;
            tx_data[2] = (uint8_t)(data >> 8);
            tx_data[3] = (uint8_t)data;
            break;  
        case 7:
            Send_id = 0x1FF;
            tx_data[4] = (uint8_t)(data >> 8);
            tx_data[5] = (uint8_t)data;
            break;  
        case 8:
            Send_id = 0x1FF;
            tx_data[6] = (uint8_t)(data >> 8);
            tx_data[7] = (uint8_t)data;
            break;
        }
        
    }
    else if(motor->type == GM6020_Voltage)
    {
        switch (motor->ID)
        {
        case 0:
            Send_id = 0x1FF;
            tx_data[0] = (uint8_t)(data >> 8);
            tx_data[1] = (uint8_t)data;
            break;  
        case 1:
            Send_id = 0x1FF;
            tx_data[2] = (uint8_t)(data >> 8);
            tx_data[3] = (uint8_t)data;
            break;  
        case 2:
            Send_id = 0x1FF;
            tx_data[4] = (uint8_t)(data >> 8);
            tx_data[5] = (uint8_t)data;
            break;  
        case 3:
            Send_id = 0x1FF;
            tx_data[6] = (uint8_t)(data >> 8);
            tx_data[7] = (uint8_t)data;
            break;  
        case 4:
            Send_id = 0x2FF;
            tx_data[0] = (uint8_t)(data >> 8);
            tx_data[1] = (uint8_t)data;
            break;  
        case 5:
            Send_id = 0x2FF;
            tx_data[2] = (uint8_t)(data >> 8);
            tx_data[3] = (uint8_t)data;
            break;  
        case 6:
            Send_id = 0x2FF;
            tx_data[4] = (uint8_t)(data >> 8);
            tx_data[5] = (uint8_t)data;
            break;  
        case 7:
            Send_id = 0x2FF;
            tx_data[6] = (uint8_t)(data >> 8);
            tx_data[7] = (uint8_t)data;
            break;
        }
    }
    else if(motor->type == GM6020_Current)
    {
        switch (motor->ID)
        {
        case 0:
            Send_id = 0x1FE;
            tx_data[0] = (uint8_t)(data >> 8);
            tx_data[1] = (uint8_t)data;
            break;  
        case 1:
            Send_id = 0x1FE;
            tx_data[2] = (uint8_t)(data >> 8);
            tx_data[3] = (uint8_t)data;
            break;  
        case 2:
            Send_id = 0x1FE;
            tx_data[4] = (uint8_t)(data >> 8);
            tx_data[5] = (uint8_t)data;
            break;  
        case 3:
            Send_id = 0x1FE;
            tx_data[6] = (uint8_t)(data >> 8);
            tx_data[7] = (uint8_t)data;
            break;  
        case 4:
            Send_id = 0x2FE;
            tx_data[0] = (uint8_t)(data >> 8);
            tx_data[1] = (uint8_t)data;
            break;  
        case 5:
            Send_id = 0x2FE;
            tx_data[2] = (uint8_t)(data >> 8);
            tx_data[3] = (uint8_t)data;
            break;  
        case 6:
            Send_id = 0x2FE;
            tx_data[4] = (uint8_t)(data >> 8);
            tx_data[5] = (uint8_t)data;
            break;  
        case 7:
            Send_id = 0x2FE;
            tx_data[6] = (uint8_t)(data >> 8);
            tx_data[7] = (uint8_t)data;
            break;
        }
    }
    CAN_Send(motor->can, Send_id, tx_data);
}
