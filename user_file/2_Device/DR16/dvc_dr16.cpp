/**
 * @file dvc_dr16.cpp
 * @brief DR16 遥控器设备对象实现
 *
 * @details
 * 本文件将 DR16 自身持有的双缓冲接收状态和解析行为收进 `Class_DR16`。
 */
#include "dvc_dr16.h"

#include <string.h>

Class_DR16 DR16_Manage_Object;

static void DR16_Receive_Callback(uint8_t *data, uint16_t len);

/**
 * @brief 根据 UART 句柄获取对应的软件管理对象
 * @param huart UART 句柄
 * @return 成功时返回管理对象指针，失败返回 NULL
 */
static Class_UART_Manage_Object *DR16_Get_UART_Manager(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return NULL;
    }

    if (huart->Instance == USART1)
    {
        return &UART1_Manage_Object;
    }

    if (huart->Instance == USART3)
    {
        return &UART3_Manage_Object;
    }

    if (huart->Instance == UART5)
    {
        return &UART5_Manage_Object;
    }

    if (huart->Instance == USART6)
    {
        return &UART6_Manage_Object;
    }

    return NULL;
}

/**
 * @brief 对输入值进行限幅
 * @param val 输入值
 * @param min_val 最小值
 * @param max_val 最大值
 * @return 限幅后的结果
 */
float Class_DR16::Clamp(float val, float min_val, float max_val)
{
    if (val < min_val)
    {
        return min_val;
    }

    if (val > max_val)
    {
        return max_val;
    }

    return val;
}

/**
 * @brief 对拨码开关值进行合法化处理
 * @param sw 原始开关值
 * @return 合法的开关值
 */
uint8_t Class_DR16::SanitizeSwitch(uint8_t sw)
{
    if ((sw == DR16_SWITCH_UP) || (sw == DR16_SWITCH_MIDDLE) || (sw == DR16_SWITCH_DOWN))
    {
        return sw;
    }

    return DR16_SWITCH_MIDDLE;
}

/**
 * @brief 初始化 DR16 设备对象
 * @param uart_handle 绑定的 UART 句柄
 */
void Class_DR16::Init(UART_HandleTypeDef *uart_handle)
{
    Class_UART_Manage_Object *uart_manage;

    huart = uart_handle;
    ready_index = 0u;
    has_new_frame = 0u;
    memset(frame_buf, 0, sizeof(frame_buf));

    if (huart == NULL)
    {
        return;
    }

    uart_manage = DR16_Get_UART_Manager(huart);
    if (uart_manage == NULL)
    {
        return;
    }

    uart_manage->Init(huart, DR16_Receive_Callback);
}

/**
 * @brief 接收一帧 DR16 原始数据并写入双缓冲
 * @param data 接收到的原始字节流
 * @param len 本次接收字节数
 */
void Class_DR16::UART_RxCpltCallback(uint8_t *data, uint16_t len)
{
    uint8_t write_index;

    if ((data == NULL) || (len < DR16_FRAME_LEN))
    {
        return;
    }

    /* 写到非 ready 缓冲区，完成后再原子切换索引。 */
    write_index = (uint8_t)(ready_index ^ 1u);
    memcpy(frame_buf[write_index], data, DR16_FRAME_LEN);
    __DMB();
    ready_index = write_index;
    has_new_frame = 1u;
}

/**
 * @brief 从双缓冲中取出最新一帧数据
 * @param out_frame 输出帧缓冲区
 * @retval 1 成功取到新帧
 * @retval 0 当前没有新帧
 */
uint8_t Class_DR16::FetchLatestFrame(uint8_t *out_frame)
{
    uint32_t primask;
    uint8_t index;

    if (out_frame == NULL)
    {
        return 0u;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (has_new_frame == 0u)
    {
        if (primask == 0u)
        {
            __enable_irq();
        }
        return 0u;
    }

    index = ready_index;
    memcpy(out_frame, frame_buf[index], DR16_FRAME_LEN);
    has_new_frame = 0u;

    if (primask == 0u)
    {
        __enable_irq();
    }

    return 1u;
}

/**
 * @brief 判断拨码开关状态及触发沿
 * @param sw 输出拨码开关状态
 * @param now 当前原始值
 * @param prev 上一周期原始值
 */
void Class_DR16::JudgeSwitch(DR16_Switch_Status_TypeDef *sw, uint8_t now, uint8_t prev)
{
    now = SanitizeSwitch(now);
    prev = SanitizeSwitch(prev);

    switch (prev)
    {
    case DR16_SWITCH_UP:
        if (now == DR16_SWITCH_UP)
        {
            *sw = DR16_SWITCH_STATUS_UP;
        }
        else if (now == DR16_SWITCH_MIDDLE)
        {
            *sw = DR16_SWITCH_STATUS_TRIG_UP_MIDDLE;
        }
        else
        {
            *sw = DR16_SWITCH_STATUS_TRIG_MIDDLE_DOWN;
        }
        break;

    case DR16_SWITCH_DOWN:
        if (now == DR16_SWITCH_DOWN)
        {
            *sw = DR16_SWITCH_STATUS_DOWN;
        }
        else if (now == DR16_SWITCH_MIDDLE)
        {
            *sw = DR16_SWITCH_STATUS_TRIG_DOWN_MIDDLE;
        }
        else
        {
            *sw = DR16_SWITCH_STATUS_TRIG_MIDDLE_UP;
        }
        break;

    case DR16_SWITCH_MIDDLE:
    default:
        if (now == DR16_SWITCH_UP)
        {
            *sw = DR16_SWITCH_STATUS_TRIG_MIDDLE_UP;
        }
        else if (now == DR16_SWITCH_DOWN)
        {
            *sw = DR16_SWITCH_STATUS_TRIG_MIDDLE_DOWN;
        }
        else
        {
            *sw = DR16_SWITCH_STATUS_MIDDLE;
        }
        break;
    }
}

/**
 * @brief 判断按键状态及触发沿
 * @param key 输出按键状态
 * @param now 当前原始值
 * @param prev 上一周期原始值
 */
void Class_DR16::JudgeKey(DR16_Key_Status_TypeDef *key, uint8_t now, uint8_t prev)
{
    if (prev == DR16_KEY_FREE)
    {
        if (now == DR16_KEY_FREE)
        {
            *key = DR16_KEY_STATUS_FREE;
        }
        else
        {
            *key = DR16_KEY_STATUS_TRIG_FREE_PRESSED;
        }
    }
    else
    {
        if (now == DR16_KEY_FREE)
        {
            *key = DR16_KEY_STATUS_TRIG_PRESSED_FREE;
        }
        else
        {
            *key = DR16_KEY_STATUS_PRESSED;
        }
    }
}

/**
 * @brief 处理一帧最新 DR16 数据
 * @param dr16 输出的应用层数据结构
 *
 * @details
 * 无新帧时直接返回，不阻塞任务。
 */
void Class_DR16::Process(DR16_DataTypeDef *dr16)
{
    uint8_t raw[DR16_FRAME_LEN];
    uint16_t ch0;
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    float rocker_denom;

    if (dr16 == NULL)
    {
        return;
    }

    if (FetchLatestFrame(raw) == 0u)
    {
        return;
    }

    ch0 = ((uint16_t)raw[0] | ((uint16_t)(raw[1] & 0x07u) << 8)) & 0x07FFu;
    ch1 = (((uint16_t)raw[1] >> 3) | ((uint16_t)(raw[2] & 0x3Fu) << 5)) & 0x07FFu;
    ch2 = (((uint16_t)raw[2] >> 6) | ((uint16_t)raw[3] << 2) | ((uint16_t)(raw[4] & 0x01u) << 10)) & 0x07FFu;
    ch3 = (((uint16_t)raw[4] >> 1) | ((uint16_t)(raw[5] & 0x0Fu) << 7)) & 0x07FFu;

    /* DJI 常用定义：s1 高 2 位，s2 次高 2 位。 */
    dr16->raw_s1 = (raw[5] >> 6) & 0x03u;
    dr16->raw_s2 = (raw[5] >> 4) & 0x03u;

    mouse_x = (int16_t)((uint16_t)raw[6] | ((uint16_t)raw[7] << 8));
    mouse_y = (int16_t)((uint16_t)raw[8] | ((uint16_t)raw[9] << 8));
    mouse_z = (int16_t)((uint16_t)raw[10] | ((uint16_t)raw[11] << 8));

    dr16->raw_mouse_l = raw[12];
    dr16->raw_mouse_r = raw[13];
    dr16->raw_key = (uint16_t)raw[14] | ((uint16_t)raw[15] << 8);

    rocker_denom = DR16_ROCKER_RANGE / 2.0f;

    dr16->right_x = Clamp((ch0 - DR16_ROCKER_OFFSET) / rocker_denom, -1.0f, 1.0f);
    dr16->right_y = Clamp((ch1 - DR16_ROCKER_OFFSET) / rocker_denom, -1.0f, 1.0f);
    dr16->left_x = Clamp((ch2 - DR16_ROCKER_OFFSET) / rocker_denom, -1.0f, 1.0f);
    dr16->left_y = Clamp((ch3 - DR16_ROCKER_OFFSET) / rocker_denom, -1.0f, 1.0f);

    dr16->mouse_x = Clamp(mouse_x / 32768.0f, -1.0f, 1.0f);
    dr16->mouse_y = Clamp(mouse_y / 32768.0f, -1.0f, 1.0f);
    dr16->mouse_z = Clamp(mouse_z / 32768.0f, -1.0f, 1.0f);

    dr16->update_count++;
    dr16->last_update_tick = HAL_GetTick();
}

/**
 * @brief 更新 DR16 拨码和按键的边沿状态
 * @param dr16 应用层数据结构
 */
void Class_DR16::Timer1msCallback(DR16_DataTypeDef *dr16)
{
    uint8_t i;
    uint8_t now_bit;
    uint8_t prev_bit;

    if (dr16 == NULL)
    {
        return;
    }

    JudgeSwitch(&dr16->left_switch, dr16->raw_s1, dr16->prev_raw_s1);
    JudgeSwitch(&dr16->right_switch, dr16->raw_s2, dr16->prev_raw_s2);

    JudgeKey(&dr16->mouse_left, dr16->raw_mouse_l, dr16->prev_raw_mouse_l);
    JudgeKey(&dr16->mouse_right, dr16->raw_mouse_r, dr16->prev_raw_mouse_r);

    for (i = 0u; i < 16u; i++)
    {
        now_bit = (dr16->raw_key >> i) & 0x01u;
        prev_bit = (dr16->prev_raw_key >> i) & 0x01u;
        JudgeKey(&dr16->key[i], now_bit, prev_bit);
    }

    dr16->prev_raw_s1 = dr16->raw_s1;
    dr16->prev_raw_s2 = dr16->raw_s2;
    dr16->prev_raw_mouse_l = dr16->raw_mouse_l;
    dr16->prev_raw_mouse_r = dr16->raw_mouse_r;
    dr16->prev_raw_key = dr16->raw_key;
}

/**
 * @brief DR16 串口接收桥接回调
 * @novel 该函数不是 HAL 官方回调，而是当前工程注册给 USART 驱动的上层接收桥接入口。
 * @param data 接收到的原始字节流
 * @param len 本次接收长度
 */
static void DR16_Receive_Callback(uint8_t *data, uint16_t len)
{
    DR16_Manage_Object.UART_RxCpltCallback(data, len);
}
