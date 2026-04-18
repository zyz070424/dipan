/**
 * @file drv_can.cpp
 * @brief CAN 驱动实现
 *
 * @details
 * 本文件实现了 CAN 驱动的软件双缓冲接收机制。
 * 设计目标是：
 * - 中断里只做必要的数据搬运和状态记录
 * - 任务里再按 FIFO 或标准 ID 的方式读取数据
 * - 同时兼容原有 C 风格外部接口
 */
#include "drv_can.h"
#include <stdint.h>

/** @brief CAN1 对应的软件管理对象实例 */
Class_CAN_Manage_Object CAN1_Manage_Object = {};
/** @brief CAN2 对应的软件管理对象实例 */
Class_CAN_Manage_Object CAN2_Manage_Object = {};

/** @brief 记录每一路 CAN 是否已经完成 `HAL_CAN_Start()` */
static uint8_t can_started[2] = {0};

namespace
{
/**
 * @brief 将 HAL CAN 句柄映射为本地启动状态数组下标
 * @param hcan HAL CAN 句柄
 * @retval 0 对应 CAN1
 * @retval 1 对应 CAN2
 * @retval -1 无法识别的 CAN 实例
 */
int CAN_Get_Index(const CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return -1;
    }

    if (hcan->Instance == CAN1)
    {
        return 0;
    }

    if (hcan->Instance == CAN2)
    {
        return 1;
    }

    return -1;
}

/**
 * @brief 根据 HAL CAN 句柄获取对应的软件管理对象
 * @param hcan HAL CAN 句柄
 * @return 成功时返回对应管理对象指针，失败返回 NULL
 */
Class_CAN_Manage_Object *CAN_Get_Manage_Object(const CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return NULL;
    }

    if (hcan->Instance == CAN1)
    {
        return &CAN1_Manage_Object;
    }

    if (hcan->Instance == CAN2)
    {
        return &CAN2_Manage_Object;
    }

    return NULL;
}
}

/**
 * @brief 初始化软件管理对象的运行时状态
 * @param can_handle 对应的 HAL CAN 句柄
 */
void Class_CAN_Manage_Object::Init(CAN_HandleTypeDef *can_handle)
{
    hcan = can_handle;
    Rx_Buffer_Active = Rx_Buffer_0;
    Rx_Length_Active = 0;
    Rx_Buffer_Ready = Rx_Buffer_1;
    Rx_Length_Ready = 0;
    Drop_Count = 0;
    Notify_Task_Handle = NULL;
    Alive_Flag = 0;
    Alive_Pre_Flag = 0;
    Alive_Online = 0;
    Alive_Changed = 0;
}

/**
 * @brief 在 Ready 区为空时，把 Active 区发布为 Ready 区
 *
 * @details
 * 该函数只在不覆盖旧 Ready 数据的前提下切换缓冲区，避免任务尚未读取
 * 的数据被新的中断数据直接冲掉。
 */
void Class_CAN_Manage_Object::PublishActiveToReady()
{
    if (Rx_Length_Active == 0)
    {
        return;
    }

    if (Rx_Length_Ready != 0)
    {
        return;
    }

    Rx_Buffer_Ready = Rx_Buffer_Active;
    Rx_Length_Ready = Rx_Length_Active;

    if (Rx_Buffer_Active == Rx_Buffer_0)
    {
        Rx_Buffer_Active = Rx_Buffer_1;
    }
    else
    {
        Rx_Buffer_Active = Rx_Buffer_0;
    }

    Rx_Length_Active = 0;
}

/**
 * @brief 在任务读取前，必要时把 Active 区提升为 Ready 区
 *
 * @details
 * 当中断已经把数据写入 Active 区，但由于 Ready 区为空且尚未来得及发布时，
 * 读取路径会调用本函数把数据尽快暴露给任务层。
 */
void Class_CAN_Manage_Object::PromoteActiveToReadyIfNeeded()
{
    if (Rx_Length_Ready == 0 && Rx_Length_Active > 0)
    {
        Rx_Buffer_Ready = Rx_Buffer_Active;
        Rx_Length_Ready = Rx_Length_Active;

        if (Rx_Buffer_Active == Rx_Buffer_0)
        {
            Rx_Buffer_Active = Rx_Buffer_1;
        }
        else
        {
            Rx_Buffer_Active = Rx_Buffer_0;
        }

        Rx_Length_Active = 0;
    }
}

/**
 * @brief 配置当前 CAN 句柄的硬件滤波器
 *
 * @note
 * 当前采用“全通 + 软件筛选”的策略，统一接收到 FIFO0。
 */
void Class_CAN_Manage_Object::FilterConfig()
{
    CAN_FilterTypeDef sFilterConfig;

    if (hcan == NULL)
    {
        return;
    }

    memset(&sFilterConfig, 0, sizeof(sFilterConfig));

    sFilterConfig.SlaveStartFilterBank = 14;
    sFilterConfig.FilterBank = (hcan->Instance == CAN2) ? 14 : 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
    {
    }
}

/**
 * @brief 启动 CAN 总线并使能 FIFO0 接收中断通知
 *
 * @details
 * 该函数带有重复启动保护：
 * - 同一路 CAN 重复调用时，不会再次执行 `HAL_CAN_Start()`
 * - 若对象尚未初始化，会先补做软件对象初始化
 */
void Class_CAN_Manage_Object::Start()
{
    int can_index = CAN_Get_Index(hcan);

    if (can_index < 0)
    {
        return;
    }

    if (Rx_Buffer_Active == NULL)
    {
        Init(hcan);
    }

    if (can_started[can_index] != 0)
    {
        return;
    }

    FilterConfig();

    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        return;
    }

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return;
    }

    can_started[can_index] = 1;
}

/**
 * @brief 注册接收就绪时需要通知的任务
 * @param task_handle 任务句柄
 */
void Class_CAN_Manage_Object::RegisterNotifyTask(TaskHandle_t task_handle)
{
    taskENTER_CRITICAL();
    Notify_Task_Handle = task_handle;
    taskEXIT_CRITICAL();
}

/**
 * @brief 发送一帧标准 CAN 数据帧
 * @param send_id 标准帧 ID
 * @param data 指向 8 字节数据的指针
 */
void Class_CAN_Manage_Object::Send(uint32_t send_id, uint8_t *data)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    if (hcan == NULL || data == NULL)
    {
        return;
    }

    tx_header.StdId = send_id;
    tx_header.ExtId = 0;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox) != HAL_OK)
    {
    }
}

/**
 * @brief FIFO0 接收中断的软件处理流程
 *
 * @details
 * 该函数运行在中断上下文，主要工作为：
 * - 从硬件 FIFO0 取出所有待处理报文
 * - 记录链路活跃计数
 * - 写入 Active 缓冲区
 * - 在合适时机把 Active 区发布为 Ready 区
 * - 如果注册了任务句柄，则通知任务层有新数据可读
 */
void Class_CAN_Manage_Object::ReceiveCallback()
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (hcan == NULL)
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        Alive_Flag++;

        if (Rx_Length_Active < CAN_RX_BUFFER_SIZE)
        {
            uint16_t write_index = Rx_Length_Active;
            Rx_Buffer_Active[write_index].rx_header = rx_header;
            memcpy(Rx_Buffer_Active[write_index].rx_data, rx_data, 8);
            Rx_Length_Active++;
        }
        else
        {
            Drop_Count++;
        }
    }

    PublishActiveToReady();

    if (Rx_Length_Ready > 0 && Notify_Task_Handle != NULL)
    {
        vTaskNotifyGiveFromISR(Notify_Task_Handle, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief 以先进先出方式读取一帧 CAN 报文
 * @param rx_message 输出报文缓存
 * @retval HAL_OK 读取成功
 * @retval HAL_ERROR 无可读报文或参数无效
 */
HAL_StatusTypeDef Class_CAN_Manage_Object::ReadMessage(CAN_RX_MESSAGE *rx_message)
{
    uint16_t i;

    if (rx_message == NULL)
    {
        return HAL_ERROR;
    }

    taskENTER_CRITICAL();

    PromoteActiveToReadyIfNeeded();

    if (Rx_Length_Ready == 0)
    {
        taskEXIT_CRITICAL();
        return HAL_ERROR;
    }

    *rx_message = Rx_Buffer_Ready[0];

    for (i = 1; i < Rx_Length_Ready; i++)
    {
        Rx_Buffer_Ready[i - 1] = Rx_Buffer_Ready[i];
    }

    Rx_Length_Ready--;

    taskEXIT_CRITICAL();

    return HAL_OK;
}

/**
 * @brief 按标准帧 ID 读取一帧匹配报文
 * @param std_id 目标标准帧 ID
 * @param rx_message 输出报文缓存
 * @retval HAL_OK 找到并读取成功
 * @retval HAL_ERROR 未找到匹配报文或参数无效
 *
 * @note
 * 读取成功后，匹配帧会从 Ready 缓冲区中移除。
 */
HAL_StatusTypeDef Class_CAN_Manage_Object::ReadMessageByStdId(uint32_t std_id, CAN_RX_MESSAGE *rx_message)
{
    uint16_t i;
    uint16_t j;

    if (rx_message == NULL)
    {
        return HAL_ERROR;
    }

    taskENTER_CRITICAL();

    PromoteActiveToReadyIfNeeded();

    for (i = 0; i < Rx_Length_Ready; i++)
    {
        if (Rx_Buffer_Ready[i].rx_header.StdId == std_id)
        {
            *rx_message = Rx_Buffer_Ready[i];

            for (j = i + 1; j < Rx_Length_Ready; j++)
            {
                Rx_Buffer_Ready[j - 1] = Rx_Buffer_Ready[j];
            }

            Rx_Length_Ready--;

            taskEXIT_CRITICAL();
            return HAL_OK;
        }
    }

    taskEXIT_CRITICAL();

    return HAL_ERROR;
}

/**
 * @brief 以 100ms 为周期更新在线状态
 *
 * @details
 * 如果本周期 `Alive_Flag` 相较上一次检查有增长，则认为链路在线；
 * 否则认为链路离线。只有在状态发生变化时，才会置位 `Alive_Changed`。
 */
void Class_CAN_Manage_Object::AliveCheck100ms()
{
    uint8_t online_new = (uint8_t)(Alive_Flag != Alive_Pre_Flag);

    Alive_Pre_Flag = Alive_Flag;

    if (online_new != Alive_Online)
    {
        Alive_Online = online_new;
        Alive_Changed = 1;
    }
}

/**
 * @brief 获取当前链路在线状态
 * @retval 0 离线
 * @retval 1 在线
 */
uint8_t Class_CAN_Manage_Object::AliveIsOnline() const
{
    return Alive_Online;
}

/**
 * @brief 消费一次“在线状态变化事件”
 * @param online 若非空，则返回当前在线状态
 * @retval 0 状态无变化
 * @retval 1 状态有变化
 */
uint8_t Class_CAN_Manage_Object::AliveTryConsumeChanged(uint8_t *online)
{
    uint8_t changed;

    taskENTER_CRITICAL();

    changed = Alive_Changed;
    if (changed != 0)
    {
        Alive_Changed = 0;
        if (online != NULL)
        {
            *online = Alive_Online;
        }
    }

    taskEXIT_CRITICAL();

    return changed;
}

/**
 * @brief HAL FIFO0 接收中断回调函数
 * @param hcan HAL CAN 句柄
 *
 * @details
 * HAL 在检测到 FIFO0 有待处理消息后会调用该函数。
 * 本驱动在这里统一转入 `CAN_Receive_Callback()`，以保持旧接口与新实现一致。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    Class_CAN_Manage_Object *manage = CAN_Get_Manage_Object(hcan);

    if (manage == NULL)
    {
        return;
    }

    if (manage->hcan == NULL)
    {
        manage->Init(hcan);
    }

    manage->ReceiveCallback();
}
