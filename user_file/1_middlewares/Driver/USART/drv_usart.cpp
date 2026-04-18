/**
 * @file drv_usart.cpp
 * @brief USART/UART 驱动实现
 */
#include "drv_usart.h"
#include <string.h>

/** @brief USART1 对应的管理对象实例 */
Class_UART_Manage_Object UART1_Manage_Object = {};
/** @brief USART3 对应的管理对象实例 */
Class_UART_Manage_Object UART3_Manage_Object = {};
/** @brief UART5 对应的管理对象实例 */
Class_UART_Manage_Object UART5_Manage_Object = {};
/** @brief USART6 对应的管理对象实例 */
Class_UART_Manage_Object UART6_Manage_Object = {};

namespace
{
/**
 * @brief 根据 HAL UART 句柄获取对应的软件管理对象
 * @param huart HAL UART 句柄
 * @return 成功时返回管理对象指针，失败返回 NULL
 */
Class_UART_Manage_Object *USART_Get_Manage_Object(UART_HandleTypeDef *huart)
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
 * @brief 进入临界区
 * @return 进入前的 PRIMASK
 */
uint32_t USART_Enter_Critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/**
 * @brief 退出临界区
 * @param primask 进入前保存的 PRIMASK
 */
void USART_Exit_Critical(uint32_t primask)
{
    if (primask == 0u)
    {
        __enable_irq();
    }
}
}

/**
 * @brief 启动一次 ReceiveToIdle 接收
 * @retval HAL 状态码
 */
HAL_StatusTypeDef Class_UART_Manage_Object::StartRx()
{
    HAL_StatusTypeDef ret;

    if ((huart == NULL) || (Rx_Buffer_Active == NULL))
    {
        return HAL_ERROR;
    }

    if (huart->hdmarx != NULL)
    {
        ret = HAL_UARTEx_ReceiveToIdle_DMA(huart, Rx_Buffer_Active, UART_BUFFER_SIZE);
        if (ret != HAL_OK)
        {
            return ret;
        }

        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        return HAL_OK;
    }

    return HAL_UARTEx_ReceiveToIdle_IT(huart, Rx_Buffer_Active, UART_BUFFER_SIZE);
}

/**
 * @brief 重新初始化接收状态
 */
void Class_UART_Manage_Object::Reinit()
{
    Rx_Buffer_Active = Rx_Buffer_0;
    Rx_Buffer_Ready = Rx_Buffer_1;
    Tx_Busy = 0u;

    (void)StartRx();
}

/**
 * @brief 初始化 UART 管理对象
 * @param uart_handle HAL UART 句柄
 * @param callback 上层接收回调函数
 */
void Class_UART_Manage_Object::Init(UART_HandleTypeDef *uart_handle, UART_Callback callback)
{
    huart = uart_handle;
    Callback_Function = callback;
    Rx_Buffer_Active = Rx_Buffer_0;
    Rx_Buffer_Ready = Rx_Buffer_1;
    Tx_Busy = 0u;

    (void)StartRx();
}

/**
 * @brief 使用 DMA 发送一段数据
 * @param data 待发送数据指针
 * @param len 待发送长度
 * @retval HAL_OK 发送成功发起
 * @retval HAL_BUSY 当前仍在发送
 * @retval HAL_ERROR 参数无效或当前不支持发送
 */
HAL_StatusTypeDef Class_UART_Manage_Object::SendData(const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef ret;
    uint32_t primask;

    if ((huart == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    if ((len == 0u) || (len > UART_TX_BUFFER_SIZE))
    {
        return HAL_ERROR;
    }

    if (huart->hdmatx == NULL)
    {
        return HAL_ERROR;
    }

    if (__get_IPSR() != 0u)
    {
        return HAL_ERROR;
    }

    primask = USART_Enter_Critical();
    if (Tx_Busy != 0u)
    {
        USART_Exit_Critical(primask);
        return HAL_BUSY;
    }
    Tx_Busy = 1u;
    USART_Exit_Critical(primask);

    memcpy(Tx_Buffer, data, len);

    ret = HAL_UART_Transmit_DMA(huart, Tx_Buffer, len);
    if (ret != HAL_OK)
    {
        primask = USART_Enter_Critical();
        Tx_Busy = 0u;
        USART_Exit_Critical(primask);
        return ret;
    }

    return HAL_OK;
}

/**
 * @brief 处理一次 ReceiveToIdle 接收完成事件
 * @param size 本次接收的有效字节数
 */
void Class_UART_Manage_Object::RxEventCallback(uint16_t size)
{
    uint8_t *ready_buffer;

    ready_buffer = Rx_Buffer_Active;
    if (Rx_Buffer_Active == Rx_Buffer_0)
    {
        Rx_Buffer_Active = Rx_Buffer_1;
    }
    else
    {
        Rx_Buffer_Active = Rx_Buffer_0;
    }

    Rx_Buffer_Ready = ready_buffer;

    (void)StartRx();

    if ((size > 0u) && (Callback_Function != NULL))
    {
        Callback_Function(Rx_Buffer_Ready, size);
    }
}

/**
 * @brief 处理一次 DMA 发送完成事件
 */
void Class_UART_Manage_Object::TxCpltCallback()
{
    Tx_Busy = 0u;
}

/**
 * @brief 处理一次 UART 错误事件
 */
void Class_UART_Manage_Object::ErrorCallback()
{
    Reinit();
}

/**
 * @brief HAL ReceiveToIdle 接收事件回调
 * @param huart HAL UART 句柄
 * @param Size 本次接收的有效字节数
 *
 * @details
 * 这是 HAL 官方回调入口，不使用 `@novel` 标记。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    Class_UART_Manage_Object *manage = USART_Get_Manage_Object(huart);

    if (manage == NULL)
    {
        return;
    }

    manage->RxEventCallback(Size);
}

/**
 * @brief HAL UART DMA 发送完成回调
 * @param huart HAL UART 句柄
 *
 * @details
 * 这是 HAL 官方回调入口，不使用 `@novel` 标记。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    Class_UART_Manage_Object *manage = USART_Get_Manage_Object(huart);

    if (manage == NULL)
    {
        return;
    }

    manage->TxCpltCallback();
}

/**
 * @brief HAL UART 错误回调
 * @param huart HAL UART 句柄
 *
 * @details
 * 这是 HAL 官方回调入口，不使用 `@novel` 标记。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Class_UART_Manage_Object *manage = USART_Get_Manage_Object(huart);

    if (manage == NULL)
    {
        return;
    }

    manage->ErrorCallback();

}
