#include "drv_usart.h"
#include "stm32f4xx_hal_uart.h"

struct Struct_UART_Manage_Object UART1_Manage_Object = {0};
struct Struct_UART_Manage_Object UART6_Manage_Object = {0};
/*
 * @brief  USART 初始化函数
 * @param  huart: UART 句柄指针
 * @param  callback: 接收完成回调函数指针
 * @retval 无
 */
void USART_Init(UART_HandleTypeDef *huart, UART_Callback callback)
{
    if (huart->Instance == USART1)
    {
        UART1_Manage_Object.huart = huart;
        UART1_Manage_Object.Callback_Function = callback;
        UART1_Manage_Object.Rx_Buffer_Active = UART1_Manage_Object.Rx_Buffer_0;
        UART1_Manage_Object.Rx_Buffer_Ready = UART1_Manage_Object.Rx_Buffer_1;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);
    }
    else if (huart->Instance == USART6)
    {
        UART6_Manage_Object.huart = huart;
        UART6_Manage_Object.Callback_Function = callback;
        UART6_Manage_Object.Rx_Buffer_Active = UART6_Manage_Object.Rx_Buffer_0;
        UART6_Manage_Object.Rx_Buffer_Ready = UART6_Manage_Object.Rx_Buffer_1;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART6_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);
    }
}
/*
 * @brief  USART 重新初始化函数（用于错误恢复）
 * @param  huart: UART 句柄指针
 * @retval 无
 */
void UART_Reinit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        UART1_Manage_Object.Rx_Buffer_Active = UART1_Manage_Object.Rx_Buffer_0;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);
    }
    else if (huart->Instance == USART6)
    {
        UART6_Manage_Object.Rx_Buffer_Active = UART6_Manage_Object.Rx_Buffer_0;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART6_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);
    }
 }
/*
 * @brief  USART 发送数据函数
 * @param  huart: UART 句柄指针  
 * @param  data: 待发送的数据指针
 * @param  len: 待发送的数据长度
 * @retval 无
 */
void USART_SendData(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit_DMA(huart, data, len);
}
/*
 * @brief  UART 接收完成回调函数（由 HAL 库调用）
 * @param  huart: UART 句柄指针
 * @param  Size: 本次接收的数据长度
 * @retval 无
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        UART1_Manage_Object.Rx_Buffer_Ready = UART1_Manage_Object.Rx_Buffer_Active;
        if (UART1_Manage_Object.Rx_Buffer_Active == UART1_Manage_Object.Rx_Buffer_0)
        {
            UART1_Manage_Object.Rx_Buffer_Active = UART1_Manage_Object.Rx_Buffer_1;
        }
        else
        {
            UART1_Manage_Object.Rx_Buffer_Active = UART1_Manage_Object.Rx_Buffer_0;
        }

        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);

        if (UART1_Manage_Object.Callback_Function != NULL)
        {
            //通知 
            UART1_Manage_Object.Callback_Function(UART1_Manage_Object.Rx_Buffer_Ready, Size);
        }
    }
    else if (huart->Instance == USART6)
    {
        UART6_Manage_Object.Rx_Buffer_Ready = UART6_Manage_Object.Rx_Buffer_Active;
        if (UART6_Manage_Object.Rx_Buffer_Active == UART6_Manage_Object.Rx_Buffer_0)
        {
            UART6_Manage_Object.Rx_Buffer_Active = UART6_Manage_Object.Rx_Buffer_1;
        }
        else
        {
            UART6_Manage_Object.Rx_Buffer_Active = UART6_Manage_Object.Rx_Buffer_0;
        }

        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART6_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);

        if (UART6_Manage_Object.Callback_Function != NULL)
        {
            //通知
            UART6_Manage_Object.Callback_Function(UART6_Manage_Object.Rx_Buffer_Ready, Size);
        }
    }
}
/*
 * @brief  UART 错误回调函数（由 HAL 库调用） 
 * @param  huart: UART 句柄指针
 * @retval 无
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UART_Reinit(huart);
}