#ifndef DRV_USART_H__
#define DRV_USART_H__

#include "main.h"
#include "stm32f4xx_hal_uart.h"

#define UART_BUFFER_SIZE 256

typedef void (*UART_Callback)(uint8_t *Buffer, uint16_t Length);

struct Struct_UART_Manage_Object
{
    // UART句柄
    UART_HandleTypeDef *huart;
    UART_Callback Callback_Function;
    // 双缓冲适配的缓冲区以及当前激活的缓冲区
    uint8_t Rx_Buffer_0[UART_BUFFER_SIZE];
    uint8_t Rx_Buffer_1[UART_BUFFER_SIZE];
    // 正在接收的缓冲区
    uint8_t *Rx_Buffer_Active;
    // 接收完毕的缓冲区
    uint8_t *Rx_Buffer_Ready;
};

void USART_Init(UART_HandleTypeDef *huart, UART_Callback callback);

void USART_SendData(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len);


#endif /* DRV_USART_H__ */