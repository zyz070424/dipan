#ifndef DRV_USB_H__
#define DRV_USB_H__

#include "main.h"
#include "usbd_cdc_if.h"

#define USB_DATA_Send_MAX 256
#define USB_BUFFER_SIZE   256

extern uint8_t USB_Tx_Buffer[USB_DATA_Send_MAX];
typedef void (*USB_Callback)(uint8_t *Buffer, uint32_t Length);

/**
 * @brief USB通信处理结构体
 */
struct Struct_USB_Manage_Object
{
    USB_Callback Callback_Function;

    // 双缓冲适配的缓冲区以及当前激活的缓冲区
    uint8_t Rx_Buffer_0[USB_BUFFER_SIZE];
    uint8_t *Rx_Buffer_Active;
    uint8_t *Rx_Buffer_Ready;

    // 接收时间戳
    uint64_t Rx_Time_Stamp;
};
void USB_Init(USB_Callback Callback_Function);
void USB_SendData(float* data, uint16_t len);
void USB_SendString(const char* str);
void USB_Rx_Callback(uint8_t* Buf, uint32_t Len);
#endif /* DRV_USB_H__ */