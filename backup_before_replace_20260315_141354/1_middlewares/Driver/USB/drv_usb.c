#include"drv_usb.h"
#include <stdint.h>
extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

struct Struct_USB_Manage_Object USB_Manage_Object = {0};

void USB_Init(USB_Callback Callback_Function)
{
    USB_Manage_Object.Callback_Function = Callback_Function;
    // 初始化双缓冲适配的缓冲区
    USB_Manage_Object.Rx_Buffer_Active = UserRxBufferFS;
}
/*
 * @brief  USB 发送数据函数
 * @param  data: 待发送的浮点数据数组指针
 * @param  len: 待发送的浮点数据个数
 * @retval 无
 */
void USB_SendData(float* data, uint16_t len)
{
    uint16_t data_len = len * sizeof(float);
    uint16_t total_len = data_len + 2;
    USB_Tx_Buffer[0] = '[';

    if (total_len > USB_DATA_Send_MAX)
    {
        return;
    }
    memcpy(&USB_Tx_Buffer[1], data, data_len);
    USB_Tx_Buffer[1 + data_len] = ']';
    CDC_Transmit_FS(USB_Tx_Buffer, total_len);



}
/*
 * @brief  USB 发送字符串函数
 * @param  str: 待发送的字符串指针
 * @retval 无
 */
void USB_SendString(const char* str)
{
    uint16_t len = strlen(str);
    if (len > USB_DATA_Send_MAX)
    {
        return;
    }
    memcpy(USB_Tx_Buffer, str, len);
    CDC_Transmit_FS(USB_Tx_Buffer, len);
}

void USB_Rx_Callback(uint8_t* Buf, uint32_t Len)
{
   USB_Manage_Object.Rx_Buffer_Ready = USB_Manage_Object.Rx_Buffer_Active;
    if (USB_Manage_Object.Rx_Buffer_Active == UserRxBufferFS)
    {
        USB_Manage_Object.Rx_Buffer_Active = USB_Manage_Object.Rx_Buffer_0;
    }
    else
    {
        USB_Manage_Object.Rx_Buffer_Active =UserRxBufferFS;
    }

    

    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, USB_Manage_Object.Rx_Buffer_Active);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);

    if (USB_Manage_Object.Callback_Function != NULL)
    {
        USB_Manage_Object.Callback_Function(USB_Manage_Object.Rx_Buffer_Ready, Len);
    }
}

