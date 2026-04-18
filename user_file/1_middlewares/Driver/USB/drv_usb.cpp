/**
 * @file drv_usb.cpp
 * @brief USB CDC 驱动实现
 */
#include "drv_usb.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** @brief 全局 USB 管理对象实例 */
Class_USB_Manage_Object USB_Manage_Object = {};

/** @brief USB 发送内部缓存，避免异步发送阶段引用上层栈内存 */
static uint8_t USB_Tx_Buffer[USB_DATA_Send_MAX];

namespace
{
/**
 * @brief 获取当前 CDC 类句柄
 * @return CDC 类句柄指针；当 USB 未配置完成时返回 NULL
 */
USBD_CDC_HandleTypeDef *USB_Get_CDC_Handle(void)
{
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return NULL;
    }

    return (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
}

/**
 * @brief 进入临界区
 * @return 进入前的 PRIMASK
 */
uint32_t USB_Enter_Critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/**
 * @brief 退出临界区
 * @param primask 进入前保存的 PRIMASK
 */
void USB_Exit_Critical(uint32_t primask)
{
    if (primask == 0u)
    {
        __enable_irq();
    }
}
}

/**
 * @brief 初始化 USB 管理对象
 * @param callback 上层接收回调函数
 */
void Class_USB_Manage_Object::Init(USB_Callback callback)
{
    uint32_t now_tick = HAL_GetTick();

    Callback_Function = callback;
    Rx_Buffer_Active = UserRxBufferFS;
    Rx_Buffer_Ready = NULL;

    Tx_Busy = 0u;
    Tx_Min_Interval_Tick = USB_TX_MIN_INTERVAL_TICK_DEFAULT;
    Tx_Last_Transmit_Tick = now_tick - USB_TX_MIN_INTERVAL_TICK_DEFAULT;
    Tx_Busy_Start_Tick = now_tick;

    Alive_Flag = 0u;
    Alive_Pre_Flag = 0u;
    Alive_Online = 0u;
    Alive_Changed = 0u;

    StartReceive();
}

/**
 * @brief 设置最小发送间隔
 * @param interval_tick 发送间隔，单位 Tick
 */
void Class_USB_Manage_Object::SetTxMinInterval(uint16_t interval_tick)
{
    Tx_Min_Interval_Tick = (interval_tick == 0u) ? 1u : interval_tick;
}

/**
 * @brief USB 接收活跃计数加一
 */
void Class_USB_Manage_Object::AliveFeed()
{
    Alive_Flag++;
}

/**
 * @brief 重新启动下一包 CDC 接收
 */
void Class_USB_Manage_Object::StartReceive()
{
    if (USB_Get_CDC_Handle() == NULL)
    {
        return;
    }

    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, Rx_Buffer_Active);
    (void)USBD_CDC_ReceivePacket(&hUsbDeviceFS);
}

/**
 * @brief 执行 USB 接收双缓冲切换
 */
void Class_USB_Manage_Object::SwapRxBuffer()
{
    uint8_t *ready_buffer = Rx_Buffer_Active;

    Rx_Buffer_Active = (ready_buffer == UserRxBufferFS) ? Rx_Buffer_0 : UserRxBufferFS;
    Rx_Buffer_Ready = ready_buffer;
}

/**
 * @brief 尝试直接发起一次底层发送
 * @param data 发送缓冲区
 * @param len 发送长度
 * @return `USBD_OK` / `USBD_BUSY` / `USBD_FAIL`
 */
uint8_t Class_USB_Manage_Object::TryTransmitNoCopy(uint8_t *data, uint16_t len)
{
    USBD_CDC_HandleTypeDef *hcdc = USB_Get_CDC_Handle();

    if (hcdc == NULL)
    {
        return USBD_BUSY;
    }

    if (hcdc->TxState != 0u)
    {
        return USBD_BUSY;
    }

    return CDC_Transmit_FS(data, len);
}

/**
 * @brief 发送一段二进制数据
 * @param data 待发送数据指针
 * @param len 待发送长度
 * @return `USBD_OK` / `USBD_BUSY` / `USBD_FAIL`
 */
uint8_t Class_USB_Manage_Object::SendData(const uint8_t *data, uint16_t len)
{
    uint8_t ret;
    uint32_t primask;
    uint32_t now_tick;
    USBD_CDC_HandleTypeDef *hcdc;

    if ((data == NULL) || (len == 0u) || (len > USB_DATA_Send_MAX))
    {
        return USBD_FAIL;
    }

    if (__get_IPSR() != 0u)
    {
        return USBD_FAIL;
    }

    now_tick = HAL_GetTick();
    if ((uint32_t)(now_tick - Tx_Last_Transmit_Tick) < Tx_Min_Interval_Tick)
    {
        return USBD_BUSY;
    }

    primask = USB_Enter_Critical();

    if ((Tx_Busy != 0u) &&
        ((uint32_t)(now_tick - Tx_Busy_Start_Tick) >= USB_TX_BUSY_TIMEOUT_TICK))
    {
        hcdc = USB_Get_CDC_Handle();
        if ((hcdc == NULL) || (hcdc->TxState == 0u))
        {
            Tx_Busy = 0u;
        }
    }

    if (Tx_Busy != 0u)
    {
        USB_Exit_Critical(primask);
        return USBD_BUSY;
    }

    Tx_Busy = 1u;
    Tx_Busy_Start_Tick = now_tick;
    USB_Exit_Critical(primask);

    memcpy(USB_Tx_Buffer, data, len);

    ret = TryTransmitNoCopy(USB_Tx_Buffer, len);
    if (ret == USBD_OK)
    {
        Tx_Last_Transmit_Tick = now_tick;
        return USBD_OK;
    }

    primask = USB_Enter_Critical();
    Tx_Busy = 0u;
    USB_Exit_Critical(primask);

    return ret;
}

/**
 * @brief 发送一段字符串
 * @param str 待发送字符串
 * @return `USBD_OK` / `USBD_BUSY` / `USBD_FAIL`
 */
uint8_t Class_USB_Manage_Object::SendString(const char *str)
{
    uint16_t len;

    if (str == NULL)
    {
        return USBD_FAIL;
    }

    len = (uint16_t)strlen(str);
    if (len == 0u)
    {
        return USBD_OK;
    }

    return SendData((const uint8_t *)str, len);
}

/**
 * @brief 处理一次 USB 接收完成事件
 * @param buf 底层 CDC 传入的缓冲区
 * @param len 接收到的有效长度
 */
void Class_USB_Manage_Object::RxCallback(uint8_t *buf, uint32_t len)
{
    (void)buf;

    if (len > 0u)
    {
        SwapRxBuffer();
        AliveFeed();

        StartReceive();

        if (Callback_Function != NULL)
        {
            Callback_Function(Rx_Buffer_Ready, len);
        }
        return;
    }

    StartReceive();
}

/**
 * @brief 处理一次 USB 发送完成事件
 */
void Class_USB_Manage_Object::TxCpltCallback()
{
    Tx_Busy = 0u;
}

/**
 * @brief 执行一次 100ms 周期的在线状态检查
 */
void Class_USB_Manage_Object::AliveCheck100ms()
{
    uint8_t online_new = (uint8_t)(Alive_Flag != Alive_Pre_Flag);

    Alive_Pre_Flag = Alive_Flag;

    if (online_new != Alive_Online)
    {
        Alive_Online = online_new;
        Alive_Changed = 1u;
    }
}

/**
 * @brief 获取当前 USB 在线状态
 * @retval 0 离线
 * @retval 1 在线
 */
uint8_t Class_USB_Manage_Object::AliveIsOnline() const
{
    return Alive_Online;
}

/**
 * @brief 消费一次 USB 在线状态变化事件
 * @param online 若非空，则输出当前在线状态
 * @retval 0 无变化
 * @retval 1 有变化
 */
uint8_t Class_USB_Manage_Object::AliveTryConsumeChanged(uint8_t *online)
{
    uint8_t changed = Alive_Changed;

    if (changed != 0u)
    {
        Alive_Changed = 0u;
        if (online != NULL)
        {
            *online = Alive_Online;
        }
    }

    return changed;
}

/**
 * @brief USB 接收软件回调入口
 * @param buf 底层 CDC 传入的缓冲区
 * @param len 接收到的有效长度
 *
 * @novel
 * 该函数是本工程在 `usbd_cdc_if.c` 中额外转调出来的非官方回调入口。
 */
void USB_Rx_Callback(uint8_t *buf, uint32_t len)
{
    USB_Manage_Object.RxCallback(buf, len);
}

/**
 * @brief USB 发送完成软件回调入口
 *
 * @novel
 * 该函数是本工程在 `usbd_cdc_if.c` 中额外转调出来的非官方回调入口。
 */
void USB_TxCplt_Callback(void)
{
    USB_Manage_Object.TxCpltCallback();
}
