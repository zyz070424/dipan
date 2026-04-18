/**
 * @file drv_usb.h
 * @brief USB CDC 驱动接口与管理对象定义
 *
 * @details
 * 当前驱动以 USB CDC 为基础，内部已经完全切到类作用域实现。
 * 对 `C` 编译单元仅保留接收和发送完成这两个必须的回调入口。
 */
#ifndef DRV_USB_H__
#define DRV_USB_H__

#include "main.h"
#include "usbd_cdc_if.h"
#include <stdint.h>

/**
 * @brief 单次 USB 发送允许的最大字节数
 */
#define USB_DATA_Send_MAX 256
/**
 * @brief USB 接收缓冲区大小
 */
#define USB_BUFFER_SIZE   256
/**
 * @brief 默认最小发送间隔，单位系统 Tick
 */
#define USB_TX_MIN_INTERVAL_TICK_DEFAULT 1
/**
 * @brief 发送忙状态超时阈值，单位系统 Tick
 */
#define USB_TX_BUSY_TIMEOUT_TICK 20

/**
 * @brief USB 上层接收回调函数类型
 * @param buffer 指向本次接收到的数据缓冲区
 * @param length 本次接收的有效字节数
 */
typedef void (*USB_Callback)(uint8_t *buffer, uint32_t length);

/**
 * @brief USB 软件管理对象
 *
 * @details
 * 该对象负责维护 USB CDC 的收发软件状态，包括：
 * - 上层接收回调
 * - 双缓冲接收区
 * - 发送忙状态与发送节流
 * - USB 接收在线检测
 */
#ifdef __cplusplus
class Class_USB_Manage_Object
{
public:
    USB_Callback Callback_Function;          /**< 上层接收回调函数 */

    uint8_t Rx_Buffer_0[USB_BUFFER_SIZE];    /**< 自维护接收缓冲区 */
    uint8_t *Rx_Buffer_Active;               /**< 当前交给底层 CDC 的接收缓冲区 */
    uint8_t *Rx_Buffer_Ready;                /**< 最近一次接收完成后可供上层读取的缓冲区 */

    volatile uint8_t Tx_Busy;                /**< 当前发送是否处于忙状态 */
    uint16_t Tx_Min_Interval_Tick;           /**< 两次发送允许的最小 Tick 间隔 */
    uint32_t Tx_Last_Transmit_Tick;          /**< 上一次成功发起发送的 Tick */
    uint32_t Tx_Busy_Start_Tick;             /**< 最近一次进入忙状态的 Tick */

    volatile uint32_t Alive_Flag;            /**< 收到有效数据时的活跃计数 */
    uint32_t Alive_Pre_Flag;                 /**< 上一次 100ms 检查时的活跃计数 */
    volatile uint8_t Alive_Online;           /**< 当前在线状态，0=离线，1=在线 */
    volatile uint8_t Alive_Changed;          /**< 在线状态是否发生变化 */

    /**
     * @brief 初始化 USB 软件管理对象
     * @param callback 上层接收回调函数
     */
    void Init(USB_Callback callback);

    /**
     * @brief 发送一段二进制数据
     * @param data 待发送数据指针
     * @param len 待发送数据长度
     * @return `USBD_OK` / `USBD_BUSY` / `USBD_FAIL`
     */
    uint8_t SendData(const uint8_t *data, uint16_t len);

    /**
     * @brief 发送一段字符串
     * @param str 待发送字符串
     * @return `USBD_OK` / `USBD_BUSY` / `USBD_FAIL`
     */
    uint8_t SendString(const char *str);

    /**
     * @brief 设置最小发送间隔
     * @param interval_tick 最小发送间隔，单位 Tick
     */
    void SetTxMinInterval(uint16_t interval_tick);

    /**
     * @brief 处理一次 USB 接收完成事件
     * @param buf 底层 CDC 传入的接收缓冲区
     * @param len 本次接收到的字节数
     */
    void RxCallback(uint8_t *buf, uint32_t len);

    /**
     * @brief 处理一次 USB 发送完成事件
     */
    void TxCpltCallback();

    /**
     * @brief 执行一次 100ms 周期的在线状态检查
     */
    void AliveCheck100ms();

    /**
     * @brief 获取当前 USB 在线状态
     * @retval 0 离线
     * @retval 1 在线
     */
    uint8_t AliveIsOnline() const;

    /**
     * @brief 消费一次 USB 在线状态变化事件
     * @param online 若非空，则返回当前在线状态
     * @retval 0 无变化
     * @retval 1 有变化
     */
    uint8_t AliveTryConsumeChanged(uint8_t *online);

private:
    /**
     * @brief USB 接收活跃计数加一
     */
    void AliveFeed();

    /**
     * @brief 重新启动下一包 CDC 接收
     */
    void StartReceive();

    /**
     * @brief 执行 USB 接收双缓冲切换
     */
    void SwapRxBuffer();

    /**
     * @brief 尝试发起一次底层发送，不做数据拷贝
     * @param data 待发送缓冲区
     * @param len 待发送字节数
     * @return `USBD_OK` / `USBD_BUSY` / `USBD_FAIL`
     */
    uint8_t TryTransmitNoCopy(uint8_t *data, uint16_t len);
};

/**
 * @brief 全局 USB 管理对象
 */
extern Class_USB_Manage_Object USB_Manage_Object;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief USB 接收软件回调入口
 * @param buf 底层 CDC 传入的接收缓冲区
 * @param len 接收到的数据长度
 *
 * @novel
 * 该函数不是 ST 官方规定必须由用户实现的回调名，而是本工程在
 * `usbd_cdc_if.c` 中额外转调出来的 USB 软件回调入口。
 */
void USB_Rx_Callback(uint8_t *buf, uint32_t len);

/**
 * @brief USB 发送完成软件回调入口
 *
 * @novel
 * 该函数不是 ST 官方规定必须由用户实现的回调名，而是本工程在
 * `usbd_cdc_if.c` 中额外转调出来的 USB 软件回调入口。
 */
void USB_TxCplt_Callback(void);

#ifdef __cplusplus
}
#endif

#endif /* DRV_USB_H__ */
