/**
 * @file drv_usart.h
 * @brief USART/UART 驱动接口与管理对象定义
 *
 * @details
 * 当前驱动主要服务于 DR16 遥控接收和 VOFA 数据发送。
 * 当前版本直接以类对象形式暴露每一路 UART 软件管理对象。
 */
#ifndef DRV_USART_H__
#define DRV_USART_H__

#include "main.h"
#include "stm32f4xx_hal_uart.h"

/**
 * @brief UART 接收双缓冲区大小
 */
#define UART_BUFFER_SIZE 256
/**
 * @brief UART DMA 发送缓存大小
 */
#define UART_TX_BUFFER_SIZE 256

/**
 * @brief 上层串口接收回调函数类型
 * @param buffer 指向本次接收到的数据缓冲区
 * @param length 本次接收的有效字节数
 */
typedef void (*UART_Callback)(uint8_t *buffer, uint16_t length);

/**
 * @brief UART 软件管理对象
 *
 * @details
 * 一个 `Class_UART_Manage_Object` 对应一路串口外设，负责：
 * - 保存 HAL UART 句柄
 * - 维护双缓冲接收区
 * - 维护 DMA 发送缓存和发送忙状态
 * - 在 HAL 回调到来时完成接收缓冲切换与发送状态释放
 */
class Class_UART_Manage_Object
{
public:
    UART_HandleTypeDef *huart;              /**< 关联的 HAL UART 句柄 */
    UART_Callback Callback_Function;        /**< 上层接收回调函数 */

    uint8_t Rx_Buffer_0[UART_BUFFER_SIZE];  /**< 接收缓冲区 0 */
    uint8_t Rx_Buffer_1[UART_BUFFER_SIZE];  /**< 接收缓冲区 1 */

    uint8_t *Rx_Buffer_Active;              /**< 当前由 HAL 写入的接收缓冲区 */
    uint8_t *Rx_Buffer_Ready;               /**< 最近一次接收完成后的可读缓冲区 */

    uint8_t Tx_Buffer[UART_TX_BUFFER_SIZE]; /**< DMA 发送缓存 */
    volatile uint8_t Tx_Busy;               /**< 当前是否存在正在进行的 DMA 发送 */

    /**
     * @brief 初始化 UART 管理对象
     * @param uart_handle HAL UART 句柄
     * @param callback 上层接收回调函数
     */
    void Init(UART_HandleTypeDef *uart_handle, UART_Callback callback);

    /**
     * @brief 发送一段数据
     * @param data 待发送数据指针
     * @param len 待发送字节数
     * @retval HAL_OK 发送成功发起
     * @retval HAL_BUSY 当前仍在发送
     * @retval HAL_ERROR 参数无效或当前不支持发送
     */
    HAL_StatusTypeDef SendData(const uint8_t *data, uint16_t len);

    /**
     * @brief 处理一次 ReceiveToIdle 接收完成事件
     * @param size 本次接收的有效字节数
     */
    void RxEventCallback(uint16_t size);

    /**
     * @brief 处理一次 DMA 发送完成事件
     */
    void TxCpltCallback();

    /**
     * @brief 处理一次 UART 错误事件
     */
    void ErrorCallback();

private:
    /**
     * @brief 启动一次 ReceiveToIdle 接收
     * @retval HAL 状态码
     */
    HAL_StatusTypeDef StartRx();

    /**
     * @brief 重新初始化接收状态
     *
     * @details
     * 主要用于在错误回调中恢复接收路径。
     */
    void Reinit();
};

/**
 * @brief USART1 对应的管理对象
 */
extern Class_UART_Manage_Object UART1_Manage_Object;
/**
 * @brief USART3 对应的管理对象
 */
extern Class_UART_Manage_Object UART3_Manage_Object;
/**
 * @brief UART5 对应的管理对象
 */
extern Class_UART_Manage_Object UART5_Manage_Object;
/**
 * @brief USART6 对应的管理对象
 */
extern Class_UART_Manage_Object UART6_Manage_Object;

#endif /* DRV_USART_H__ */
