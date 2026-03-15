#include "drv_can.h"
#include <string.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"
QueueHandle_t xQueue_CAN[2] = {0};

/*
 * @brief  CAN 滤波器配置函数
 * @param  hcan: CAN句柄
 * @retval 无
 */
void CAN_Filter_Config(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0; // 使用滤波器0
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // 过滤模式：掩码模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 滤波器位宽：32位
    sFilterConfig.FilterIdHigh = 0x0000; // 标识符高16位（不使用）
    sFilterConfig.FilterIdLow = 0x0000;  // 标识符低16位（不使用）
    sFilterConfig.FilterMaskIdHigh = 0x0000; // 掩码高16位（全通）
    sFilterConfig.FilterMaskIdLow = 0x0000;  // 掩码低16位（全通）
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 分配到FIFO0
    sFilterConfig.FilterActivation = ENABLE; // 激活滤波器

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
    {
        // 处理错误
    }
}

/*
 * @brief  CAN 启动函数
 * @param  hcan: CAN句柄
 * @retval 无
 */
void CAN_Start(CAN_HandleTypeDef *hcan)
{
    if(hcan->Instance == CAN1)
    {
        xQueue_CAN[0] = xQueueCreate(10, sizeof( CAN_RX_MESSAGE));
        if (xQueue_CAN[0] == NULL)
        {
            // 队列创建失败处理
            return;
        }
    }
    else if(hcan->Instance == CAN2)
    {
        xQueue_CAN[1] = xQueueCreate(10, sizeof( CAN_RX_MESSAGE));
        if (xQueue_CAN[1] == NULL)
        {
            // 队列创建失败处理
           return;
        }
    }
    else
    {
        // 处理未知CAN实例的情况
        return;
    }
    
    // 配置滤波器
    CAN_Filter_Config(hcan);
    
    // 启动CAN
    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        // 处理错误
    }
    
    // 激活接收中断
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        // 处理错误
    }
}

/*
 * @brief  CAN 发送控制命令函数
 * @param  hcan: CAN句柄
 * @param  Send_id: 发送的CAN ID
 * @param  current: 包含四个电机当前值的数组指针
 * @retval 无
*/
void CAN_Send(CAN_HandleTypeDef *hcan, uint32_t Send_id, uint8_t *data)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    
    // 设置CAN ID
    tx_header.StdId = Send_id;
    tx_header.ExtId = 0;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;
    
   
    // 发送消息
    if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox) != HAL_OK)
    {
        // 处理错误
    }
}

/*
 * @brief  CAN 接收回调函数
 * @param  hcan: CAN句柄
 * @retval 无
 */
void CAN_Receive_Callback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    
    // 获取接收到的消息
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {   
        // 检查队列是否有效
        if (hcan->Instance == CAN1 && xQueue_CAN[0] != NULL)
        {
            CAN_RX_MESSAGE rx_message;
            rx_message.rx_header = rx_header;
            memcpy(rx_message.rx_data, rx_data, 8);
            
            BaseType_t ret = xQueueSendFromISR(xQueue_CAN[0], &rx_message, 0);
            if (ret != pdPASS)
            {
                // 处理队列发送失败的情况（队列满）
            }
        }
        else if (hcan->Instance == CAN2 && xQueue_CAN[1] != NULL)
        {
            CAN_RX_MESSAGE rx_message;
            rx_message.rx_header = rx_header;
            memcpy(rx_message.rx_data, rx_data, 8);
            
            BaseType_t ret = xQueueSendFromISR(xQueue_CAN[1], &rx_message, 0);
            if (ret != pdPASS)
            {
                // 处理队列发送失败的情况（队列满）
            }
        }
        else
        {
            // 处理未知CAN实例或未创建队列的情况
        }

    }   
}
/*
 * @brief  CAN 接收中断回调函数
 * @param  hcan: CAN句柄
 * @retval 无
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Receive_Callback(hcan);
}