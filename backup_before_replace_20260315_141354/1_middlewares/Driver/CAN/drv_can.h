#ifndef __DRV_CAN_H__
#define __DRV_CAN_H__
#include "main.h"
#include "stm32f4xx_hal_can.h"

// FreeRTOS相关头文件
#include "FreeRTOS.h"
#include "queue.h"

typedef struct 
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
} CAN_RX_MESSAGE;

// 全局队列句柄声明
extern QueueHandle_t xQueue_CAN[2];
void CAN_Filter_Config(CAN_HandleTypeDef *hcan);
void CAN_Start(CAN_HandleTypeDef *hcan);
void CAN_Send(CAN_HandleTypeDef *hcan, uint32_t Send_id, uint8_t *data);
#endif /* __DRV_CAN_H__ */