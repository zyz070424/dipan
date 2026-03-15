#include "dvc_dr16.h"
#include "FreeRTOS.h"
#include "queue.h"
xQueueHandle xUART1_Queue;
xQueueHandle xUART6_Queue;

static void DR16_Receive_Callback(uint8_t *data, uint16_t len)
{
    UART_RxMsg_t msg = {data, len};
    xQueueSendFromISR(xUART1_Queue, &msg, NULL);
}

// 内部函数：判断开关边沿
/**
 * @brief  判断开关边沿
 * @param  sw: 开关状态指针
 * @param  now: 当前开关状态
 * @param  prev: 上一个开关状态
 * @retval None
 */
static void JudgeSwitch(DR16_Switch_Status_TypeDef *sw, uint8_t now, uint8_t prev)
{
    switch (prev) {
        case DR16_SWITCH_UP:
            switch (now) {
                case DR16_SWITCH_UP:      *sw = DR16_SWITCH_STATUS_UP; break;
                case DR16_SWITCH_DOWN:    *sw = DR16_SWITCH_STATUS_TRIG_MIDDLE_DOWN; break;
                case DR16_SWITCH_MIDDLE:  *sw = DR16_SWITCH_STATUS_TRIG_UP_MIDDLE; break;
            } break;
        case DR16_SWITCH_DOWN:
            switch (now) {
                case DR16_SWITCH_UP:      *sw = DR16_SWITCH_STATUS_TRIG_MIDDLE_UP; break;
                case DR16_SWITCH_DOWN:    *sw = DR16_SWITCH_STATUS_DOWN; break;
                case DR16_SWITCH_MIDDLE:  *sw = DR16_SWITCH_STATUS_TRIG_DOWN_MIDDLE; break;
            } break;
        case DR16_SWITCH_MIDDLE:
            switch (now) {
                case DR16_SWITCH_UP:      *sw = DR16_SWITCH_STATUS_TRIG_MIDDLE_UP; break;
                case DR16_SWITCH_DOWN:    *sw = DR16_SWITCH_STATUS_TRIG_MIDDLE_DOWN; break;
                case DR16_SWITCH_MIDDLE:  *sw = DR16_SWITCH_STATUS_MIDDLE; break;
            } break;
        default:
            *sw = DR16_SWITCH_STATUS_MIDDLE;
    }
}

// 内部函数：判断按键边沿
/** 
 * @brief  判断按键边沿
 * @param  key: 按键状态指针
 * @brief  判断按键边沿
 * @param  now: 当前按键状态
 * @param  prev: 上一个按键状态
 * @retval None
 */
static void JudgeKey(DR16_Key_Status_TypeDef *key, uint8_t now, uint8_t prev)
{
    if (prev == DR16_KEY_FREE) {
        if (now == DR16_KEY_FREE)
            *key = DR16_KEY_STATUS_FREE;
        else
            *key = DR16_KEY_STATUS_TRIG_FREE_PRESSED;
    } else { // prev == PRESSED
        if (now == DR16_KEY_FREE)
            *key = DR16_KEY_STATUS_TRIG_PRESSED_FREE;
        else
            *key = DR16_KEY_STATUS_PRESSED;
    }
}
/**
 * @brief  初始化DR16遥控器
 * @param  huart: UART句柄指针
 * @retval None
 */
void DR16_Init(UART_HandleTypeDef *huart)
{
    xQueueCreate(10, sizeof(UART_RxMsg_t));
    USART_Init(huart, DR16_Receive_Callback);
    
}


/**
 * @brief  处理DR16遥控器数据
 * @param  dr16: DR16数据指针
 * @retval None
 */
void DR16_Process(DR16_DataTypeDef *dr16)
{
    UART_RxMsg_t msg;
    if (xQueueReceive(xUART1_Queue, &msg, portMAX_DELAY) == pdPASS)
    {
        if (msg.len < 18) return;
        uint8_t *raw = msg.data;

        // 临时保存通道（你结构体里没有 raw_ch0~raw_ch3）
        uint16_t ch0 = ((uint16_t)raw[0] | ((uint16_t)(raw[1] & 0x07) << 8)) & 0x07FF;
        uint16_t ch1 = (((uint16_t)raw[1] >> 3) | ((uint16_t)(raw[2] & 0x3F) << 5)) & 0x07FF;
        uint16_t ch2 = (((uint16_t)raw[2] >> 6) | ((uint16_t)raw[3] << 2) | ((uint16_t)(raw[4] & 0x01) << 10)) & 0x07FF;
        uint16_t ch3 = (((uint16_t)raw[4] >> 1) | ((uint16_t)(raw[5] & 0x0F) << 7)) & 0x07FF;

        dr16->raw_s1 = (raw[5] >> 4) & 0x03;
        dr16->raw_s2 = (raw[5] >> 6) & 0x03;

        int16_t mouse_x = (int16_t)((uint16_t)raw[6]  | ((uint16_t)raw[7]  << 8));
        int16_t mouse_y = (int16_t)((uint16_t)raw[8]  | ((uint16_t)raw[9]  << 8));
        int16_t mouse_z = (int16_t)((uint16_t)raw[10] | ((uint16_t)raw[11] << 8));

        dr16->raw_mouse_l = raw[12];
        dr16->raw_mouse_r = raw[13];
        dr16->raw_key     = (uint16_t)raw[14] | ((uint16_t)raw[15] << 8);

        // 归一化
        dr16->right_x = (ch0 - DR16_ROCKER_OFFSET) / (DR16_ROCKER_RANGE / 2.0f);
        dr16->right_y = (ch1 - DR16_ROCKER_OFFSET) / (DR16_ROCKER_RANGE / 2.0f);
        dr16->left_x  = (ch2 - DR16_ROCKER_OFFSET) / (DR16_ROCKER_RANGE / 2.0f);
        dr16->left_y  = (ch3 - DR16_ROCKER_OFFSET) / (DR16_ROCKER_RANGE / 2.0f);

        dr16->mouse_x = mouse_x / 32768.0f;
        dr16->mouse_y = mouse_y / 32768.0f;
        dr16->mouse_z = mouse_z / 32768.0f;
    }
}


/**
 * @brief  1ms回调函数，用于处理DR16遥控器数据
 * @param  dr16: DR16数据指针
 * @retval None
 */
void DR16_Timer1msCallback(DR16_DataTypeDef *dr16)
{
    // 判断开关边沿
    JudgeSwitch(&dr16->left_switch, dr16->raw_s1, dr16->prev_raw_s1);
    JudgeSwitch(&dr16->right_switch, dr16->raw_s2, dr16->prev_raw_s2);

    // 判断鼠标按键
    JudgeKey(&dr16->mouse_left, dr16->raw_mouse_l, dr16->prev_raw_mouse_l);
    JudgeKey(&dr16->mouse_right, dr16->raw_mouse_r, dr16->prev_raw_mouse_r);

    // 判断键盘按键
    for (int i = 0; i < 16; i++) {
        uint8_t now_bit = (dr16->raw_key >> i) & 0x01;
        uint8_t prev_bit = (dr16->prev_raw_key >> i) & 0x01;
        JudgeKey(&dr16->key[i], now_bit, prev_bit);
    }

    // 保存本次原始值，作为下一次的上一次
    dr16->prev_raw_s1 = dr16->raw_s1;
    dr16->prev_raw_s2 = dr16->raw_s2;
    dr16->prev_raw_mouse_l = dr16->raw_mouse_l;
    dr16->prev_raw_mouse_r = dr16->raw_mouse_r;
    dr16->prev_raw_key = dr16->raw_key;
}


   

