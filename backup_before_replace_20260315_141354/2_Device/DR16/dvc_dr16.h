#ifndef __DVC_DR16_H__
#define __DVC_DR16_H__

#include "drv_usart.h"


typedef struct {
    uint8_t *data;
    uint16_t len;
} UART_RxMsg_t;

#define DR16_SWITCH_UP      1
#define DR16_SWITCH_DOWN    2
#define DR16_SWITCH_MIDDLE  3

// 按键值
#define DR16_KEY_FREE       0
#define DR16_KEY_PRESSED    1

// 摇杆范围
#define DR16_ROCKER_OFFSET  1024
#define DR16_ROCKER_RANGE   1320   // 1684-364

// 键位宏定义
#define DR16_KEY_W 0
#define DR16_KEY_S 1
#define DR16_KEY_A 2
#define DR16_KEY_D 3
#define DR16_KEY_SHIFT 4
#define DR16_KEY_CTRL 5
#define DR16_KEY_Q 6
#define DR16_KEY_E 7
#define DR16_KEY_R 8
#define DR16_KEY_F 9
#define DR16_KEY_G 10
#define DR16_KEY_Z 11
#define DR16_KEY_X 12
#define DR16_KEY_C 13
#define DR16_KEY_V 14
#define DR16_KEY_B 15

// 开关状态（带触发）
typedef enum {
    DR16_SWITCH_STATUS_UP = 0,
    DR16_SWITCH_STATUS_MIDDLE,
    DR16_SWITCH_STATUS_DOWN,
    DR16_SWITCH_STATUS_TRIG_UP_MIDDLE,      // 上->中
    DR16_SWITCH_STATUS_TRIG_MIDDLE_UP,      // 中->上
    DR16_SWITCH_STATUS_TRIG_MIDDLE_DOWN,    // 中->下
    DR16_SWITCH_STATUS_TRIG_DOWN_MIDDLE,    // 下->中
} DR16_Switch_Status_TypeDef;

// 按键状态（带触发）
typedef enum {
    DR16_KEY_STATUS_FREE = 0,
    DR16_KEY_STATUS_PRESSED,
    DR16_KEY_STATUS_TRIG_FREE_PRESSED,      // 松开->按下
    DR16_KEY_STATUS_TRIG_PRESSED_FREE,      // 按下->松开
} DR16_Key_Status_TypeDef;


// 解析后的遥控器数据（供应用程序使用）
typedef struct {
    // 摇杆归一化值 (-1.0 ~ 1.0)
    float right_x;
    float right_y;
    float left_x;
    float left_y;

    // 鼠标归一化值 (-1.0 ~ 1.0)
    float mouse_x;
    float mouse_y;
    float mouse_z;

    // 开关状态（带触发）
    DR16_Switch_Status_TypeDef left_switch;
    DR16_Switch_Status_TypeDef right_switch;

    // 鼠标按键状态（带触发）
    DR16_Key_Status_TypeDef mouse_left;
    DR16_Key_Status_TypeDef mouse_right;

    // 键盘按键状态（16个键）
    DR16_Key_Status_TypeDef key[16];

    // 内部使用的原始数据（用于边沿检测）
    uint8_t raw_s1;
    uint8_t raw_s2;
    uint8_t raw_mouse_l;
    uint8_t raw_mouse_r;
    uint16_t raw_key;

    // 上一次原始数据（边沿检测用）
    uint8_t prev_raw_s1;
    uint8_t prev_raw_s2;
    uint8_t prev_raw_mouse_l;
    uint8_t prev_raw_mouse_r;
    uint16_t prev_raw_key;
} DR16_DataTypeDef;


void DR16_Init(UART_HandleTypeDef *huart);
void DR16_Process(DR16_DataTypeDef *dr16);
void DR16_Timer1msCallback(DR16_DataTypeDef *dr16);
#endif /* __DVC_DR16_H__ */
