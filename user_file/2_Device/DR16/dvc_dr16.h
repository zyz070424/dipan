/**
 * @file dvc_dr16.h
 * @brief DR16 遥控器设备对象定义与对外接口
 *
 * @details
 * 本文件定义 DR16 遥控器对象及其解析结果数据结构。
 * 当前版本把 DR16 自身持有的双缓冲接收状态与解析逻辑收进对象本身，
 * 同时保持应用层直接访问 `DR16_DataTypeDef` 的方式不变。
 */
#ifndef __DVC_DR16_H__
#define __DVC_DR16_H__

#include <stdint.h>
#include "drv_usart.h"

/**
 * @brief DR16 一帧原始数据长度
 */
#define DR16_FRAME_LEN 18

/**
 * @brief DR16 拨码开关原始值定义
 */
#define DR16_SWITCH_UP 1
#define DR16_SWITCH_DOWN 2
#define DR16_SWITCH_MIDDLE 3

/**
 * @brief DR16 按键原始值定义
 */
#define DR16_KEY_FREE 0
#define DR16_KEY_PRESSED 1

/**
 * @brief DR16 摇杆零点偏移
 */
#define DR16_ROCKER_OFFSET 1024
/**
 * @brief DR16 摇杆总量程
 */
#define DR16_ROCKER_RANGE 1320

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

/**
 * @brief DR16 拨码开关状态
 */
typedef enum
{
    DR16_SWITCH_STATUS_UP = 0,
    DR16_SWITCH_STATUS_MIDDLE,
    DR16_SWITCH_STATUS_DOWN,
    DR16_SWITCH_STATUS_TRIG_UP_MIDDLE,
    DR16_SWITCH_STATUS_TRIG_MIDDLE_UP,
    DR16_SWITCH_STATUS_TRIG_MIDDLE_DOWN,
    DR16_SWITCH_STATUS_TRIG_DOWN_MIDDLE,
} DR16_Switch_Status_TypeDef;

/**
 * @brief DR16 按键状态
 */
typedef enum
{
    DR16_KEY_STATUS_FREE = 0,
    DR16_KEY_STATUS_PRESSED,
    DR16_KEY_STATUS_TRIG_FREE_PRESSED,
    DR16_KEY_STATUS_TRIG_PRESSED_FREE,
} DR16_Key_Status_TypeDef;

/**
 * @brief 解析后的 DR16 数据
 *
 * @details
 * 该结构保持当前工程原有定义不变，供应用层直接访问。
 */
typedef struct
{
    float right_x;  /**< 右摇杆 X，归一化到 [-1, 1] */
    float right_y;  /**< 右摇杆 Y，归一化到 [-1, 1] */
    float left_x;   /**< 左摇杆 X，归一化到 [-1, 1] */
    float left_y;   /**< 左摇杆 Y，归一化到 [-1, 1] */

    float mouse_x;  /**< 鼠标 X，归一化到 [-1, 1] */
    float mouse_y;  /**< 鼠标 Y，归一化到 [-1, 1] */
    float mouse_z;  /**< 鼠标 Z，归一化到 [-1, 1] */

    DR16_Switch_Status_TypeDef left_switch;   /**< 左侧拨码状态 */
    DR16_Switch_Status_TypeDef right_switch;  /**< 右侧拨码状态 */

    DR16_Key_Status_TypeDef mouse_left;   /**< 鼠标左键状态 */
    DR16_Key_Status_TypeDef mouse_right;  /**< 鼠标右键状态 */

    DR16_Key_Status_TypeDef key[16];  /**< 键盘 16 个按键状态 */

    uint8_t raw_s1;            /**< 原始左拨码值 */
    uint8_t raw_s2;            /**< 原始右拨码值 */
    uint8_t raw_mouse_l;       /**< 原始鼠标左键值 */
    uint8_t raw_mouse_r;       /**< 原始鼠标右键值 */
    uint16_t raw_key;          /**< 原始键盘位图 */
    uint32_t update_count;     /**< 成功解析帧计数 */
    uint32_t last_update_tick; /**< 最近一次解析成功的系统时刻 */

    uint8_t prev_raw_s1;       /**< 上一周期左拨码原始值 */
    uint8_t prev_raw_s2;       /**< 上一周期右拨码原始值 */
    uint8_t prev_raw_mouse_l;  /**< 上一周期鼠标左键原始值 */
    uint8_t prev_raw_mouse_r;  /**< 上一周期鼠标右键原始值 */
    uint16_t prev_raw_key;     /**< 上一周期键盘位图 */
} DR16_DataTypeDef;

/**
 * @brief DR16 遥控器设备对象
 *
 * @details
 * 该对象持有 DR16 自身的双缓冲接收状态，并完成：
 * - 新帧接收
 * - 原始帧提取
 * - 摇杆、鼠标和按键解析
 * - 开关与按键边沿判断
 */
class Class_DR16
{
public:
    UART_HandleTypeDef *huart = NULL; /**< 当前绑定的 UART 句柄 */

    /**
     * @brief 初始化 DR16 设备对象
     * @param uart_handle 绑定的 UART 句柄
     */
    void Init(UART_HandleTypeDef *uart_handle);

    /**
     * @brief 处理一次 DR16 串口接收完成
     * @param data 接收到的原始字节流
     * @param len 本次接收长度
     */
    void UART_RxCpltCallback(uint8_t *data, uint16_t len);

    /**
     * @brief 处理一帧最新 DR16 数据
     * @param dr16 输出的应用层数据结构
     */
    void Process(DR16_DataTypeDef *dr16);

    /**
     * @brief 更新拨码和按键边沿状态
     * @param dr16 输出的应用层数据结构
     */
    void Timer1msCallback(DR16_DataTypeDef *dr16);

private:
    uint8_t frame_buf[2][DR16_FRAME_LEN] = {0}; /**< DR16 接收双缓冲 */
    volatile uint8_t ready_index = 0u;          /**< 当前可读缓冲区索引 */
    volatile uint8_t has_new_frame = 0u;        /**< 是否存在尚未处理的新帧 */

    /**
     * @brief 对输入值进行限幅
     * @param val 输入值
     * @param min_val 最小值
     * @param max_val 最大值
     * @return 限幅后的结果
     */
    static float Clamp(float val, float min_val, float max_val);

    /**
     * @brief 对拨码开关值进行合法化处理
     * @param sw 原始拨码值
     * @return 合法拨码值
     */
    static uint8_t SanitizeSwitch(uint8_t sw);

    /**
     * @brief 从双缓冲中取出最新一帧
     * @param out_frame 输出帧缓冲区
     * @retval 1 成功取到新帧
     * @retval 0 当前没有新帧
     */
    uint8_t FetchLatestFrame(uint8_t *out_frame);

    /**
     * @brief 判断拨码开关状态及触发沿
     * @param sw 输出拨码状态
     * @param now 当前原始值
     * @param prev 上一周期原始值
     */
    static void JudgeSwitch(DR16_Switch_Status_TypeDef *sw, uint8_t now, uint8_t prev);

    /**
     * @brief 判断按键状态及触发沿
     * @param key 输出按键状态
     * @param now 当前原始值
     * @param prev 上一周期原始值
     */
    static void JudgeKey(DR16_Key_Status_TypeDef *key, uint8_t now, uint8_t prev);
};

/**
 * @brief 全局 DR16 管理对象
 */
extern Class_DR16 DR16_Manage_Object;

#endif /* __DVC_DR16_H__ */
