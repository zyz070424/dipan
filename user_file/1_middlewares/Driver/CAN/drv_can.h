/**
 * @file drv_can.h
 * @brief CAN 驱动对外接口与管理对象定义
 *
 * @details
 * 当前驱动已经完全切到 C++ 类作用域实现。
 * 运行机制仍然保持原工程的设计：
 * 1. 中断中只负责从硬件 FIFO 取出数据并搬运到软件缓冲区。
 * 2. 任务中通过读取 Ready 缓冲区获取最新 CAN 帧。
 * 3. 使用 Active/Ready 双缓冲减少中断与任务对同一块缓冲区的竞争。
 * 4. 通过 Alive 标志位做 100ms 周期在线检测。
 */
#ifndef __DRV_CAN_H__
#define __DRV_CAN_H__

#include "main.h"
#include "stm32f4xx_hal_can.h"
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/**
 * @brief 单个 CAN 管理对象的软件接收缓冲深度
 *
 * @note
 * 该值表示 Active 或 Ready 任意一块缓冲区能够暂存的最大报文数，
 * 不是 CAN 硬件 FIFO 的深度。
 */
#define CAN_RX_BUFFER_SIZE 32

/**
 * @brief 一帧 CAN 接收消息的软件表示
 *
 * @details
 * 该结构体用于把 HAL 读出的帧头与 8 字节数据统一封装，便于中断和任务
 * 之间传递。当前驱动只处理标准帧接收场景，因此任务层通常关注
 * `rx_header.StdId` 和 `rx_data`。
 */
typedef struct
{
    CAN_RxHeaderTypeDef rx_header; /**< HAL 提供的接收帧头信息 */
    uint8_t rx_data[8];            /**< 实际收到的 8 字节负载数据 */
} CAN_RX_MESSAGE;

/**
 * @brief CAN 总线的软件管理对象
 *
 * @details
 * 一个 `Class_CAN_Manage_Object` 对应一路物理 CAN 外设。
 * 它主要负责：
 * - 维护 Active/Ready 双缓冲接收区
 * - 维护可选的任务通知句柄
 * - 提供报文读取接口
 * - 提供链路在线检测状态
 */
class Class_CAN_Manage_Object
{
public:
    CAN_HandleTypeDef *hcan; /**< 关联的 HAL CAN 句柄 */

    CAN_RX_MESSAGE Rx_Buffer_0[CAN_RX_BUFFER_SIZE]; /**< 双缓冲区 0 */
    CAN_RX_MESSAGE Rx_Buffer_1[CAN_RX_BUFFER_SIZE]; /**< 双缓冲区 1 */

    CAN_RX_MESSAGE *Rx_Buffer_Active; /**< 当前由中断写入的缓冲区 */
    uint16_t Rx_Length_Active;        /**< Active 缓冲区已写入帧数 */

    CAN_RX_MESSAGE *Rx_Buffer_Ready; /**< 当前可被任务读取的缓冲区 */
    uint16_t Rx_Length_Ready;        /**< Ready 缓冲区当前有效帧数 */

    uint32_t Drop_Count; /**< 因软件缓冲区写满导致的累计丢帧数 */

    TaskHandle_t Notify_Task_Handle; /**< Ready 缓冲区就绪后可被通知的任务句柄 */

    volatile uint32_t Alive_Flag;   /**< 每收到一帧加一，用于在线检测 */
    uint32_t Alive_Pre_Flag;        /**< 上一次 100ms 检查时记录的计数值 */
    volatile uint8_t Alive_Online;  /**< 当前在线状态，0=离线，1=在线 */
    volatile uint8_t Alive_Changed; /**< 在线状态自上次消费后是否发生变化 */

    /**
     * @brief 初始化软件管理对象的运行时状态
     * @param can_handle 对应的 HAL CAN 句柄
     *
     * @details
     * 该函数只负责建立软件对象和 HAL 句柄之间的关联，并重置双缓冲、
     * 通知句柄以及在线检测状态，不会启动硬件外设。
     */
    void Init(CAN_HandleTypeDef *can_handle);

    /**
     * @brief 按当前句柄配置 CAN 滤波器
     *
     * @details
     * 当前实现采用“全接收”策略，把收到的标准帧统一送入 FIFO0，
     * 由后续的软件层按标准 ID 再做筛选。
     */
    void FilterConfig();

    /**
     * @brief 启动 CAN 总线并打开接收中断通知
     *
     * @note
     * 该函数内部带有重复启动保护；同一路 CAN 重复调用时不会再次执行
     * `HAL_CAN_Start()`。
     */
    void Start();

    /**
     * @brief 注册 Ready 缓冲就绪时需要唤醒的任务
     * @param task_handle 任务句柄
     *
     * @details
     * 注册后，当中断接收到报文并成功发布到 Ready 缓冲区时，驱动会在
     * ISR 中使用任务通知唤醒该任务。
     */
    void RegisterNotifyTask(TaskHandle_t task_handle);

    /**
     * @brief 发送一帧标准 CAN 数据帧
     * @param send_id 标准帧 ID
     * @param data 指向 8 字节发送数据的指针
     */
    void Send(uint32_t send_id, uint8_t *data);

    /**
     * @brief CAN FIFO0 接收中断的软件处理入口
     *
     * @details
     * 该函数运行在中断上下文：
     * - 从硬件 FIFO0 读取所有待处理报文
     * - 写入 Active 缓冲区
     * - 刷新在线检测计数
     * - 在合适时机把 Active 发布为 Ready
     * - 如有需要，通知等待接收的任务
     */
    void ReceiveCallback();

    /**
     * @brief 以先进先出方式读取一帧报文
     * @param rx_message 输出报文缓存
     * @retval HAL_OK 读取成功
     * @retval HAL_ERROR 当前没有可读报文或参数无效
     */
    HAL_StatusTypeDef ReadMessage(CAN_RX_MESSAGE *rx_message);

    /**
     * @brief 按标准帧 ID 读取一帧匹配报文
     * @param std_id 目标标准帧 ID
     * @param rx_message 输出报文缓存
     * @retval HAL_OK 找到并读取成功
     * @retval HAL_ERROR 未找到匹配报文或参数无效
     *
     * @note
     * 成功读取后，匹配到的帧会从 Ready 缓冲区移除。
     */
    HAL_StatusTypeDef ReadMessageByStdId(uint32_t std_id, CAN_RX_MESSAGE *rx_message);

    /**
     * @brief 执行一次 100ms 周期的在线状态检查
     *
     * @details
     * 通过比较 `Alive_Flag` 与 `Alive_Pre_Flag` 是否增长来判断最近一个
     * 周期是否收到过数据。
     */
    void AliveCheck100ms();

    /**
     * @brief 获取当前链路在线状态
     * @retval 0 离线
     * @retval 1 在线
     */
    uint8_t AliveIsOnline() const;

    /**
     * @brief 消费一次“在线状态发生变化”的事件
     * @param online 若非空，则返回当前在线状态
     * @retval 0 自上次消费后状态无变化
     * @retval 1 自上次消费后状态发生过变化
     */
    uint8_t AliveTryConsumeChanged(uint8_t *online);

private:
    /**
     * @brief 当 Ready 缓冲区为空时，把 Active 缓冲区发布为 Ready
     *
     * @note
     * 该函数不会覆盖尚未被任务读取的 Ready 缓冲区。
     */
    void PublishActiveToReady();

    /**
     * @brief 在任务上下文读取前，必要时把 Active 提升为 Ready
     *
     * @details
     * 某些时刻中断已经收到了数据，但还未来得及通过发布逻辑把它切换成
     * Ready；读取前调用该函数可以减少数据在 Active 缓冲区滞留的时间。
     */
    void PromoteActiveToReadyIfNeeded();
};

/**
 * @brief CAN1 对应的软件管理对象
 */
extern Class_CAN_Manage_Object CAN1_Manage_Object;

/**
 * @brief CAN2 对应的软件管理对象
 */
extern Class_CAN_Manage_Object CAN2_Manage_Object;

#endif /* __DRV_CAN_H__ */
