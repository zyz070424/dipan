/**
 * @file drv_spi.h
 * @brief SPI 驱动接口与管理对象定义
 *
 * @details
 * 当前 SPI 驱动主要服务于 BMI088 的寄存器读写场景。
 * 当前版本直接以类对象形式暴露 `SPI1_Manage_Object`。
 */
#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

#include "main.h"
#include "projdefs.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"
#include <stdint.h>

/**
 * @brief SPI 逻辑设备枚举
 *
 * @details
 * BMI088 的温度寄存器属于 ACCEL 寄存器组，因此 `TEMP` 仅作为逻辑设备标识存在，
 * 片选仍然与 `ACCEL` 共用。
 */
enum SPI_Device
{
    ACCEL = 0, /**< BMI088 加速度计寄存器组 */
    GYRO = 1,  /**< BMI088 陀螺仪寄存器组 */
    TEMP = 2,  /**< BMI088 温度寄存器逻辑设备 */
};

/** @brief 拉高 ACCEL 片选引脚 */
#define ACCEL_CS_HIGH()  HAL_GPIO_WritePin(ACCEL_CSB1_GPIO_Port, ACCEL_CSB1_Pin, GPIO_PIN_SET)
/** @brief 拉低 ACCEL 片选引脚 */
#define ACCEL_CS_LOW()   HAL_GPIO_WritePin(ACCEL_CSB1_GPIO_Port, ACCEL_CSB1_Pin, GPIO_PIN_RESET)

/** @brief 拉高 GYRO 片选引脚 */
#define GYRO_CS_HIGH()   HAL_GPIO_WritePin(GYRO_CSB2_GPIO_Port, GYRO_CSB2_Pin, GPIO_PIN_SET)
/** @brief 拉低 GYRO 片选引脚 */
#define GYRO_CS_LOW()    HAL_GPIO_WritePin(GYRO_CSB2_GPIO_Port, GYRO_CSB2_Pin, GPIO_PIN_RESET)

/**
 * @brief 当前 SPI 事务上下文
 *
 * @details
 * 一次 DMA 事务启动前，会把本次访问设备、接收目标缓冲区以及有效数据长度
 * 记录在此结构体中，供中断完成回调阶段做统一收尾。
 */
typedef struct Struct_SPI_Current_Transaction
{
    uint8_t device;         /**< 当前事务访问的逻辑设备 */
    uint8_t *user_rx_buf;   /**< 用户提供的接收缓冲区 */
    uint16_t valid_size;    /**< 用户真正关心的有效字节数 */
} Struct_SPI_Current_Transaction;

/**
 * @brief SPI 软件管理对象
 *
 * @details
 * 当前工程中该驱动只维护一路 `SPI1_Manage_Object`，主要负责：
 * - 管理 DMA 同步信号量
 * - 保存一组固定长度的收发缓冲区
 * - 保存当前事务上下文
 * - 处理事务完成收尾
 * - 维护 SPI 链路在线检测状态
 */
class Class_SPI_Manage_Object
{
public:
    SPI_HandleTypeDef *hspi;                  /**< 关联的 HAL SPI 句柄 */
    SemaphoreHandle_t Done_Sem;              /**< DMA 事务完成信号量 */
    uint8_t DMA_Inited;                      /**< DMA 同步资源是否已初始化 */

    uint8_t Tx_Buffer[16];                   /**< SPI 发送缓冲区 */
    uint8_t Rx_Buffer[16];                   /**< SPI 接收缓冲区 */

    Struct_SPI_Current_Transaction Current_Transaction; /**< 当前事务上下文 */

    volatile uint8_t Transfer_State;         /**< 当前事务状态 */

    volatile uint32_t Alive_Flag;            /**< 成功事务计数 */
    uint32_t Alive_Pre_Flag;                 /**< 上一次 100ms 检查时的计数值 */
    volatile uint8_t Alive_Online;           /**< 当前在线状态，0=离线，1=在线 */
    volatile uint8_t Alive_Changed;          /**< 在线状态是否发生过变化 */

    /**
     * @brief 初始化 DMA 同步资源和软件状态
     *
     * @details
     * 该函数不会直接发起任何硬件访问，只负责建立软件侧的同步对象和清空运行时状态。
     */
    void InitDMA();

    /**
     * @brief 向指定逻辑设备写寄存器
     * @param spi_handle 当前使用的 HAL SPI 句柄
     * @param device 目标逻辑设备
     * @param reg 寄存器地址
     * @param data 待写入的 1 字节数据
     * @retval HAL_OK 写入成功
     * @retval HAL_ERROR 参数错误或事务失败
     * @retval HAL_TIMEOUT 等待 DMA 完成超时
     */
    HAL_StatusTypeDef WriteReg(SPI_HandleTypeDef *spi_handle, uint8_t device, uint8_t reg, uint8_t data);

    /**
     * @brief 从指定逻辑设备读取寄存器
     * @param spi_handle 当前使用的 HAL SPI 句柄
     * @param device 目标逻辑设备
     * @param reg 起始寄存器地址
     * @param rx_data 用户接收缓冲区
     * @param valid_size 期望读取的有效字节数
     * @retval HAL_OK 读取成功
     * @retval HAL_ERROR 参数错误或事务失败
     * @retval HAL_TIMEOUT 等待 DMA 完成超时
     */
    HAL_StatusTypeDef ReadReg(SPI_HandleTypeDef *spi_handle, uint8_t device, uint8_t reg, uint8_t *rx_data, uint16_t valid_size);

    /**
     * @brief 处理 SPI DMA 收发完成事件
     */
    void TxRxCpltCallback();

    /**
     * @brief 处理 SPI DMA 发送完成事件
     */
    void TxCpltCallback();

    /**
     * @brief 处理 SPI DMA 错误事件
     */
    void ErrorCallback();

    /**
     * @brief 执行一次 100ms 周期的在线状态检查
     */
    void AliveCheck100ms();

    /**
     * @brief 获取当前在线状态
     * @retval 0 离线
     * @retval 1 在线
     */
    uint8_t AliveIsOnline() const;

    /**
     * @brief 消费一次在线状态变化事件
     * @param online 若非空，则输出当前在线状态
     * @retval 0 无变化
     * @retval 1 有变化
     */
    uint8_t AliveTryConsumeChanged(uint8_t *online);

private:
    /**
     * @brief 判断逻辑设备编号是否有效
     * @param device 逻辑设备编号
     * @retval 1 有效
     * @retval 0 无效
     */
    uint8_t DeviceIsValid(uint8_t device) const;

    /**
     * @brief 获取指定逻辑设备读取返回数据的偏移量
     * @param device 逻辑设备编号
     * @return 有效数据在 DMA 接收缓冲区中的起始偏移
     */
    uint8_t DeviceGetReadOffset(uint8_t device) const;

    /**
     * @brief 控制指定逻辑设备的片选引脚
     * @param device 逻辑设备编号
     * @param level_high 0 表示拉低，1 表示拉高
     */
    void DeviceCSWrite(uint8_t device, uint8_t level_high);

    /**
     * @brief 判断当前是否处于中断上下文
     * @retval 1 当前位于 ISR 中
     * @retval 0 当前位于任务上下文中
     */
    uint8_t IsInISR() const;

    /**
     * @brief 清空 DMA 完成信号量中的残留计数
     */
    void ClearDoneSemaphore();

    /**
     * @brief 在事务成功时喂一次在线检测计数
     */
    void AliveFeed();

    /**
     * @brief 等待当前 DMA 事务完成
     * @retval HAL_OK 事务成功
     * @retval HAL_ERROR 事务失败
     * @retval HAL_TIMEOUT 等待超时
     */
    HAL_StatusTypeDef WaitTransferDone();

    /**
     * @brief 在中断上下文中统一完成事务收尾
     * @param transfer_ok 1 表示事务成功，0 表示事务失败
     * @param copy_rx 1 表示需要把接收数据拷贝到用户缓冲区
     */
    void TransferFinishFromISR(uint8_t transfer_ok, uint8_t copy_rx);
};

/**
 * @brief SPI1 对应的软件管理对象
 */
extern Class_SPI_Manage_Object SPI1_Manage_Object;

#endif /* __DRV_SPI_H__ */
