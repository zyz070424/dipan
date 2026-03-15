#include "drv_spi.h"
#include "projdefs.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"
#include <stdint.h>

// --- 1. FreeRTOS 同步对象 ---
static SemaphoreHandle_t SPI_Done_Sem; // 用于等待 DMA 完成
static SemaphoreHandle_t SPI_Bus_Mutex;    // 用于保护 SPI 总线竞争

// --- 2. 静态缓冲区 (避免栈溢出) ---
static uint8_t SPI_Tx_Buffer[16]; 
static uint8_t SPI_Rx_Buffer[16];
// --- 3. 临时全局上下文 (仅在 DMA 传输期间有效，受 Mutex 保护) ---
typedef struct {
    uint8_t device;
    uint8_t *user_rx_buf; // 用于加速度计去 Dummy
    uint16_t valid_size;  // 用于加速度计去 Dummy
    uint8_t device_type;  //ACCEL or GYRO
} SPI_Current_Transaction_t;

static SPI_Current_Transaction_t g_current_txn;

// --- 初始化函数 (在 main.c 或 SPI 初始化时调用一次) ---
/*
 * @brief  SPI DMA 初始化函数
 * @param  无
 * @retval 无
 */
void SPI_DMA_Init(void) 
{
    
    SPI_Done_Sem = xSemaphoreCreateCounting(1, 0); // 初始为 0，表示未完成
    SPI_Bus_Mutex = xSemaphoreCreateMutex();
}
static void SPI_CS_High(uint8_t device)
{
    switch(device)
    {
    case ACCEL:
        ACCEL_CS_HIGH(); 
        break;
    case GYRO:
        GYRO_CS_HIGH(); 
        break;
    default:
        break;           
    }
}
/*
 * @brief  SPI CS引脚低电平
 * @param  device: 设备类型 (ACCEL, GYRO, TEMP)
 * @retval 无
 */
static void SPI_CS_Low(uint8_t device)
{
    switch(device)
    {
    case ACCEL:
        ACCEL_CS_LOW();  
        break;
    case GYRO:
        GYRO_CS_LOW();   
        break;
    default:
        break;          
    }
}
/*
 * @brief  SPI 读寄存器函数 (单寄存器或连续寄存器)
 * @param  hspi: SPI句柄
 * @param  device: 设备类型 (ACCEL, GYRO, TEMP)
 * @param  reg: 寄存器地址
 * @param  rx_data: 用户提供的接收缓冲区指针
 * @param  valid_size: 用户期望的有效数据字节数（不包括加速度计的 Dummy 字节）
 * @retval HAL 状态码
 */
HAL_StatusTypeDef SPI_ReadReg(SPI_HandleTypeDef *hspi, uint8_t device, uint8_t reg, uint8_t *rx_data, uint16_t valid_size)
{
     // 超时时间设为 100ms，防止死锁
    if (xSemaphoreTake(SPI_Bus_Mutex, 100) != pdTRUE) 
    {
        return HAL_BUSY; 
    }

    uint16_t total_len;
    if (device == ACCEL) 
    {
        total_len = valid_size + 2; // 加速度计 
    } 
    else 
    {
        total_len = valid_size + 1; // 陀螺仪
    }

    if (total_len > sizeof(SPI_Tx_Buffer)) 
    {
        xSemaphoreGive(SPI_Bus_Mutex);
        return HAL_ERROR;
    }

    //  构造 TX 数据
    SPI_Tx_Buffer[0] = reg | 0x80;
    for (int i = 1; i < total_len; i++) 
    {
        SPI_Tx_Buffer[i] = 0x00;
    }

    // 设置当前事务上下文
    g_current_txn.device = device;
    g_current_txn.device_type = device;
    g_current_txn.user_rx_buf = rx_data;
    g_current_txn.valid_size = valid_size;
    
    // 拉低 CS
    SPI_CS_Low(device);

    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive_DMA(hspi, SPI_Tx_Buffer, SPI_Rx_Buffer, total_len);

    if (ret != HAL_OK) 
    {
        SPI_CS_High(device);
        xSemaphoreGive(SPI_Bus_Mutex); // 释放锁
        return ret;
    }

    //等待 DMA 完成 (阻塞当前任务，让出 CPU 给其他任务)
    // 超时设为 50ms
    if (xSemaphoreTake(SPI_Done_Sem, 50) != pdTRUE) 
    {
        // 超时说明 DMA 卡死了
        SPI_CS_High(g_current_txn.device);
        xSemaphoreGive(SPI_Bus_Mutex);
        return HAL_TIMEOUT;
    }

    xSemaphoreGive(SPI_Bus_Mutex);

    return HAL_OK;
}

/*
 * @brief  SPI 读连续寄存器函数（简化版，直接调用 SPI_ReadReg）
 * @param  hspi: SPI句柄
 * @param  device: 设备类型 (ACCEL, GYRO, TEMP)
 * @param  reg: 起始寄存器地址
 * @param  rx_data: 用户提供的接收缓冲区指针
 * @param  valid_size: 用户期望的有效数据字节数（不包括加速度计的 Dummy 字节）
 * @retval HAL 状态码
 */
HAL_StatusTypeDef SPI_WriteReg(SPI_HandleTypeDef *hspi, uint8_t device, uint8_t reg, uint8_t data)
{
    if (xSemaphoreTake(SPI_Bus_Mutex, 100) != pdTRUE)
    {
        return HAL_BUSY;
    }

    static uint8_t tx_buf[2];
    tx_buf[0] = reg & 0x7F;
    tx_buf[1] = data;

    //保留上下问
    g_current_txn.device = device;
    SPI_CS_Low(device);

    HAL_StatusTypeDef ret = HAL_SPI_Transmit_DMA(hspi, tx_buf, 2);
    if (ret != HAL_OK) 
    {
        SPI_CS_High(device);
        xSemaphoreGive(SPI_Bus_Mutex);
        return ret;
    }

    if (xSemaphoreTake(SPI_Done_Sem, 50) != pdTRUE) 
    {
        SPI_CS_High(g_current_txn.device);
        xSemaphoreGive(SPI_Bus_Mutex);
        return HAL_TIMEOUT;
    }

    xSemaphoreGive(SPI_Bus_Mutex);
    return HAL_OK;
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) 
    {
        // 1. 拉高 CS (使用全局上下文中的信息)
        SPI_CS_High(g_current_txn.device);

        // 2. 【关键】如果是加速度计，剔除 Dummy
        if (g_current_txn.device_type == ACCEL) 
        {
            uint8_t *raw = g_current_txn.user_rx_buf;
            uint16_t len = g_current_txn.valid_size;
            for (int i = 0; i < len; i++) 
            {
                raw[i] = SPI_Rx_Buffer[i + 2]; // 跳过前两个 Dummy 字节
            }
        }
        else if (g_current_txn.device_type == GYRO)
        {
            uint8_t *raw = g_current_txn.user_rx_buf;
            uint16_t len = g_current_txn.valid_size;
            for (int i = 0; i < len; i++) 
            {
                raw[i] = SPI_Rx_Buffer[i + 1]; // 跳过前一个 Dummy 字节
            }
        }
        
        xSemaphoreGiveFromISR(SPI_Done_Sem, NULL);
        
       
        
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) 
    {
        SPI_CS_High(g_current_txn.device);
        
        xSemaphoreGiveFromISR(SPI_Done_Sem, NULL);
    }
}
