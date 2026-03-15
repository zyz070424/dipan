#ifndef __BMI088_H__
#define __BMI088_H__
#include "drv_spi.h"
#include "alg_quaternion.h"


void BMI088_Init(SPI_HandleTypeDef *hspi);
void BMI088_ReadAccel(SPI_HandleTypeDef *hspi, imu_data_t *data);
void BMI088_ReadGyro(SPI_HandleTypeDef *hspi, imu_data_t *data);
void BMI088_ReadTemp(SPI_HandleTypeDef *hspi, imu_data_t *data);
euler_t BMI088_Complementary_Filter(imu_data_t *data, float dt, float kp, float ki);


//数据处理
#define BMI088_ACCEL_SENSITIVITY_24G 1365.0f
#define BMI088_ACCEL_SENSITIVITY_12G 2730.0f
#define BMI088_ACCEL_SENSITIVITY_6G  5460.0f
#define BMI088_ACCEL_SENSITIVITY_3G  10920.0f

// --- 陀螺仪 ---
#define BMI088_GYRO_SENSITIVITY_2000_DPS 16.384f
#define BMI088_GYRO_SENSITIVITY_1000_DPS 32.8f
#define BMI088_GYRO_SENSITIVITY_500_DPS  65.6f
#define BMI088_GYRO_SENSITIVITY_250_DPS  131.2f
#define BMI088_GYRO_SENSITIVITY_125_DPS  262.4f



// --- 芯片 ID ---
#define BMI088_ACCEL_CHIP_ID        0x1E
#define BMI088_GYRO_CHIP_ID         0x0F

// --- 软复位命令 ---
#define BMI088_SOFT_RESET_CMD       0xB6
//ACCEL
/// --- 地址定义 ---
#define BMI088_REG_ACCEL_CHIP_ID    0x00
#define BMI088_REG_ACCEL_ERR_REG    0x02
#define BMI088_REG_ACCEL_STATUS     0x03
#define BMI088_REG_ACCEL_X_LSB      0x12
#define BMI088_REG_ACCEL_X_MSB      0x13
#define BMI088_REG_ACCEL_Y_LSB      0x14
#define BMI088_REG_ACCEL_Y_MSB      0x15
#define BMI088_REG_ACCEL_Z_LSB      0x16
#define BMI088_REG_ACCEL_Z_MSB      0x17
#define BMI088_REG_ACCEL_SENSORTIME_0 0x18
#define BMI088_REG_ACCEL_SENSORTIME_1 0x19
#define BMI088_REG_ACCEL_SENSORTIME_2 0x1A
#define BMI088_REG_ACCEL_INT_STAT_1 0x1D
#define BMI088_REG_ACCEL_TEMP_LSB   0x22
#define BMI088_REG_ACCEL_TEMP_MSB   0x23
#define BMI088_REG_ACCEL_CONF       0x40
#define BMI088_REG_ACCEL_RANGE      0x41
#define BMI088_REG_ACCEL_INT1_IO_CONF 0x53
#define BMI088_REG_ACCEL_INT2_IO_CONF 0x54
#define BMI088_REG_ACCEL_INT_MAP_DATA 0x58
#define BMI088_REG_ACCEL_SELF_TEST  0x6D
#define BMI088_REG_ACCEL_PWR_CONF   0x7C
#define BMI088_REG_ACCEL_PWR_CTRL   0x7D
#define BMI088_REG_ACCEL_SOFTRESET  0x7E

// --- 配置值宏 (ACC_CONF 0x40) ---
// 格式: [3:0] ODR, [7:4] 带宽/滤波模式
// ODR: 0xC=1600Hz, 0xB=800Hz, 0xA=400Hz, 0x9=200Hz, 0x8=100Hz, 0x7=50Hz...
// 带宽: 0x8=OSR4, 0x9=OSR2, 0xA=Normal, 0xB=CIC
#define BMI088_ACCEL_ODR_1600_HZ    0x0C
#define BMI088_ACCEL_ODR_800_HZ     0x0B
#define BMI088_ACCEL_ODR_400_HZ     0x0A
#define BMI088_ACCEL_ODR_200_HZ     0x09
#define BMI088_ACCEL_ODR_100_HZ     0x08
#define BMI088_ACCEL_ODR_50_HZ      0x07

#define BMI088_ACCEL_BWP_NORMAL     0xA0 // Normal mode (推荐)
#define BMI088_ACCEL_BWP_OSR4       0x80 // OSR4
#define BMI088_ACCEL_BWP_OSR2       0x90 // OSR2
#define BMI088_ACCEL_BWP_CIC        0xB0 // CIC

// 常用组合示例: 1600Hz + Normal Mode
#define BMI088_ACCEL_CONF_1600HZ_NORMAL  (BMI088_ACCEL_ODR_1600_HZ | BMI088_ACCEL_BWP_NORMAL)

// --- 量程宏 (ACC_RANGE 0x41) ---
#define BMI088_ACCEL_RANGE_3G       0x00
#define BMI088_ACCEL_RANGE_6G       0x01
#define BMI088_ACCEL_RANGE_12G      0x02
#define BMI088_ACCEL_RANGE_24G      0x03

// --- 电源控制宏 (ACC_PWR_CTRL 0x7D) ---
#define BMI088_ACCEL_PWR_ENABLE     0x04
#define BMI088_ACCEL_PWR_DISABLE    0x00

// --- 中断映射宏 (INT_MAP_DATA 0x58) ---
#define BMI088_ACCEL_INT1_DRDY_MAP  0x04 // Map Data Ready to INT1
#define BMI088_ACCEL_INT2_DRDY_MAP  0x40 // Map Data Ready to INT2

// --- 中断引脚配置宏 (INTx_IO_CONF 0x53/0x54) ---
// Bit3: Output Enable, Bit2: Open Drain(0=PushPull), Bit1: Level(1=High, 0=Low)
#define BMI088_ACCEL_INT_CFG_PUSH_PULL_HIGH  0x0A // En=1, OD=0, Lvl=1
#define BMI088_ACCEL_INT_CFG_PUSH_PULL_LOW   0x08 // En=1, OD=0, Lvl=0

//GYRO
// --- 地址定义 ---
#define BMI088_REG_GYRO_CHIP_ID     0x00
#define BMI088_REG_GYRO_X_LSB       0x02
#define BMI088_REG_GYRO_X_MSB       0x03
#define BMI088_REG_GYRO_Y_LSB       0x04
#define BMI088_REG_GYRO_Y_MSB       0x05
#define BMI088_REG_GYRO_Z_LSB       0x06
#define BMI088_REG_GYRO_Z_MSB       0x07
#define BMI088_REG_GYRO_INT_STAT_1  0x0A
#define BMI088_REG_GYRO_RANGE       0x0F
#define BMI088_REG_GYRO_BANDWIDTH   0x10
#define BMI088_REG_GYRO_LPM1        0x11
#define BMI088_REG_GYRO_SOFTRESET   0x14
#define BMI088_REG_GYRO_INT_CTRL    0x15
#define BMI088_REG_GYRO_INT3_INT4_IO_CONF 0x16
#define BMI088_REG_GYRO_INT3_INT4_IO_MAP  0x18
#define BMI088_REG_GYRO_SELF_TEST   0x3C

// --- 带宽与 ODR 宏 (GYRO_BANDWIDTH 0x10) ---
// 高 4 位固定为 1 (只读)，低 4 位配置 ODR/BW
// 0x00: 2000Hz/532Hz, 0x01: 2000Hz/230Hz, 0x02: 1000Hz/116Hz, 0x03: 400Hz/47Hz...
#define BMI088_GYRO_BW_532_HZ       0x80 // ODR 2000Hz
#define BMI088_GYRO_BW_230_HZ       0x81 // ODR 2000Hz
#define BMI088_GYRO_BW_116_HZ       0x82 // ODR 1000Hz
#define BMI088_GYRO_BW_47_HZ        0x83 // ODR 400Hz (常用)
#define BMI088_GYRO_BW_23_HZ        0x84 // ODR 200Hz
#define BMI088_GYRO_BW_12_HZ        0x85 // ODR 100Hz
#define BMI088_GYRO_BW_64_HZ        0x87 // ODR 400Hz (不同滤波)

// --- 量程宏 (GYRO_RANGE 0x0F) ---
#define BMI088_GYRO_RANGE_2000_DPS  0x00
#define BMI088_GYRO_RANGE_1000_DPS  0x01
#define BMI088_GYRO_RANGE_500_DPS   0x02
#define BMI088_GYRO_RANGE_250_DPS   0x03
#define BMI088_GYRO_RANGE_125_DPS   0x04

// --- 低功耗模式宏 (GYRO_LPM1 0x11) ---
#define BMI088_GYRO_MODE_NORMAL     0x00
#define BMI088_GYRO_MODE_DEEP_SUSPEND 0x20
#define BMI088_GYRO_MODE_SUSPEND    0x80

// --- 中断控制宏 (GYRO_INT_CTRL 0x15) ---
#define BMI088_GYRO_INT_DRDY_ENABLE 0x80 // Bit 7: Enable Data Ready Interrupt

// --- 中断引脚配置宏 (INT3_INT4_IO_CONF 0x16) ---
// Bit0: Int3 Level, Bit1: Int3 OD, Bit2: Int4 Level, Bit3: Int4 OD
// 示例: Int3 Push-Pull High Active
#define BMI088_GYRO_INT3_CFG_PUSH_PULL_HIGH 0x01 
// 示例: Int4 Push-Pull High Active
#define BMI088_GYRO_INT4_CFG_PUSH_PULL_HIGH 0x04 

// --- 中断映射宏 (INT3_INT4_IO_MAP 0x18) ---
#define BMI088_GYRO_INT3_DRDY_MAP   0x01 // Map Data Ready to INT3
#define BMI088_GYRO_INT4_DRDY_MAP   0x80 // Map Data Ready to INT4
#define BMI088_GYRO_INT3_INT4_DRDY_MAP 0x81 // Map to Both

#endif /* __BMI088_H__ */