#include "bmi088.h"
#include "drv_spi.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <stdint.h>

/** 
 * @brief  BMI088 初始化函数
 * @param  hspi: SPI句柄
 * @retval 无
 * @note 还没有写错误处理
 * @note 还没有写错误处理
 * @note 还没有写错误处理
 * @note 还没有写错误处理
 * @note 还没有写错误处理
 */
void BMI088_Init(SPI_HandleTypeDef *hspi)
{
    SPI_DMA_Init(); // 初始化 SPI DMA 和相关同步对象
    //假读以确认通信正常
    uint8_t ACC_ID = 0;
    SPI_ReadReg(hspi, ACCEL, BMI088_REG_ACCEL_CHIP_ID, &ACC_ID, 1);

    SPI_WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_SOFTRESET, BMI088_SOFT_RESET_CMD);
    SPI_WriteReg(hspi, GYRO, BMI088_REG_GYRO_SOFTRESET, BMI088_SOFT_RESET_CMD);
    vTaskDelay(100);
    //配置加速度计
    SPI_WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_PWR_CTRL, BMI088_ACCEL_PWR_ENABLE);
    vTaskDelay(50);

    SPI_WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_CONF, BMI088_ACCEL_CONF_1600HZ_NORMAL);
    SPI_WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_RANGE,BMI088_ACCEL_RANGE_24G);

    SPI_WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_INT1_IO_CONF, BMI088_ACCEL_INT_CFG_PUSH_PULL_HIGH);
    SPI_WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_INT_MAP_DATA, BMI088_ACCEL_INT1_DRDY_MAP);

    //配置陀螺仪
    SPI_WriteReg(hspi, GYRO, BMI088_REG_GYRO_BANDWIDTH, BMI088_GYRO_BW_532_HZ);
    SPI_WriteReg(hspi, GYRO, BMI088_REG_GYRO_RANGE, BMI088_GYRO_RANGE_2000_DPS);
    SPI_WriteReg(hspi, GYRO, BMI088_REG_GYRO_INT_CTRL, BMI088_GYRO_INT_DRDY_ENABLE);

    SPI_WriteReg(hspi, GYRO, BMI088_REG_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_CFG_PUSH_PULL_HIGH);
    SPI_WriteReg(hspi, GYRO, BMI088_REG_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_INT3_DRDY_MAP);
}
/**
 * @brief  BMI088 读取加速度计数据函数
 * @param  hspi: SPI句柄
 * @param  data: 存储加速度计数据的结构体指针
 * @retval 无
 */
void BMI088_ReadAccel(SPI_HandleTypeDef *hspi, imu_data_t *data)
{
    uint8_t accel_data_raw[6];
    int16_t accel_data[3];
    SPI_ReadReg(hspi, ACCEL, BMI088_REG_ACCEL_X_LSB, accel_data_raw, 6);
    accel_data[0] = (int16_t)((uint16_t)accel_data_raw[1] << 8 | accel_data_raw[0]);
    accel_data[1] = (int16_t)((uint16_t)accel_data_raw[3] << 8 | accel_data_raw[2]);
    accel_data[2] = (int16_t)((uint16_t)accel_data_raw[5] << 8 | accel_data_raw[4]);
    data->acc.x = accel_data[0] / BMI088_ACCEL_SENSITIVITY_24G;
    data->acc.y = accel_data[1] / BMI088_ACCEL_SENSITIVITY_24G;
    data->acc.z = accel_data[2] / BMI088_ACCEL_SENSITIVITY_24G;
}
/**
 * @brief  BMI088 读取陀螺仪数据函数
 * @param  hspi: SPI句柄
 * @param  data: 存储陀螺仪数据的结构体指针
 * @retval 无
 */
void BMI088_ReadGyro(SPI_HandleTypeDef *hspi, imu_data_t *data)
{
    uint8_t gyro_data_raw[6];
    int16_t gyro_data[3];
    SPI_ReadReg(hspi, GYRO, BMI088_REG_GYRO_X_LSB, gyro_data_raw, 6);
    gyro_data[0] = (int16_t)((uint16_t)gyro_data_raw[1] << 8 | gyro_data_raw[0]);
    gyro_data[1] = (int16_t)((uint16_t)gyro_data_raw[3] << 8 | gyro_data_raw[2]);
    gyro_data[2] = (int16_t)((uint16_t)gyro_data_raw[5] << 8 | gyro_data_raw[4]);
    data->gyro.x = gyro_data[0] / BMI088_GYRO_SENSITIVITY_2000_DPS;
    data->gyro.y = gyro_data[1] / BMI088_GYRO_SENSITIVITY_2000_DPS;
    data->gyro.z = gyro_data[2] / BMI088_GYRO_SENSITIVITY_2000_DPS;
}  
/**
 * @brief  BMI088 读取温度数据函数
 * @param  hspi: SPI句柄
 * @param  data: 存储温度数据的结构体指针
 * @retval 无
 */
void BMI088_ReadTemp(SPI_HandleTypeDef *hspi, imu_data_t *data)
{
    uint8_t temp_data_raw[2];
    int16_t temp_data;
    SPI_ReadReg(hspi, ACCEL, BMI088_REG_ACCEL_TEMP_LSB, temp_data_raw, 2);
    temp_data = (int16_t)((uint16_t)temp_data_raw[1] << 8 | temp_data_raw[0]);
    data->temp = (temp_data / 326.8f) + 23.0f;



}
/**
 * @brief  BMI088 互补滤波函数
 * @param  hspi: SPI句柄
 * @param  data: 存储IMU数据的结构体指针
 * @param  dt: 采样时间（秒）
 * @param  kp: 比例增益
 * @param  ki: 积分增益
 * @retval euler_t 输出欧拉角（角度）
 */
euler_t BMI088_Complementary_Filter(imu_data_t *data, float dt, float kp, float ki)
{

    static quat_t q = {1.0f, 0.0f, 0.0f, 0.0f};
    data->dt = dt;
    data->gyro.x *= (M_PI / 180.0f);
    data->gyro.y *= (M_PI / 180.0f);
    data->gyro.z *= (M_PI / 180.0f);
    mahony_update(&q, *data, kp, ki);
    euler_t euler = quat_to_euler(q);
    return euler;
}