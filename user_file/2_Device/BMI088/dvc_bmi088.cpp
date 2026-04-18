/**
 * @file dvc_bmi088.cpp
 * @brief BMI088 IMU 设备对象实现
 *
 * @details
 * 本文件保持原工程中的寄存器配置流程和量程换算不变，
 * 仅将 BMI088 自身持有的跨周期状态收进 `Class_BMI088`：
 * - 姿态解算使用的四元数
 * - Yaw 连续展开状态
 * - 当前绑定的 SPI 句柄
 */
#include "dvc_bmi088.h"
#include "task.h"

#include <math.h>

#define BMI088_OK_OR_RETURN(x) do { if ((x) != HAL_OK) return HAL_ERROR; } while (0)

Class_BMI088 BMI088_Manage_Object;

/**
 * @brief 更新 BMI088 对象当前绑定的 SPI 句柄
 * @param spi_handle SPI 句柄
 */
void Class_BMI088::SetSPIHandle(SPI_HandleTypeDef *spi_handle)
{
    if (spi_handle != NULL)
    {
        hspi = spi_handle;
    }
}

/**
 * @brief 根据系统当前状态执行毫秒级延时
 * @param ms 延时毫秒数
 */
void Class_BMI088::DelayMs(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        HAL_Delay(ms);
    }
    else
    {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
}

/**
 * @brief 将角度限制到 [-180, 180) 区间
 * @param angle_deg 输入角度，单位 deg
 * @return 限幅后的角度
 */
float Class_BMI088::Wrap180(float angle_deg)
{
    while (angle_deg >= 180.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

/**
 * @brief 将 Yaw 连续展开状态恢复到初始值
 */
void Class_BMI088::YawContinuousReset()
{
    yaw_continuous_inited = 0u;
    yaw_zero_raw_deg = 0.0f;
    yaw_last_rel_wrapped_deg = 0.0f;
    yaw_continuous_deg = 0.0f;
}

/**
 * @brief 重置 BMI088 对象内部状态
 *
 * @details
 * 该函数只重置 BMI088 对象自己持有的四元数与连续 yaw 状态，
 * 不额外修改 Mahony 滤波器模块的其它全局观测量。
 */
void Class_BMI088::ResetState()
{
    quat.w = 1.0f;
    quat.x = 0.0f;
    quat.y = 0.0f;
    quat.z = 0.0f;
    YawContinuousReset();
}

/**
 * @brief 将原始 Yaw 角转换为连续展开角
 * @param raw_yaw_deg 原始 Yaw 角，单位 deg
 * @return 连续展开后的 Yaw 角
 */
float Class_BMI088::YawToContinuous(float raw_yaw_deg)
{
    float yaw_rel_wrapped_deg;
    float dyaw_deg;

    if (isfinite(raw_yaw_deg) == 0)
    {
        return yaw_continuous_deg;
    }

    if (yaw_continuous_inited == 0u)
    {
        yaw_zero_raw_deg = raw_yaw_deg;
        yaw_last_rel_wrapped_deg = 0.0f;
        yaw_continuous_deg = 0.0f;
        yaw_continuous_inited = 1u;
        return 0.0f;
    }

    yaw_rel_wrapped_deg = Wrap180(raw_yaw_deg - yaw_zero_raw_deg);
    dyaw_deg = yaw_rel_wrapped_deg - yaw_last_rel_wrapped_deg;

    if (dyaw_deg > 180.0f)
    {
        dyaw_deg -= 360.0f;
    }
    else if (dyaw_deg < -180.0f)
    {
        dyaw_deg += 360.0f;
    }

    yaw_continuous_deg += dyaw_deg;
    yaw_last_rel_wrapped_deg = yaw_rel_wrapped_deg;

    return yaw_continuous_deg;
}

/**
 * @brief 初始化 BMI088 设备对象
 * @param spi_handle SPI 句柄
 * @retval HAL_StatusTypeDef 初始化结果
 */
HAL_StatusTypeDef Class_BMI088::Init(SPI_HandleTypeDef *spi_handle)
{
    uint8_t acc_id = 0;
    uint8_t gyro_id = 0;
    uint8_t dummy = 0;

    if (spi_handle == NULL)
    {
        return HAL_ERROR;
    }

    SetSPIHandle(spi_handle);
    ResetState();

    SPI1_Manage_Object.InitDMA();

    /* 先对两个传感器都进行软重置。 */
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_SOFTRESET, BMI088_SOFT_RESET_CMD));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, GYRO, BMI088_REG_GYRO_SOFTRESET, BMI088_SOFT_RESET_CMD));
    DelayMs(50);

    /*
     * 上电/复位后，ACC 以 I2C 模式启动。
     * 第一次 SPI 读取用于切换接口，可能会返回无效数据。
     */
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.ReadReg(hspi, ACCEL, BMI088_REG_ACCEL_CHIP_ID, &dummy, 1));
    DelayMs(1);

    /* 验证重置后的芯片 ID。 */
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.ReadReg(hspi, ACCEL, BMI088_REG_ACCEL_CHIP_ID, &acc_id, 1));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.ReadReg(hspi, GYRO, BMI088_REG_GYRO_CHIP_ID, &gyro_id, 1));
    if ((acc_id != BMI088_ACCEL_CHIP_ID) || (gyro_id != BMI088_GYRO_CHIP_ID))
    {
        return HAL_ERROR;
    }

    /* 加速度计电源和配置。 */
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_PWR_CONF, 0x00));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_PWR_CTRL, BMI088_ACCEL_PWR_ENABLE));
    DelayMs(10);

    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_CONF, BMI088_ACCEL_CONF_1600HZ_NORMAL));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_RANGE, BMI088_ACCEL_RANGE_24G));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_INT1_IO_CONF, BMI088_ACCEL_INT_CFG_PUSH_PULL_HIGH));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, ACCEL, BMI088_REG_ACCEL_INT_MAP_DATA, BMI088_ACCEL_INT1_DRDY_MAP));

    /* 陀螺仪配置。 */
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, GYRO, BMI088_REG_GYRO_LPM1, BMI088_GYRO_MODE_NORMAL));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, GYRO, BMI088_REG_GYRO_BANDWIDTH, BMI088_GYRO_BW_532_HZ));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, GYRO, BMI088_REG_GYRO_RANGE, BMI088_GYRO_RANGE_2000_DPS));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, GYRO, BMI088_REG_GYRO_INT_CTRL, BMI088_GYRO_INT_DRDY_ENABLE));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, GYRO, BMI088_REG_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_CFG_PUSH_PULL_HIGH));
    BMI088_OK_OR_RETURN(SPI1_Manage_Object.WriteReg(hspi, GYRO, BMI088_REG_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_INT3_DRDY_MAP));

    return HAL_OK;
}

/**
 * @brief 读取 BMI088 加速度计数据
 * @param data 加速度输出结构体
 */
void Class_BMI088::ReadAccel(imu_data_t *data)
{
    uint8_t accel_data_raw[6];
    int16_t accel_data[3];

    if ((hspi == NULL) || (data == NULL))
    {
        return;
    }

    if (SPI1_Manage_Object.ReadReg(hspi, ACCEL, BMI088_REG_ACCEL_X_LSB, accel_data_raw, 6) != HAL_OK)
    {
        return;
    }

    accel_data[0] = (int16_t)((uint16_t)accel_data_raw[1] << 8 | accel_data_raw[0]);
    accel_data[1] = (int16_t)((uint16_t)accel_data_raw[3] << 8 | accel_data_raw[2]);
    accel_data[2] = (int16_t)((uint16_t)accel_data_raw[5] << 8 | accel_data_raw[4]);

    data->acc.x = accel_data[0] / BMI088_ACCEL_SENSITIVITY_24G;
    data->acc.y = accel_data[1] / BMI088_ACCEL_SENSITIVITY_24G;
    data->acc.z = accel_data[2] / BMI088_ACCEL_SENSITIVITY_24G;
}

/**
 * @brief 读取 BMI088 陀螺仪数据
 * @param data 陀螺仪输出结构体
 */
void Class_BMI088::ReadGyro(imu_data_t *data)
{
    uint8_t gyro_data_raw[6];
    int16_t gyro_data[3];

    if ((hspi == NULL) || (data == NULL))
    {
        return;
    }

    if (SPI1_Manage_Object.ReadReg(hspi, GYRO, BMI088_REG_GYRO_X_LSB, gyro_data_raw, 6) != HAL_OK)
    {
        return;
    }

    gyro_data[0] = (int16_t)((uint16_t)gyro_data_raw[1] << 8 | gyro_data_raw[0]);
    gyro_data[1] = (int16_t)((uint16_t)gyro_data_raw[3] << 8 | gyro_data_raw[2]);
    gyro_data[2] = (int16_t)((uint16_t)gyro_data_raw[5] << 8 | gyro_data_raw[4]);

    data->gyro.x = gyro_data[0] / BMI088_GYRO_SENSITIVITY_2000_DPS;
    data->gyro.y = gyro_data[1] / BMI088_GYRO_SENSITIVITY_2000_DPS;
    data->gyro.z = gyro_data[2] / BMI088_GYRO_SENSITIVITY_2000_DPS;
}

/**
 * @brief 读取 BMI088 温度
 * @param data 温度输出结构体
 */
void Class_BMI088::ReadTemp(imu_data_t *data)
{
    uint8_t temp_data_raw[2];
    int16_t temp_data;

    if ((hspi == NULL) || (data == NULL))
    {
        return;
    }

    if (SPI1_Manage_Object.ReadReg(hspi, ACCEL, BMI088_REG_ACCEL_TEMP_LSB, temp_data_raw, 2) != HAL_OK)
    {
        return;
    }

    temp_data = (int16_t)((uint16_t)temp_data_raw[1] << 8 | temp_data_raw[0]);
    data->temp = (temp_data / 326.8f) + 23.0f;
}

/**
 * @brief 执行一次 BMI088 姿态融合并输出欧拉角
 * @param data IMU 输入数据
 * @param dt 时间步长，单位 s
 * @param kp Mahony 比例增益
 * @param ki Mahony 积分增益
 * @return 当前欧拉角，单位 deg
 */
euler_t Class_BMI088::ComplementaryFilter(imu_data_t *data, float dt, float kp, float ki)
{
    euler_t euler = {0};
    imu_data_t imu;

    if (data == NULL)
    {
        return euler;
    }

    /* 使用本地副本避免改写原始传感器数据。 */
    imu = *data;
    imu.dt = dt;

    Quaternion_Filter.MahonyUpdate(&quat, imu, kp, ki);
    euler = Quaternion_Filter.QuatToEuler(quat);
    euler.yaw = YawToContinuous(euler.yaw);

    return euler;
}
