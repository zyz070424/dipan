/**
 * @file dvc_bmi088.h
 * @brief BMI088 IMU 设备对象定义与对外接口
 *
 * @details
 * 本文件定义一个轻量 BMI088 设备类，用来收纳 BMI088 模块真正跨周期保存的状态：
 * - 当前绑定的 SPI 句柄
 * - 姿态解算使用的四元数
 * - Yaw 连续展开状态
 */
#ifndef __BMI088_H__
#define __BMI088_H__

#include "drv_spi.h"
#include "alg_quaternion.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <stdint.h>

/**
 * @brief BMI088 设备对象
 *
 * @details
 * 该对象是当前工程中对 BMI088 设备层最贴合的类化形式。
 * 它保留原有寄存器读写和姿态融合逻辑不变，只把原先散落在文件级 `static`
 * 变量中的内部状态收进对象本身，而不越层去承担上层 Posture 模块的职责。
 */
class Class_BMI088
{
public:
    SPI_HandleTypeDef *hspi = NULL; /**< 当前绑定的 SPI 句柄 */

    /**
     * @brief 更新当前绑定的 SPI 句柄
     * @param spi_handle SPI 句柄
     *
     * @details
     * 该接口主要供旧 C 风格桥接函数使用，用来兼容仍然传入 `SPI_HandleTypeDef *`
     * 的原有调用方式。
     */
    void SetSPIHandle(SPI_HandleTypeDef *spi_handle);

    /**
     * @brief 初始化 BMI088 设备
     * @param spi_handle SPI 句柄
     * @retval HAL_StatusTypeDef 初始化结果
     */
    HAL_StatusTypeDef Init(SPI_HandleTypeDef *spi_handle);

    /**
     * @brief 读取加速度计数据
     * @param data 输出 IMU 数据结构体
     */
    void ReadAccel(imu_data_t *data);

    /**
     * @brief 读取陀螺仪数据
     * @param data 输出 IMU 数据结构体
     */
    void ReadGyro(imu_data_t *data);

    /**
     * @brief 读取温度数据
     * @param data 输出 IMU 数据结构体
     */
    void ReadTemp(imu_data_t *data);

    /**
     * @brief 重置连续 Yaw 展开状态
     */
    void YawContinuousReset();

    /**
     * @brief 执行一次姿态融合并输出欧拉角
     * @param data 输入 IMU 数据
     * @param dt 时间步长，单位 s
     * @param kp Mahony 比例增益
     * @param ki Mahony 积分增益
     * @return 欧拉角结果，单位 deg
     */
    euler_t ComplementaryFilter(imu_data_t *data, float dt, float kp, float ki);

private:
    quat_t quat = {1.0f, 0.0f, 0.0f, 0.0f}; /**< 当前姿态四元数 */
    uint8_t yaw_continuous_inited = 0u;     /**< 连续 Yaw 状态是否已完成首帧初始化 */
    float yaw_zero_raw_deg = 0.0f;          /**< 连续 Yaw 的零点参考角 */
    float yaw_last_rel_wrapped_deg = 0.0f;  /**< 上一周期相对 Yaw 包裹角 */
    float yaw_continuous_deg = 0.0f;        /**< 连续展开后的 Yaw 角 */

    /**
     * @brief 按当前系统运行状态执行毫秒延时
     * @param ms 延时毫秒数
     */
    static void DelayMs(uint32_t ms);

    /**
     * @brief 将角度限制到 [-180, 180) 区间
     * @param angle_deg 输入角度，单位 deg
     * @return 限幅后的角度
     */
    static float Wrap180(float angle_deg);

    /**
     * @brief 将原始 Yaw 转为连续展开角
     * @param raw_yaw_deg 原始 Yaw 角
     * @return 连续展开后的 Yaw 角
     */
    float YawToContinuous(float raw_yaw_deg);

    /**
     * @brief 重置设备对象内部状态
     */
    void ResetState();
};

/**
 * @brief 全局 BMI088 管理对象
 */
extern Class_BMI088 BMI088_Manage_Object;

/* 数据处理 */
#define BMI088_ACCEL_SENSITIVITY_24G 1365.0f
#define BMI088_ACCEL_SENSITIVITY_12G 2730.0f
#define BMI088_ACCEL_SENSITIVITY_6G  5460.0f
#define BMI088_ACCEL_SENSITIVITY_3G  10920.0f

/* 陀螺仪灵敏度 */
#define BMI088_GYRO_SENSITIVITY_2000_DPS 16.384f
#define BMI088_GYRO_SENSITIVITY_1000_DPS 32.8f
#define BMI088_GYRO_SENSITIVITY_500_DPS  65.6f
#define BMI088_GYRO_SENSITIVITY_250_DPS  131.2f
#define BMI088_GYRO_SENSITIVITY_125_DPS  262.4f

/* 芯片 ID */
#define BMI088_ACCEL_CHIP_ID 0x1E
#define BMI088_GYRO_CHIP_ID  0x0F

/* 软复位命令 */
#define BMI088_SOFT_RESET_CMD 0xB6

/* ACCEL */
#define BMI088_REG_ACCEL_CHIP_ID       0x00
#define BMI088_REG_ACCEL_ERR_REG       0x02
#define BMI088_REG_ACCEL_STATUS        0x03
#define BMI088_REG_ACCEL_X_LSB         0x12
#define BMI088_REG_ACCEL_X_MSB         0x13
#define BMI088_REG_ACCEL_Y_LSB         0x14
#define BMI088_REG_ACCEL_Y_MSB         0x15
#define BMI088_REG_ACCEL_Z_LSB         0x16
#define BMI088_REG_ACCEL_Z_MSB         0x17
#define BMI088_REG_ACCEL_SENSORTIME_0  0x18
#define BMI088_REG_ACCEL_SENSORTIME_1  0x19
#define BMI088_REG_ACCEL_SENSORTIME_2  0x1A
#define BMI088_REG_ACCEL_INT_STAT_1    0x1D
#define BMI088_REG_ACCEL_TEMP_LSB      0x22
#define BMI088_REG_ACCEL_TEMP_MSB      0x23
#define BMI088_REG_ACCEL_CONF          0x40
#define BMI088_REG_ACCEL_RANGE         0x41
#define BMI088_REG_ACCEL_INT1_IO_CONF  0x53
#define BMI088_REG_ACCEL_INT2_IO_CONF  0x54
#define BMI088_REG_ACCEL_INT_MAP_DATA  0x58
#define BMI088_REG_ACCEL_SELF_TEST     0x6D
#define BMI088_REG_ACCEL_PWR_CONF      0x7C
#define BMI088_REG_ACCEL_PWR_CTRL      0x7D
#define BMI088_REG_ACCEL_SOFTRESET     0x7E

#define BMI088_ACCEL_ODR_1600_HZ 0x0C
#define BMI088_ACCEL_ODR_800_HZ  0x0B
#define BMI088_ACCEL_ODR_400_HZ  0x0A
#define BMI088_ACCEL_ODR_200_HZ  0x09
#define BMI088_ACCEL_ODR_100_HZ  0x08
#define BMI088_ACCEL_ODR_50_HZ   0x07

#define BMI088_ACCEL_BWP_NORMAL 0xA0
#define BMI088_ACCEL_BWP_OSR4   0x80
#define BMI088_ACCEL_BWP_OSR2   0x90
#define BMI088_ACCEL_BWP_CIC    0xB0

#define BMI088_ACCEL_CONF_1600HZ_NORMAL (BMI088_ACCEL_ODR_1600_HZ | BMI088_ACCEL_BWP_NORMAL)

#define BMI088_ACCEL_RANGE_3G  0x00
#define BMI088_ACCEL_RANGE_6G  0x01
#define BMI088_ACCEL_RANGE_12G 0x02
#define BMI088_ACCEL_RANGE_24G 0x03

#define BMI088_ACCEL_PWR_ENABLE  0x04
#define BMI088_ACCEL_PWR_DISABLE 0x00

#define BMI088_ACCEL_INT1_DRDY_MAP 0x04
#define BMI088_ACCEL_INT2_DRDY_MAP 0x40

#define BMI088_ACCEL_INT_CFG_PUSH_PULL_HIGH 0x0A
#define BMI088_ACCEL_INT_CFG_PUSH_PULL_LOW  0x08

/* GYRO */
#define BMI088_REG_GYRO_CHIP_ID            0x00
#define BMI088_REG_GYRO_X_LSB              0x02
#define BMI088_REG_GYRO_X_MSB              0x03
#define BMI088_REG_GYRO_Y_LSB              0x04
#define BMI088_REG_GYRO_Y_MSB              0x05
#define BMI088_REG_GYRO_Z_LSB              0x06
#define BMI088_REG_GYRO_Z_MSB              0x07
#define BMI088_REG_GYRO_INT_STAT_1         0x0A
#define BMI088_REG_GYRO_RANGE              0x0F
#define BMI088_REG_GYRO_BANDWIDTH          0x10
#define BMI088_REG_GYRO_LPM1               0x11
#define BMI088_REG_GYRO_SOFTRESET          0x14
#define BMI088_REG_GYRO_INT_CTRL           0x15
#define BMI088_REG_GYRO_INT3_INT4_IO_CONF  0x16
#define BMI088_REG_GYRO_INT3_INT4_IO_MAP   0x18
#define BMI088_REG_GYRO_SELF_TEST          0x3C

#define BMI088_GYRO_BW_532_HZ 0x80
#define BMI088_GYRO_BW_230_HZ 0x81
#define BMI088_GYRO_BW_116_HZ 0x82
#define BMI088_GYRO_BW_47_HZ  0x83
#define BMI088_GYRO_BW_23_HZ  0x84
#define BMI088_GYRO_BW_12_HZ  0x85
#define BMI088_GYRO_BW_64_HZ  0x87

#define BMI088_GYRO_RANGE_2000_DPS 0x00
#define BMI088_GYRO_RANGE_1000_DPS 0x01
#define BMI088_GYRO_RANGE_500_DPS  0x02
#define BMI088_GYRO_RANGE_250_DPS  0x03
#define BMI088_GYRO_RANGE_125_DPS  0x04

#define BMI088_GYRO_MODE_NORMAL       0x00
#define BMI088_GYRO_MODE_DEEP_SUSPEND 0x20
#define BMI088_GYRO_MODE_SUSPEND      0x80

#define BMI088_GYRO_INT_DRDY_ENABLE 0x80

#define BMI088_GYRO_INT3_CFG_PUSH_PULL_HIGH 0x01
#define BMI088_GYRO_INT4_CFG_PUSH_PULL_HIGH 0x04

#define BMI088_GYRO_INT3_DRDY_MAP      0x01
#define BMI088_GYRO_INT4_DRDY_MAP      0x80
#define BMI088_GYRO_INT3_INT4_DRDY_MAP 0x81

#endif /* __BMI088_H__ */
