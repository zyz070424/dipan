#ifndef __CLASSIS_H__
#define __CLASSIS_H__

#include "dvc_dr16.h"
#include "dvc_bmi088.h"
#include "dvc_motor.h"
#include "main.h"
#include <stdint.h>
//预留接口，用于扩展任务
#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_WHEEL_COUNT 4

#define SQRT2_2      0.707106781f
#define WHEEL_RADIUS 0.0815f
#define ROBOT_RADIUS 0.2125f

// 每轮方向补偿（非常关键）：
// 电机正转方向与公式正方向一致 -> 填 1
// 电机正转方向与公式正方向相反 -> 填 -1
#define CHASSIS_DIR_FRONT_LEFT   (1)
#define CHASSIS_DIR_FRONT_RIGHT  (1)
#define CHASSIS_DIR_BACK_LEFT    (1)
#define CHASSIS_DIR_BACK_RIGHT   (1)

// 控制循环参数（先给可用默认值，后续按车调参）
#define CHASSIS_CTRL_DT_S                0.001f
#define CHASSIS_MAX_LINEAR_SPEED_MPS     3.0f
#define CHASSIS_MAX_ANGULAR_SPEED_RADPS  4.0f
// 自动自旋角速度（rad/s），按场地和裁判规则自行调整，这个数值据ai考证比较合适
#define CHASSIS_AUTO_SPIN_SPEED_RADPS    6.0f
#define CHASSIS_AUTO_SPIN_SPEED_MIN_RADPS 0.5f
#define CHASSIS_AUTO_SPIN_SPEED_MAX_RADPS 6.0f
#define CHASSIS_AUTO_SPIN_ADJUST_RADPS_PER_S 3.0f
#define CHASSIS_AUTO_SPIN_ADJUST_DEADZONE 0.05f

extern DR16_DataTypeDef g_dr16_data;

void Classis_Init(void *params);

// 全向轮逆解：由底盘 vx/vy/w -> 四个轮子的目标角速度
void Classis_Control(const DR16_DataTypeDef *dr16,
                     float yaw_deg,
                     uint8_t yaw_valid,
                     float *target_speed,
                     float max_speed,
                     float max_angular_velocity);

// 底盘主循环：读取遥控、读取电机反馈、PID计算、发送CAN
void Classis_RunTask(void *params);
void Classis_BMI088_Task(void *params);
void Classis_PID_TestTask(void *params);
void Classis_DR16_TestTask(void *params);
void Classis_GetBMI088ImuData(imu_data_t *imu_data);
void Classis_GetBMI088EulerData(euler_t *euler_deg);
uint8_t Classis_IsBMI088Ready(void);
HAL_StatusTypeDef Classis_GetBMI088InitStatus(void);

// 1ms 定时回调入口：用于 DR16 边沿状态更新
void Classis_Timer1msCallback(void);

#ifdef __cplusplus
}
#endif

#endif /* __CLASSIS_H__ */
