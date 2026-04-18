/**
 * @file MyTask.cpp
 * @brief 任务层入口实现
 *
 * @details
 * 当前任务层仍保留 `MyTask_Init()` / `MyTask_Run()` 这组对外入口，
 * 但内部已经切换为调度新的 `Chassis` 模块入口。
 */
#include "MyTask.h"

#include "Chassis.h"
#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief 底盘控制任务栈大小
 */
#define MYTASK_CHASSIS_CTRL_STACK_SIZE 2500
/**
 * @brief BMI088 任务栈大小
 */
#define MYTASK_CHASSIS_BMI088_STACK_SIZE 1000

static uint8_t g_task_started = 0u;

/**
 * @brief 底盘主循环任务入口
 * @param argument 任务参数
 */
static void MyTask_Chassis_Task(void *argument)
{
    Chassis.RunTask(argument);
}

/**
 * @brief BMI088 周期读取任务入口
 * @param argument 任务参数
 */
static void MyTask_Chassis_BMI088_Task(void *argument)
{
    Chassis.BMI088Task(argument);
}

/**
 * @brief 任务初始化函数
 */
void MyTask_Init(void)
{
    Chassis.Init(NULL);
}

/**
 * @brief 任务运行函数
 */
void MyTask_Run(void)
{
    TaskHandle_t chassis_task_handle;
    TaskHandle_t chassis_bmi088_task_handle;

    if (g_task_started != 0u)
    {
        return;
    }

    g_task_started = 1u;

    xTaskCreate(MyTask_Chassis_Task,
                "Task_Chassis",
                MYTASK_CHASSIS_CTRL_STACK_SIZE,
                NULL,
                osPriorityNormal,
                &chassis_task_handle);

    xTaskCreate(MyTask_Chassis_BMI088_Task,
                "Task_Chassis_BMI088",
                MYTASK_CHASSIS_BMI088_STACK_SIZE,
                NULL,
                osPriorityAboveNormal,
                &chassis_bmi088_task_handle);
}
