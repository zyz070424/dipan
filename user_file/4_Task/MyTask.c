#include "MyTask.h"

#include "Classis.h"
#include "FreeRTOS.h"
#include "task.h"

#define MYTASK_CLASSIS_CTRL_STACK_SIZE   2500
#define MYTASK_CLASSIS_BMI088_STACK_SIZE 1000

static uint8_t g_task_started = 0u;

/**
 * @brief 任务初始化函数
 *
 */
void MyTask_Init(void)
{
    // 统一做一次底盘初始化：遥控、CAN、电机、PID、定时器中断
    Classis_Init(NULL);
}

/**
 * @brief 任务运行函数
 *
 */
void MyTask_Run(void)
{
    TaskHandle_t classis_task_handle;
    TaskHandle_t classis_bmi088_task_handle;

    if (g_task_started != 0u)
    {
        return;
    }

    g_task_started = 1u;

    xTaskCreate(Classis_RunTask,
                "Task_Classis",
                MYTASK_CLASSIS_CTRL_STACK_SIZE,
                NULL,
                osPriorityNormal,
                &classis_task_handle);
    xTaskCreate(Classis_BMI088_Task,
                "Task_Classis_BMI088",
                MYTASK_CLASSIS_BMI088_STACK_SIZE,
                NULL,
                osPriorityAboveNormal,
                &classis_bmi088_task_handle);
}
