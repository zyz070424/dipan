#include "MyTask.h"

#include "Classis.h"

void MyTask_Init(void)
{
    // 统一做一次底盘初始化：遥控、CAN、电机、PID、定时器中断
    Classis_Init(NULL);
}

void MyTask_Run(void)
{
    // 这里进入底盘主循环，不会返回
    Classis_RunTask(NULL);
}
