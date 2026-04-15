#ifndef __TASK_H__
#define __TASK_H__

#include "main.h"
#include "cmsis_os.h"
//预留接口，用于扩展任务
#ifdef __cplusplus
extern "C" {
#endif

// 任务入口包装：把底盘模块接到 FreeRTOS 线程
void MyTask_Init(void);
void MyTask_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_H__ */
