/**
 * @file MyTask.h
 * @brief 任务层对外入口声明
 */
#ifndef __MYTASK_H__
#define __MYTASK_H__

#include "main.h"
#include "cmsis_os.h"
/* 预留接口，用于扩展任务 */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化任务层
 */
void MyTask_Init(void);
/**
 * @brief 运行任务层
 */
void MyTask_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* __MYTASK_H__ */
