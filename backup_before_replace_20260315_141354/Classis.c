#include "Classis.h"
#include "motor.h"
#include "can.h"
#include <stdint.h>
DR16_DataTypeDef DR16_Data;
Motor_TypeDef Motor_front_left;
Motor_TypeDef Motor_front_right;
Motor_TypeDef Motor_back_left;
Motor_TypeDef Motor_back_right;
void Classis_Init(void*params)
{
    DR16_Init(&huart1);
    Motor_Init(&Motor_front_left,1,M3508,&hcan1,DJI_Control_Method_Speed);
    Motor_Init(&Motor_front_right,2,M3508,&hcan1,DJI_Control_Method_Speed);
    Motor_Init(&Motor_back_left,3,M3508,&hcan1,DJI_Control_Method_Speed);
    Motor_Init(&Motor_back_right,4,M3508,&hcan1,DJI_Control_Method_Speed);
}
//Moter0 = (-√2/2 * vx + √2/2 * vy + ω * r)/s
//Moter1 = (-√2/2 * vx - √2/2 * vy + ω * r)/s
//Moter3 = (√2/2 * vx - √2/2 * vy + ω * r)/s
//Moter4 = (√2/2 * vx + √2/2 * vy + ω * r)/s
//r = 425mm/2 = 212.5mm
//s = 81.5mm
void Classis_Control(DR16_DataTypeDef *dr16,float *Target_Speed,float Max_Speed,float Max_Angular_Velocity)
{
    float vx = dr16->left_x * Max_Speed;
    float vy = dr16->left_y * Max_Speed;\
    float omega = dr16->right_x * Max_Angular_Velocity;

    Target_Speed[0] = (-SQRT2_2 * vx + SQRT2_2 * vy + omega * ROBOT_RADIUS)/WHEEL_RADIUS;
    Target_Speed[1] = (-SQRT2_2 * vx - SQRT2_2 * vy + omega * ROBOT_RADIUS)/WHEEL_RADIUS;
    Target_Speed[2] = (SQRT2_2 * vx - SQRT2_2 * vy + omega * ROBOT_RADIUS)/WHEEL_RADIUS;
    Target_Speed[3] = (SQRT2_2 * vx + SQRT2_2 * vy + omega * ROBOT_RADIUS)/WHEEL_RADIUS;
   
}

void Classis_DR16_Control(void*params)
{
     
    while(1)
    {
        DR16_Process(&DR16_Data);
    }
}








/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if(htim->Instance == TIM2)
  {
    DR16_Timer1msCallback(&DR16_Data);
  }
  /* USER CODE END Callback 1 */
}