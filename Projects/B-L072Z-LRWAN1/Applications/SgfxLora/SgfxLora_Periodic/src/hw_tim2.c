/*******************************************************************************
 * @file    hw_tim2.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   manages the timer 2
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other 
 *    contributors to this software may be used to endorse or promote products 
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this 
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under 
 *    this license is void and will automatically terminate your rights under 
 *    this license. 
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "utilities.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define  TIM2_PULSE_WIDTH  ((uint32_t) 2 )       
#define  TIM2_PERIOD       ((uint32_t) 400)        /* 12,5us  */
#define  SPI_LAUNCH_DELAY  ((uint32_t) 69)       
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static TIM_HandleTypeDef TimHandle;

void HW_TIM2_Init(void)
{
  /* -1- Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef sConfig;

  TimHandle.Instance = TIM2;

  TimHandle.Init.Period        = TIM2_PERIOD-1;
  TimHandle.Init.Prescaler     = 0;
  TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TimHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;

  HAL_TIM_OC_Init(&TimHandle);

  /*##-2- Configure the Output Compare channels*/
  /* NSS */
  sConfig.OCMode     = TIM_OCMODE_PWM1;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.Pulse = TIM2_PULSE_WIDTH;

  HAL_TIM_OC_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1);

  /*DMA req*/
  sConfig.OCMode     = TIM_OCMODE_ACTIVE;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.Pulse = SPI_LAUNCH_DELAY;

  HAL_TIM_OC_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2);

  /*##-3- Start signals generation #######################################*/ 
  /* Start channel 2 in Output compare mode */ 

  __HAL_TIM_ENABLE_DMA(&TimHandle, TIM_DMA_CC2);
}

void HW_TIM2_Start(void)
{
  __HAL_TIM_SET_COUNTER(&TimHandle, 0) ;

  HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_1);

  HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_2);
}

void HW_TIM2_Stop(void)
{
  /* Clear the update interrupt flag */
 __HAL_TIM_CLEAR_FLAG(&TimHandle, TIM_FLAG_UPDATE);
  /* Wait for the update interrupt flag */
  while(__HAL_TIM_GET_FLAG(&TimHandle, TIM_FLAG_UPDATE) != SET);
   /* stop the timer */
  HAL_TIM_OC_Stop(&TimHandle, TIM_CHANNEL_1);  
     /* stop the timer */
  HAL_TIM_OC_Stop(&TimHandle, TIM_CHANNEL_2);
   /* reset counter value to 0 the timer */
  __HAL_TIM_SET_COUNTER(&TimHandle, 0) ;
   /* clear counter flag*/
  __HAL_TIM_CLEAR_FLAG(&TimHandle, TIM_FLAG_CC1);
}

void HW_TIM2_SetPeriod(uint32_t periodInit)
{
  TimHandle.Instance = TIM2;
  TimHandle.Init.Period        = periodInit-1;
  TimHandle.Init.Prescaler     = 0;
  TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TimHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;

  HAL_TIM_OC_Init(&TimHandle);
}

uint32_t HW_TIM2_GetPeriod( void )
{
  return TimHandle.Init.Period+1;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
