/********************************************************************************
 * @file    scheduler.h
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   scheduler interface
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#ifdef __cplusplus
extern "C" {
#endif

  /* Includes ------------------------------------------------------------------*/
#include "utilities_conf.h"
  /* Exported types ------------------------------------------------------------*/
  /* Exported constants --------------------------------------------------------*/
  /* External variables --------------------------------------------------------*/
  /* Exported macros -----------------------------------------------------------*/
  /* Exported functions ------------------------------------------------------- */

  /**
   * @brief This is called by the scheduler in critical section (PRIMASK bit) when
   *          - there is no more tasks to be executed
   *          - there is no pending event or the pending event is still not set
   *        The application should enter low power mode
   *
   * @param  None
   * @retval None
   */
   



  void SCH_Idle( void );

  /**
   * @brief This requests the scheduler to execute all pending tasks. When no task are pending, it calls SCH_Idle();
   *
   * @param  None
   * @retval None
   */
  void SCH_Run( void );

  /**
   * @brief This registers a task in the scheduler.
   *        A task can be either Set, Pause or Resume.
   *
   * @param task_id: id of the task ( It shall be different for all tasks)
   * @param task: Callback to the task to be executed
   *
   * @retval None
   */
  void SCH_RegTask( uint32_t task_id, void (*task)(void) );

  /**
   * @brief  Request a task to be executed
   *
   * @param  task_id: The Id of the task
   * @retval None
   */
  void SCH_SetTask( uint32_t task_id );

  /**
   * @brief Prevents a task to be called by the scheduler even when set with SCH_SetTask()
   *        By default, all tasks are executed by the scheduler when set with SCH_SetTask()
   *
   * @param task_id
   * @retval None
   */
  void SCH_PauseTask( uint32_t task_id );

  /**
   * @brief Allows a task to be called by the scheduler if set with SCH_SetTask()
   *        By default, all tasks are executed by the scheduler when set with SCH_SetTask()
   *
   * @param task_id
   * @retval None
   */
  void SCH_ResumeTask( uint32_t task_id );

  /**
   * @brief It waits for a specific event to be set. The scheduler loops SCH_EvtIdle() until the event is set
   *        When called recursively, it acts as a First in / Last out mechanism. The scheduler waits for the
   *        last event requested to be set even though one of the already requested event has been set.
   *
   * @param evt_id
   * @retval None
   */
  void SCH_WaitEvt( uint32_t evt_id );

  /**
   * @brief It sets an event that is waited with SCH_WaitEvt()
   *
   * @param evt_id
   * @retval None
   */
  void SCH_SetEvt( uint32_t evt_id );

  /**
   * @brief It clears an event  SCH_WaitEvt()
   * 
   * @param evt_id
   * @retval None
   */
  void SCH_ClrEvt( uint32_t evt_id );
  /**
   * @brief The scheduler loops in that function until the waited event is set
   *        The application may either enter low power mode or call SCH_Run()
   *        When not implemented by the application, it calls call SCH_Run()
   *
   * @param  None
   * @retval None
   */
  void SCH_EvtIdle( void );

#ifdef __cplusplus
}
#endif

#endif /*__SCHEDULER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
