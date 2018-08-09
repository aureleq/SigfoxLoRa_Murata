/********************************************************************************
 * @file    scheduler.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   it schedules tasks
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
#include "scheduler.h"

#define CFG_SCH_TASK_NBR 16

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if( __CORTEX_M == 0)
static const uint8_t clz_table_4bit[16] = { 4, 3, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
#endif
static uint32_t TaskSet = 0;
static uint32_t TaskMask = (~0);
static uint32_t EvtSet = 0;
static uint32_t EvtWaited = 0;
static void (*TaskCb[CFG_SCH_TASK_NBR])( void );


/* Global variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#if( __CORTEX_M == 0)
static uint32_t CountLeadZero(uint32_t value);
#endif

/* Functions Definition ------------------------------------------------------*/

void SCH_Run( void )
{
  uint32_t bit_nbr;

  while( TaskSet &  TaskMask )
  {
#if( __CORTEX_M == 0) 
   bit_nbr = CountLeadZero( TaskSet &  TaskMask );
#else
   bit_nbr =__CLZ( TaskSet &  TaskMask );
#endif
    BACKUP_PRIMASK();
    
    DISABLE_IRQ() ;
    
    TaskSet &= ~( 1 << (31 - bit_nbr) );
    
    RESTORE_PRIMASK();

    TaskCb[31 - bit_nbr]();
  }

  BACKUP_PRIMASK();
  
  DISABLE_IRQ() ;

  if ( !( (TaskSet & TaskMask ) || (EvtSet & EvtWaited) ) )
  {
    SCH_Idle();
  }
  RESTORE_PRIMASK();

  return;
}


void SCH_RegTask( uint32_t task_id, void (*task)(void) )
{
  BACKUP_PRIMASK();
  
  DISABLE_IRQ() ;

  TaskCb[task_id] = task;

  RESTORE_PRIMASK();

  return;
}

void SCH_SetTask( uint32_t task_id )
{
  BACKUP_PRIMASK();
  
  DISABLE_IRQ() ;

  TaskSet |= (1 << task_id);

  RESTORE_PRIMASK();

  return;
}

void SCH_PauseTask( uint32_t task_id )
{
  BACKUP_PRIMASK();
  
  DISABLE_IRQ() ;

  TaskMask &= ~ ( 1 << task_id );

  RESTORE_PRIMASK();

  return;
}

void SCH_ResumeTask( uint32_t task_id )
{
  BACKUP_PRIMASK();
  
  DISABLE_IRQ() ;

  TaskMask |= ( 1 << task_id );

  RESTORE_PRIMASK();

  return;
}

void SCH_SetEvt( uint32_t evt_id )
{
  BACKUP_PRIMASK();
  
  DISABLE_IRQ() ;

  EvtSet |= ( 1 << evt_id);

  RESTORE_PRIMASK();

  return;
}

void SCH_ClrEvt( uint32_t evt_id )
{
  BACKUP_PRIMASK();
  
  DISABLE_IRQ() ;

  EvtSet &= ~( 1 << evt_id);

  RESTORE_PRIMASK();

  return;
}

void SCH_WaitEvt( uint32_t evt_id )
{
  uint32_t event_waited_id_backup;

  event_waited_id_backup = EvtWaited;
  EvtWaited = ( 1 << evt_id );
  while( (EvtSet & EvtWaited) == 0)
  {
    SCH_EvtIdle();
  }
  EvtSet &= (~EvtWaited);
  EvtWaited = event_waited_id_backup;

  return;
}

__weak void SCH_EvtIdle( void )
{
  /**
   * execute scheduler if not implemented by the application
   */
  SCH_Run();

  return;
}

__weak void SCH_Idle( void )
{
  /**
   * Stay in run mode if not implemented by the application
   */
  return;
}

#if( __CORTEX_M == 0) 
static uint32_t CountLeadZero(uint32_t value)
{
  uint32_t n = 0;

  if ((value & 0xFFFF0000) == 0)  { n  = 16; value <<= 16;  }
  if ((value & 0xFF000000) == 0)  { n +=  8; value <<=  8;  }
  if ((value & 0xF0000000) == 0)  { n +=  4; value <<=  4;  }

  n += (uint32_t)clz_table_4bit[value >> (32-4)];

  return n;
}
#endif






/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
