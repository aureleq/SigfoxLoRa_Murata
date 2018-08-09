/******************************************************************************
 * @file    mcu_api.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   mcu library interface
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


#include "utils_common.h"
#include "sigfox_types.h"
#include "sigfox_api.h"
#include "mcu_api.h"
#include "sgfx_sx1276_driver.h"
#include "st_lowlevel.h"
#include "hw_msp.h"
#include "timeServer.h"
#include "scheduler.h"
#include "hw_eeprom.h"
#include "debug.h"
/******** DEFINE **************************************************************/

#define LIBRARY_MEM_SIZE_MAX 200

/******* GLOBAL VARIABLES *****************************************************/  
/******* LOCAL VARIABLES ******************************************************/

/*The allocated memory must be word aligned */
static uint8_t LibraryMem[LIBRARY_MEM_SIZE_MAX] ALIGN(4); 

static TimerEvent_t Timer_delayMs; 

static void OnTimerDelayEvt( void ); 

/*timer to handle low power delays*/
static TimerEvent_t Timer_Timeout; 

static void OnTimerTimeoutEvt( void );

/*timer to handleCarrier Sense Timeout*/
static TimerEvent_t Timer_TimeoutCs; 


/********************************************************
 *  GLOBALS : these variables are shared with the RF 
 ********************************************************/
/*!
 * @brief Wait delay_ms milliseconds in a blocking way.
 * @param[in] uint32_t delay_ms: the number of milliseconds to wait.
 * @retval None
 */
static void Delay_Lp(uint32_t delay_ms);
#define MCU_VER "MCU_API_V1.0"
static sfx_u8 mcu_api_version[30]= MCU_VER;

/*******************************************************************/
sfx_u8 MCU_API_malloc(sfx_u16 size, sfx_u8 **returned_pointer)
{
  sfx_error_t error = SFX_ERR_NONE;
  // ------------------------------------------------------
  // PSEUDO code
  // ------------------------------------------------------
  // Allocate a memory : static or dynamic allocation
  // knowing that the sigfox library will ask for a buffer once at the SIGFOX_API_open() call.
  // This buffer will be released after SIGFOX_API_close() call.
  if(size <= LIBRARY_MEM_SIZE_MAX )
  { 
    /* The memory block is free, we can allocate it */
    *returned_pointer = LibraryMem ;
  }
  else
  {
    /* No block available */
    error = MCU_ERR_API_MALLOC;
  }
  return error;
}

/*******************************************************************/
sfx_u8 MCU_API_free(sfx_u8 *ptr)
{
  return SFX_ERR_NONE;
}

/*******************************************************************/
sfx_u8 MCU_API_get_voltage_temperature(sfx_u16 *voltage_idle, sfx_u16 *voltage_tx, sfx_s16 *temperature)
{
    /* TBD_DOWNLINK : Implement the proper values for each sensor */
    *voltage_idle = (uint16_t) HW_GetBatteryLevel(); /* mV */ 
    *voltage_tx = 0;   /* mV */
    *temperature = (uint16_t) ((HW_GetTemperatureLevel() * 10 )>>8);  /* */
  
    DBG_PRINTF("temp=%d , ", (int32_t) *temperature);
    DBG_PRINTF("voltage=%u\n\r", (uint32_t) *voltage_idle);

  return SFX_ERR_NONE;
}

/*******************************************************************/
sfx_u8 MCU_API_delay(sfx_delay_t delay_type)
{

    /* Local variables */
    sfx_error_t err = SFX_ERR_NONE;
  
    st_sfx_rc_t SgfxRc = E2pData.SgfxRc;

    /* Switch/case */
    switch(delay_type)
    {
    case SFX_DLY_INTER_FRAME_TRX :
        /* Delay  is 500ms  in FCC and ETSI
         * In ARIB : minimum delay is 50 ms */
        if( SgfxRc.id== RC3C_ID )
        {
          Delay_Lp( 50 ); /* 50 ms */ 

        }
        else
        {          
          Delay_Lp( 450 ); /* 500 ms */
          /*comment from sigfox iso 500 was measured 565, spec={500,525ms} so removed 50*/
        }
        break;

    case SFX_DLY_INTER_FRAME_TX :
        /* Start delay 0 seconds to 2 seconds in FCC and ETSI*/
        /* In ARIB : minimum delay is 50 ms */
        if( SgfxRc.id== RC3C_ID )
        {
           Delay_Lp( 50 ); /* 50 ms */
        }
        else
        {
           Delay_Lp( 1000 ); /* 1000 ms */
        }
        break;

    case SFX_DLY_OOB_ACK :
        /* Start delay between 1.4 seconds to 4 seconds in FCC and ETSI */
        Delay_Lp(1600);
       /*comment from sigfox iso 1400 was measured 1300, spec={1,4-4s}so added 200*/
        break;

    case SFX_DLY_CS_SLEEP :
        Delay_Lp( 500 );/* 500 ms */
        break;

    default : 
        err = MCU_ERR_API_DLY;
        break;
    }
    return err;
}


/*******************************************************************/
sfx_u8 MCU_API_get_nv_mem(sfx_u8 read_data[SFX_NVMEM_BLOCK_SIZE])
{
  int i;
  for ( i=0; i< SFX_NVMEM_BLOCK_SIZE;i++)
  {
    read_data[i] =E2pData.lib_sgfx_mcu_nvm[i];
  }

 return SFX_ERR_NONE;
}

/*******************************************************************/
sfx_u8 MCU_API_set_nv_mem(sfx_u8 data_to_write[SFX_NVMEM_BLOCK_SIZE])
{
  int i;
  for ( i=0; i< SFX_NVMEM_BLOCK_SIZE;i++)
  {
    HW_EEPROM_WRITE( E2pData.lib_sgfx_mcu_nvm[i], data_to_write[i]);
  }
  return SFX_ERR_NONE;

}

/*******************************************************************/
sfx_u8 MCU_API_timer_start_carrier_sense(sfx_u16 time_duration_in_ms)
{
  DBG_PRINTF_NOW();
  DBG_PRINTF("CS timeout Started= %d msec\n\r", time_duration_in_ms);
  TimerInit( &Timer_TimeoutCs, OnTimerTimeoutCsEvt );
  TimerSetValue( &Timer_TimeoutCs,  time_duration_in_ms  );
  TimerStart( &Timer_TimeoutCs ); 

  return SFX_ERR_NONE;
}

/*******************************************************************/
sfx_u8 MCU_API_timer_start(sfx_u32 time_duration_in_s)
{
  DBG_PRINTF_NOW();
  DBG_PRINTF("TIM timeout Started= %d sec\n\r", time_duration_in_s);
  TimerInit( &Timer_Timeout, OnTimerTimeoutEvt );
  TimerSetValue( &Timer_Timeout,  time_duration_in_s*1000  );
  TimerStart( &Timer_Timeout ); 
  
  return SFX_ERR_NONE;
}

/*******************************************************************/
sfx_u8 MCU_API_timer_stop(void)
{
  DBG_PRINTF_NOW();
  
  DBG_PRINTF("timer_stop\n\r");

  TimerStop( &Timer_Timeout );

  return SFX_ERR_NONE;
}

/*******************************************************************/
sfx_u8 MCU_API_timer_stop_carrier_sense(void)
{
  DBG_PRINTF_NOW();
  
  DBG_PRINTF("CS timer_stop\n\r");

  TimerStop( &Timer_TimeoutCs );

  return SFX_ERR_NONE;
}

/*******************************************************************/
sfx_u8 MCU_API_timer_wait_for_end(void)
{
  DBG_PRINTF_NOW();

  DBG_PRINTF("Rx Wait Start\n\r");

  SCH_WaitEvt( TIMOUT_EVT );

  DBG_PRINTF_NOW();

  DBG_PRINTF("Rx Wait End\n\r");
  return SFX_ERR_NONE;
}

/*******************************************************************/
sfx_u8 MCU_API_report_test_result(sfx_bool status, sfx_s16 rssi)
{
  DBG_PRINTF_NOW();
  if( status == SFX_TRUE)
  {
    /* Set the LED3 to inform the user that we received a frame for the device */
    BSP_LED_On(LED_GREEN );
    HAL_Delay( 50 );
    BSP_LED_Off( LED_GREEN );

    if (E2pData.AtEcho == E2P_SET)
    {
      PRINTF("RX OK. RSSI= %d dBm\n\r", (int32_t) rssi );
    }
  }
  else
  {
    BSP_LED_On( LED_BLUE );
    HAL_Delay( 50 );
    BSP_LED_Off( LED_BLUE );
    
    if (E2pData.AtEcho == E2P_SET)
    {
      PRINTF("RX KO. RSSI= %d dBm\n\r", (int32_t)rssi );
    }
  }

  return SFX_ERR_NONE;
}

/*******************************************************************/
sfx_u8 MCU_API_get_version(sfx_u8 **version, sfx_u8 *size)
{
  *version = (sfx_u8*)mcu_api_version;
  if(size == SFX_NULL)
  {
    return MCU_ERR_API_GET_VERSION;
  }
  *size = sizeof(mcu_api_version);
  
  return SFX_ERR_NONE;
}


static void Delay_Lp(uint32_t delay_ms)
{
  DBG_PRINTF_NOW();
  
  DBG_PRINTF("Delay= %d ms\n\r",delay_ms);
  if ( delay_ms > 5 )
  {
    TimerInit( &Timer_delayMs, OnTimerDelayEvt );
    TimerSetValue( &Timer_delayMs,  delay_ms);
    TimerStart( &Timer_delayMs );
    SCH_WaitEvt( DELAY_EVT );
  }
  else
  {
    HAL_Delay( delay_ms );
  }
}

/* Private function definition -----------------------------------------------*/
static void OnTimerDelayEvt( void )
{
  SCH_SetEvt( DELAY_EVT );
}

static void OnTimerTimeoutEvt( void )
{
  SCH_SetEvt( TIMOUT_EVT );
}
