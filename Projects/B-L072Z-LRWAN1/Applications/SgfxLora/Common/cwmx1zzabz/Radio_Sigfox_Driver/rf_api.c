/******************************************************************************
 * @file    rf_api.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   rf library interface
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
  /******************************************************************************
 * @file    rf_api.c
 * @author  MCD Application Team
 * @version V1.1.0.RC1
 * @date    15-April-2018
 * @brief   rf library interface
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
*/
/********************************************************
 *  GLOBALS : these variables are shared with the MCU
 ********************************************************/

#include "utils_common.h"
#include "sigfox_types.h"
#include "sigfox_api.h"
#include "rf_api.h"
#include "st_lowlevel.h"
#include "sgfx_sx1276_driver.h"
#include "scheduler.h"
#include "hw_eeprom.h"
#include "debug.h"
#define SIZE_OF_MODULATION_LIB_VERSION  30 /* MODULATION_LIB_VERSION defined in ST_SGFX_SX1276_xxx.lib */
#define MAX_CS_RSSI_AVG 8
static sfx_u8 rf_api_version[SIZE_OF_MODULATION_LIB_VERSION];

/*******************************************************************/
sfx_u8 RF_API_init(sfx_rf_mode_t rf_mode)
{
  DBG_PRINTF_NOW();
    // ------------------------------------------------------
    // PSEUDO code
    // ------------------------------------------------------
  // Put here all RF initialization concerning the RC IC you are using.
  // this function is dependant of SPI driver or equivalent.
  switch (rf_mode)
  {
    case SFX_RF_MODE_TX :
      DBG_PRINTF("RF_API_init in TX\n\r");

      SGFX_SX1276_tx_config( );
      // set the RF IC in TX mode : this could be DBPSK100 or 600:  this distinction will be done during the RF_API_send
      // you can add some code to switch on TCXO or warm up quartz for stabilization : in case of frequency drift issue
    break;
    case SFX_RF_MODE_RX :
      DBG_PRINTF("RF_API_init in RX\n\r");
      SGFX_SX1276_rx_config( );
      // set the RF IC in RX mode : GFSK 600bps is the only mode now
      // Enable interrupts on RX fifo
      // RF IC must configure the SYNCHRO BIT = 0xAAAAAAAAAA and synchro frame = 0xB227
    break;
    case SFX_RF_MODE_CS200K_RX :

      DBG_PRINTF("RF_API_init in CS200\n\r");
      //DBG_PRINTF("RxBw=200kHz\n\r");

      SGFX_SX1276_rx_config( );
      SGFX_SX1276_rx_setbw(CS_BW_200KHZ);
      // configure the RF IC into sensing 200KHz bandwidth to be able to read out RSSI level
      // RSSI level will outputed during the RF_API_wait_for_clear_channel function
    break;
    case SFX_RF_MODE_CS300K_RX :
      DBG_PRINTF("RF_API_init in CS300\n\r");
      //DBG_PRINTF("RxBw=300kHz\n\r");

      SGFX_SX1276_rx_config( );
      SGFX_SX1276_rx_setbw(CS_BW_300KHZ);
      // configure the RF IC into sensing 300KHz bandwidth to be able to read out RSSI level
      // This is poosible to make this carrier sense in 2 * 200Kz if you respect the regulation time for listening
      // RSSI level will outputed during the RF_API_wait_for_clear_channel function.
    break;

    default :
    break;
  }
  return SFX_ERR_NONE;
};

/*******************************************************************/
sfx_u8 RF_API_stop(void)
{
  DBG_PRINTF_NOW();

  DBG_PRINTF("RF_API_stop\n\r");

  SGFX_SX1276_stop();

  return SFX_ERR_NONE;
}

/*******************************************************************/
sfx_u8 RF_API_send(sfx_u8 *stream, sfx_modulation_type_t type, sfx_u8 size)
{
  sfx_u8 status =SFX_ERR_NONE;
  BSP_LED_On( LED_RED2 );

  DBG_PRINTF_NOW();

  DBG_PRINTF("TX START\n\r");
  PRINTF("Start");
  switch (type)
  {
    case SFX_DBPSK_100BPS :
      if ( SGFX_SX1276_tx( (uint8_t *)stream, (uint8_t) size, 100 ) != MOD_SUCCESS)
      {
       status = RF_ERR_API_SEND;
      }
      break;
    case SFX_DBPSK_600BPS :
      if ( SGFX_SX1276_tx( (uint8_t *)stream, (uint8_t) size, 600 ) != MOD_SUCCESS)
      {
       status = RF_ERR_API_SEND;
      }
      break;

    default :
      status = RF_ERR_API_SEND;
      break;
  }
  DBG_PRINTF_NOW();

  DBG_PRINTF("TX END\n\r");

  BSP_LED_Off( LED_RED2 );

  return status;
}

/*******************************************************************/
sfx_u8 RF_API_start_continuous_transmission (sfx_modulation_type_t type)
{
  sfx_u8 status = SFX_ERR_NONE;
   // ------------------------------------------------------
    // PSEUDO code
    // ------------------------------------------------------
  switch (type)
  {
    case SFX_NO_MODULATION :
      if (SGFX_SX1276_start_txtest_cw()!= MOD_SUCCESS)
      {
       status = RF_ERR_API_START_CONTINUOUS_TRANSMISSION;
      }
      // Configure the RF IC into pure carrier CW : no modulation
      // This mode is available on many RF ICs for type approval tests or manufacturing tests.
      // The frequency is chosen in the RF_API_change_frequency by the sigfox lib
      // i.e. : SPI_DRV_write(CW)
    break;
    case SFX_DBPSK_100BPS :
      if (SGFX_SX1276_start_txtest_prbs9( 100 )!= MOD_SUCCESS)
      {
       status = RF_ERR_API_START_CONTINUOUS_TRANSMISSION;
      }
      // Make an infinite DBPSK 100bps modulation on the RF IC at the frequency given by the RF_API_change_frequency()
      break;
    case SFX_DBPSK_600BPS :
      // Make an infinite DBPSK 600bps modulation on the RF IC at the frequency given by the RF_API_change_frequency()
      if (SGFX_SX1276_start_txtest_prbs9( 600 )!= MOD_SUCCESS)
      {
       status = RF_ERR_API_START_CONTINUOUS_TRANSMISSION;
      }
      break;
    default :
      status = RF_ERR_API_SEND;
      break;
  }
  return status;
}

/*******************************************************************/
sfx_u8 RF_API_stop_continuous_transmission (void)
{
   // ------------------------------------------------------
    // PSEUDO code
    // ------------------------------------------------------
  // Stop the RC IC : you can switch off this part and use RF IC for other modulation / protocole : SFX LIB does not use it after this call
  // Stop also the TCXO if you have one or external PA.
  if (SGFX_SX1276_stop_txtest() != MOD_SUCCESS)
  {
    return RF_ERR_API_STOP_CONTINUOUS_TRANSMISSION;
  }
  return SFX_ERR_NONE;
}

/*******************************************************************/
sfx_u8 RF_API_change_frequency(sfx_u32 frequency)
{
  DBG_PRINTF_NOW();

  DBG_PRINTF("RF at Freq %d\n\r", frequency);

  STLL_Radio_SetFreq(frequency);

  return SFX_ERR_NONE;
}

/*******************************************************************/
sfx_u8 RF_API_wait_frame(sfx_u8 *frame, sfx_s16 *rssi, sfx_rx_state_enum_t * state)
{
  sfx_error_t status;
  DBG_PRINTF_NOW();

  DBG_PRINTF("RX START\n\r");

  SGFX_SX1276_rx_start();

  if( STLL_WaitEndOfRxFrame( ) == STLL_SET )
  {
    SGFX_SX1276_rx_stop(frame);

    *rssi = STLL_SGFX_SX1276_GetSyncRssi();
    status = SFX_ERR_NONE;
    *state = DL_PASSED;
  }
  else
  {
    *rssi = (sfx_s8) 0;
    *state = DL_TIMEOUT;
    status = SFX_ERR_NONE;
  }

  DBG_PRINTF_NOW();

  DBG_PRINTF("RX END\n\r");

  return status;
}


/*******************************************************************/
sfx_u8 RF_API_wait_for_clear_channel(sfx_u8 cs_min, sfx_s8 cs_threshold, sfx_rx_state_enum_t * state )
{

  sfx_s16 rssiVec[MAX_CS_RSSI_AVG];
  sfx_s16 rssi=0;
  int cur_idx=0;

  sfx_rx_state_enum_t cs_state  = DL_TIMEOUT;
  /*default in case below never true*/
  /*Init so that it waits at least 5*/
  for (int i=0; i<MAX_CS_RSSI_AVG; i++)
  {
    rssiVec[i]=INT16_MAX;
  }


  if (cs_min >= MAX_CS_RSSI_AVG)
  {
    return RF_ERR_API_WAIT_CLEAR_CHANNEL;
  }

  /*init the cs status to Reset*/
  STLL_RxCarrierSenseInitStatus();
   /*starts the receiver to sense channel*/
  DBG_PRINTF_NOW(); DBG_PRINTF("CS START\n\r");

  SGFX_SX1276_rx_start();

  BSP_LED_On( LED_GREEN );

  while (STLL_RxCarrierSenseGetStatus( ) == STLL_RESET )
  {
    HAL_Delay(1);
    // integrate the rssi during cs_min miliseconds
    rssiVec[cur_idx] = STLL_RxCarrierSenseGetRssi();

    cur_idx++;
    if (cur_idx==cs_min)
    {
      cur_idx=0;
    }

    //Av in dB.. Should be done in linear to be exact.
    rssi = 0;
    for(int i=0; i<cs_min; i++)
    {
      rssi+=rssiVec[i];
    }
    rssi= rssi / (cs_min);

    if ( rssi < cs_threshold )
    {
       cs_state= DL_PASSED;
       break;
    }

  }
  BSP_LED_Off( LED_GREEN );

  DBG_PRINTF_NOW();

  DBG_PRINTF("CS STOP rssi=%d>> ", rssi);

  if (cs_state== DL_PASSED)
  {
    DBG_PRINTF("LBT Channel Free\n\r");
  }
  else
  {
    DBG_PRINTF("LBT Channel Busy\n\r");
  }

  *state= cs_state;

  return SFX_ERR_NONE;
}

/*******************************************************************/
sfx_u8 RF_API_get_version(sfx_u8 **version, sfx_u8 *size)
{
  sfx_u8 ret = SFX_ERR_API_GET_VERSION;
  int len=  sprintf( (char*) rf_api_version, "%s", SGFX_SX1276_get_version() );

  *version = (sfx_u8*)rf_api_version;
  if(size != SFX_NULL)
  {
    *size = len;
    ret = SFX_ERR_NONE;
  }

  return ret;
}
