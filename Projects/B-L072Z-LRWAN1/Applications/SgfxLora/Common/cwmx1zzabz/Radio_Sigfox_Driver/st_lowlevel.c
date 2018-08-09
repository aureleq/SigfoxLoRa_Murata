/******************************************************************************
 * @file    st_lowlevel.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   modulation library low level implementation
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
#include "st_lowlevel.h"
#include "scheduler.h"
#include "radio.h"
#include "sx1276.h"
#include "hw_eeprom.h"
#include "low_power_manager.h"
#include "mlm32l07x01.h"
#include "st_sigfox_api.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static int16_t meas_rssi_dbm = 0;
static STLL_flag rxPacketReceived= STLL_RESET;
static STLL_flag rxCarrierSenseFlag= STLL_RESET;


/* Private function prototypes -----------------------------------------------*/

static void RADIO_rx_packet_interrupt_handler(void);
/*!
 * @brief  Is called at the end of the synchro Ok
 * @note  Reads RSSI when syncro has occured
 * @param[in] none
 * @retval none.
 */
static void STLL_RX_SYNC_IRQHandler_CB( void );


/* Private function definition ------------------------------------------------*/

/* Public function definition ------------------------------------------------*/


/* Radio Sub group */
uint8_t STLL_Radio_ReadReg(uint8_t address)
{
  return  SX1276Read( address);
}

void STLL_Radio_WriteReg( uint8_t addr, uint8_t data )
{
  SX1276Write(  addr,  data );
}

void STLL_Radio_ReadFifo(uint8_t size, uint8_t* buffer)
{
  SX1276ReadBuffer( 0, buffer, size );
}

void STLL_Radio_WriteFifo(uint8_t size, uint8_t* buffer)
{
  SX1276WriteBuffer( 0, buffer, size );
}

void STLL_Radio_Init( void )
{
  RadioEvents_t events = {NULL};
  SX1276Init( &events );
}

void STLL_Radio_DeInit( void )
{
  RadioEvents_t events = {NULL};

  SX1276Init( &events );
  
  SX1276Write(0x40 , 0x01 );
}

void STLL_Radio_IoInit( void )
{
  SX1276IoInit();
  
  HW_GPIO_SetIrq( RADIO_DIO_0_PORT, RADIO_DIO_0_PIN, IRQ_PRIORITY_RADIO, RADIO_rx_packet_interrupt_handler );
  HW_GPIO_SetIrq( RADIO_DIO_4_PORT, RADIO_DIO_4_PIN, IRQ_PRIORITY_RADIO, STLL_RX_SYNC_IRQHandler_CB );
}

void STLL_Radio_IoDeInit( void )
{
  HW_GPIO_SetIrq( RADIO_DIO_0_PORT, RADIO_DIO_0_PIN, IRQ_PRIORITY_RADIO, NULL );  
  HW_GPIO_SetIrq( RADIO_DIO_1_PORT, RADIO_DIO_1_PIN, IRQ_PRIORITY_RADIO, NULL );
  HW_GPIO_SetIrq( RADIO_DIO_2_PORT, RADIO_DIO_2_PIN, IRQ_PRIORITY_RADIO, NULL );
  HW_GPIO_SetIrq( RADIO_DIO_3_PORT, RADIO_DIO_3_PIN, IRQ_PRIORITY_RADIO, NULL );
  HW_GPIO_SetIrq( RADIO_DIO_4_PORT, RADIO_DIO_4_PIN, IRQ_PRIORITY_RADIO, NULL );

  SX1276IoDeInit();
}

void STLL_Radio_SetOpMode(uint8_t opMode)
{
  SX1276SetOpMode( opMode );
}

void STLL_Radio_SetFreq( uint32_t Freq )
{
  SX1276SetChannel( Freq );
}

void STLL_RadioPowerSetEeprom( int8_t power)
{
  HW_EEPROM_WRITE( E2pData.TxPower, power) ;

}

int8_t STLL_RadioPowerGet( void )
{
  return E2pData.TxPower;
}

void STLL_RadioPowerSetBoard( int8_t power)
{
  SX1276SetRfTxPower( power );
}

STLL_flag STLL_WaitEndOfTxFrame( void )
{
  /* Wait that flag EOFTX_EVT is set*/
  SCH_WaitEvt( EOFTX_EVT );
  
  return STLL_SET;
}

void STLL_SetEndOfTxFrame( void )
{
  SCH_SetEvt( EOFTX_EVT );
}

/* SPI subgroup*/
void STLL_Transmit_DMA_Start( uint16_t *pDataSource, uint16_t Size)
{
  HW_SPI_Transmit_DMA_Start( pDataSource, Size);
}

void STLL_Transmit_DMA_Stop( void )
{
  HW_SPI_Transmit_DMA_Stop();
}

/* timer2 subgroup*/
void STLL_TIM2_Start( void )
{
  HW_TIM2_Start();
}

void STLL_TIM2_Stop( void )
{
  HW_TIM2_Stop();
}

void STLL_TIM2_SetPeriod( uint32_t period )
{
  HW_TIM2_SetPeriod( period );
}

uint32_t STLL_TIM2_GetPeriod(  void )
{
  return HW_TIM2_GetPeriod( );
}

/* low power subgroup*/

void STLL_LowPower(STLL_State State)
{
  if ( State == STLL_ENABLE)
  {
    LPM_SetStopMode( LPM_LIB_Id, LPM_Enable);
  }
  else
  {
    LPM_SetStopMode( LPM_LIB_Id, LPM_Disable);
  }
}

void STLL_SetClockSource( stll_clockType_e clocktype)
{
  if ( clocktype == HSI_SOURCE )
  {
    HW_SetHSIasSysClock() ;
  }
  else
  {
    HW_SetHSEasSysClock() ;
  }
}

int16_t STLL_SGFX_SX1276_GetSyncRssi(void)
{
  return meas_rssi_dbm;
}

STLL_flag STLL_WaitEndOfRxFrame( void )
{
  rxPacketReceived = STLL_RESET;
  
  SCH_ClrEvt( TIMOUT_EVT );
  
  SCH_WaitEvt( TIMOUT_EVT );
  
  return rxPacketReceived;
}

int16_t STLL_RxCarrierSenseGetRssi(void)
{
  return  (-( STLL_Radio_ReadReg( REG_RSSIVALUE ) >> 1 ) -13 + E2pData.rssi_cal) ;
}

void STLL_RxCarrierSenseInitStatus( void )
{
  /*Initialises the Flag*/
   rxCarrierSenseFlag =STLL_RESET;  
}

STLL_flag STLL_RxCarrierSenseGetStatus( void )
{
  return rxCarrierSenseFlag ;  
}

void OnTimerTimeoutCsEvt( void )
{
  rxCarrierSenseFlag = STLL_SET;
}

/* Private function  ------------------------------------------------*/
static void RADIO_rx_packet_interrupt_handler( void )
{
  rxPacketReceived = STLL_SET;  
  /*wakes up the MCU at line SCH_WaitEvt( TIMOUT_EVT );*/
  SCH_SetEvt( TIMOUT_EVT );  
}

static void STLL_RX_SYNC_IRQHandler_CB( void )
{
  meas_rssi_dbm =  -( ((int16_t) STLL_Radio_ReadReg( REG_RSSIVALUE )) >> 1 ) -13 + E2pData.rssi_cal;
}


/******************* (C) COPYRIGHT  STMicroelectronics *****END OF FILE****/
