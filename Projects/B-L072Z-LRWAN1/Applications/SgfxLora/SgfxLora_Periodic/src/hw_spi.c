// @aureleq Merge with i-cube-lrwan v1.1.2
/*******************************************************************************
 * @file    hw_spi.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   manages the SPI interface
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
#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_spi.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static SPI_HandleTypeDef SpiHandle;

/* Private function prototypes -----------------------------------------------*/

/*!
 * @brief Calculates Spi Divisor based on Spi Frequency and Mcu Frequency
 *
 * @param [IN] Spi Frequency
 * @retval Spi divisor
 */
static uint32_t SpiFrequency( uint32_t hz );

/* Exported functions ---------------------------------------------------------*/

/*!
 * @brief Initializes the SPI object and MCU peripheral
 *
 * @param [IN] none
 */
void HW_SPI_Init( void )
{

  /*##-1- Configure the SPI peripheral */
  /* Set the SPI parameters */

  SpiHandle.Instance = SPI1;

  // aureleq: 10000000 for lrwan
  SpiHandle.Init.BaudRatePrescaler = SpiFrequency( 16000000 );
  SpiHandle.Init.Direction      = SPI_DIRECTION_2LINES;
  SpiHandle.Init.Mode           = SPI_MODE_MASTER;
  SpiHandle.Init.CLKPolarity    = SPI_POLARITY_LOW;
  SpiHandle.Init.CLKPhase       = SPI_PHASE_1EDGE;
  SpiHandle.Init.DataSize       = SPI_DATASIZE_8BIT;
  SpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.FirstBit       = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.NSS            = SPI_NSS_SOFT;
  SpiHandle.Init.TIMode         = SPI_TIMODE_DISABLE;

  //SPI_CLK_ENABLE(); 

  if(HAL_SPI_Init( &SpiHandle) != HAL_OK)
  {
    /* Initialization Error */
     Error_Handler();
  }

  /*##-2- Configure the SPI GPIOs */
  HW_SPI_IoInit(  );
}


/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */



/*!
 * @brief De-initializes the SPI object and MCU peripheral
 *
 * @param [IN] none
 */
void HW_SPI_DeInit( void )
{
  HAL_SPI_DeInit( &SpiHandle);

    /*##-1- Reset peripherals ####*/
  __HAL_RCC_SPI1_FORCE_RESET();

  __HAL_RCC_SPI1_RELEASE_RESET();

  /*##-2- Configure the SPI GPIOs */
  HW_SPI_IoDeInit( );
}

void HW_SPI_IoInit( void )
{
  GPIO_InitTypeDef initStruct={0};


  initStruct.Mode =GPIO_MODE_AF_PP;
  initStruct.Pull = GPIO_PULLDOWN;
  initStruct.Speed = GPIO_SPEED_HIGH;
  initStruct.Alternate= SPI1_AF ;

  HW_GPIO_Init( RADIO_SCLK_PORT, RADIO_SCLK_PIN, &initStruct);
  HW_GPIO_Init( RADIO_MISO_PORT, RADIO_MISO_PIN, &initStruct);
  HW_GPIO_Init( RADIO_MOSI_PORT, RADIO_MOSI_PIN, &initStruct);

  initStruct.Mode = GPIO_MODE_OUTPUT_PP;
  initStruct.Pull = GPIO_PULLUP;

  HW_GPIO_Init(  RADIO_NSS_PORT, RADIO_NSS_PIN, &initStruct );

  HW_GPIO_Write ( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );
}


void HW_SPI_IoNssSwInit( void )
{
  /*in Software Mode the SPi is 8 bits: 8bits Address and then 8bits Data*/
  LL_SPI_SetDataWidth(SpiHandle.Instance,  LL_SPI_DATAWIDTH_8BIT );


  RADIO_NSS_GPIO_CLK_ENABLE();
  LL_GPIO_SetPinMode(RADIO_NSS_PORT, RADIO_NSS_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(RADIO_NSS_PORT, RADIO_NSS_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(RADIO_NSS_PORT, RADIO_NSS_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_NOPULL);

  LL_GPIO_SetOutputPin(RADIO_NSS_PORT, RADIO_NSS_PIN);

}

void HW_SPI_IoNssHwInit( void )
{
  /*in Hardware Mode the SPi is 16 bit (Ad<<8 | Data)*/
  LL_SPI_SetDataWidth(SpiHandle.Instance,  LL_SPI_DATAWIDTH_16BIT );

  RADIO_NSS_GPIO_CLK_ENABLE();
  LL_GPIO_SetPinMode(RADIO_NSS_PORT, RADIO_NSS_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinOutputType(RADIO_NSS_PORT, RADIO_NSS_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(RADIO_NSS_PORT, RADIO_NSS_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(RADIO_NSS_PORT, RADIO_NSS_PIN, GPIO_PULLUP);
  LL_GPIO_SetAFPin_8_15(RADIO_NSS_PORT, RADIO_NSS_PIN, TIMx_GPIO_AF_CHANNEL1);
#if 0
  __GPIOA_CLK_ENABLE();
  LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinOutputType(GPIOA, GPIO_PIN_1, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOA, GPIO_PIN_1, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOA, GPIO_PIN_1, GPIO_PULLUP);
  LL_GPIO_SetAFPin_0_7(GPIOA, GPIO_PIN_1, GPIO_AF2_TIM2);
#endif
}


void HW_SPI_IoDeInit( void )
{
  GPIO_InitTypeDef initStruct={0};

  initStruct.Mode =GPIO_MODE_OUTPUT_PP;

  initStruct.Pull =GPIO_PULLDOWN  ;
  HW_GPIO_Init ( RADIO_MOSI_PORT, RADIO_MOSI_PIN, &initStruct );
  HW_GPIO_Write( RADIO_MOSI_PORT, RADIO_MOSI_PIN, 0 );

  initStruct.Pull =GPIO_PULLDOWN;
  HW_GPIO_Init ( RADIO_MISO_PORT, RADIO_MISO_PIN, &initStruct );
  HW_GPIO_Write( RADIO_MISO_PORT, RADIO_MISO_PIN, 0 );

  initStruct.Pull =GPIO_PULLDOWN  ;
  HW_GPIO_Init ( RADIO_SCLK_PORT, RADIO_SCLK_PIN, &initStruct );
  HW_GPIO_Write(  RADIO_SCLK_PORT, RADIO_SCLK_PIN, 0 );

  initStruct.Pull = GPIO_PULLUP;
  HW_GPIO_Init ( RADIO_NSS_PORT, RADIO_NSS_PIN , &initStruct );
  HW_GPIO_Write( RADIO_NSS_PORT, RADIO_NSS_PIN , 1 );
}

uint16_t HW_SPI_InOut( uint16_t txData )
{
  uint16_t rxData ;

  HAL_SPI_TransmitReceive( &SpiHandle, ( uint8_t * ) &txData, ( uint8_t* ) &rxData, 1, HAL_MAX_DELAY);

  return rxData;
}

void HW_SPI_Transmit_DMA_Start( uint16_t *pDataSource, uint16_t Size)
{
  HW_SPI_IoNssHwInit() ;

  HAL_SPI_Transmit_DMA(&SpiHandle, (uint8_t *) pDataSource, Size);
}

void HW_SPI_Transmit_DMA_Stop( void )
{
  DMA_HandleTypeDef *hdma= SpiHandle.hdmatx;

  HAL_SPI_DMAStop( &SpiHandle );

  __HAL_DMA_DISABLE_IT(hdma, DMA_IT_HT);

  __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TC);

  HW_SPI_IoNssSwInit( );
}

void HW_SPI_ClearIRQ( void )
{
  DMA_HandleTypeDef *hdma= SpiHandle.hdmatx;

  /* Clear the transfer complete flag */
  __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma) );

  /* Clear the Half transfer complete flag */
  __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma));
}

/* Private functions ---------------------------------------------------------*/

static uint32_t SpiFrequency( uint32_t hz )
{
  uint32_t divisor = 0;
  uint32_t SysClkTmp = SystemCoreClock;
  uint32_t baudRate;

  while( SysClkTmp > hz)
  {
    divisor++;
    SysClkTmp= ( SysClkTmp >> 1);

    if (divisor >= 7)
      break;
  }

  // aureleq: lrwan version different
  baudRate = divisor << SPI_CR1_BR_Pos;

  return baudRate;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
