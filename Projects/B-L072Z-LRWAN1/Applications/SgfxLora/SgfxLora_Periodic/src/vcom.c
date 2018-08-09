// @aureleq: Not Merged. TBC
/******************************************************************************
 * @file    vcom.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   manages virtual com port
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
#include <stdarg.h>
#include "hw.h"
#include "vcom.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_lpuart.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_dma.h"
#include "low_power_manager.h"
#include "tiny_vsnprintf.h"
#include "scheduler.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef TRACE
#define BUFSIZE_TX 512
#else
#define BUFSIZE_TX 128
#endif

#define BUFSIZE_RX 8

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

typedef struct {
  char buff[BUFSIZE_TX];   /* buffer to transmit */
  __IO int iw;             /* 1st free index in BuffTx */
  int ir;                  /* next char to read in buffTx */
  __IO int dmabuffSize;
} circ_buff_tx_t;

typedef struct {
  char buff[BUFSIZE_RX];   /* buffer to receive */
  __IO int iw;             /* 1st free index in BuffRx */
  int ir;                  /* next char to read in buffRx */
} circ_buff_rx_t;

static struct {
  circ_buff_rx_t rx;        /* UART rx buffer context*/
  circ_buff_tx_t tx;        /* UART tx buffer context */
} uart_context;             /* UART context*/

static struct {
  char buffer[10];        /* low power buffer*/
  int len;                /* low power buffer length */
} SleepBuff;              /* low power structure*/


/* Private function prototypes -----------------------------------------------*/
/**
 * @brief  Takes one character that has been received and save it in uart_context.buffRx
 * @param  received character
 */
static void receive(char rx);

/**
 * @brief  prepare DMA print
 * @param  None
 */
static void vcom_PrintDMA(void);

/**
 * @brief  Starts DMA transfer into UART
 * @param  buffer adress to start
 * @param  length of buffer to transfer
 */
static void vcom_StartDMA(char* buf, uint16_t buffLen);


/* Functions Definition ------------------------------------------------------*/

void vcom_Init(void)
{
  LL_LPUART_InitTypeDef LPUART_InitStruct;
  /*## Configure the UART peripheral ######################################*/


  LL_RCC_SetLPUARTClockSource(LL_RCC_LPUART1_CLKSOURCE_HSI);
  UARTX_CLK_ENABLE();
  vcom_IoInit();

    /*##-3- Configure the NVIC for UART ########################################*/
  /* NVIC for UART */
  HAL_NVIC_SetPriority(UARTX_IRQn, IRQ_PRIORITY_USARTX, 0);
  HAL_NVIC_EnableIRQ(UARTX_IRQn);

  LPUART_InitStruct.BaudRate = 9600;
  LPUART_InitStruct.DataWidth = LL_LPUART_DATAWIDTH_8B;
  LPUART_InitStruct.StopBits = LL_LPUART_STOPBITS_1;
  LPUART_InitStruct.Parity = LL_LPUART_PARITY_NONE;
  LPUART_InitStruct.TransferDirection = LL_LPUART_DIRECTION_TX_RX;
  LPUART_InitStruct.HardwareFlowControl = LL_LPUART_HWCONTROL_NONE;

  LL_LPUART_Init(UARTX, &LPUART_InitStruct);
    /* Configuring the LPUART specific LP feature - the wakeup from STOP */
  LL_LPUART_EnableInStopMode(UARTX);

  LL_LPUART_Enable(UARTX);

  while (LL_LPUART_IsActiveFlag_TEACK(UARTX) == RESET);
  while (LL_LPUART_IsActiveFlag_REACK(UARTX) == RESET);
}

void vcom_DeInit(void)
{
  LL_LPUART_DeInit(UARTX);
}

void vcom_IoInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Alternate = UARTX_TX_AF;

  HW_GPIO_Init(UARTX_TX_GPIO_PORT, UARTX_TX_PIN, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Alternate = UARTX_RX_AF;

  HW_GPIO_Init(UARTX_RX_GPIO_PORT, UARTX_RX_PIN, &GPIO_InitStruct);
}

void vcom_IoDeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;

  HW_GPIO_Init(UARTX_TX_GPIO_PORT, UARTX_TX_PIN, &GPIO_InitStructure);

  HW_GPIO_Init(UARTX_RX_GPIO_PORT, UARTX_RX_PIN, &GPIO_InitStructure);
}

void vcom_ReceiveInit(void)
{
  /* enable RXNE */
  LL_LPUART_EnableIT_RXNE(UARTX);
  /* WakeUp from stop mode on start bit detection*/
  LL_LPUART_SetWKUPType(UARTX, LL_LPUART_WAKEUP_ON_STARTBIT);

  LL_LPUART_EnableIT_WKUP(UARTX);
  /* Enable the UART Parity Error Interrupt */
  LL_LPUART_EnableIT_PE(UARTX);
  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  LL_LPUART_EnableIT_ERROR(UARTX);
}


void vcom_Send( const char *format, ... )
{
  va_list args;
  va_start(args, format);
 
  uint8_t lenTop;
  char tempBuff[BUFSIZE_TX];
  
  int32_t freebuff;
  int len=0;

  if (SleepBuff.len!=0)
  {
    /*if SleepBuff has been filled before entering lowpower, prepend it */
    memcpy(&tempBuff[0], SleepBuff.buffer, SleepBuff.len);
    len = tiny_vsnprintf_like(&tempBuff[SleepBuff.len], sizeof(tempBuff), format, args); 
    len += SleepBuff.len;
    /*erase SleepBuff*/
    memset(SleepBuff.buffer, 0,sizeof(SleepBuff.buffer) );
    SleepBuff.len=0;
  }
  else
  {
  /*convert into string at buff[0] of length iw*/
    len = tiny_vsnprintf_like(&tempBuff[0], sizeof(tempBuff), format, args); 
  }
  
  /* calculate free buffer size*/
  freebuff = BUFSIZE_TX - ((BUFSIZE_TX+uart_context.tx.iw-uart_context.tx.ir)%BUFSIZE_TX);

  while (len>freebuff)
  {
    /*wait enough free char in buff*/
    /*1 char at 9600 lasts approx 1ms*/
    /*even in the while loop, DMA IRQ handler will update iw-uart_context.tx.ir*/
    DelayMs(1);
    freebuff = BUFSIZE_TX - ((BUFSIZE_TX+uart_context.tx.iw-uart_context.tx.ir)%BUFSIZE_TX);
    
    /*in case vcom_Send called by higher or equal priority irq than DMA priority, 
     vcom_Dma_IRQHandler should be triggered by software*/
    BACKUP_PRIMASK();

    DISABLE_IRQ( );
    if (HAL_NVIC_GetPendingIRQ(DMA1_Channel4_5_6_7_IRQn)== 1)
    {
      vcom_Dma_IRQHandler();
    }
    ENABLE_IRQ( );
  }

  if ((uart_context.tx.iw+len)<BUFSIZE_TX)
  {
    memcpy( &uart_context.tx.buff[uart_context.tx.iw], &tempBuff[0], len);
    uart_context.tx.iw+=len;
  }
  else
  {
    /*cut buffer in high/low part*/
    lenTop= BUFSIZE_TX - (uart_context.tx.iw);
    /*copy beginning at top part of the circ buf*/
    memcpy( &uart_context.tx.buff[uart_context.tx.iw], &tempBuff[0], lenTop);
     /*copy end at bottom part of the circ buf*/
    memcpy( &uart_context.tx.buff[0], &tempBuff[lenTop], len-lenTop);
    uart_context.tx.iw = len-lenTop;
  }

  if (! LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_7) )
  {
    vcom_PrintDMA();
  }
  
  
  va_end(args);
}

void vcom_Send_Lp(const char *format, ...)
{
  /*special vcomsend to avoid waking up any time MCU goes to sleep*/
  va_list args;
  va_start(args, format);
  
  SleepBuff.len = tiny_vsnprintf_like(&SleepBuff.buffer[0], sizeof(SleepBuff.buffer), format, args); 
  
  va_end(args);
}

FlagStatus IsNewCharReceived(void)
{
  FlagStatus status;
  
  BACKUP_PRIMASK();
  DISABLE_IRQ();
  
  status = ((uart_context.rx.iw == uart_context.rx.ir) ? RESET : SET);
  
  RESTORE_PRIMASK();
  return status;
}

uint8_t GetNewChar(void)
{
  uint8_t NewChar;

  BACKUP_PRIMASK();
  DISABLE_IRQ();

  NewChar = uart_context.rx.buff[uart_context.rx.ir];
  uart_context.rx.ir = (uart_context.rx.ir + 1) % sizeof(uart_context.rx.buff);

  RESTORE_PRIMASK();
  return NewChar;
}

void vcom_IRQHandler(void)
{
  if ( LL_LPUART_IsActiveFlag_TC(UARTX) && (LL_LPUART_IsEnabledIT_TC(UARTX) != RESET) )/*tx*/
  {
    /*last uart char has just been sent out to terminal*/
    LL_LPUART_ClearFlag_TC(UARTX);
    /*enable lowpower since finished*/
    LPM_SetStopMode(LPM_UART_TX_Id, LPM_Enable);
  }
  /*rx*/
  {
    __IO int rx_ready = 0;
    char rx = AT_ERROR_RX_CHAR;
    
    /* UART Wake Up interrupt occured ------------------------------------------*/
    if (LL_LPUART_IsActiveFlag_WKUP(UARTX) && (LL_LPUART_IsEnabledIT_WKUP(UARTX) != RESET))
    {
      LL_LPUART_ClearFlag_WKUP(UARTX);

      /* forbid stop mode */
      LPM_SetStopMode(LPM_UART_RX_Id, LPM_Disable);
    }

    if (LL_LPUART_IsActiveFlag_RXNE(UARTX) && (LL_LPUART_IsEnabledIT_RXNE(UARTX) != RESET))
    {
      /* no need to clear the RXNE flag because it is auto cleared by reading the data*/
      rx = LL_LPUART_ReceiveData8(UARTX);
      rx_ready = 1;
      
      /* allow stop mode*/
      LPM_SetStopMode(LPM_UART_RX_Id, LPM_Enable);
    }

    if (LL_LPUART_IsActiveFlag_PE(UARTX) || LL_LPUART_IsActiveFlag_FE(UARTX) || LL_LPUART_IsActiveFlag_ORE(UARTX) || LL_LPUART_IsActiveFlag_NE(UARTX))
    {
      DBG_PRINTF("Error when receiving\n\r");
      /* clear error IT */
      LL_LPUART_ClearFlag_PE(UARTX);
      LL_LPUART_ClearFlag_FE(UARTX);
      LL_LPUART_ClearFlag_ORE(UARTX);
      LL_LPUART_ClearFlag_NE(UARTX);
      
      rx = AT_ERROR_RX_CHAR;
      
      rx_ready = 1;
    }
    
    if (rx_ready == 1)
    {
      receive(rx);
    }
  }
}

static void receive(char rx)
{
  int next_free;

  /** no need to clear the RXNE flag because it is auto cleared by reading the data*/
  uart_context.rx.buff[uart_context.rx.iw] = rx;
  next_free = (uart_context.rx.iw + 1) % sizeof(uart_context.rx.buff);
  if (next_free != uart_context.rx.iw)
  {
    /* this is ok to read as there is no buffer overflow in input */
    uart_context.rx.iw = next_free;
  }
  else
  {
    /* force the end of a command in case of overflow so that we can process it */
    uart_context.rx.buff[uart_context.rx.iw] = '\r';
    DBG_PRINTF("uart_context.buffRx buffer overflow %d\r\n");
  }
  SCH_SetTask(VCOM_TASK);
}

void vcom_Dma_IRQHandler( void )
{
  if (LL_DMA_IsActiveFlag_TC7(DMA1) )
  {
    /*clear interrupt and flag*/
    LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_7);
    
    LL_DMA_ClearFlag_TC7(DMA1);
    /* update tx read index*/
    uart_context.tx.ir += uart_context.tx.dmabuffSize;
    
    uart_context.tx.ir = (uart_context.tx.ir)%BUFSIZE_TX;
    
    LL_DMA_DisableChannel( DMA1, LL_DMA_CHANNEL_7);
    
    LL_LPUART_DisableDMAReq_TX(UARTX);
  }
  if ( uart_context.tx.ir!= uart_context.tx.iw)
  {
    /*continue if more has been written in buffer meanwhile*/
    vcom_PrintDMA();
  }
}

static void vcom_PrintDMA(void)
{
  BACKUP_PRIMASK();
  
  DISABLE_IRQ( );
  
  uint16_t write_idx=  (uart_context.tx.iw);
  uint16_t read_idx=   (uart_context.tx.ir);
  /*shall not go in stop mode while printing*/
  LPM_SetStopMode(LPM_UART_TX_Id, LPM_Disable);

  if (write_idx > read_idx)
  {
    /*contiguous buffer[ir..iw]*/
    uart_context.tx.dmabuffSize= write_idx - read_idx;

    vcom_StartDMA( &uart_context.tx.buff[read_idx], uart_context.tx.dmabuffSize);
  }
  else
  {
    /*[ir:BUFSIZE_TX-1] and [0:iw]. */
     uart_context.tx.dmabuffSize= BUFSIZE_TX-read_idx;
     /*only [ir:BUFSIZE_TX-1] sent, rest will be sent in dma  handler*/
     vcom_StartDMA( &uart_context.tx.buff[read_idx], uart_context.tx.dmabuffSize);
  }
  RESTORE_PRIMASK();
}

static void vcom_StartDMA(char* buf, uint16_t buffLen)
{
  LL_DMA_InitTypeDef DMA_InitStruct;
  /*switch dma clock ON*/
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  /*dma initialisation*/  
  DMA_InitStruct.Direction= LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStruct.PeriphOrM2MSrcAddress=(uint32_t) &UARTX->TDR;
  DMA_InitStruct.PeriphOrM2MSrcDataSize= LL_DMA_PDATAALIGN_BYTE;
  DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;  
  DMA_InitStruct.Mode= LL_DMA_MODE_NORMAL;
  DMA_InitStruct.MemoryOrM2MDstAddress= (uint32_t) buf;
  DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStruct.MemoryOrM2MDstDataSize= LL_DMA_MDATAALIGN_BYTE;  
  DMA_InitStruct.NbData= buffLen;  
  DMA_InitStruct.PeriphRequest=LL_DMA_REQUEST_5;
  DMA_InitStruct.Priority=LL_DMA_PRIORITY_LOW;
  LL_DMA_Init(DMA1, LL_DMA_CHANNEL_7,&DMA_InitStruct );
    /*enable DMA transmit complete interrupt*/
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_7);
  /* enable DMA nvic*/
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, IRQ_PRIORITY_USARTX, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
  /* enable LPUART DMA request*/
  LL_LPUART_EnableDMAReq_TX(UARTX);
  /*enable DMA channel*/
  LL_DMA_EnableChannel( DMA1, LL_DMA_CHANNEL_7); 
  /*enable LPUART transmitt complete interrupt*/
  LL_LPUART_EnableIT_TC(UARTX);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
