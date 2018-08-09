/******************************************************************************
 * @file    hw_eeprom.h
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   contains all eeprom driver
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
#ifndef __HW_EEPROM_H__
#define __HW_EEPROM_H__

#include "hw_eeprom_conf.h"

#ifdef __cplusplus
 extern "C" {
#endif
/* Exported types ------------------------------------------------------------*/  
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
extern restorable_eeprom_t E2pData;

/* Exported macros -----------------------------------------------------------*/
/*!
 * @brief AT_EEPROM place the variable in section IO
 * @note in the scatter file, the following must be added to force section IO at 0x80800000
   EEPROM 0x08080000
   {
     EEPROM 0x08080000
     {
       .ANY ( "EEPROM")
     }
   } 
 * @retval None
 */
#if defined(__CC_ARM)
#define AT_EEPROM __attribute__((section("eeprom_sect"), zero_init))
#elif defined(__ICCARM__)
#define AT_EEPROM @ "eeprom_sect"
#else  /* __GNUC__ */
#define AT_EEPROM __attribute__((section(".eeprom_sect")))
#endif   


/* Exported functions ------------------------------------------------------- */ 

/*!
 * @brief Initializes the Eeprom
 * @note  DEFAULT_FACTORY_SETTINGS are written E2pData
 * @param  None
 * @retval None
 */
void HW_EEPROM_Init( void );

/*!
 * @brief Initializes the Eeprom
 * @note  DEFAULT_FACTORY_SETTINGS are written E2pData
 * @param  None
 * @retval None
 */
void HW_EEPROM_RestoreFs( void );

void HW_EEPROM_Unlock( void );

void HW_EEPROM_Lock( void );

#define HW_EEPROM_WRITE( VAR, X )                                             \
                         do{                                                  \
                             HW_EEPROM_Unlock();                              \
                             VAR = X;                                         \
                             HW_EEPROM_Lock();                                \
                         } while(0)
                         
                         
#ifdef __cplusplus
}
#endif

#endif /* __HW_EEPROM_H__ */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
