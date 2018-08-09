/******************************************************************************
 * @file    hw_eeprom_conf.h
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   configures eeprom driver
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
#ifndef __HW_EEPROM_CONF_H__
#define __HW_EEPROM_CONF_H__

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "st_sigfox_api.h"
#include "se_nvm.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{ 
#if defined(__CC_ARM) /* when changing tool chain the EEPROM shall be re-initialized */
  E2P_SET = 0xAA ,  /* Set by Keil */
#elif defined(__ICCARM__)
  E2P_SET = 0xBB ,  /* Set by IAR */
#else  /* __GNUC__ */
  E2P_SET = 0xCC ,  /* Set by GCC */
#endif  
  E2P_RST= 0     /* EEPROM has not yet been set or has been erased */
}E2P_flagStatus_t;
   
typedef struct {
 int16_t rssi_cal;
 E2P_flagStatus_t AtEcho;
 int8_t TxPower;
 st_sfx_rc_t SgfxRc;
 sfx_key_type_t SgfxKey;
 sfx_u32 macroch_config_words_rc2[3];
 sfx_u32 macroch_config_words_rc3[3];
 sfx_u32 macroch_config_words_rc4[3];
 uint8_t lib_sgfx_se_nvm[SFX_SE_NVMEM_BLOCK_SIZE];
 uint8_t lib_sgfx_mcu_nvm[SFX_NVMEM_BLOCK_SIZE];
} restorable_eeprom_t;

/* Exported constants --------------------------------------------------------*/
#define DEFAULT_FACTORY_SETTINGS { 0, /*calibartion RSSI*/ \
                                   E2P_SET,            /* AtEcho  =  Set */   \
                                   14,                 /* TxPower = 14dBm in RC1, 20dBm in Rc2 and RC4 */   \
                                   ST_RC1,             /* SgfxRcz = RC1  */   \
                                   CREDENTIALS_KEY_PRIVATE,/*Private KEY*/     \
                                   {RC2_SET_STD_CONFIG_SM_WORD_0, RC2_SET_STD_CONFIG_SM_WORD_1, RC2_SET_STD_CONFIG_SM_WORD_2 }, \
                                   RC3C_CONFIG, \
                                   {RC4_SET_STD_CONFIG_SM_WORD_0, RC4_SET_STD_CONFIG_SM_WORD_1, RC4_SET_STD_CONFIG_SM_WORD_2 }, \
                                   {0xFF,0,0,0x0F,0xFF},  \
                                   {0,0,0,0} }

/* External variables --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#ifdef __cplusplus
}
#endif

#endif /* __HW_EEPROM_CONF_H__ */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
