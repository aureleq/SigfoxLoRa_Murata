/******************************************************************************
 * @file    se_nvm.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   manages SE nvm datas
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
#include <stdint.h>
#include <string.h>
#include "hw_eeprom.h"
#include "se_nvm.h"
#include "sgfx_credentials.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

sfx_u8 SE_NVM_get(sfx_u8 read_data[SFX_SE_NVMEM_BLOCK_SIZE])
{
  int i;
  for ( i=0; i< SFX_SE_NVMEM_BLOCK_SIZE;i++)
  {
    read_data[i] =E2pData.lib_sgfx_se_nvm[i];
  }

  return SFX_ERR_NONE;
}

sfx_u8 SE_NVM_set(sfx_u8 data_to_write[SFX_SE_NVMEM_BLOCK_SIZE])
{
  int i;
  for ( i=0; i< SFX_SE_NVMEM_BLOCK_SIZE;i++)
  {
    HW_EEPROM_WRITE( E2pData.lib_sgfx_se_nvm[i], data_to_write[i]);
  }
  return SFX_ERR_NONE;
}

sfx_key_type_t SE_NVM_get_key_type( void )
{
  return  E2pData.SgfxKey;
}

void  SE_NVM_set_key_type( sfx_key_type_t keyType )
{
  HW_EEPROM_WRITE(  E2pData.SgfxKey,  keyType );
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
