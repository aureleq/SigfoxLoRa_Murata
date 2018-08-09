/**
  @page Sigfox Readme file
 
  @verbatim
  ******************** (C) COPYRIGHT 2017 STMicroelectronics *******************
  * @file    Sigfox/Sgfx_push_button/readme.txt 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    30-April-2018 
  * @brief   This application is a simple demo of a Sigfox Modem connecting to 
  *          a Sigfox Network. 
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
   @endverbatim

@par Description

 This directory contains a set of source files that implements a simple sigfox node. 
 the node sends the  environemental sensor data on user button pressure to the 
 sigfox network. The Sigfox modem run on  B-L072Z-LRWAN1.
  ******************************************************************************

 On user push-button event or reset, this application reads the temperature,  
 humidity and atmospheric pressure from the sensors through I2C. These three data  
 (temperature, humidity, atmospheric pressure) are sent to the Sigfox™ network. 
 

@par Directory contents 

  - Sgfx_push_button/Inc/hw_conf.h                file to manage Cube SW family used and debug switch
  - Sgfx_push_button/Inc/stm32lXxx_hal_conf.h     Library Configuration file
  - Sgfx_push_button/Inc/stm32lXxx_it.h           Header for stm32lXxx_it.c
  - Sgfx_push_button/Inc/stm32lXxx_hw_conf.h      Header for stm32lXxx_hw_conf.c
  - Sgfx_push_button/Inc/hw_spi.h                 Header for hw_spi.c
  - Sgfx_push_button/Inc/hw_rtc.h                 Header for hw_rtc.c
  - Sgfx_push_button/Inc/hw_gpio.h                Header for hw_gpio.c
  - Sgfx_push_button/Inc/hw_tim2.h                Header for hw_tim2.c
  - Sgfx_push_button/Src/bsp.h                    bsp  header
  - Sgfx_push_button/Inc/hw.h                     group all hw interface
  - Sgfx_push_button/Inc/vcom.h                   interface to vcom.c 
  - Sgfx_push_button/Inc/tiny_sscanf.h            interface to tiny_sscanf.c 
  - Sgfx_push_button/Inc/tiny_printf.h            interface to tiny_printf.c 
  - Sgfx_push_button/Inc/debug.h                  interface to debug functionally
  - Sgfx_push_button/Inc/Comissioning.h           End device comissioning parameters
  - Sgfx_push_button/Inc/version .h               version file
  
  - Sgfx_push_button/Src/main.c                   Main program file
  - Sgfx_push_button/Src/stm32lXxx_it.c           STM32lXxx Interrupt handlers
  - Sgfx_push_button/Src/stm32lXxx_hal_msp.c      stm32lXxx specific hardware HAL code
  - Sgfx_push_button/Src/stm32lXxx_hw.c           stm32lXxx specific hardware driver code
  - Sgfx_push_button/Src/hw_spi.c                 spi driver
  - Sgfx_push_button/Src/hw_rtc.c                 rtc driver
  - Sgfx_push_button/Src/hw_gpio.c                gpio driver
  - Sgfx_push_button/Src/bsp.c                    read the bsp sensor
  - Sgfx_push_button/Inc/hw_tim2.c                timer 2 driver
  - Sgfx_push_button/Src/vcom.c                   virtual com port interface on Terminal
  - Sgfx_push_button/Inc/tiny_sscanf.c            low foot print sscanf
  - Sgfx_push_button/Inc/tiny_printf.c            low foot print printf
  - Sgfx_push_button/Src/debug.c                  debug driver
 
@par Hardware and Software environment 

  - This example runs on STM32L072/82 embedded in the module
    
  - This application has been tested with STMicroelectronics:
    B-L072Z-LRWAN1 RevC boards 
  -Set Up:

             --------------------------  V    V  --------------------------
             |     B-L072Z-LRWAN1     |  |    |  |      Sigfox Netork     |
             |         with           |  |    |  |                        |
   ComPort<--|      Sigfox button     |--|    |--|                        |-->Web Server
             |       end-node         |          |                        |
             --------------------------          --------------------------

  - X_NUCLEO_IKS01A2 can be plug on the top if user prefers to use its external sensors.
    In that case the line /* #define LOW_POWER_DISABLE */ should be uncommented in hw_conf.h  

  - Defining DEBUG in hw_conf.h allows keeping the debugger attached
    If DEBUG flag is not defined (hw_conf.h) the debugger disconnects in order to reduce power consumption

  - Depending on tool chain in order to connect the debugger again (when in low power) can be necessary 
    to keep the reset button pressed when starting the next download or
    to erase the chip memory. This to avoid the device to enter low power before debugger connection.
    It can also be necessary to reset the board after the download.

  - Defining TRACE in hw_conf.h allows to see the activity by connecting with an hyperterminal 
    (DEBUG should be defined too)

  - Defining the flag STDBY_ON in main.c the device goes in standby after sending data.
    This allows to reduce further the consumption by a factor 3.
    Instead of the push button, the reset button shall be use to wake up the MCU and send new data.

  - By default this application works for RC=1 (Radio Configuration Zone = Europe, Oman, South Africa)
    For changing the zone adapt following definition in main.c: #define APPLI_RC   ST_RC1

  - In order to access Sigfox Network each device needs to be "Personalised" and "Activate".
    See ../SignatureGenerator/readme.txt
    Otherwise (for test purposes) a Sigfox Network Emulator can be used instead of real Sigfox Network 
    by replacing main.c @ line 131 with:  HW_EEPROM_WRITE( E2pData.SgfxKey, CREDENTIALS_KEY_PRIVATE);

@par How to use it ? 
In order to make the program work, you must do the following :
  - Open your preferred toolchain 
  - Rebuild all files, erase the chip memory/eeprom and load your image into target memory
  - Run the example
  - JP9 must have pin 1 and 2 connected (TCXO_VCC managed by MCU) 
  - The solder bridge SB26 must be closed.(DIO4 from radio connected to PA5)
  - The solder bridge SB13 must be closed. (TCXO_OUT connected to PH0-OSC-IN)
To see the logs on hyperterminal:
  - Open 1 Terminal connected to the sigfox modem 
  - Terminal Config = 9600, 8b, 1 stopbit, no parity, no flow control ( in src/vcom.c)
  
   
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
