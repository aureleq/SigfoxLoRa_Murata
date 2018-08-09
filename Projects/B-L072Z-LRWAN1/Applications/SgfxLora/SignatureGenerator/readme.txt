/**
  @page Sigfox Readme file
 
  @verbatim
  ******************** (C) COPYRIGHT 2017 STMicroelectronics *******************
  * @file    Sigfox/SignatureGenerator/readme.txt 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    15-December-2017 
  * @brief   This binary file is used to generate a unique credentials per device 
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

  When compiling and loading the firmware using the default sigfox_data.h, 
  default sigfox credentials are loaded in the device. It only allows to test 
  the sigfox murata device locally in the lab.
  For the sigfox murata device to send data to the sigfox backend server, 
  two steps are required:
  - Personalization. Every sigfox device must be loaded with unique credentials per device. 
    This credentials are known as ID, PAC and Private Key. They are necessary 
    to activate the device and send data to the sigfox data server. This step is a process 
    for a muRata module user to obtain its credentials
  - Activation. Once the device is personalized, the device needs to be recorded 
    by the sigfox back-end server. This step requires to log-on the Sigfox backend server. 

  SignatureGenerator.bin is a binary file that can be used to generate a unique credentials 
  per device. It generates a 128 bits signature retrivable on UART pins PA2 and PA9.  

@par Directory contents 

  - SignatureGenerator/SignatureGenerator.bin

  - This example runs on STM32L072/82 embedded in the module
    
  - This application has been tested with STMicroelectronics:
    B-L072Z-LRWAN1 RevC boards 
  -Set Up:
    -------                 --------------------------  
    |     |                 |     B-L072Z-LRWAN1     | 
    | PC  |-->    ComPort<--|         with           |  
    |     |                 |      Murata module     | 
    -------                 --------------------------   

@par How to use it ? 

  SignatureGenerator.bin must be loaded in the device. 
  Loading can either be done by using ST-Link Utility or  by ‘drag&drop’.
  A com Port is emulated over the USB where a terminal can be connected. 

  After a hardware reset, the signature will be sent on the com ports. 
  The following should be display (note that the signature is device dependent):
  E.g.: signature = D33AA579E7B9D7209EAB354801574D15

  Getting credendials from Murata server
  Log onto myMurata.com online Portal and copy/paste the signature. 
  A device dependent sigfox_data.h (together with its equivalent binary format 
  sigfox_data.bin) will be sent by mail if the signature is authenticated.

  Loading your credentials in the Murata module
  They are two ways to load the credentials. 
  - 1st method: replace the default sigfox_data.h located in 
                Projects\B-L072Z-LRWAN1\Applications\Sgfx\ModemAT\inc 
                by the one received from Murata. 
                After project compilation and load, the device is personalized.
  - 2nd method: the other method is to overwrite the already loaded sigfox_data 
                located at 0x08017000. After the application with default sigfox_data 
                is loaded into the device, they can be erased and replaced directly 
                from using the sigfox_data.bin. To do so, the following ST-Link Utility 
                command line can be executed: 
                ST-LINK_CLI.exe -c swd ur -SE 736 -P sigfox_data.bin 0x08017000 -hardrst –v
                Note that option ‘-SE 736’ erases the sector 736 (@ 0x08017000).
                Once ST-LINK command is executed, the device is personalized.

  Warning: the credentials received from myMurata is device specific. 
  The received credentials can NOT be used on any other devices!

  Activation
  - In your terminal, use AT_commands  AT$ID?<CR> and  AT$PAC?<CR> from ATmodem application 
    in order to get SigFox ID & PAC. 
  - Then go to https://backend.sigfox.com/activate/
  - Click on muRata logo.
  - Select your country.
  - Enter your ID & PAC.
  - Enter your account details and click “Subscribe”.
  - Your kit is now activated on SigFox Network !


 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */