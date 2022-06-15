# @aureleq - July 2018
# Makefile to compile Sigfox examples for ST B-L072Z-LRWAN1 eval board
#
# Based on Makefile for mkrfox1300 firmware by:
#   Copyright 2018 Fabio Baltieri (fabio.baltieri@gmail.com)
#
# Based on the original ben-wpan code written by:
#   Werner Almesberger, Copyright 2010-2011
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
SHELL := /bin/bash

NAME = main

CROSS_COMPILE = arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE = $(CROSS_COMPILE)size
FLASH = st-flash

LINKER_SCRIPT = Projects/B-L072Z-LRWAN1/Applications/SgfxLora/SgfxLora_Periodic/SW4STM32/mlm32l07x01/STM32L072CZYx_FLASH.ld

CFLAGS  = -g -Os -Wall -Wextra -Wno-unused-parameter \
	  -mcpu=cortex-m0plus -mthumb \
	  -std=c99 -ffunction-sections -fdata-sections -march=armv6-m -mthumb -mabi=aapcs -mfloat-abi=soft

USER_LIBS_DIR = \
		-L"./Middlewares/Third_Party/Sgfx/Crypto" \
		-L"./Middlewares/Third_Party/Sgfx/SigfoxLib" \
		-L"./Middlewares/Third_Party/Sgfx/SigfoxLibTest" \
		-L"./Projects/B-L072Z-LRWAN1/Applications/SgfxLora/Common/cwmx1zzabz/Credentials_libs" \
		-L"./Projects/B-L072Z-LRWAN1/Applications/SgfxLora/Common/cwmx1zzabz/Radio_Sigfox_Driver"

# Sigfox stack libs & credentials management
USER_LIBS = \
	-lSgfxAddonV020_CM0_GCC -lSgfxCoreV231_CM0_GCC -lSgfxCmacV100_CM0_GCC \
	-lSgfxSTModemSx1276V123_CM0_GCC -lSgfxCredentialsV011_CM0_GCC

LDFLAGS = \
	  -Wl,--gc-sections,--no-undefined \
	  -T$(LINKER_SCRIPT) --specs=nosys.specs -specs=nano.specs\
	  -lm

DEFINES = \
	  -DSTM32L072xx \
	  -DUSE_B_L072Z_LRWAN1 \
	  -DUSE_FULL_LL_DRIVER \
		-DUSE_RADIO_LL_DRIVER\
		-DUSE_HAL_DRIVER \
	  -DREGION_EU868

INCLUDES = \
	   -IProjects/B-L072Z-LRWAN1/Applications/SgfxLora/Common/cwmx1zzabz/Credentials_libs \
		 -IProjects/B-L072Z-LRWAN1/Applications/SgfxLora/Common/cwmx1zzabz/Radio_Sigfox_Driver \
		 -IProjects/B-L072Z-LRWAN1/Applications/SgfxLora/SgfxLora_Periodic/inc \
		 -IDrivers/BSP/B-L072Z-LRWAN1 \
	   -IDrivers/BSP/MLM32L07X01 \
		 -IDrivers/BSP/X_NUCLEO_IKS01A2 \
		 -IDrivers/BSP/Components/Common \
	   -IDrivers/BSP/Components/sx1276 \
		 -IDrivers/BSP/Components/hts221 \
		 -IDrivers/BSP/Components/lis3mdl \
		 -IDrivers/BSP/Components/lps22hb \
		 -IDrivers/BSP/Components/lps25hb \
		 -IDrivers/BSP/Components/lsm6ds0 \
		 -IDrivers/BSP/Components/lsm6ds3 \
		 -IDrivers/BSP/Components/lsm6dsl \
		 -IDrivers/BSP/Components/lsm303agr \
	   -IDrivers/CMSIS/Device/ST/STM32L0xx/Include \
	   -IDrivers/CMSIS/Include \
		 -IDrivers/STM32L0xx_HAL_Driver/Inc \
	   -IMiddlewares/Third_Party/Sgfx/Crypto \
	   -IMiddlewares/Third_Party/Sgfx/SigfoxLib \
	   -IMiddlewares/Third_Party/Sgfx/SigfoxLibTest \
	   -IMiddlewares/Third_Party/Sgfx/utils \
		 -IMiddlewares/Third_Party/Lora/Mac/region \
		 -IMiddlewares/Third_Party/Lora/Crypto \
		 -IMiddlewares/Third_Party/Lora/Mac \
		 -IMiddlewares/Third_Party/Lora/Phy \
		 -IMiddlewares/Third_Party/Lora/Utilities \
		 -IMiddlewares/Third_Party/Lora/Core


OBJS = \
			Drivers/BSP/B-L072Z-LRWAN1/b-l072z-lrwan1.o \
			Drivers/CMSIS/Device/ST/STM32L0xx/Source/Templates/system_stm32l0xx.o \
			Drivers/BSP/Components/hts221/HTS221_Driver.o \
			Drivers/BSP/Components/hts221/HTS221_Driver_HL.o \
			Drivers/BSP/Components/lps22hb/LPS22HB_Driver.o \
			Drivers/BSP/Components/lps22hb/LPS22HB_Driver_HL.o \
			Drivers/BSP/Components/lps25hb/LPS25HB_Driver.o \
			Drivers/BSP/Components/lps25hb/LPS25HB_Driver_HL.o \
			Drivers/BSP/Components/sx1276/sx1276.o \
			Drivers/BSP/X_NUCLEO_IKS01A2/x_nucleo_iks01a2.o \
			Drivers/BSP/X_NUCLEO_IKS01A2/x_nucleo_iks01a2_humidity.o \
			Drivers/BSP/X_NUCLEO_IKS01A2/x_nucleo_iks01a2_pressure.o \
			Drivers/BSP/X_NUCLEO_IKS01A2/x_nucleo_iks01a2_temperature.o \
			Drivers/BSP/MLM32L07X01/mlm32l07x01.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_adc.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_adc_ex.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_cortex.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_dma.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_flash.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_flash_ex.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_gpio.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_i2c.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_pwr.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_pwr_ex.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rcc.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rcc_ex.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rtc.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rtc_ex.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_spi.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_tim.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_tim_ex.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_uart.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_uart_ex.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_dma.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_lpuart.o \
			Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_rcc.o \
			Middlewares/Third_Party/Sgfx/Crypto/se_nvm.o \
			Middlewares/Third_Party/Sgfx/Crypto/sigfox_data.o \
			Middlewares/Third_Party/Sgfx/utils/low_power_manager.o \
			Middlewares/Third_Party/Sgfx/utils/scheduler.o \
			Middlewares/Third_Party/Sgfx/utils/timeServer.o \
			Middlewares/Third_Party/Sgfx/utils/utilities.o \
			Middlewares/Third_Party/Lora/Mac/region/Region.o \
			Middlewares/Third_Party/Lora/Mac/region/RegionAS923.o \
			Middlewares/Third_Party/Lora/Mac/region/RegionAU915.o \
			Middlewares/Third_Party/Lora/Mac/region/RegionCN470.o \
			Middlewares/Third_Party/Lora/Mac/region/RegionCN779.o \
			Middlewares/Third_Party/Lora/Mac/region/RegionCommon.o \
			Middlewares/Third_Party/Lora/Mac/region/RegionEU433.o \
			Middlewares/Third_Party/Lora/Mac/region/RegionEU868.o \
			Middlewares/Third_Party/Lora/Mac/region/RegionIN865.o \
			Middlewares/Third_Party/Lora/Mac/region/RegionKR920.o \
			Middlewares/Third_Party/Lora/Mac/region/RegionUS915.o \
			Middlewares/Third_Party/Lora/Mac/LoRaMac.o \
			Middlewares/Third_Party/Lora/Mac/LoRaMacCrypto.o \
			Middlewares/Third_Party/Lora/Crypto/aes.o \
			Middlewares/Third_Party/Lora/Crypto/cmac.o \
			Middlewares/Third_Party/Lora/Core/lora.o \
			Projects/B-L072Z-LRWAN1/Applications/SgfxLora/SgfxLora_Periodic/SW4STM32/startup_stm32l072xx.o \
			Projects/B-L072Z-LRWAN1/Applications/SgfxLora/SgfxLora_Periodic/src/bsp.o \
			Projects/B-L072Z-LRWAN1/Applications/SgfxLora/SgfxLora_Periodic/src/debug.o \
			Projects/B-L072Z-LRWAN1/Applications/SgfxLora/SgfxLora_Periodic/src/hw_eeprom.o \
			Projects/B-L072Z-LRWAN1/Applications/SgfxLora/SgfxLora_Periodic/src/hw_gpio.o \
			Projects/B-L072Z-LRWAN1/Applications/SgfxLora/SgfxLora_Periodic/src/hw_rtc.o \
			Projects/B-L072Z-LRWAN1/Applications/SgfxLora/SgfxLora_Periodic/src/hw_spi.o \
			Projects/B-L072Z-LRWAN1/Applications/SgfxLora/SgfxLora_Periodic/src/hw_tim2.o \
			Projects/B-L072Z-LRWAN1/Applications/SgfxLora/SgfxLora_Periodic/src/main.o \
			Projects/B-L072Z-LRWAN1/Applications/SgfxLora/SgfxLora_Periodic/src/mlm32l0xx_hal_msp.o \
			Projects/B-L072Z-LRWAN1/Applications/SgfxLora/SgfxLora_Periodic/src/mlm32l0xx_hw.o \
			Projects/B-L072Z-LRWAN1/Applications/SgfxLora/SgfxLora_Periodic/src/mlm32l0xx_it.o \
			Projects/B-L072Z-LRWAN1/Applications/SgfxLora/SgfxLora_Periodic/src/tiny_sscanf.o \
			Projects/B-L072Z-LRWAN1/Applications/SgfxLora/SgfxLora_Periodic/src/tiny_vsnprintf.o \
			Projects/B-L072Z-LRWAN1/Applications/SgfxLora/SgfxLora_Periodic/src/vcom.o \
			Projects/B-L072Z-LRWAN1/Applications/SgfxLora/Common/cwmx1zzabz/Radio_Sigfox_Driver/mcu_api.o \
			Projects/B-L072Z-LRWAN1/Applications/SgfxLora/Common/cwmx1zzabz/Radio_Sigfox_Driver/rf_api.o \
			Projects/B-L072Z-LRWAN1/Applications/SgfxLora/Common/cwmx1zzabz/Radio_Sigfox_Driver/st_lowlevel.o


# ----- Verbosity control -----------------------------------------------------

CC_normal	:= $(CC)
BUILD_normal	:=
DEPEND_normal	:= $(CC) -MM -MG

CC_quiet	= @echo "  CC       " $@ && $(CC_normal)
BUILD_quiet	= @echo "  BUILD    " $@ && $(BUILD_normal)
DEPEND_quiet	= @$(DEPEND_normal)

ifeq ($(V),1)
    CC		= $(CC_normal)
    BUILD	= $(BUILD_normal)
    DEPEND	= $(DEPEND_normal)
else
    CC		= $(CC_quiet)
    BUILD	= $(BUILD_quiet)
    DEPEND	= $(DEPEND_quiet)
endif

# ----- Rules -----------------------------------------------------------------

.PHONY:		all clean

all:		$(NAME)_text.bin

$(NAME).elf: $(OBJS)
	$(CC) $(CFLAGS) $(USER_LIBS_DIR) $(LDFLAGS) -o $@ $(OBJS) $(USER_LIBS)
	$(SIZE) $@

%_text.bin: %.elf
	$(BUILD) $(OBJCOPY) -O binary $< $@


# ----- Cleanup ---------------------------------------------------------------

clean:
		rm -f $(NAME).bin $(NAME).elf $(NAME).hex
		rm -f $(NAME)_text.{bin,hex}
		rm -f $(OBJS) $(OBJS:.o=.d)
		rm -f *~

# ----- Dependencies ----------------------------------------------------------

MKDEP =									\
	$(DEPEND) $(CFLAGS) $(DEFINES) $(INCLUDES) $< |							\
	  sed 								\
	    -e 's|^$(basename $(notdir $<)).o:|$@:|'			\
	    -e '/^\(.*:\)\? */{p;s///;s/ *\\\?$$/ /;s/  */:\n/g;H;}'	\
	    -e '$${g;p;}'						\
	    -e d >$(basename $@).d;					\
	  [ "$${PIPESTATUS[*]}" = "0 0" ] ||				\
	  { rm -f $(basename $@).d; exit 1; }

%.o: %.c
	$(CC) $(CFLAGS) $(DEFINES) $(INCLUDES) -c $< -o $@
	$(MKDEP)

%.o: %.s
	$(CC) $(CFLAGS) $(DEFINES) $(INCLUDES) -D__ASSEMBLY__ -c $< -o $@
	$(MKDEP)

-include $(OBJS:.o=.d)

# ----- Programming and device control ----------------------------------------

.PHONY: flash

flash: $(NAME)_text.bin
	$(FLASH) write $(NAME)_text.bin 0x08000000
