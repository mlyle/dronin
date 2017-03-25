#
# Hardcoded linker script names for now
#
#
CDEFS				+= -DSTM32F10X_MD
CDEFS				+= -DHSE_VALUE=$(OSCILLATOR_FREQ)
CDEFS 				+= -DUSE_STDPERIPH_DRIVER
ARCHFLAGS			+= -mcpu=cortex-m3 -march=armv7e-m
FLOATABI			= soft

#
# PIOS device library source and includes
#
EXTRAINCDIRS		+=	$(PIOS_DEVLIB)/inc

#
# CMSIS for the F1
#
include $(PIOSCOMMONLIB)/CMSIS/library.mk
CMSIS3_DEVICEDIR	:=	$(PIOS_DEVLIB)/Libraries/CMSIS/Core/CM3
SRC			+=      $(BOARD_INFO_DIR)/cmsis_system.c
EXTRAINCDIRS		+=	$(CMSIS3_DEVICEDIR)

#
# ST Peripheral library
#
PERIPHLIB		 =	$(PIOS_DEVLIB)/Libraries/STM32F10x_StdPeriph_Driver
EXTRAINCDIRS		+=	$(PERIPHLIB)/inc
SRC			+=	$(wildcard $(PERIPHLIB)/src/*.c)

#
# ST USB Device Lib
#
USBDEVLIB                       =       $(PIOS_DEVLIB)/Libraries/STM32_USB-FS-Device_Driver
EXTRAINCDIRS                    +=      $(USBDEVLIB)/inc
SRC                             +=      $(wildcard $(USBDEVLIB)/src/*.c)

