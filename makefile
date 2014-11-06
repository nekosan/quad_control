# makefile for application

## Comment or uncomment below to make build settings
## to suit your preferense and environment.
## You don't have to change makefile or lib/makefile.

# select board name
#STM32_P103 = 1
#STM32_H103 = 1
#CQ_STARM = 1
#CQ_ST103Z = 1
#STM3210E_EVAL = 1
#STBee = 1
STBee_Mini = 1
#STBee_Mini_SWD = 1
#STM32_VLD = 1

# select way to write flash
# ST-LINK via SWD but not with STVP is done by Debug button 
# but not by make target 
#FT2232_JTAG = 1
#STLINK_JTAG_GDB = 1
#STLINK_SWD_STVP = 1
#VERSALOON_SWD = 1
DFU = 1
#UART = 1

# select serial port for debug
#USART1 = 1
#USART2 = 1
#USART3 = 1
#DCC = 1
VCP = 1

# Uncomment below if you use USB
USE_USB = 1

# CPU can stop at start up until serial data send. This is for a wait until VCP connection.
#STOP_AT_START_UP = -DSTOP_AT_STARTUP

# Uncomment below if you use Free-RTOS
#USE_FREERTOS = 1

# select way to connect SD card
#SD_SDIO = 1
SD_SPI = 1

# select toolchain
YAGARTO = 1
#DEFAULT_PATH = 1
#CODESOURCERY = 1

# Uncomment below if you make debug build
DEBUG = 1

# select ROM size to write using STVP
STVP_ALL = 1
#STVP_32K = 1
#STVP_20K = 1

# designate serial port number to connect UART1 using ST Flash Loader
FLASH_LOADER_PORT = 25

############# NO NEED TO CHANGE LINES BELOW ###############
# define xtal clock and chip type
ifdef STM32_P103
XTAL_FREQ = ((uint32_t)8000000)
SYSCLK_FREQ_SELECT = SYSCLK_FREQ_72MHz
SYSCLK_FREQ_VALE = 72000000
CHIP_TYPE = STM32F10X_MD
CHIP_NAME = STM32F103xB
BOARD_USED = USE_STM32_P103
DFU_BINARY = DFU_STM32-P103.bin
FLASH_LOADER_MAP = STM32_Med-density_128K
 ifdef DFU
 LD_SCRIPT = 128_20_dfu.ld
 endif
 ifndef DFU
 LD_SCRIPT = 128_20.ld
 endif
 ifdef USE_FREERTOS
 START_UP = lib/Startup/startup_stm32f10x_md_rtos.o
 endif
 ifndef USE_FREERTOS
 START_UP = lib/Startup/startup_stm32f10x_md_nortos.o
 endif
endif
ifdef STM32_H103
XTAL_FREQ = ((uint32_t)8000000)
SYSCLK_FREQ_SELECT = SYSCLK_FREQ_72MHz
SYSCLK_FREQ_VALE = 72000000
CHIP_TYPE = STM32F10X_MD
CHIP_NAME = STM32F103xB
BOARD_USED = USE_STM32_H103
DFU_BINARY = DFU_STM32_H103.bin
FLASH_LOADER_MAP = STM32_Med-density_128K
 ifdef DFU
 LD_SCRIPT = 128_20_dfu.ld
 endif
 ifndef DFU
 LD_SCRIPT = 128_20.ld
 endif
  ifdef USE_FREERTOS
 START_UP = lib/Startup/startup_stm32f10x_md_rtos.o
 endif
 ifndef USE_FREERTOS
 START_UP = lib/Startup/startup_stm32f10x_md_nortos.o
 endif
endif
ifdef CQ_STARM
XTAL_FREQ = ((uint32_t)8000000)
SYSCLK_FREQ_SELECT = SYSCLK_FREQ_72MHz
SYSCLK_FREQ_VALE = 72000000
CHIP_TYPE = STM32F10X_MD
CHIP_NAME = STM32F103xB
BOARD_USED = USE_CQ_STARM
DFU_BINARY = DFU_CQ-STARM.bin
FLASH_LOADER_MAP = STM32_Med-density_128K
 ifdef DFU
 LD_SCRIPT = 128_20_dfu.ld
 endif
 ifndef DFU
 LD_SCRIPT = 128_20.ld
 endif
 ifdef USE_FREERTOS
 START_UP = lib/Startup/startup_stm32f10x_md_rtos.o
 endif
 ifndef USE_FREERTOS
 START_UP = lib/Startup/startup_stm32f10x_md_nortos.o
 endif
endif
ifdef CQ_ST103Z
XTAL_FREQ = ((uint32_t)8000000)
SYSCLK_FREQ_SELECT = SYSCLK_FREQ_72MHz
SYSCLK_FREQ_VALE = 72000000
CHIP_TYPE = STM32F10X_HD
CHIP_NAME = STM32F103xE
BOARD_USED = USE_CQ_ST103Z
DFU_BINARY = DFU_CQ-ST103Z.bin
FLASH_LOADER_MAP = STM32_High-density_512K
 ifdef DFU
 LD_SCRIPT = 512_64_dfu.ld
 endif
 ifndef DFU
 LD_SCRIPT = 512_64.ld
 endif
 ifdef USE_FREERTOS
 START_UP = lib/Startup/startup_stm32f10x_hd_rtos.o
 endif
 ifndef USE_FREERTOS
 START_UP = lib/Startup/startup_stm32f10x_hd_nortos.o
 endif
endif
ifdef STM3210E_EVAL
XTAL_FREQ = ((uint32_t)8000000)
SYSCLK_FREQ_SELECT = SYSCLK_FREQ_72MHz
SYSCLK_FREQ_VALE = 72000000
CHIP_TYPE = STM32F10X_HD
CHIP_NAME = STM32F103xE
BOARD_USED = USE_STM3210E_EVAL
DFU_BINARY = DFU_ST3210E-EVAL.bin
FLASH_LOADER_MAP = STM32_High-density_512K
 ifdef DFU
 LD_SCRIPT = 512_64_dfu.ld
 endif
 ifndef DFU
 LD_SCRIPT = 512_64.ld
 endif
 ifdef USE_FREERTOS
 START_UP = lib/Startup/startup_stm32f10x_hd_rtos.o
 endif
 ifndef USE_FREERTOS
 START_UP = lib/Startup/startup_stm32f10x_hd_nortos.o
 endif
endif
ifdef STBee
XTAL_FREQ = ((uint32_t)12000000)
SYSCLK_FREQ_SELECT = SYSCLK_FREQ_72MHz
SYSCLK_FREQ_VALE = 72000000
CHIP_TYPE = STM32F10X_HD
CHIP_NAME = STM32F103xE
BOARD_USED = USE_STBEE
DFU_BIN = DFU_STBEE.bin
DFU_HEX = DFU_STBEE.hex
FLASH_LOADER_MAP = STM32_High-density_512K
 ifdef DFU
 LD_SCRIPT = 512_64_dfu.ld
 endif
 ifndef DFU
 LD_SCRIPT = 512_64.ld
 endif
 ifdef USE_FREERTOS
 START_UP = lib/Startup/startup_stm32f10x_hd_rtos.o
 endif
 ifndef USE_FREERTOS
 START_UP = lib/Startup/startup_stm32f10x_hd_nortos.o
 endif
endif
ifdef STBee_Mini
XTAL_FREQ = ((uint32_t)12000000)
SYSCLK_FREQ_SELECT = SYSCLK_FREQ_72MHz
SYSCLK_FREQ_VALE = 72000000
CHIP_TYPE = STM32F10X_MD
CHIP_NAME = STM32F103xC
BOARD_USED = USE_STBEE_MINI
#DFU_BINARY = DFU_STM32-P103.bin
LDLIBS += -lc -lm -lgcc -lnosys
FLASH_LOADER_MAP = STM32_Med-density_128K
 ifdef DFU
 LD_SCRIPT = 128_20_dfu.ld
 endif
 ifndef DFU
 LD_SCRIPT = 128_20.ld
 endif
 ifdef USE_FREERTOS
 START_UP = lib/Startup/startup_stm32f10x_md_rtos.o
 endif
 ifndef USE_FREERTOS
 START_UP = lib/Startup/startup_stm32f10x_md_nortos.o
 endif
endif
ifdef STBee_Mini_SWD
XTAL_FREQ = ((uint32_t)12000000)
SYSCLK_FREQ_SELECT = SYSCLK_FREQ_72MHz
SYSCLK_FREQ_VALE = 72000000
CHIP_TYPE = STM32F10X_MD
CHIP_NAME = STM32F103xC
BOARD_USED = USE_STBEE_MINI_SWD
#DFU_BINARY = DFU_STM32-P103.bin
FLASH_LOADER_MAP = STM32_Med-density_128K
 ifdef DFU
 LD_SCRIPT = 128_20_dfu.ld
 endif
 ifndef DFU
 LD_SCRIPT = 128_20.ld
 endif
 ifdef USE_FREERTOS
 START_UP = lib/Startup/startup_stm32f10x_md_rtos.o
 endif
 ifndef USE_FREERTOS
 START_UP = lib/Startup/startup_stm32f10x_md_nortos.o
 endif
endif
ifdef STM32_VLD
XTAL_FREQ = ((uint32_t)8000000)
SYSCLK_FREQ_SELECT = SYSCLK_FREQ_24MHz
SYSCLK_FREQ_VALE = 24000000
CHIP_TYPE = STM32F10X_MD_VL
CHIP_NAME = STM32F100xBxxB
BOARD_USED = USE_STM32_VLD
FLASH_LOADER_MAP = STM32_Med-density-value_128K
 ifdef DFU
 #There is no USB device port
 endif
 ifndef DFU
 LD_SCRIPT = 128_8.ld
 endif
 ifdef USE_FREERTOS
 START_UP = lib/Startup/startup_stm32f10x_md_vl_rtos.o
 endif
 ifndef USE_FREERTOS
 START_UP = lib/Startup/startup_stm32f10x_md_vl_nortos.o
 endif
endif


ifdef DFU
WRITE_BY_DFU = -DUSE_DFU
endif

# select serial port for debug
ifdef USART1
DEBUG_SERIAL = USE_USART1
endif
ifdef USART2
DEBUG_SERIAL = USE_USART2
endif
ifdef USART3
DEBUG_SERIAL = USE_USART3
endif
ifdef DCC
DEBUG_SERIAL = USE_DCC
endif
ifdef VCP
DEBUG_SERIAL = USE_VCP
endif

ifdef DEFAULT_PATH
TOOLCHAIN_PATH = 
CC_NAME = arm-none-eabi-gcc
CXX_NAME = arm-none-eabi-g++
AR_NAME = arm-none-eabi-ar
OBJCOPY_NAME =arm-none-eabi-objcopy
GDB_NAME = arm-none-eabi-gdb
endif

ifdef YAGARTO
TOOLCHAIN_PATH = ../../toolchain/yagarto/bin/
CC_NAME = arm-none-eabi-gcc
CXX_NAME = arm-none-eabi-g++
AR_NAME = arm-none-eabi-ar
OBJCOPY_NAME =arm-none-eabi-objcopy
GDB_NAME = arm-none-eabi-gdb
endif

ifdef CODESOURCERY
TOOLCHAIN_PATH = C:/Program Files/CodeSourcery/Sourcery G++ Lite/bin/
CC_NAME = arm-none-eabi-gcc
CXX_NAME = arm-none-eabi-g++
AR_NAME = arm-none-eabi-ar
OBJCOPY_NAME =arm-none-eabi-objcopy
GDB_NAME = arm-none-eabi-gdb
endif

ifdef STVP_ALL
API_SAMPLE_EXE = APISample.exe
endif
ifdef STVP_32K
API_SAMPLE_EXE = APISampleSTM32_32K.exe
endif
ifdef STVP_20K
API_SAMPLE_EXE = APISampleSTM32_20K.exe
endif

# Define board specific options
BOARD_OPTS = -DHSE_VALUE=$(XTAL_FREQ) -USTM32F10X_XL -D$(CHIP_TYPE) \
             -D$(BOARD_USED) $(WRITE_BY_DFU) -D$(DEBUG_SERIAL) \
             -D$(SYSCLK_FREQ_SELECT)=$(SYSCLK_FREQ_VALE) $(STOP_AT_START_UP)

###############################################################################

# making this makefile works even if cygwin is installed
SHELL=cmd.exe

export DEBUG
export MESSAGES

TARGET_ARCH = -mcpu=cortex-m3 -mthumb

INCLUDE_DIRS = -I . -I inc \
               -I lib \
               -I lib/CMSIS/CM3/CoreSupport -I lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x \
               -I lib/STM32F10x_StdPeriph_Driver/inc \
               -I lib/SD/inc \
               -I lib/USART/inc -I lib/COM/inc -I lib/UTIL/inc\
               -I lib/Startup \
               -I lib/Platform 
ifdef VCP
INCLUDE_DIRS += -I lib/Virtual_COM_Port/inc
endif
ifdef USE_USB
INCLUDE_DIRS += -I lib/STM32_USB-FS-Device_Driver/inc
endif
ifdef USE_FREERTOS
INCLUDE_DIRS += -I lib/FreeRTOS/Source/include \
                -I lib/FreeRTOS/Source/portable/GCC/ARM_CM3
endif
ifdef SD_SDIO
INCLUDE_DIRS += -I lib/SD_SDIO/inc
endif
ifdef SD_SPI
INCLUDE_DIRS += -I lib/SD_SPI/inc
endif
               
LIBRARY_DIRS = -L ./lib

COMPILE_OPTS = -fsigned-char $(WARNINGS) $(FIRMWARE_OPTS) $(BOARD_OPTS) $(FREERTOS_OPTS) $(TARGET_OPTS) $(MESSAGES) $(INCLUDE_DIRS)
#WARNINGS =  -W $(W_CONVERSION) -Winline
#WARNINGS = -Wall -W $(W_CONVERSION) -Wshadow -Wcast-qual -Wwrite-strings -Winline
WARNINGS = -Wall -W $(W_CONVERSION)
#W_CONVERSION = -Wconversion

WARNINGS_CXX = -Weffc++ $(W_OLD_STYLE_CAST)
#W_OLD_STYLE_CAST = -Wold-style-cast
#F_NO_EXCEPTIONS = -fno-exceptions	# disabling exceptions saves code space

FIRMWARE_OPTS = -DUSE_STDPERIPH_DRIVER

MESSAGES = -fmessage-length=0

FREERTOS_OPTS = -D GCC_ARMCM3

ifdef DEBUG
 TARGET_OPTS = -O0 -g3
# TARGET_OPTS = -O2 -g3
 DEBUG_MACRO = -DDEBUG
else
 F_INLINE = -finline
 F_INLINE_ONCE = -finline-functions-called-once
 #F_UNROLL_LOOPS = -funroll-loops
 TARGET_OPTS = -Os $(F_INLINE) $(F_INLINE_ONCE) $(F_UNROLL_LOOPS)
endif

CPPFLAGS = $(DEBUG_MACRO)

CC = $(TOOLCHAIN_PATH)$(CC_NAME)
CFLAGS = -std=gnu99 $(COMPILE_OPTS)

CXX = $(TOOLCHAIN_PATH)$(CXX_NAME)
CXXFLAGS = -std=gnu++98 $(COMPILE_OPTS) $(WARNINGS_CXX) $(F_NO_EXCEPTIONS)

AS = $(CC)
ASFLAGS = -x assembler-with-cpp -c $(TARGET_ARCH) $(COMPILE_OPTS) 

LD = $(CC)
LDFLAGS = -Wl,--gc-sections,-Map=$(MAIN_MAP),-cref -T ld/$(LD_SCRIPT) $(INCLUDE_DIRS) $(LIBRARY_DIRS)

AR = $(TOOLCHAIN_PATH)$(AR_NAME)
ARFLAGS = cr

OBJCOPY = $(TOOLCHAIN_PATH)$(OBJCOPY_NAME)
OBJCOPY_BIN_FLAGS = -O binary
OBJCOPY_HEX_FLAGS = -O ihex

MAIN_OUT = bin\main.elf
MAIN_MAP = $(MAIN_OUT:%.elf=%.map)
MAIN_BIN = $(MAIN_OUT:%.elf=%.bin)
MAIN_HEX = $(MAIN_OUT:%.elf=%.hex)
MAIN_DFU = $(MAIN_OUT:%.elf=%.dfu)

GDB = $(TOOLCHAIN_PATH)$(GDB_NAME)

MAIN_OBJS = $(sort \
            $(patsubst %.cpp,%.o,$(wildcard *.cpp)) \
            $(patsubst %.cc,%.o,$(wildcard *.cc)) \
            $(patsubst %.c,%.o,$(wildcard *.c)) \
            $(patsubst %.s,%.o,$(wildcard *.s)) \
            $(patsubst %.cpp,%.o,$(wildcard src/*.cpp)) \
            $(patsubst %.cc,%.o,$(wildcard src/*.cc)) \
            $(patsubst %.c,%.o,$(wildcard src/*.c)) \
            $(patsubst %.s,%.o,$(wildcard src/*.s)))

LINK_FWLIB = lib/lib.a

LIB_OUT = lib/lib.a

LIB_OBJS = $(sort \
 $(patsubst %.c,%.o,$(wildcard lib/Platform/*.c)) \
 $(patsubst %.s,%.o,$(wildcard lib/Platform/*.s)) \
 $(patsubst %.c,%.o,$(wildcard lib/USART/src/*.c)) \
 $(patsubst %.s,%.o,$(wildcard lib/USART/src/*.s)) \
 $(patsubst %.c,%.o,$(wildcard lib/COM/src/*.c)) \
 $(patsubst %.s,%.o,$(wildcard lib/COM/src/*.s)) \
 $(patsubst %.c,%.o,$(wildcard lib/UTIL/src/*.c)) \
 $(patsubst %.s,%.o,$(wildcard lib/UTIL/src/*.s)) \
 $(patsubst %.c,%.o,$(wildcard lib/STM32F10x_StdPeriph_Driver/src/*.c)) \
 $(patsubst %.s,%.o,$(wildcard lib/STM32F10x_StdPeriph_Driver/src/*.s)) \
 $(patsubst %.c,%.o,$(wildcard lib/CMSIS/CM3/CoreSupport/*.c)) \
 $(patsubst %.s,%.o,$(wildcard lib/CMSIS/CM3/CoreSupport/*.s)) \
 $(patsubst %.c,%.o,$(wildcard lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/*.c)) \
 $(patsubst %.s,%.o,$(wildcard lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/*.s)) \
 $(patsubst %.c,%.o,$(wildcard lib/Startup/*.c)) \
 $(patsubst %.s,%.o,$(wildcard lib/Startup/*.s)))
ifdef VCP
LIB_OBJS += $(sort \
 $(patsubst %.c,%.o,$(wildcard lib/Virtual_COM_Port/src/*.c)) \
 $(patsubst %.s,%.o,$(wildcard lib/Virtual_COM_Port/src/*.s)))
endif
ifdef USE_USB
LIB_OBJS += $(sort \
 $(patsubst %.c,%.o,$(wildcard lib/STM32_USB-FS-Device_Driver/src/*.c)) \
 $(patsubst %.s,%.o,$(wildcard lib/STM32_USB-FS-Device_Driver/src/*.s)))
endif
ifdef USE_FREERTOS
LIB_OBJS += $(sort \
 $(patsubst %.c,%.o,$(wildcard lib/FreeRTOS/Source/*.c)) \
 $(patsubst %.s,%.o,$(wildcard lib/FreeRTOS/Source/*.s)) \
 $(patsubst %.c,%.o,$(wildcard lib/FreeRTOS/Source/portable/GCC/ARM_CM3/*.c)) \
 $(patsubst %.s,%.o,$(wildcard lib/FreeRTOS/Source/portable/GCC/ARM_CM3/*.s)) \
 $(patsubst %.c,%.o,$(wildcard lib/FreeRTOS/Source/portable/MemMang/*.c)) \
 $(patsubst %.s,%.o,$(wildcard lib/FreeRTOS/Source/portable/MemMang/*.s)))
endif
ifdef SD_SDIO
LIB_OBJS += $(sort \
 $(patsubst %.c,%.o,$(wildcard lib/SD_SDIO/src/*.c)) \
 $(patsubst %.s,%.o,$(wildcard lib/SD_SDIO/src/*.s)))
endif
ifdef SD_SPI
LIB_OBJS += $(sort \
 $(patsubst %.c,%.o,$(wildcard lib/SD_SPI/src/*.c)) \
 $(patsubst %.s,%.o,$(wildcard lib/SD_SPI/src/*.s)))
endif

## make targets

# all

.PHONY: all
all: lib main

# all_clean

.PHONY: all_clean
all_clean: lib_clean main_clean

# main

.PHONY: main
main: $(MAIN_BIN) $(MAIN_HEX)

$(MAIN_OUT): $(MAIN_OBJS) $(START_UP) $(LINK_FWLIB)  
	$(LD) $(LDFLAGS) $(TARGET_ARCH) $^ -o $@ $(LDLIBS)

$(MAIN_OBJS): $(wildcard *.h) $(wildcard inc/*.h)

$(MAIN_BIN): $(MAIN_OUT)
	$(OBJCOPY) $(OBJCOPY_BIN_FLAGS) $< $@

$(MAIN_HEX): $(MAIN_OUT)
	$(OBJCOPY) $(OBJCOPY_HEX_FLAGS) $< $@
	
	@copy $(MAIN_OUT) jtag\flash.elf


# flash

.PHONY: flash
flash: flash-elf

.PHONY: flash-elf
flash-elf: all

ifdef DFU	
# download using DFU and DFU Tools from KSK
	@cd bin && DfuConvert.exe -n Internal_Flash -v 0483 -p DF11 -b 0 -ReadFileName main.hex -CreateFileName main.dfu
	@cd bin && DfuUpgrade.exe -DownFileName main.dfu -TargetIdSel 0 -v 0483 -p DF11 -b 0
	@cd bin && DfuVerify.exe -DownFileName main.dfu -TargetIdSel 0 -v 0483 -p DF11 -b 0
endif

ifdef VERSALOON_SWD
# download using VERSALOON SWD and OpenOCD
	@cd jtag && ..\..\..\OpenOCD\0.5.0-dev\bin\openocd -f flash-elf_swd.cfg
endif

ifdef FT2232_JTAG
# download using FT2232 JTAG and OpenOCD
	@cd jtag && ..\..\..\OpenOCD\0.5.0-dev\bin\openocd -f flash-elf.cfg
endif
	
ifdef STLINK_JTAG_GDB
# download using ST-LINK and ST-LINK GDB server	
	@cd jtag && $(GDB) flash.elf -x gdb_stlink.ini
endif

ifdef UART
# download using UART and Flash Loader Demonstrator	
	@cd bin && ..\jtag\Flash_Loader.bat $(FLASH_LOADER_PORT) $(FLASH_LOADER_MAP)
endif

ifdef STLINK_SWD_STVP
# download using UART and Flash Loader Demonstrator	
	@copy $(MAIN_HEX) "..\..\STVP\"
	@cd ..\..\STVP &&$(API_SAMPLE_EXE) -BoardName=ST-LINK -Device=$(CHIP_NAME) -Port=USB -ProgMode=SWD -no_loop -no_log -progress -erase
	@cd ..\..\STVP &&$(API_SAMPLE_EXE) -BoardName=ST-LINK -Device=$(CHIP_NAME) -Port=USB -ProgMode=SWD -no_loop -no_log -progress -FileProg=main.hex
	@del "..\..\STVP\main.hex"
endif

# main_asm

.PHONY: main_asm
main_asm:
	$(CC) $(COMPILE_OPTS) -S \
             $(wildcard *.c) \
             $(wildcard src/*.c) 

# main_clean

.PHONY: main_clean
main_clean:
	-del /f /q *.o src\*.o $(MAIN_OUT) $(MAIN_MAP) $(MAIN_BIN) $(MAIN_HEX) $(MAIN_DFU) *.s jtag\flash.elf jtag\flash.bin

# lib

.PHONY: lib
lib: $(LIB_OUT)

$(LIB_OUT): $(LIB_OBJS)
	$(AR) $(ARFLAGS) $@ $(LIB_OBJS)

$(LIB_OBJS): $(wildcard lib/Platform/*.h) \
             $(wildcard lib/USART/src/*.h) \
             $(wildcard lib/COM/src/*.h) \
             $(wildcard lib/UTIL/src/*.h) \
             $(wildcard lib/SD_SPI/src/*.h) \
             $(wildcard lib/SD_SDIO/src/*.h) \
             $(wildcard lib/STM32F10x_StdPeriph_Driver/src/*.h) \
             $(wildcard lib/CMSIS/CM3/CoreSupport/*.h) \
             $(wildcard lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/*.h) \
             $(wildcard lib/STM32_USB-FS-Device_Driver/src/*.h) \
             $(wildcard lib/FreeRTOS/Source/*.h) \
             $(wildcard lib/lib/FreeRTOS/Source/portable/GCC/ARM_CM3/*.h) \
             $(wildcard lib/FreeRTOS/Source/portable/MemMang/*.h) \
             $(wildcard lib/Virtual_COM_Port/src/*.h) \
             $(wildcard lib/Startup/*.h) 
             
# lib_asm

.PHONY: lib_asm
lib_asm:
	$(CC) $(COMPILE_OPTS) -S \
             $(wildcard lib/STM32F10x_StdPeriph_Driver/src/*.c) \
             $(wildcard lib/USART/src/*.c) \
             $(wildcard lib/COM/src/*.c) \
             $(wildcard lib/UTIL/src/*.c) \
             $(wildcard lib/SD_SDIO/src/*.c) \
             $(wildcard lib/SD_SPI/src/*.c) \
             $(wildcard lib/CMSIS/CM3/CoreSupport/*.c) \
             $(wildcard lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/*.c) \
             $(wildcard lib/STM32_USB-FS-Device_Driver/src/*.c) \
             $(wildcard lib/FreeRTOS/Source/*.c) \
             $(wildcard lib/FreeRTOS/Source/portable/GCC/ARM_CM3/*.c) \
             $(wildcard lib/FreeRTOS/Source/portable/MemMang/*.c) \
             $(wildcard lib/Virtual_COM_Port/src/*.c) \
             $(wildcard lib/Startup/*.c) \
             $(wildcard lib/Platform/*.c)
             
# lib_clean

.PHONY: lib_clean
lib_clean:
	@del /f /q lib\STM32F10x_StdPeriph_Driver\src\*.o lib\COM\src\*.o lib\USART\src\*.o lib\UTIL\src\*.o lib\SD_SDIO\src\*.o lib\SD_SPI\src\*.o lib\CMSIS\CM3\CoreSupport\*.o lib\CMSIS\CM3\DeviceSupport\ST\STM32F10x\*.o lib\FreeRTOS\Source\*.o lib\FreeRTOS\Source\portable\GCC\ARM_CM3\*.o lib\FreeRTOS\Source\portable\MemMang\*.o lib\Startup\*.o lib\Platform\*.o lib\*.s lib\lib.a
ifdef VCP
	@del /f /q lib\Virtual_COM_Port\src\*.o
endif
ifdef USE_USB
	@del /f /q lib\STM32_USB-FS-Device_Driver\src\*.o
endif

# restore_dfu

.PHONY: restore_dfu
restore_dfu:
#	@copy jtag\DFU\$(DFU_BINARY) jtag\flash.bin

ifdef VERSALOON_SWD
# download using VERSALOON SWD and OpenOCD
	@copy jtag\DFU\$(DFU_BIN) jtag\flash.bin
	@cd jtag && ..\..\..\OpenOCD\0.5.0-dev\bin\openocd -f flash-dfu_swd.cfg
	@del jtag\flash.bin
endif

ifdef FT2232_JTAG
# download using FT2232 JTAG and OpenOCD
	@copy jtag\DFU\$(DFU_BIN) jtag\flash.bin
	@cd jtag && ..\..\..\OpenOCD\0.5.0-dev\bin\openocd -f flash-dfu.cfg
	@del jtag\flash.bin
endif
	
ifdef STLINK_JTAG_GDB
# download using ST-LINK and ST-LINK GDB server	
	@copy jtag\DFU\$(DFU_BIN) jtag\flash.bin
	@cd jtag && $(GDB) flash.bin -x gdb_stlink.ini
	@del jtag\flash.bin
endif

ifdef UART
# download using UART and Flash Loader Demonstrator	
	@copy jtag\DFU\$(DFU_BIN) bin\flash.bin
	@cd bin && ..\jtag\Flash_Loader_DFU.bat $(FLASH_LOADER_PORT) $(FLASH_LOADER_MAP)
	@del bin\flash.bin
endif

ifdef STLINK_SWD_STVP
# download using UART and Flash Loader Demonstrator	
	$(OBJCOPY) -I binary -O ihex --change-addresses 0x08000000 jtag\DFU\$(DFU_BIN) ..\..\STVP\flash.hex
	@cd ..\..\STVP &&$(API_SAMPLE_EXE) -BoardName=ST-LINK -Device=$(CHIP_NAME) -Port=USB -ProgMode=SWD -no_loop -no_log -progress -erase
	@cd ..\..\STVP &&$(API_SAMPLE_EXE) -BoardName=ST-LINK -Device=$(CHIP_NAME) -Port=USB -ProgMode=SWD -no_loop -no_log -progress -FileProg=flash.hex
	@del "..\..\STVP\flash.hex"
endif
