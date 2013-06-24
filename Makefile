# Makefile for AutoQuad Flight Controller firmware
#
# ! Use of this makefile requires setup of a compatible development environment.
# ! For latest development recommendations, check here: http://autoquad.org/wiki/wiki/development/
# ! This file is ignored when building with CrossWorks Studio.
#
# All paths are relative to Makefile location
# Usage examples:
# 		make all											# default Release type builds .hex and .elf binaries
#		make all BUILD_TYPE=Debug					# build with compiler debugging flags/options enabled
#		make all BOARD_REV=1 INCR_BUILDNUM=0	# build for rev 1 (post Oct-2012) hardware, don't increment the buildnumber

# Defaults - modify here or on command line
#
# Output folder name; Use 'Debug' to set debug compiler options;
BUILD_TYPE ?= Release
# Path to source files - no trailing slash
SRC_PATH ?= .
# Board version to build for (6)
BOARD_VER ?= 6
# Board revision to build for (0 = v6 initial release revision, 1 = v6 Oct. 2012 revision)
BOARD_REV ?= 0
# Increment build number? (0|1)  This is automatically disabled for debug builds.
INCR_BUILDNUM ?= 1
# Use the single-folder source file organization from AQ repo? (0|1)
FLAT_SRC ?= 1
# Produced binaries file name prefix
BIN_NAME ?= aqv6.7
# Build debug version? (0|1; true by default if build_type contains the word "debug")
ifeq ($(findstring Debug, $(BUILD_TYPE)), Debug)
	DEBUG_BUILD ?= 1
else 
	DEBUG_BUILD ?= 0
endif
# Build with HW flow control disabled? (0|1)
HW_FC_NONE ?= 0
# Flashing interface (Linux only)
USB_DEVICE ?= /dev/ttyUSB0

# You may also use BIN_SUFFIX to append text 
# to generated bin file name after version string;
# BIN_SUFFIX = 


# System-specific folder paths and commands
#
# compiler base path
CC_PATH ?= /usr/share/crossworks_for_arm_2.2
#CC_PATH ?= C:/devel/gcc/crossworks_for_arm_2.2

# shell commands (Windows needs Cygwin or MSys)
EXE_AWK ?= gawk 
EXE_MV ?= mv 
#EXE_AWK := C:/cygwin/bin/gawk
#EXE_MV := C:/cygwin/bin/mv

# Path to mavlink and stm32 includes
AQLIB_PATH ?= ..
#AQLIB_PATH = C:/devel/AQ/lib

# Where to put the built objects and binaries.
# A sub-folder is created along this path, named as the BUILD_TYPE.
BUILD_PATH ?= .
#BUILD_PATH ?= ..


#
## probably don't need to change anything below here ##
#

# build/object directory
OBJ_PATH = $(BUILD_PATH)/$(BUILD_TYPE)/obj
# bin file(s) output path
BIN_PATH = $(BUILD_PATH)/$(BUILD_TYPE)

# command to execute (later, if necessary) for increasing build number in buildnum.h
CMD_BUILDNUMBER = $(shell $(EXE_AWK) '$$2 ~ /BUILDNUMBER/{ $$NF=$$NF+1 } 1' $(SRC_PATH)/buildnum.h > $(SRC_PATH)/tmp_buildnum.h && $(EXE_MV) $(SRC_PATH)/tmp_buildnum.h $(SRC_PATH)/buildnum.h)

# get current revision and build numbers
REV_NUM := $(shell $(EXE_AWK) '$$2 ~ /REVISION/{print $$4}' $(SRC_PATH)/buildnum.h)
BUILD_NUM := $(shell $(EXE_AWK) '$$2 ~ /BUILDNUMBER/{print $$NF}' $(SRC_PATH)/buildnum.h)
ifeq ($(INCR_BUILDNUM), 1)
	BUILD_NUM := $(shell echo $$[$(BUILD_NUM)+1])
endif

# Resulting bin file names before extension
ifeq ($(DEBUG_BUILD), 1)
	BIN_NAME := $(BIN_NAME)-debug
	INCR_BUILDNUM = 0
else
	BIN_NAME := $(BIN_NAME).r$(REV_NUM).b$(BUILD_NUM)-rev$(BOARD_REV)
	ifdef BIN_SUFFIX
		BIN_NAME := $(BIN_NAME)-$(BIN_SUFFIX)
	endif
endif

# Compiler-specific paths
CC_BIN_PATH = $(CC_PATH)/gcc/arm-unknown-eabi/bin
CC_LIB_PATH = $(CC_PATH)/lib
CC_INC_PATH = $(CC_PATH)/include
CC = $(CC_BIN_PATH)/cc1
AS = $(CC_BIN_PATH)/as
LD = $(CC_BIN_PATH)/ld
OBJCP = $(CC_BIN_PATH)/objcopy

# External libraries required by AQ
MAVINC_PATH = $(AQLIB_PATH)/mavlink/include/autoquad
STMLIB_PATH = $(AQLIB_PATH)/STM32
# also STM32f4.ld should be here

# Some supporting files/libraries are kept within the working source code base for now.
# (see corresponding object lists below for which files live in these folders)
# It can be convenient to break up these source files into actual subfolders.
# The official AQ source repo keeps everything in one folder since CrossWorks
# Studio uses virtual folders for code organization.
ifeq ($(FLAT_SRC), 1)
	STM32SYS_PATH = $(SRC_PATH)
	COOS_PATH = $(SRC_PATH)
	DSP_PATH = $(SRC_PATH)
	FATFS_PATH = $(SRC_PATH)
	DIGI_PATH = $(SRC_PATH)
	AQINC_PATHS = $(SRC_PATH)
else
	AQINC_PATH = .
	STM32SYS_PATH = $(AQINC_PATH)/STM32_System
	COOS_PATH = $(AQINC_PATH)/CoOS
	DSP_PATH = $(AQINC_PATH)/DSPLIB
	FATFS_PATH = $(AQINC_PATH)/fatFS
	DIGI_PATH = $(SRC_PATH)/digitalIMU
	AQINC_PATHS = $(SRC_PATH) $(COOS_PATH) $(DSP_PATH) $(FATFS_PATH) $(STM32SYS_PATH)
endif

# all include flags for the compiler
CC_INCLUDES :=  $(addprefix -I, $(AQINC_PATHS)) -I$(STMLIB_PATH)/include -I$(MAVINC_PATH) -I$(CC_INC_PATH)

# compiler flags
CC_OPTS = -mcpu=cortex-m4 -mthumb -mlittle-endian -mfpu=fpv4-sp-d16 -mfloat-abi=hard -nostdinc -fsingle-precision-constant -fno-gcse -Wall -finline-functions -Wdouble-promotion -std=c99 \
	-fno-dwarf2-cfi-asm -fno-builtin -ffunction-sections -fdata-sections -fno-common -fmessage-length=0 -quiet -MD $(basename $@).d -MQ $@

# macro definitions to pass via compiler command line
#
CC_VARS = -D__ARM_ARCH_7EM__ -D__CROSSWORKS_ARM -D__ARM_ARCH_FPV4_SP_D16__ -D__TARGET_PROCESSOR=STM32F407VG -D__TARGET_F4XX= -DSTM32F4XX= -D__FPU_PRESENT \
	-DARM_MATH_CM4 -D__THUMB -DNESTED_INTERRUPTS -DCTL_TASKING -DUSE_STDPERIPH_DRIVER 

CC_VARS += -DBOARD_VERSION=$(BOARD_VER) -DBOARD_REVISION=$(BOARD_REV)

ifeq ($(HW_FC_NONE),1)
	CC_VARS += -DMAVLINK_SERIAL_PORT_FLOW_NONE
endif

# Additional target(s) to build based on conditionals
#
EXTRA_TARGETS =
ifeq ($(INCR_BUILDNUM), 1)
	EXTRA_TARGETS = BUILDNUMBER
endif

# build type flags/defs (debug vs. release)
# (exclude STARTUP_FROM_RESET in debug builds if using Rowley debugger)
ifeq ($(DEBUG_BUILD), 1)
	BT_CFLAGS = -DDEBUG -DSTARTUP_FROM_RESET -DUSE_FULL_ASSERT -O1 -ggdb -g2
else
	BT_CFLAGS = -DNDEBUG -DSTARTUP_FROM_RESET -O2
endif


# all compiler options
CFLAGS = $(CC_OPTS) $(CC_INCLUDES) $(CC_VARS) $(BT_CFLAGS)

# assembler options
AS_OPTS = --traditional-format -mcpu=cortex-m4 -mthumb -EL -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# linker (ld) options
LINKER_OPTS = -ereset_handler --omagic -defsym=__do_debug_operation=__do_debug_operation_mempoll -u__do_debug_operation_mempoll -defsym=__vfprintf=__vfprintf_double_long_long -u__vfprintf_double_long_long \
	-defsym=__vfscanf=__vfscanf_double_long_long -u__vfscanf_double_long_long --fatal-warnings -EL --gc-sections -T$(STMLIB_PATH)/STM32f4.ld -Map $(OBJ_PATH)/aqv6.6.map -u_vectors

# eabi linker libs
# ! These are proprietary Rowley libraries, approved for personal use with the AQ project (see http://forum.autoquad.org/viewtopic.php?f=31&t=44&start=50#p8476 )
EXTRA_LIB_FILES = libm_v7em_fpv4_sp_d16_hard_t_le_eabi.a libc_v7em_fpv4_sp_d16_hard_t_le_eabi.a libcpp_v7em_fpv4_sp_d16_hard_t_le_eabi.a \
	libdebugio_v7em_fpv4_sp_d16_hard_t_le_eabi.a libc_targetio_impl_v7em_fpv4_sp_d16_hard_t_le_eabi.a libc_user_libc_v7em_fpv4_sp_d16_hard_t_le_eabi.a
EXTRA_LIBS := $(addprefix $(CC_LIB_PATH)/, $(EXTRA_LIB_FILES))


# AQ code objects to create (correspond to .c source to compile)
AQV6_OBJS := 1wire.o adc.o algebra.o analog.o aq_init.o aq_mavlink.o aq_timer.o \
	comm.o command.o compass.o config.o control.o can.o \
	digital.o esc32.o eeprom.o \
	filer.o flash.o fpu.o futaba.o \
	gimbal.o gps.o getbuildnum.o grhott.o imu.o util.o logger.o \
	main_ctl.o motors.o \
	nav.o nav_ukf.o pid.o ppm.o pwm.o \
	radio.o rotations.o rcc.o rtc.o run.o \
	sdio.o serial.o signaling.o spektrum.o spi.o srcdkf.o supervisor.o \
	telemetry.o ublox.o vn100.o

## other objects to create

# STM32 related including preprocessor and startup 
STM32_SYS_OBJ_FILES =  misc.o stm32f4xx_adc.o stm32f4xx_can.o stm32f4xx_dma.o stm32f4xx_exti.o stm32f4xx_flash.o stm32f4xx_gpio.o stm32f4xx_pwr.o stm32f4xx_rcc.o stm32f4xx_rtc.o stm32f4xx_sdio.o \
	stm32f4xx_spi.o stm32f4xx_syscfg.o stm32f4xx_tim.o stm32f4xx_usart.o system_stm32f4xx.o STM32_Startup.o thumb_crt0.o
ifeq ($(STM32SYS_PATH),$(SRC_PATH))
	STM32_SYS_OBJS := $(STM32_SYS_OBJ_FILES)
	STM32_OBJ_TARGET := $(OBJ_PATH)
else
	STM32_SYS_OBJS := $(addprefix $(STM32SYS_PATH)/, $(STM32_SYS_OBJ_FILES))
	STM32_OBJ_TARGET := $(OBJ_PATH)/$(STM32SYS_PATH)
endif

# CoOS
COOS_OBJ_FILES = arch.o core.o event.o flag.o kernelHeap.o mbox.o mm.o mutex.o port.o queue.o sem.o serviceReq.o task.o time.o timer.o utility.o
ifeq ($(COOS_PATH),$(SRC_PATH))
	COOS_OBJS := $(COOS_OBJ_FILES)
else
	COOS_OBJS := $(addprefix $(COOS_PATH)/, $(COOS_OBJ_FILES))
endif

# ARM
DSPLIB_OBJ_FILES = arm_fill_f32.o arm_copy_f32.o arm_mat_init_f32.o arm_mat_inverse_f32.o arm_mat_trans_f32.o arm_mat_mult_f32.o \
	arm_mat_add_f32.o arm_mat_sub_f32.o arm_mean_f32.o arm_scale_f32.o arm_std_f32.o
ifeq ($(DSP_PATH),$(SRC_PATH))
	DSPLIB_OBJS := $(DSPLIB_OBJ_FILES)
else
	DSPLIB_OBJS := $(addprefix $(DSP_PATH)/, $(DSPLIB_OBJ_FILES))
endif

# FATfs
FATFS_OBJ_FILES = ff.o
ifeq ($(FATFS_PATH),$(SRC_PATH))
	FATFS_OBJS := $(FATFS_OBJ_FILES)
else
	FATFS_OBJS := $(addprefix $(FATFS_PATH)/, $(FATFS_OBJ_FILES))
endif

# Digital IMU
DIGI_OBJ_FILES = d_imu.o hmc5983.o mpu6000.o ms5611.o
ifeq ($(DIGI_PATH),$(SRC_PATH))
	DIGI_OBJS := $(DIGI_OBJ_FILES)
else
	DIGI_OBJS := $(addprefix $(DIGI_PATH)/, $(DIGI_OBJ_FILES))
endif


## all objects
C_OBJECTS := $(addprefix $(OBJ_PATH)/, $(AQV6_OBJS) $(STM32_SYS_OBJS) $(COOS_OBJS) $(DSPLIB_OBJS) $(FATFS_OBJS) $(DIGI_OBJS))

# dependency files generated by previous make runs
DEPS := $(C_OBJECTS:.o=.d)


#
## Target definitions
#

.PHONY: all clean

all: CREATE_BUILD_FOLDER $(EXTRA_TARGETS) $(BIN_PATH)/$(BIN_NAME).hex

clean:
	rm -fr $(OBJ_PATH)
#	rm -f $(BIN_PATH)/*.*

# include auto-generated depenency targets
-include $(DEPS)

$(OBJ_PATH)/%.o: $(SRC_PATH)/%.c
	@echo ""
	@echo "## Compiling $< -> $@ ##"
	$(CC) $(CFLAGS) $< -o $(basename $@).lst
	@echo "## Assembling --> $@ ##"
	$(AS) $(AS_OPTS) $(basename $@).lst -o $@
	@rm -f $(basename $@).lst

$(STM32_OBJ_TARGET)/STM32_Startup.o: $(STMLIB_PATH)/STM32_Startup.s
	@echo ""
	@echo "## Compiling $< -> $@ ##"
	$(CC) $(CFLAGS) -E -lang-asm $< -o $(basename $@).lst
	@echo "## Assembling --> $@ ##"
	$(AS) $(AS_OPTS) -gdwarf-2 $(basename $@).lst -o $@
	@rm -f $(basename $@).lst

$(STM32_OBJ_TARGET)/thumb_crt0.o: $(STM32SYS_PATH)/thumb_crt0.s
	@echo ""
	@echo "## Compiling $< -> $@ ##"
	$(CC) $(CFLAGS) -E -lang-asm $< -o $(basename $@).lst
	@echo "## Assembling --> $@ ##"
	$(AS) $(AS_OPTS) -gdwarf-2 $(basename $@).lst -o $@
	@rm -f $(basename $@).lst

$(BIN_PATH)/$(BIN_NAME).elf: $(C_OBJECTS)
	@echo ""
	@echo "## Linking --> $@ ##"
	$(LD) -X $(LINKER_OPTS) -o $@ --start-group $(C_OBJECTS) $(EXTRA_LIBS) --end-group

$(BIN_PATH)/$(BIN_NAME).bin: $(BIN_PATH)/$(BIN_NAME).elf
	@echo ""
	@echo "## Objcopy $< --> $@ ##"
	$(OBJCP) -O binary $< $@

$(BIN_PATH)/$(BIN_NAME).hex: $(BIN_PATH)/$(BIN_NAME).elf
	@echo ""
	@echo "## Objcopy $< --> $@ ##"
	$(OBJCP) -O ihex $< $@

CREATE_BUILD_FOLDER :
	@echo "Creating Build folders if necessary"
	@mkdir -p $(OBJ_PATH)

BUILDNUMBER :
	@echo "Incrementing Build Number"
	$(CMD_BUILDNUMBER)

## Flash-Loader (Linux only) 			##
## Requires AQ ground tools sources	##
$(SRC_PATH)/../ground/loader: $(SRC_PATH)/../ground/loader.c
	(cd $(SRC_PATH)/../ground/ ; make loader)

flash: $(SRC_PATH)/../ground/loader
	$(SRC_PATH)/../ground/loader -p $(USB_DEVICE) -b 115200 -f $(BIN_PATH)/$(BIN_NAME).hex
