# Makefile for AutoQuad Flight Controller firmware
#
# ! Use of this makefile requires setup of a compatible development environment.
# ! For latest development recommendations, check the README file distributed with this project.
# ! This file is ignored when building with CrossWorks Studio.
#
# All paths are relative to Makefile location.  Possible make targets:
#  all         build firmware .elf, .hex, and .bin binaries
#  hex         build firmware .elf, and .hex binaries
#  bin         build firmware .elf, and .bin binaries
#  flash       attempt to build ../ground/loader and flash firmware to board (linux only)
#  pack        create .zip archive of generated .hex and .bin files (requires GNU zip)
#  pack-hex    create .zip archive of generated .hex files
#  pack-bin    create .zip archive of generated .bin files
#  clean       delete all built objects (not binaries or archives)
#  clean-bin   delete all binaries created in build folder (*.elf, *.bin, *.hex)
#  clean-pack  delete all archives in build folder (*.zip)
#  clean-all   run all the above clean* steps.
#  size        run a memory report on an elf file (also automatically run after building elf file)
#
# Read comments below under "External libraries required by AQ" for dependency details.
#
# Usage examples:
#   make all                                   # default Release type builds .elf, .hex, and .bin binaries
#   make all BUILD_TYPE=Debug                  # build with compiler debugging flags/options enabled
#   make all BOARD_REV=1 INCR_BUILDNUM=0       # build for rev 1 (post Oct-2012) hardware, don't increment the buildnumber
#   make hex BOARD_REV=1 DIMU_VER=1.1          # build only .hex file for rev 1 hardware with DIMU add-on board
#   make bin BOARD_VER=8 BOARD_REV=3 QUATOS=1  # build .bin file for AQ M4 hardware using Quatos (see note on QUATOS option, below)
#
# 
# To get a proper size report after building, the GCC program "size" is needed (arm-none-eabi-size[.exe]). 
#  The SIZE variable (below) specifies its location. By default this is in the same folder as the compiler/build chain. However, 
#  the CrossWorks build chain does not provide one.  A copy can be obtained from many sources, 
#  eg. any recent version of https://launchpad.net/gcc-arm-embedded, Yagarto, etc.
#
# Windows needs some core GNU tools in your %PATH% (probably same place your "make" is). 
#    Required: gawk, mv, echo, rm, mkdir (see note)
#    Optional: expr (auto-incrmenent buildnumber), zip (to compress hex files using "make pack")
#   MKDIR Note: Check the EXE_MKDIR variable below -- due to a naming conflict with the Windows "mkdir", you may need to specify a full path for it.
#   Recommend GnuWin32 CoreUtils http://gnuwin32.sourceforge.net/packages/coreutils.htm
#

# Include user-specific settings file, if any, in regular Makefile format.
# This file can set any default variable values you wish to override (all defaults are listed below).
# The .user file is not included with the source code distribution, so it will not be overwritten.
-include Makefile.user

# Defaults - modify here, on command line, or in Makefile.user
#
# Output folder name; Use 'Debug' to set debug compiler options;
BUILD_TYPE ?= Release

# Path to source files - no trailing slash
SRC_PATH ?= src

# Where to put the built objects and binaries. Use any relative or absolute path (directory must alrady exist).
# A sub-folder is created along this path, named as the BUILD_TYPE (eg. build/Release).
BUILD_PATH ?= ../build

# Board version to build for (6=AQv6, 7=AQv7, 8=AQ M4)
BOARD_VER ?= 6

# Board revision to build for 
# For AQv6: 0=initial release, 1=Oct. 2012 revision)
# For AQ M4: 1-3=early prototypes, 4-5=beta boards, 6=Dec 2014 production (aka M4v2)
BOARD_REV ?= 1

# Specify a DIMU version number to enable DIMU support in AQv6, zero to disable (eg. DIMU_VER=1.1)
ifneq ($(findstring 6, $(BOARD_VER)),)
	DIMU_VER ?= 1.1
else
	DIMU_VER ?= 0
endif

# Increment build number? (0|1)  This is automatically disabled for debug builds.
INCR_BUILDNUM ?= 0

# Produced binaries file name prefix (version/revision/build/hardware info will be automatically appended)
BIN_NAME ?= aq

# You may use BIN_SUFFIX to append text to generated bin file name after version string;
BIN_SUFFIX ?=

# Build debug version? (0|1; true by default if build_type contains the word "debug")
ifneq ($(findstring Debug, $(BUILD_TYPE)),)
	DEBUG_BUILD ?= 1
else 
	DEBUG_BUILD ?= 0
endif

# Build with Quatos controller enabled (0=no, 1=yes)
# NOTE: Must have pre-compiled quatos library file in lib/quatos (it is not distrubuted with this GPL source code)
#  Quatos is a proprietary attitude controller from drone-controls.com
QUATOS ?= 0

# Build with HILS enabled (0=no, 1=yes)
# NOTE: Must have pre-compiled hilSim library file in lib/hilsim
#  hilSim is a hardware-in-the-loop simulation support library from worlddesign.com
HIL_SIM ?= 0

# Build with specific default parameters file (eg. CONFIG_FILE=config_default_m4.h)
CONFIG_FILE ?=

# Build with specific board (hardware) definitions file (eg. BOARD_FILE=board_custom.h)
BOARD_FILE ?=

# Add preprocessor definitions to CDEFS (eg. CC_ADD_VARS=-DCOMM_DISABLE_FLOW_CONTROL1 to disable flow control on USART 1)
CC_ADD_VARS ?=

# Additional C objects to compile
EXTRA_OBJECTS ?=


# System-specific folder paths and commands
#
# compiler base path
# eg: CC_PATH ?= C:/devel/gcc/crossworks_for_arm_2.3
CC_PATH ?= /usr/share/crossworks_for_arm_2.3

# Absolute or relative path to libraries/includes, no trailing slash.
# eg: AQLIB_PATH = ../lib or /usr/lib/aq
AQLIB_PATH ?= lib

# Generated MAVLink header files (https://github.com/AutoQuad/mavlink_headers/tree/master/include)
MAVINC_PATH ?= $(AQLIB_PATH)/mavlink/include/autoquad
# STM32F4 libs from ST (http://www.st.com/web/en/catalog/tools/PF257901)
STM32DRIVER_PATH ?= $(AQLIB_PATH)/STM32F4xx_StdPeriph_Driver
STM32CMSIS_PATH ?= $(AQLIB_PATH)/CMSIS
STM32DSPLIB_PATH ?= $(STM32CMSIS_PATH)/DSP_Lib
# Proprietary Quatos attitude controller
QUATOS_PATH ?= $(AQLIB_PATH)/quatos
# Proprietary HILS library
HIL_SIM_PATH ?= $(AQLIB_PATH)/hilsim

# shell commands
EXE_AWK ?= gawk 
EXE_MKDIR ?= mkdir
#EXE_MKDIR = C:/cygwin/bin/mkdir
EXE_TEST ?= test
EXE_ZIP ?= zip -j
# file extention for compressed files (gz for gzip, etc)
ZIP_EXT ?= zip

# Flashing interface (Linux only)
USB_DEVICE ?= /dev/ttyUSB0

# defaults end


#
## probably don't need to change anything below here ##
#

# build/object directory
OBJ_PATH = $(BUILD_PATH)/$(BUILD_TYPE)/obj
# bin file(s) output path
BIN_PATH = $(BUILD_PATH)/$(BUILD_TYPE)

# clean up legacy options
ifeq ($(BIN_SUFFIX),0)
	BIN_SUFFIX =
endif
ifeq ($(CONFIG_FILE),0)
	CONFIG_FILE =
endif
ifeq ($(BOARD_FILE),0)
	BOARD_FILE =
endif

# prevent HILS build with AIMU
ifneq ($(HIL_SIM), 0)
	ifeq ($(BOARD_VER), 6)
		ifeq ($(DIMU_VER), 0)
			HIL_SIM = 0
		endif
	endif
endif

#
## Determine build version and final name for binary target
#

VERSION_FILE = $(SRC_PATH)/aq_version.h
ifeq ("$(wildcard $(VERSION_FILE))","")
	VERSION_FILE = $(SRC_PATH)/getbuildnum.h
endif
# get current version and build numbers
FW_VER := $(shell $(EXE_AWK) 'BEGIN { FS = "[ \"\t]+" }$$2 ~ /FI(MR|RM)WARE_VER(SION|_MAJ)/{print $$NF}' $(VERSION_FILE))
ifeq ($(findstring ., $(FW_VER)),)
	FW_VER := $(FW_VER).$(shell $(EXE_AWK) '$$2 ~ /FIMRWARE_VER_MIN/{print $$NF}' $(VERSION_FILE))
endif

BUILD_NUM := $(shell $(EXE_AWK) '$$2 ~ /BUILDNUMBER/{print $$NF}' $(SRC_PATH)/buildnum.h)
ifeq ($(INCR_BUILDNUM), 1)
	BUILD_NUM := $(shell expr $(BUILD_NUM) + 1)
endif

# Resulting bin file names before extension
ifeq ($(DEBUG_BUILD), 1)
	# debug build gets a consistent name to simplify dev setup
	TARGET := $(BIN_NAME)-debug
	override INCR_BUILDNUM = 0
else
	TARGET := $(BIN_NAME)v$(FW_VER).$(BUILD_NUM)-hwv$(BOARD_VER).$(BOARD_REV)
	ifneq ($(DIMU_VER), 0)
		TARGET := $(TARGET)-dimu$(DIMU_VER)
	endif
	ifneq ($(QUATOS), 0)
		TARGET := $(TARGET)-quatos
	endif
	ifneq ($(HIL_SIM), 0)
		TARGET := $(TARGET)-hils
	endif
	ifneq ($(BIN_SUFFIX),)
		TARGET := $(TARGET)-$(BIN_SUFFIX)
	endif
endif

#
## Build tools specs and flags
#

# Compiler-specific paths
CC_BIN_PATH = $(CC_PATH)/gcc/arm-unknown-eabi/bin
CC_LIB_PATH = $(CC_PATH)/lib
CC_INC_PATH = $(CC_PATH)/include
CC = $(CC_BIN_PATH)/cc1
AS = $(CC_BIN_PATH)/as
LD = $(CC_BIN_PATH)/ld
OBJCP = $(CC_BIN_PATH)/objcopy
SIZE ?= $(CC_BIN_PATH)/size

# all include flags for the compiler
CC_INCLUDES ?=
CC_INCLUDES += -I$(SRC_PATH)
CC_INCLUDES += -I$(SRC_PATH)/co_os
CC_INCLUDES += -I$(SRC_PATH)/drivers
CC_INCLUDES += -I$(SRC_PATH)/drivers/can
CC_INCLUDES += -I$(SRC_PATH)/drivers/disk
CC_INCLUDES += -I$(SRC_PATH)/drivers/usb
CC_INCLUDES += -I$(SRC_PATH)/math
CC_INCLUDES += -I$(SRC_PATH)/radio
CC_INCLUDES += -I$(SRC_PATH)/targets
CC_INCLUDES += -I$(STM32DRIVER_PATH)/inc
CC_INCLUDES += -I$(STM32CMSIS_PATH)/Include
CC_INCLUDES += -I$(STM32CMSIS_PATH)/Device/ST/STM32F4xx/Include
CC_INCLUDES += -I$(MAVINC_PATH)
CC_INCLUDES += -I$(CC_INC_PATH)
CC_INCLUDES += -I$(HIL_SIM_PATH)/include

# path hints for make
VPATH ?=
VPATH += $(SRC_PATH) $(STM32DRIVER_PATH)/src $(STM32DSPLIB_PATH)/Source

# macro definitions to pass via compiler command line
#
CDEFS ?=
CDEFS += \
	-D__SIZEOF_WCHAR_T=4              \
	-D__ARM_ARCH_7EM__                \
	-D__CROSSWORKS_ARM                \
	-D__ARM_ARCH_FPV4_SP_D16__        \
	-D__TARGET_PROCESSOR=STM32F407VG  \
	-D__CROSSWORKS_MAJOR_VERSION=2    \
	-D__CROSSWORKS_MINOR_VERSION=3    \
	-D__CROSSWORKS_REVISION=2         \
	-D__TARGET_PROCESSOR_STM32F407VG  \
	-DSTM32F4XX -DSTM32F40_41xxx      \
	-D__FPU_PRESENT=1                 \
	-DARM_MATH_CM4                    \
	-D__THUMB                         \
	-DNESTED_INTERRUPTS               \
	-DCTL_TASKING                     \
	-DUSE_STDPERIPH_DRIVER            \
	-DBOARD_VERSION=$(BOARD_VER)      \
	-DBOARD_REVISION=$(BOARD_REV)     \
	-DDIMU_VERSION=$(subst .,,$(DIMU_VER))

ifneq ($(CONFIG_FILE),)
	CDEFS += -DCONFIG_DEFAULTS_FILE=\"$(CONFIG_FILE)\"
endif
ifneq ($(BOARD_FILE),)
	CDEFS += -DBOARD_HEADER_FILE=\"$(BOARD_FILE)\"
endif

# compiler flags
CFLAGS ?=
CFLAGS += \
	-ggdb -g2 \
	-mcpu=cortex-m4 \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-mthumb \
	-mlittle-endian \
	-fsingle-precision-constant	\
	-quiet \
	-Wall \
	-Wdouble-promotion \
	-Winline \
	-finline-functions \
	-std=c99 \
	-nostdinc \
	-fno-dwarf2-cfi-asm \
	-fno-builtin \
	-fno-common \
	-ffunction-sections \
	-fdata-sections \
	-fmessage-length=0 \
	--param large-function-insns=5400

CFLAGS += $(CC_INCLUDES)
CFLAGS += $(CDEFS)
CFLAGS += $(CC_ADD_VARS)

# assembler options
ASMFLAGS = --traditional-format -mcpu=cortex-m4 -mthumb -EL -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# linker options
LDFLAGS = -ereset_handler --omagic --fatal-warnings -EL --gc-sections -T$(SRC_PATH)/targets/autoquad.ld -Map $(OBJ_PATH)/autoquad.map -u_vectors \
	-defsym=__vfprintf=__vfprintf_double_long_long -u__vfprintf_double_long_long -defsym=__vfscanf=__vfscanf_double_long_long -u__vfscanf_double_long_long

# external ARM libs
# ! These are proprietary Rowley libraries, approved for personal use with the AQ project (see http://forum.autoquad.org/viewtopic.php?f=31&t=44&start=50#p8476 )
LDLIBS_FILES = libm libc libcpp libc_user_libc

ARM_LIB_QUAL ?= _v7em_fpv4_sp_d16_hard_t_le_eabi

# build type flags/defs (debug vs. release)
# (exclude STARTUP_FROM_RESET in debug builds if using Rowley debugger)
ifeq ($(DEBUG_BUILD), 1)
	CFLAGS += -O1
	CDEFS += -DDEBUG -DUSE_FULL_ASSERT -DSTARTUP_FROM_RESET
	LDFLAGS += -defsym=__do_debug_operation=__do_debug_operation_mempoll -u__do_debug_operation_mempoll
	LDLIBS_FILES += libdebugio libc_targetio_impl
else
	CFLAGS += -O2
	CDEFS += -DNDEBUG -DSTARTUP_FROM_RESET 
endif

LDLIBS ?=
LDLIBS += $(addprefix $(CC_LIB_PATH)/, $(addsuffix $(ARM_LIB_QUAL).a, $(LDLIBS_FILES)))

ifneq ($(QUATOS), 0)
	ifeq ($(QUATOS), 1)
		LDLIBS += $(QUATOS_PATH)/quatos.a
	endif
	CFLAGS += -DHAS_QUATOS
endif

ifneq ($(HIL_SIM), 0)
	ifeq ($(HIL_SIM), 1)
		LDLIBS += $(HIL_SIM_PATH)/hilSim.a
	endif
	CFLAGS += -DHAS_HIL_SIM_MP
endif

# end of build tools options


#
## What to build
#

# Objects to create (correspond to .c source to compile)

# core code in src/
AQ_OBJ += \
	alt_ukf.o aq_init.o aq_mavlink.o aq_timer.o \
	calib.o comm.o command.o compass.o config.o control.o \
	d_imu.o esc32.o filer.o gimbal.o gps.o imu.o logger.o \
	main_ctl.o motors.o nav.o nav_ukf.o pid.o rc.o run.o \
	signaling.o supervisor.o \
	telem_sPort.o telemetry.o util.o

# Math lib
AQ_OBJ += \
	math/algebra.o \
	math/rotations.o \
	math/srcdkf.o 

# RC radio handlers
AQ_OBJ += \
	radio/dsm.o \
	radio/futaba.o \
	radio/grhott.o \
	radio/mlinkrx.o \
	radio/ppm.o \
	radio/radio.o  \
	radio/spektrum.o 

# Target hardware-specific in src/targets
AQ_OBJ += \
	targets/system_stm32f4xx.o \
	targets/STM32_Startup.o \
	targets/thumb_crt0.o

# CoOS in src/co_os/
AQ_OBJ += $(addprefix co_os/, \
	arch.o core.o event.o flag.o kernelHeap.o mbox.o mm.o mutex.o \
	port.o queue.o sem.o serviceReq.o task.o time.o timer.o utility.o \
)

# CAN drivers and functions
AQ_OBJ += \
	drivers/can/can.o \
	drivers/can/canCalib.o \
	drivers/can/canOSD.o \
	drivers/can/canSensors.o \
	drivers/can/canUart.o

# Disk I/O and FatFS drivers
AQ_OBJ += \
	drivers/disk/ff.o \
	drivers/disk/sdio.o

# Target hardware-level drivers in src/drivers
AQ_OBJ += $(addprefix drivers/, \
	1wire.o adc.o analog.o cyrf6936.o eeprom.o ext_irq.o digital.o \
	flash.o fpu.o  hmc5983.o max21100.o mpu6000.o ms5611.o \
	pwm.o rcc.o rtc.o serial.o spi.o ublox.o \
)

# USB drivers in src/drivers/usb/
AQ_OBJ += $(addprefix drivers/usb/, \
	usb.o usb_bsp.o usb_core.o usb_dcd.o usb_dcd_int.o \
	usbd_core.o  usbd_desc.o  usbd_ioreq.o  usbd_req.o usbd_storage_msd.o \
	usbd_cdc_msc_core.o usbd_msc_bot.o usbd_msc_data.o usbd_msc_scsi.o \
)

# STM32 drivers from STM32F4xx_StdPeriph_Driver/src/
LIB_OBJ += $(addprefix $(STM32DRIVER_PATH)/src/, \
	misc.o \
	$(addprefix stm32f4xx_, \
		adc.o can.o dbgmcu.o dma.o exti.o flash.o gpio.o hash.o hash_md5.o \
		pwr.o rcc.o rtc.o sdio.o spi.o syscfg.o tim.o usart.o \
	) \
)

# ARM drivers from CMSIS/DSP_Lib/Source/
LIB_OBJ += $(addprefix $(STM32DSPLIB_PATH)/Source/, \
	BasicMathFunctions/arm_scale_f32.o \
	MatrixFunctions/arm_mat_init_f32.o \
	MatrixFunctions/arm_mat_inverse_f32.o \
	MatrixFunctions/arm_mat_trans_f32.o \
	MatrixFunctions/arm_mat_mult_f32.o \
	MatrixFunctions/arm_mat_add_f32.o \
	MatrixFunctions/arm_mat_sub_f32.o \
	StatisticsFunctions/arm_mean_f32.o \
	StatisticsFunctions/arm_std_f32.o \
	SupportFunctions/arm_fill_f32.o \
	SupportFunctions/arm_copy_f32.o \
)


# combine objects
ALL_OBJ = $(addprefix $(OBJ_PATH)/, $(addprefix $(SRC_PATH)/, $(AQ_OBJ)) $(LIB_OBJ) $(EXTRA_OBJECTS))

# dependency files generated by previous make runs
DEPS := $(ALL_OBJ:.o=.d)

# Additional target(s) to build based on conditionals
#
EXTRA_TARGET_DEPS ?= #makefile-debug
ifeq ($(INCR_BUILDNUM), 1)
	EXTRA_TARGET_DEPS = BUILDNUMBER
endif


#
## Misc. config
#

# command to execute (later, if necessary) for increasing build number in buildnum.h
CMD_BUILDNUMBER = $(shell $(EXE_AWK) '$$2 ~ /BUILDNUMBER/{ $$NF=$$NF+1 } 1' $(SRC_PATH)/buildnum.h > $(SRC_PATH)/tmp_buildnum.h && mv $(SRC_PATH)/tmp_buildnum.h $(SRC_PATH)/buildnum.h)

# target memory locations and sizes in KB (from autoquad.ld script)
MEM_START_FLASH = 0x8000000
MEM_START_CCM   = 0x10000000
MEM_START_SRAM1 = 0x20000000
MEM_START_SRAM2 = 0x2001c000

MEM_SIZE_FLASH = 1024
MEM_SIZE_CCM   = 64
MEM_SIZE_SRAM1 = 112
MEM_SIZE_SRAM2 = 16

# script to run for reporting allocated memory sizes
CMD_SIZE_REPORT = `$(SIZE) -A -x $(BIN_PATH)/$(TARGET).elf | $(EXE_AWK) -n '\
	BEGIN { \
		c=0; r=0; r2=0; f=0; \
		printf("\n---- Size report ----\n\nSection Details:\n%-10s\t%7s\t\t%10s\t%s\n", "section", "size(B)", "addr", "loc") \
	} \
	NR > 2 && $$3 != 0x0 && $$3 != "" { \
		printf("%-10s\t%7d\t\t0x%08x\t", $$1, $$2, $$3); \
		if      ($$1 == ".data")            {f += $$2; print "F"}   \
		else if ($$3 >= $(MEM_START_SRAM2)) {r2 += $$2; print "S2"} \
		else if ($$3 + $$2 >= $(MEM_START_SRAM2)) {t = $$3 + $$2 - $(MEM_START_SRAM2); r2 += t; r += $$2 - t; print "S1+S2"} \
		else if ($$3 >= $(MEM_START_SRAM1)) {r += $$2; print "S1"}  \
		else if ($$3 >= $(MEM_START_CCM))   {c += $$2; print "C"}   \
		else if ($$3 >= $(MEM_START_FLASH)) {f += $$2; print "F"}   \
	} \
	END { \
		printf("\nTotals: %10s\t  usage\tof ttl\tKB free\n Flash: %10.2f\t%6.2f%\t%6d\t%7.2f\n   CCM: %10.2f\t%6.2f%\t%6d\t%7.2f\n SRAM1: %10.2f\t%6.2f%\t%6d\t%7.2f\n SRAM2: %10.2f\t%6.2f%\t%6d\t%7.2f\n", "KB", \
		f/1024, f/($(MEM_SIZE_FLASH)*1024)*100, $(MEM_SIZE_FLASH), $(MEM_SIZE_FLASH) - f/1024, \
		c/1024, c/($(MEM_SIZE_CCM)*1024)*100, $(MEM_SIZE_CCM), $(MEM_SIZE_CCM) - c/1024, \
		r/1024, r/($(MEM_SIZE_SRAM1)*1024)*100, $(MEM_SIZE_SRAM1), $(MEM_SIZE_SRAM1) - r/1024, \
		r2/1024, r2/($(MEM_SIZE_SRAM2)*1024)*100, $(MEM_SIZE_SRAM2), $(MEM_SIZE_SRAM2) - r2/1024 ); \
	}'`


#
## Target definitions
#

.PHONY: all elf hex bin clean-all clean clean-bin clean-pack pack pack-hex pack-bin size BUILDNUMBER $(OBJ_PATH)
.IGNORE: size 

all: elf hex bin
elf: $(BIN_PATH)/$(TARGET).elf
hex: $(BIN_PATH)/$(TARGET).hex
bin: $(BIN_PATH)/$(TARGET).bin

clean-all: clean clean-bin clean-pack

clean:
	-rm -fr $(OBJ_PATH)
	
clean-bin:
	-rm -f $(BIN_PATH)/*.elf
	-rm -f $(BIN_PATH)/*.bin
	-rm -f $(BIN_PATH)/*.hex

clean-pack:
	-rm -f $(BIN_PATH)/*.$(ZIP_EXT)
	
pack: pack-hex pack-bin


# include auto-generated depenency targets
ifeq ($(findstring clean, $(MAKECMDGOALS)),)
-include $(DEPS)
endif

$(OBJ_PATH)/%.o: %.c
	@$(EXE_MKDIR) -p $(@D)
	@echo "## Compiling $< -> $@ ##"
	$(CC) $(CFLAGS) -MD $(basename $@).d -MQ $@ $< -o $(basename $@).lst
	@echo "## Assembling --> $@ ##"
	$(AS) $(ASMFLAGS) $(basename $@).lst -o $@
	@rm -f $(basename $@).lst

$(OBJ_PATH)/%.o: %.s
	@$(EXE_MKDIR) -p $(@D)
	@echo "## Compiling $< -> $@ ##"
	$(CC) $(CFLAGS) -MD $(basename $@).d -MQ $@ -E -lang-asm $< -o $(basename $@).lst
	@echo "## Assembling --> $@ ##"
	$(AS) $(ASMFLAGS) -gdwarf-2 $(basename $@).lst -o $@
	@rm -f $(basename $@).lst

$(BIN_PATH)/$(TARGET).elf: $(EXTRA_TARGET_DEPS) $(ALL_OBJ)
	@echo
	@echo "## Linking --> $@ ##"
	$(LD) -X $(LDFLAGS) -o $@ --start-group $(ALL_OBJ) $(LDLIBS) --end-group
	@$(EXE_TEST) -f $(SIZE) && echo "${CMD_SIZE_REPORT}";

$(BIN_PATH)/$(TARGET).bin: $(BIN_PATH)/$(TARGET).elf
	@echo
	@echo "## Objcopy $< --> $@ ##"
	$(OBJCP) -O binary $< $@

$(BIN_PATH)/$(TARGET).hex: $(BIN_PATH)/$(TARGET).elf
	@echo
	@echo "## Objcopy $< --> $@ ##"
	$(OBJCP) -O ihex $< $@

BUILDNUMBER :
	@echo
	@echo "Incrementing Build Number"
	$(CMD_BUILDNUMBER)
	@echo

size:
	@if $(EXE_TEST) -f $(SIZE) AND $(EXE_TEST) -f $(BIN_PATH)/$(TARGET).elf; then echo "${CMD_SIZE_REPORT}"; else echo ; echo "$(SIZE) or elf file not found!"; fi

makefile-debug:
	@echo
	@echo AQ_OBJ = $(AQ_OBJ)
	@echo
	@echo LIB_OBJ = $(LIB_OBJ)
	@echo
	@echo ALL_OBJ = $(ALL_OBJ)

pack-hex pack-bin: pack-% : %
	@echo
	@echo "Compressing $< files... "
	$(EXE_ZIP) $(BIN_PATH)/$(TARGET).$(ZIP_EXT) $(BIN_PATH)/$(TARGET).$<
	
## Flash-Loader (Linux only) 			##
## Requires AQ ground tools sources	##
$(SRC_PATH)/../ground/loader: $(SRC_PATH)/../ground/loader.c
	(cd $(SRC_PATH)/../ground/ ; make loader)

flash: $(SRC_PATH)/../ground/loader
	$(SRC_PATH)/../ground/loader -p $(USB_DEVICE) -b 115200 -f $(BIN_PATH)/$(TARGET).hex
