DEPS_SUBMODULES += lib/FreeRTOS-Kernel lib/lwip lib/tinyusb

TOP := $(shell realpath .)
# $(info Top directory is $(TOP))

# ---------------------------------------
# Common make definition for all targets
# ---------------------------------------

BOARD = stm32f4

# Build directory
BUILD := _build/$(BOARD)

PROJECT := $(notdir $(CURDIR))

# Handy check parameter function
check_defined = \
    $(strip $(foreach 1,$1, \
    $(call __check_defined,$1,$(strip $(value 2)))))
__check_defined = \
    $(if $(value $1),, \
    $(error Undefined make flag: $1$(if $2, ($2))))

FREERTOS_STATS := 0
LWIP_STATS := 0

#-------------- Select the board to build for. ------------

include $(BOARD)/board.mk

SRC_C += $(wildcard $(BOARD)/*.c)
INC   += $(BOARD)

# Fetch submodules depended by family
fetch_submodule_if_empty = $(if $(wildcard $(TOP)/$1/*),,$(info $(shell git -C $(TOP) submodule update --init $1)))
ifdef DEPS_SUBMODULES
  $(foreach s,$(DEPS_SUBMODULES),$(call fetch_submodule_if_empty,$(s)))
endif

#-------------- Cross Compiler  ------------
# Can be set by board, default to ARM GCC
CROSS_COMPILE ?= arm-none-eabi-

CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++
GDB = $(CROSS_COMPILE)gdb
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE = $(CROSS_COMPILE)size
MKDIR = mkdir
SED = sed
CP = cp
RM = rm
PYTHON = python3

#-------------- Source files and compiler flags --------------

# Compiler Flags
CFLAGS += \
  -ggdb \
  -fdata-sections \
  -ffunction-sections \
  -fsingle-precision-constant \
  -fno-strict-aliasing \
  -Wdouble-promotion \
  -Wstrict-prototypes \
  -Wstrict-overflow \
  -Wall \
  -Wextra \
  -Werror \
  -Wfatal-errors \
  -Werror-implicit-function-declaration \
  -Wfloat-equal \
  -Wundef \
  -Wshadow \
  -Wwrite-strings \
  -Wsign-compare \
  -Wmissing-format-attribute \
  -Wunreachable-code \
  -Wcast-align \
  -Wcast-function-type \
  -Wcast-qual \
  -Wnull-dereference

# Debugging/Optimization
ifeq ($(DEBUG), 1)
  CFLAGS += -Og
else
  CFLAGS += -Os
endif

# Log level is mapped to TUSB DEBUG option
ifneq ($(LOG),)
  CMAKE_DEFSYM +=	-DLOG=$(LOG)
  CFLAGS += -DCFG_TUSB_DEBUG=$(LOG)
endif

# Logger: default is uart, can be set to rtt or swo
ifneq ($(LOGGER),)
	CMAKE_DEFSYM +=	-DLOGGER=$(LOGGER)
endif

ifeq ($(LOGGER),rtt)
  CFLAGS += -DLOGGER_RTT -DSEGGER_RTT_MODE_DEFAULT=SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL
  RTT_SRC = lib/SEGGER_RTT
  INC   += $(TOP)/$(RTT_SRC)/RTT
  SRC_C += $(RTT_SRC)/RTT/SEGGER_RTT.c
else ifeq ($(LOGGER),swo)
  CFLAGS += -DLOGGER_SWO
endif

# Application source
INC += \
	src \
	src/FreeRTOSConfig

SRC_C += $(wildcard src/*.c)

# FreeRTOS source, all files in port folder
INC += \
	lib/FreeRTOS-Kernel/include \
	lib/FreeRTOS-Kernel/portable/GCC/$(FREERTOS_PORT)

SRC_C += \
	lib/FreeRTOS-Kernel/list.c \
	lib/FreeRTOS-Kernel/queue.c \
	lib/FreeRTOS-Kernel/tasks.c \
	lib/FreeRTOS-Kernel/timers.c \
  lib/FreeRTOS-Kernel/stream_buffer.c \
	$(wildcard lib/FreeRTOS-Kernel/portable/GCC/$(FREERTOS_PORT)/*.c)

ifeq ($(FREERTOS_STATS), 1)
  CFLAGS += -DFREERTOS_STATS_DISPLAY
  SRC_C += lib/FreeRTOS-Kernel/portable/MemMang/heap_4.c
endif

# Suppress FreeRTOS warnings
CFLAGS += -Wno-error=cast-qual

# FreeRTOS (lto + Os) linker issue
LDFLAGS += -Wl,--undefined=vTaskSwitchContext

# lwip sources
INC += \
  lib/lwip/src/include \
  lib/lwip/src/include/ipv4 \
  lib/lwip/src/include/lwip/apps \
  lib/tinyusb/lib/networking/

SRC_C += \
  lib/lwip/src/core/altcp.c \
  lib/lwip/src/core/altcp_alloc.c \
  lib/lwip/src/core/altcp_tcp.c \
  lib/lwip/src/core/def.c \
  lib/lwip/src/core/dns.c \
  lib/lwip/src/core/inet_chksum.c \
  lib/lwip/src/core/init.c \
  lib/lwip/src/core/ip.c \
  lib/lwip/src/core/mem.c \
  lib/lwip/src/core/memp.c \
  lib/lwip/src/core/netif.c \
  lib/lwip/src/core/pbuf.c \
  lib/lwip/src/core/raw.c \
  lib/lwip/src/core/stats.c \
  lib/lwip/src/core/sys.c \
  lib/lwip/src/core/tcp.c \
  lib/lwip/src/core/tcp_in.c \
  lib/lwip/src/core/tcp_out.c \
  lib/lwip/src/core/timeouts.c \
  lib/lwip/src/core/udp.c \
  lib/lwip/src/core/ipv4/autoip.c \
  lib/lwip/src/core/ipv4/dhcp.c \
  lib/lwip/src/core/ipv4/etharp.c \
  lib/lwip/src/core/ipv4/icmp.c \
  lib/lwip/src/core/ipv4/igmp.c \
  lib/lwip/src/core/ipv4/ip4.c \
  lib/lwip/src/core/ipv4/ip4_addr.c \
  lib/lwip/src/core/ipv4/ip4_frag.c \
  lib/lwip/src/core/ipv6/dhcp6.c \
  lib/lwip/src/core/ipv6/ethip6.c \
  lib/lwip/src/core/ipv6/icmp6.c \
  lib/lwip/src/core/ipv6/inet6.c \
  lib/lwip/src/core/ipv6/ip6.c \
  lib/lwip/src/core/ipv6/ip6_addr.c \
  lib/lwip/src/core/ipv6/ip6_frag.c \
  lib/lwip/src/core/ipv6/mld6.c \
  lib/lwip/src/core/ipv6/nd6.c \
  lib/lwip/src/netif/ethernet.c \
  lib/lwip/src/netif/slipif.c \
  lib/lwip/src/apps/http/httpd.c \
  lib/lwip/src/apps/http/fs.c \
  lib/tinyusb/lib/networking/dhserver.c \
  lib/tinyusb/lib/networking/rndis_reports.c

ifeq ($(LWIP_STATS), 1)
  CFLAGS += -DLWIP_STATS_DISPLAY
endif

# Suppress warning caused by lwip
CFLAGS += \
  -Wno-error=null-dereference \
  -Wno-error=unused-parameter \
  -Wno-error=unused-variable

# TinyUSB Stack source
SRC_C += \
	lib/tinyusb/src/tusb.c \
	lib/tinyusb/src/common/tusb_fifo.c \
	lib/tinyusb/src/device/usbd.c \
	lib/tinyusb/src/device/usbd_control.c \
	lib/tinyusb/src/class/audio/audio_device.c \
	lib/tinyusb/src/class/cdc/cdc_device.c \
	lib/tinyusb/src/class/dfu/dfu_device.c \
	lib/tinyusb/src/class/dfu/dfu_rt_device.c \
	lib/tinyusb/src/class/hid/hid_device.c \
	lib/tinyusb/src/class/midi/midi_device.c \
	lib/tinyusb/src/class/msc/msc_device.c \
	lib/tinyusb/src/class/net/ecm_rndis_device.c \
	lib/tinyusb/src/class/net/ncm_device.c \
	lib/tinyusb/src/class/usbtmc/usbtmc_device.c \
	lib/tinyusb/src/class/video/video_device.c \
  lib/tinyusb/src/class/vendor/vendor_device.c

INC += lib/tinyusb/src

# ---------------------------------------
# Compiler Flags
# ---------------------------------------

# libc
LIBS += -lgcc -lm -lnosys -lc

CFLAGS += $(addprefix -I,$(INC))

# LTO makes it difficult to analyze map file for optimizing size purpose
# We will run this option in ci
ifeq ($(NO_LTO),1)
CFLAGS := $(filter-out -flto,$(CFLAGS))
endif

LDFLAGS += $(CFLAGS) -Wl,-T,$(TOP)/$(LD_FILE) -Wl,-Map=$@.map -Wl,-cref -Wl,-gc-sections

ASFLAGS += $(CFLAGS)

# Assembly files can be name with upper case .S, convert it to .s
SRC_S := $(SRC_S:.S=.s)

# Due to GCC LTO bug https://bugs.launchpad.net/gcc-arm-embedded/+bug/1747966
# assembly file should be placed first in linking order
# '_asm' suffix is added to object of assembly file
OBJ += $(addprefix $(BUILD)/obj/, $(SRC_S:.s=_asm.o))
OBJ += $(addprefix $(BUILD)/obj/, $(SRC_C:.c=.o))

# Verbose mode
ifeq ("$(V)","1")
$(info CFLAGS  $(CFLAGS) ) $(info )
$(info LDFLAGS $(LDFLAGS)) $(info )
$(info ASFLAGS $(ASFLAGS)) $(info )
endif

# ---------------------------------------
# Rules
# ---------------------------------------

# Set all as default goal
.DEFAULT_GOAL := all

all: clean web $(BUILD)/$(PROJECT).bin $(BUILD)/$(PROJECT).hex compile_commands size
	@echo Building $(PROJECT) in $(BUILD)

OBJ_DIRS = $(sort $(dir $(OBJ)))
$(OBJ): | $(OBJ_DIRS)
$(OBJ_DIRS):
	@$(MKDIR) -p $@

$(BUILD)/$(PROJECT).elf: $(OBJ)
	@echo LINK $@
	@$(CC) -o $@ $(LDFLAGS) $^ -Wl,--start-group $(LIBS) -Wl,--end-group

$(BUILD)/$(PROJECT).bin: $(BUILD)/$(PROJECT).elf
	@echo CREATE $@
	@$(OBJCOPY) -O binary $^ $@

$(BUILD)/$(PROJECT).hex: $(BUILD)/$(PROJECT).elf
	@echo CREATE $@
	@$(OBJCOPY) -O ihex $^ $@



# We set vpath to point to the top of the tree so that the source files
# can be located. By following this scheme, it allows a single build rule
# to be used to compile all .c files.
vpath %.c . $(TOP)
$(BUILD)/obj/%.o: %.c
	@echo CC $(notdir $@)
	@$(CC) $(CFLAGS) -c -MD -o $@ $<

# ASM sources lower case .s
vpath %.s . $(TOP)
$(BUILD)/obj/%_asm.o: %.s
	@echo AS $(notdir $@)
	@$(CC) -x assembler-with-cpp $(ASFLAGS) -c -o $@ $<

# ASM sources upper case .S
vpath %.S . $(TOP)
$(BUILD)/obj/%_asm.o: %.S
	@echo AS $(notdir $@)
	@$(CC) -x assembler-with-cpp $(ASFLAGS) -c -o $@ $<

size: $(BUILD)/$(PROJECT).elf
	-@echo ''
	@$(SIZE) $<
	-@echo ''

# linkermap must be install previously at https://github.com/hathach/linkermap
linkermap: $(BUILD)/$(PROJECT).elf
	@linkermap -v $<.map

.PHONY: clean web compile_commands
clean:
	$(RM) -rf $(BUILD)

web: 
	cd web && ./makefsdata && cp ./fsdata.c ../src/asset/fsdata_mb.c

compile_commands:
	@./generate_compile_commands.sh FREERTOS_STATS=$(FREERTOS_STATS) LWIP_STATS=$(LWIP_STATS)
# ---------------------------------------
# Flash Targets
# ---------------------------------------

# Flash STM32 MCU using stlink with STM32 Cube Programmer CLI
flash-stlink: $(BUILD)/$(PROJECT).elf
	STM32_Programmer_CLI --connect port=swd --write $< --go

# flash target ROM bootloader
flash-dfu: $(BUILD)/$(PROJECT).bin
	dfu-util -R -a 0 --dfuse-address 0x08000000 -D $<

# flash target using on-board stlink
flash: flash-stlink
