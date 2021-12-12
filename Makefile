DEPS_SUBMODULES += lib/FreeRTOS-Kernel lib/lwip lib/tinyusb

TOP := $(shell realpath .)
# $(info Top directory is $(TOP))

CURRENT_PATH := $(shell realpath --relative-to=$(TOP) `pwd`)
# $(info Path from top is $(CURRENT_PATH))

include ./make.mk

# Application source
INC += \
	src \
	src/FreeRTOSConfig

SRC_C += $(addprefix $(CURRENT_PATH)/, $(wildcard src/*.c))

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
  
# Suppress FreeRTOS warnings
CFLAGS += -Wno-error=cast-qual

# FreeRTOS (lto + Os) linker issue
LDFLAGS += -Wl,--undefined=vTaskSwitchContext

# suppress warning caused by lwip
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

include ./rules.mk
