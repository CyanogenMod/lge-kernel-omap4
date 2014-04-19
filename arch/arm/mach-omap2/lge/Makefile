#
# Makefile for the LGE Common
#

obj-y += lge-common.o
obj-y += lge-common_musb.o
obj-y += lge_input.o

#[SJIT] mohamed.khadri@lge.com - Added wifi control device for BCM4330
obj-y += board-lg-wifi.o 

obj-$(CONFIG_LGE_HANDLE_PANIC) += lge_handle_panic.o
obj-$(CONFIG_CMDLINE_MANIPULATE) += lge_cmdline.o
obj-$(CONFIG_LGE_SUSPEND_AUTOTEST) += lge_suspend_autotest.o
obj-$(CONFIG_MACH_LGE) += lge_blocking_monitor.o

-include $(src)/Makefile.board
