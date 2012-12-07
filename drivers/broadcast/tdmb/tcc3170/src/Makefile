#
# Makefile for the TDMB TCC3170 drivers.
#

# Each configuration option enables a list of files.
# LGE
obj-$(CONFIG_LGE_BROADCAST_TDMB_TCC3170)		+= broadcast_tcc3170_drv_if.o
obj-$(CONFIG_LGE_BROADCAST_TDMB_TCC3170)		+= broadcast_tcc3170.o
obj-$(CONFIG_LGE_BROADCAST_TDMB_TCC3170)		+= tdmb_tunerbbdrv_tcc3170.o

# Add to here for OEM files
obj-$(CONFIG_LGE_BROADCAST_TDMB_TCC3170)   += tcbd_hal.o
obj-$(CONFIG_LGE_BROADCAST_TDMB_TCC3170)   += tcbd_drv_io.o
obj-$(CONFIG_LGE_BROADCAST_TDMB_TCC3170)   += tcbd_drv_rf.o
obj-$(CONFIG_LGE_BROADCAST_TDMB_TCC3170)   += tcbd_drv_peri.o
obj-$(CONFIG_LGE_BROADCAST_TDMB_TCC3170)   += tcbd_drv_dual.o
obj-$(CONFIG_LGE_BROADCAST_TDMB_TCC3170)   += tcbd_api_common.o
obj-$(CONFIG_LGE_BROADCAST_TDMB_TCC3170)   += tcbd_drv_component.o

obj-$(CONFIG_LGE_BROADCAST_TDMB_TCC3170)   += tcbd_diagnosis/ 
obj-$(CONFIG_LGE_BROADCAST_TDMB_TCC3170)   += tcbd_stream_parser/
obj-$(CONFIG_LGE_BROADCAST_TDMB_TCC3170)   += tcbd_fic_decoder/
obj-$(CONFIG_LGE_BROADCAST_TDMB_TCC3170)   += tcbd_test/ 
obj-$(CONFIG_LGE_BROADCAST_TDMB_TCC3170)   += tcpal_linux/ 

EXTRA_CFLAGS += -Idrivers/broadcast/tdmb/tcc3170/inc
EXTRA_CFLAGS += -Idrivers/broadcast/tdmb/tcc3170/inc/tcbd_diagnosis
EXTRA_CFLAGS += -Idrivers/broadcast/tdmb/tcc3170/inc/tcbd_stream_parser
EXTRA_CFLAGS += -Idrivers/broadcast/tdmb/tcc3170/inc/tcbd_fic_decoder
EXTRA_CFLAGS += -Idrivers/broadcast/tdmb/tcc3170/inc/tcpal
