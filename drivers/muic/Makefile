# MUIC Class Driver
obj-$(CONFIG_MUIC)		+= muic_class.o muic_client_class.o
obj-$(CONFIG_MUIC_TS5USBA33402) += muic_ts5usba33402.o
ifeq ($(CONFIG_MACH_LGE_CX2), y)
obj-$(CONFIG_MUIC_TSU5611) += muic_tsu5611_cx2.o
else ifeq ($(CONFIG_MACH_LGE_P2), y)
obj-$(CONFIG_MUIC_TSU5611) += muic_tsu5611_p2.o
else
obj-$(CONFIG_MUIC_TSU5611) += muic_tsu5611.o
endif

obj-$(CONFIG_MUIC_MAX14526) += muic_max14526.o
#obj-$(CONFIG_MUIC_MAX14526) += cosmo_muic.o
#obj-$(CONFIG_MUIC_MAX14526) += cosmo_muic_ti.o
#obj-$(CONFIG_MUIC_MAX14526) += cosmo_fota.o

obj-$(CONFIG_DP3T)		+= dp3t.o
obj-$(CONFIG_USIF)		+= usif.o
obj-$(CONFIG_FOTA)		+= fota.o

ifeq ($(TARGET_PRODUCT), p2_lgu_kr)
    EXTRA_CFLAGS += -DTARGET_CARRIER_LGU
endif
