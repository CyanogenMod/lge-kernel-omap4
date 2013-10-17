# bcmdhd
SRCROOT = drivers/net/wireless/bcmdhd/src
DHDCFLAGS = -Wall -Wstrict-prototypes -Werror -Dlinux -DBCMDRIVER \
	-DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 -DWLBTAMP -DBCMFILEIMAGE  \
	-DDHDTHREAD -DDHD_GPL -DDHD_SCHED -DBDC -DTOE    \
	-DDHD_BCMEVENTS -DSHOW_EVENTS -DDONGLEOVERLAYS -DBCMDBG               \
	-DMMC_SDIO_ABORT -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS -DWLP2P     \
	-DNEW_COMPAT_WIRELESS -DWIFI_ACT_FRAME -DARP_OFFLOAD_SUPPORT          \
	-DKEEP_ALIVE -DCSCAN -DPKT_FILTER_SUPPORT -DWL_ENABLE_P2P_IF    \
	-DEMBEDDED_PLATFORM -DENABLE_INSMOD_NO_FW_LOAD -DPNO_SUPPORT          \
	-I$(SRCROOT) \
	-I$(SRCROOT)/include \
	-I$(SRCROOT)/shared \
	-I$(SRCROOT)/dhd/sys \
	-I$(SRCROOT)/dongle \
	-I$(SRCROOT)/wl/sys \
	-I$(SRCROOT)/wl/bcmwifi/src \
	-I$(SRCROOT)/wl/bcmwifi/include

# common patch  - should be defiend!!!!
DHDCFLAGS += -DCONFIG_COMMON_PATCH

# debug
#DHDCFLAGS += -DDHD_DEBUG -DSDTEST

# for P2P
DHDCFLAGS += -DP2P_PATCH

ifeq ($(CONFIG_WIFI_CONTROL_FUNC),y)
DHDCFLAGS += -DCONFIG_WIFI_CONTROL_FUNC -DCUSTOMER_HW2
endif

#LGE_CHANGE_S, real-wifi@lge.com by kyunghoe.kim, 20120626, for ccx
# For CCX
ifeq ($(CONFIG_BRCM_CCX),y)
DHDCFLAGS += -DBCMCCX
endif
#LGE_CHANGE_E, real-wifi@lge.com by kyunghoe.kim, 20120626, for ccx

#DHDCFLAGS += -DSUPPORT_PM2_ONLY

ifeq ($(CONFIG_BCMDHD_HW_OOB),y)
DHDCFLAGS += -DHW_OOB -DOOB_INTR_ONLY
#LGE_CHANGE_S, moon-wifi@lge.com by wo0ngs 2012-03-18
ifneq ($(CONFIG_ARCH_OMAP),y)
DHDCFLAGS += -DCUSTOM_OOB_GPIO_NUM=$(CONFIG_BCMDHD_GPIO_WL_HOSTWAKEUP)
endif
#LGE_CHANGE_E, moon-wifi@lge.com by wo0ngs 2012-03-18
else
DHDCFLAGS += -DSDIO_ISR_THREAD
endif

# For Static Buffer
ifeq ($(CONFIG_BCMDHD_USE_STATIC_BUF),y)
DHDCFLAGS += -DCONFIG_DHD_USE_STATIC_BUF
endif

#LGE_CHANGE_S, moon-wifi@lge.com by wo0ngs 2012-05-01, CONFIG_VOIP_KR : Disable
# For DTIM Disable/Enable
ifeq ($(TARGET_COUNTRY), KR)
DHDCFLAGS += -DCONFIG_VOIP_KR  
endif
#LGE_CHANGE_E, moon-wifi@lge.com by wo0ngs 2012-05-01

DHDOFILES = \
		src/bcmsdio/sys/bcmsdh.o \
		src/bcmsdio/sys/bcmsdh_linux.o \
		src/bcmsdio/sys/bcmsdh_sdmmc.o \
		src/bcmsdio/sys/bcmsdh_sdmmc_linux.o \
		src/dhd/sys/dhd_bta.o \
		src/dhd/sys/dhd_cdc.o \
		src/dhd/sys/dhd_common.o \
		src/dhd/sys/dhd_custom_gpio.o \
		src/dhd/sys/dhd_linux.o \
		src/dhd/sys/dhd_linux_sched.o \
		src/dhd/sys/dhd_sdio.o \
		src/shared/aiutils.o \
		src/shared/bcmutils.o   \
		src/shared/bcmevent.o \
		src/shared/hndpmu.o  \
		src/shared/linux_osl.o	\
		src/shared/sbutils.o \
		src/shared/siutils.o \
		src/wl/sys/wl_android.o	\
		src/wl/sys/wldev_common.o \
		src/wl/sys/wl_linux_mon.o \
		src/wl/bcmwifi/src/bcmwifi_channels.o 

obj-$(CONFIG_BCMDHD_USE_STATIC_BUF) += brcm_static_buf.o

obj-$(CONFIG_BCMDHD) += bcmdhd.o
bcmdhd-objs += $(DHDOFILES)
ifneq ($(CONFIG_WIRELESS_EXT),)
bcmdhd-objs += src/wl/sys/wl_iw.o
DHDCFLAGS += -DSOFTAP
endif
ifneq ($(CONFIG_CFG80211),)
bcmdhd-objs += src/wl/sys/wl_cfg80211.o \
		src/wl/sys/wl_cfgp2p.o \
		src/dhd/sys/dhd_cfg80211.o
DHDCFLAGS += -DWL_CFG80211
endif
EXTRA_CFLAGS = $(DHDCFLAGS)
ifeq ($(CONFIG_BCMDHD),m)
EXTRA_LDFLAGS += --strip-debug
endif
