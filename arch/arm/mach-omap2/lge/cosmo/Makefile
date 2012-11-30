#
# Makefile for the LGE p2 board
#

obj-y += cosmo.o \
		 cosmo_i2c.o \
		 cosmo_mux.o \
		 cosmo_emif.o \
		 cosmo_pdev.o \
		 cosmo_mmc.o \
		 cosmo_spi.o \
		 cosmo_power.o \
		 cosmo_display.o \
		 cosmo_keypad.o \
		 cosmo_cmdline.o \
		 cosmo_misc.o

# ericjs.kim@lge.com GPS added
obj-$(CONFIG_GPS) += ../../gps_gpio.o

