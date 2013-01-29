obj-$(CONFIG_LGE_VIBRATOR)	+= vibrator.o
obj-$(CONFIG_SENSORS_APDS9900)	+= apds9900.o
obj-$(CONFIG_SENSORS_APDS9190)  += apds9190.o
ifeq ($(CONFIG_MACH_LGE_COSMO),y)
obj-$(CONFIG_BACKLIGHT_LM3528)	+= lm3528_bl_cosmo.o
else
obj-$(CONFIG_BACKLIGHT_LM3528)	+= lm3528_bl.o
endif
ifeq ($(CONFIG_MACH_LGE_P2),y)
obj-$(CONFIG_BACKLIGHT_LM3530)	+= lm3530_bl_p2.o
else
obj-$(CONFIG_BACKLIGHT_LM3530)	+= lm3530_bl.o
endif
obj-$(CONFIG_BACKLIGHT_LM3533)	+= lm3533_bl.o
obj-$(CONFIG_CAMERAFLASH_LM3559)	+= lm3559.o
obj-$(CONFIG_KEYPAD_LEDS_GPIO)	+= leds-keypad-gpio.o
obj-$(CONFIG_LGE_GPIO_CONTROL)	+= gpio_control.o
