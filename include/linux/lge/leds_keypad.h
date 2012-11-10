/* include/linux/lge/leds_keypad.h
 */

#ifndef __LEDS_KEYPAD_H
#define __LEDS_KEYPAD_H

struct leds_keypad_platform_data {
	const char *name;
	int use_hold_key;
	int keypad_gpio;
	int hold_key_gpio;
};

#endif
