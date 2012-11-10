/* include/linux/lge/lge_input.h
 *
 * This header is for LGE specific input
 */

#ifndef __LGE_INPUT_H
#define __LGE_INPUT_H

#if defined(CONFIG_MACH_LGE)
extern int lge_input_set(void *dev);
extern struct input_dev *lge_input_get(void);

extern int lge_input_set_touch(void *dev);
extern struct input_dev *lge_input_get_touch(void);

#else
static inline int lge_input_set(void *dev) { return 0; }
static inline struct input_dev *lge_input_get(void) { return NULL; }

static inline int lge_input_set_touch(void *dev) { return 0; }
static inline struct input_dev *lge_input_get_touch(void) { return NULL; }
#endif

#if defined(CONFIG_MHL_INPUT_RCP)
extern void hdmi_common_register_keys(void *dev);
extern void hdmi_common_send_keyevent(unsigned char code);
extern void hdmi_common_send_uevent(int x, int y, int action, int tvCtl_x, int tvCtl_y);
#endif


/* gkpd */
struct key_table {
	char out;
	int in;
};

struct gkpd_platform_data {
	struct key_table *keys;
	unsigned int size;
};

#if defined(CONFIG_INPUT_LGE_GKPD)
extern int gkpd_get_test_mode(void);
extern void gkpd_write_value(int value);
extern void gkpd_report_key(int code, int value);
// LGE_CHANGE_S [younglae.kim@lge.com] 2012-06-28, EXPORT functions for AT%KEYLOCK
extern int get_key_lock_status(void);
// LGE_CHANGE_E [younglae.kim@lge.com] 2012-06-28
#else
static inline int gkpd_get_test_mode(void) { return 0; }
static inline void gkpd_write_value(int value) { }
static inline void gkpd_report_key(int code, int value) { }
#endif

#endif
