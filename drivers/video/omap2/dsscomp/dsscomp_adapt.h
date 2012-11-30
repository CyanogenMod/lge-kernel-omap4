#ifndef _DSSCOMP_ADAPT_H
#define _DSSCOMP_ADAPT_H


struct dsscomp_adapt_info;

int dsscomp_adapt_init(struct dsscomp_dev *cdev_);
void dsscomp_adapt_deinit(void);

struct dsscomp_adapt_info* dsscomp_adapt_need(struct dsscomp_adapt_info* adapt_ino,
		struct dss2_ovl_info *oi, struct dss2_mgr_info *mgr,
		struct dss2_ovl_info *oi_ret);
void dsscomp_adapt_start_process(struct dsscomp_adapt_info* adapt_ino);
int dsscomp_adapt_wait_complete(struct dsscomp_adapt_info *adapt_info, unsigned long msec, int *ret);
void dsscomp_adapt_free(struct dsscomp_adapt_info *adapt_info);
void dsscomp_adapt_force_stop(struct dsscomp_adapt_info *adapt_info);

//@todo append debug fs
void dsscomp_adapt_dbg_buffer(struct seq_file *s);
void dsscomp_adapt_dbg_action_history(struct seq_file *s);

#endif

