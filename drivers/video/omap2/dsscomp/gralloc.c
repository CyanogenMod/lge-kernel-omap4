#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <mach/tiler.h>
#include <video/dsscomp.h>
#include <plat/dsscomp.h>
#include "dsscomp.h"

#if defined(CONFIG_MACH_LGE_COSMO_3D_DISPLAY) || defined(CONFIG_MACH_LGE_CX2_3D_DISPLAY)//##hwcho_20120522
#ifdef CONFIG_DSSCOMP_ADAPT
#include "dsscomp_adapt.h"
#endif
#endif //##
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
static bool blanked;

#ifdef CONFIG_DSSCOMP_COPY_FOR_ROT

#include <linux/ion.h>
#include <linux/omap_ion.h>
#include <plat/dma.h>

extern struct ion_device *omap_ion_device;

struct dsscomp_gralloc_t;

/**
 * Purpose :
 *     When address is not tiler and rotation is requested,
 *     allocates tiler buffer and copy contents to it.
 */
struct tiler_2d_buffer {
	int             width, height;
	bool    allocated;      //allocation flag
	bool    in_using;       //whether dss is using this buffer

	int             bpp;                    //byte per pixel

	u32             mgr_mask;                       //connected DSS manager 
	struct dsscomp_gralloc_t        *gsync; //gsync object

	u32             src_phy_addr;   //source physical address
	size_t  src_stride;             //source stride

	struct omap_ion_tiler_alloc_data alloc_data;
	ion_phys_addr_t phy;    //physical address of ION alloc
	size_t  len;
	struct tiler_view_t view;

	wait_queue_head_t dma_wait_q;   //DMA wait queue
	bool    dma_copying;                    //flag for dma copy progress
	int             dma_ch;                                 //DMA channel


	struct timespec last_use;
};

#define MAX_BUF_NUM_FOR_ROT    6
struct tiler_2d_buffer_mgr {
	struct mutex lock;
	struct ion_client *ion;
	struct tiler_2d_buffer buffers[MAX_BUF_NUM_FOR_ROT];
};

static struct tiler_2d_buffer_mgr rot_buf_mgr;

static void dsscomp_rotbuf_mgr_check(struct dss2_ovl_info *oi, struct dsscomp_gralloc_t *gsync, u32 ch);
static void dsscomp_rotbuf_mgr_put_buf(struct tiler_2d_buffer *buf);
static int dsscomp_rotbuf_mgr_start_dma_copy_of_gralloc(struct dsscomp_gralloc_t *gsync, u32 ch);
static int dsscomp_rotbuf_mgr_wait_dma_copy_of_gralloc(struct dsscomp_gralloc_t *gsymc, u32 ch);
#endif

#define NUM_TILER1D_SLOTS 2

static struct tiler1d_slot {
	struct list_head q;
	tiler_blk_handle slot;
	u32 phys;
	u32 size;
	u32 *page_map;
} slots[NUM_TILER1D_SLOTS];
static struct list_head free_slots;
static struct dsscomp_dev *cdev;
static DEFINE_MUTEX(mtx);
static struct semaphore free_slots_sem =
				__SEMAPHORE_INITIALIZER(free_slots_sem, 0);

/* gralloc composition sync object */
struct dsscomp_gralloc_t {
	void (*cb_fn)(void *, int);
	void *cb_arg;
	struct list_head q;
	struct list_head slots;
	atomic_t refs;
	bool early_callback;
	bool programmed;
#ifdef CONFIG_DSSCOMP_COPY_FOR_ROT
	struct tiler_2d_buffer *rot_bufs[MAX_BUF_NUM_FOR_ROT];
#endif
#if defined(CONFIG_MACH_LGE_COSMO_3D_DISPLAY) || defined(CONFIG_MACH_LGE_CX2_3D_DISPLAY)//##hwcho_20120522
#ifdef CONFIG_DSSCOMP_ADAPT
	struct dsscomp_adapt_info *adapt_info;
#endif
#endif //##
};

/* queued gralloc compositions */
static LIST_HEAD(flip_queue);

static u32 ovl_use_mask[MAX_MANAGERS];

static unsigned int tiler1d_slot_size(struct dsscomp_dev *cdev)
{
	struct dsscomp_platform_data *pdata;
	pdata = (struct dsscomp_platform_data *)cdev->pdev->platform_data;
	return pdata->tiler1d_slotsz;
}

static void unpin_tiler_blocks(struct list_head *slots)
{
	struct tiler1d_slot *slot;

	/* unpin any tiler memory */
	list_for_each_entry(slot, slots, q) {
		tiler_unpin_block(slot->slot);
		up(&free_slots_sem);
	}

	/* free tiler slots */
	list_splice_init(slots, &free_slots);
}

static void dsscomp_gralloc_cb(void *data, int status)
{
	struct dsscomp_gralloc_t *gsync = data, *gsync_;
	bool early_cbs = true;
	LIST_HEAD(done);

	mutex_lock(&mtx);
	if (gsync->early_callback && status == DSS_COMPLETION_PROGRAMMED)
		gsync->programmed = true;

	if (status & DSS_COMPLETION_RELEASED) {
		if (atomic_dec_and_test(&gsync->refs))
#ifdef CONFIG_DSSCOMP_COPY_FOR_ROT
		{
			int i;
			for(i=0;i<MAX_BUF_NUM_FOR_ROT;i++)
				if ( gsync->rot_bufs[i]!=NULL )
				{
					dsscomp_rotbuf_mgr_put_buf(gsync->rot_bufs[i]);
					gsync->rot_bufs[i] = NULL;
				}
#elif defined(CONFIG_MACH_LGE_COSMO_3D_DISPLAY) || defined(CONFIG_MACH_LGE_CX2_3D_DISPLAY)//##hwcho_20120522
#ifdef CONFIG_DSSCOMP_ADAPT
		{
			if ( gsync->adapt_info!=NULL )
			{
				dsscomp_adapt_free(gsync->adapt_info);
				gsync->adapt_info = NULL;
			}
#endif
#endif //##
			unpin_tiler_blocks(&gsync->slots);
#if defined(CONFIG_DSSCOMP_ADAPT)  || defined(CONFIG_DSSCOMP_COPY_FOR_ROT)
		}
#endif
		log_event(0, 0, gsync, "--refs=%d on %s",
				atomic_read(&gsync->refs),
				(u32) log_status_str(status));
	}

	/* get completed list items in order, if any */
	list_for_each_entry_safe(gsync, gsync_, &flip_queue, q) {
		if (gsync->cb_fn) {
			early_cbs &= gsync->early_callback && gsync->programmed;
			if (early_cbs) {
				gsync->cb_fn(gsync->cb_arg, 1);
				gsync->cb_fn = NULL;
			}
		}
		if (gsync->refs.counter && gsync->cb_fn)
			break;
		if (gsync->refs.counter == 0)
			list_move_tail(&gsync->q, &done);
	}
	mutex_unlock(&mtx);

	/* call back for completed composition with mutex unlocked */
	list_for_each_entry_safe(gsync, gsync_, &done, q) {
		if (debug & DEBUG_GRALLOC_PHASES)
			dev_info(DEV(cdev), "[%p] completed flip\n", gsync);

		log_event(0, 0, gsync, "calling %pf [%p]",
				(u32) gsync->cb_fn, (u32) gsync->cb_arg);

		if (gsync->cb_fn)
			gsync->cb_fn(gsync->cb_arg, 1);
		kfree(gsync);
	}
}

/* This is just test code for now that does the setup + apply.
   It still uses userspace virtual addresses, but maps non
   TILER buffers into 1D */
int dsscomp_gralloc_queue_ioctl(struct dsscomp_setup_dispc_data *d)
{
	struct tiler_pa_info *pas[MAX_OVERLAYS];
	s32 ret;
	u32 i;

	/* convert virtual addresses to physical and get tiler pa infos */
	for (i = 0; i < d->num_ovls; i++) {
		struct dss2_ovl_info *oi = d->ovls + i;
		u32 addr = (u32) oi->address;

		pas[i] = NULL;

		/* assume virtual NV12 for now */
		if (oi->cfg.color_mode == OMAP_DSS_COLOR_NV12)
			oi->uv = tiler_virt2phys(addr +
					oi->cfg.height * oi->cfg.stride);
		else
			oi->uv = 0;
		oi->ba = tiler_virt2phys(addr);

		/* map non-TILER buffers to 1D */
		if ((oi->ba < 0x60000000 || oi->ba >= 0x80000000) && oi->ba)
			pas[i] = user_block_to_pa(addr & PAGE_MASK,
				PAGE_ALIGN(oi->cfg.height * oi->cfg.stride +
					(addr & ~PAGE_MASK)) >> PAGE_SHIFT);
	}
	ret = dsscomp_gralloc_queue(d, pas, false, NULL, NULL);
	for (i = 0; i < d->num_ovls; i++)
		tiler_pa_free(pas[i]);
	return ret;
}

int dsscomp_gralloc_queue(struct dsscomp_setup_dispc_data *d,
			struct tiler_pa_info **pas,
			bool early_callback,
			void (*cb_fn)(void *, int), void *cb_arg)
{
	u32 i;
	int r = 0;
	struct omap_dss_device *dev;
	struct omap_overlay_manager *mgr;
	static DEFINE_MUTEX(local_mtx);
	dsscomp_t comp[MAX_MANAGERS];
	u32 ovl_new_use_mask[MAX_MANAGERS];
	u32 mgr_set_mask = 0;
	u32 ovl_set_mask = 0;
	struct tiler1d_slot *slot = NULL;
	u32 slot_used = 0;
#ifdef CONFIG_DEBUG_FS
	u32 ms = ktime_to_ms(ktime_get());
#endif
	u32 channels[ARRAY_SIZE(d->mgrs)], ch;
	int skip;
	struct dsscomp_gralloc_t *gsync;
	struct dss2_rect_t win = { .w = 0 };
#if defined(CONFIG_MACH_LGE_COSMO_3D_DISPLAY) || defined(CONFIG_MACH_LGE_CX2_3D_DISPLAY)//##hwcho_20120522
#ifdef CONFIG_DSSCOMP_ADAPT
	struct dsscomp_adapt_info* adapt_info = NULL;
#endif
#endif //##
	/* reserve tiler areas if not already done so */
	dsscomp_gralloc_init(cdev);

	dump_total_comp_info(cdev, d, "queue");
	for (i = 0; i < d->num_ovls; i++)
		dump_ovl_info(cdev, d->ovls + i);

	mutex_lock(&local_mtx);

	mutex_lock(&mtx);

	/* create sync object with 1 temporary ref */
	gsync = kzalloc(sizeof(*gsync), GFP_KERNEL);
	gsync->cb_arg = cb_arg;
	gsync->cb_fn = cb_fn;
	gsync->refs.counter = 1;
	gsync->early_callback = early_callback;
	INIT_LIST_HEAD(&gsync->slots);
	list_add_tail(&gsync->q, &flip_queue);
	if (debug & DEBUG_GRALLOC_PHASES)
		dev_info(DEV(cdev), "[%p] queuing flip\n", gsync);

	log_event(0, ms, gsync, "new in %pf (refs=1)",
			(u32) dsscomp_gralloc_queue, 0);

	/* ignore frames while we are blanked */
	skip = blanked;
	if (skip && (debug & DEBUG_PHASES))
		dev_info(DEV(cdev), "[%p,%08x] ignored\n", gsync, d->sync_id);

	/* mark blank frame by NULL tiler pa pointer */
	if (!skip && pas == NULL)
		blanked = true;

	mutex_unlock(&mtx);

	d->num_mgrs = min(d->num_mgrs, (u16) ARRAY_SIZE(d->mgrs));
	d->num_ovls = min(d->num_ovls, (u16) ARRAY_SIZE(d->ovls));

	memset(comp, 0, sizeof(comp));
	memset(ovl_new_use_mask, 0, sizeof(ovl_new_use_mask));

	if (skip)
		goto skip_comp;

	d->mode = DSSCOMP_SETUP_DISPLAY;

	/* mark managers we are using */
	for (i = 0; i < d->num_mgrs; i++) {
		/* verify display is valid & connected, ignore if not */
		if (d->mgrs[i].ix >= cdev->num_displays)
			continue;
		dev = cdev->displays[d->mgrs[i].ix];
		if (!dev) {
			dev_warn(DEV(cdev), "failed to get display%d\n",
								d->mgrs[i].ix);
			continue;
		}
		mgr = dev->manager;
		if (!mgr) {
			dev_warn(DEV(cdev), "no manager for display%d\n",
								d->mgrs[i].ix);
			continue;
		}
		channels[i] = ch = mgr->id;
		mgr_set_mask |= 1 << ch;

		/* swap red & blue if requested */
		if (d->mgrs[i].swap_rb)
			swap_rb_in_mgr_info(d->mgrs + i);
	}

	/* create dsscomp objects for set managers (including active ones) */
	for (ch = 0; ch < MAX_MANAGERS; ch++) {
		if (!(mgr_set_mask & (1 << ch)) && !ovl_use_mask[ch])
			continue;

		mgr = cdev->mgrs[ch];

		comp[ch] = dsscomp_new(mgr);
		if (IS_ERR(comp[ch])) {
			comp[ch] = NULL;
			dev_warn(DEV(cdev), "failed to get composition on %s\n",
								mgr->name);
			continue;
		}

		/* set basic manager information for blanked managers */
		if (!(mgr_set_mask & (1 << ch))) {
			struct dss2_mgr_info mi = {
				.alpha_blending = true,
				.ix = comp[ch]->frm.mgr.ix,
			};
			dsscomp_set_mgr(comp[ch], &mi);
		}

		comp[ch]->must_apply = true;
		r = dsscomp_setup(comp[ch], d->mode, win);
		if (r)
			dev_err(DEV(cdev), "failed to setup comp (%d)\n", r);
	}

	/* configure manager data from gralloc composition */
	for (i = 0; i < d->num_mgrs; i++) {
		ch = channels[i];
		r = dsscomp_set_mgr(comp[ch], d->mgrs + i);
		if (r)
			dev_err(DEV(cdev), "failed to set mgr%d (%d)\n", ch, r);
	}

	/* NOTE: none of the dsscomp sets should fail as composition is new */
	for (i = 0; i < d->num_ovls; i++) {
		struct dss2_ovl_info *oi = d->ovls + i;
		u32 mgr_ix = oi->cfg.mgr_ix;
		u32 size;

		/* verify manager index */
		if (mgr_ix >= d->num_mgrs) {
			dev_err(DEV(cdev), "invalid manager for ovl%d\n",
								oi->cfg.ix);
			continue;
		}
		ch = channels[mgr_ix];

		/* skip overlays on compositions we could not create */
		if (!comp[ch])
			continue;

		/* swap red & blue if requested */
		if (d->mgrs[mgr_ix].swap_rb)
			swap_rb_in_ovl_info(d->ovls + i);

		/* copy prior overlay to avoid mapping layers twice to 1D */
		if (oi->addressing == OMAP_DSS_BUFADDR_OVL_IX) {
			unsigned int j = oi->ba;
			if (j >= i) {
				WARN(1, "Invalid clone layer (%u)", j);
				goto skip_buffer;
			}

			oi->ba = d->ovls[j].ba;
			oi->uv = d->ovls[j].uv;
#ifdef CONFIG_DSSCOMP_COPY_FOR_ROT
			dsscomp_rotbuf_mgr_check(oi, gsync, ch);
#endif
			goto skip_map1d;
		} else if (oi->addressing == OMAP_DSS_BUFADDR_FB) {
			/* get fb */
			int fb_ix = (oi->ba >> 28);
			int fb_uv_ix = (oi->uv >> 28);
			struct fb_info *fbi = NULL, *fbi_uv = NULL;
			size_t size = oi->cfg.height * oi->cfg.stride;
			if (fb_ix >= num_registered_fb ||
			    (oi->cfg.color_mode == OMAP_DSS_COLOR_NV12 &&
			     fb_uv_ix >= num_registered_fb)) {
				WARN(1, "display has no framebuffer");
				goto skip_buffer;
			}

			fbi = fbi_uv = registered_fb[fb_ix];
			if (oi->cfg.color_mode == OMAP_DSS_COLOR_NV12)
				fbi_uv = registered_fb[fb_uv_ix];

			if (size + oi->ba > fbi->fix.smem_len ||
			    (oi->cfg.color_mode == OMAP_DSS_COLOR_NV12 &&
			     (size >> 1) + oi->uv > fbi_uv->fix.smem_len)) {
				WARN(1, "image outside of framebuffer memory");
				goto skip_buffer;
			}

			oi->ba += fbi->fix.smem_start;
			oi->uv += fbi_uv->fix.smem_start;
			goto skip_map1d;
		}

		/* map non-TILER buffers to 1D */

		/* skip 2D and disabled layers */
		if (!pas[i] || !oi->cfg.enabled)
			goto skip_map1d;

		if (!slot) {
			if (down_timeout(&free_slots_sem,
						msecs_to_jiffies(100))) {
				dev_warn(DEV(cdev), "could not obtain tiler slot");
				goto skip_buffer;
			}
			mutex_lock(&mtx);
			slot = list_first_entry(&free_slots, typeof(*slot), q);
			list_move(&slot->q, &gsync->slots);
			mutex_unlock(&mtx);
		}

		size = oi->cfg.stride * oi->cfg.height;
		if (oi->cfg.color_mode == OMAP_DSS_COLOR_NV12)
			size += size >> 2;
		size = DIV_ROUND_UP(size, PAGE_SIZE);

		if (slot_used + size > slot->size) {
			dev_err(DEV(cdev), "tiler slot not big enough for frame %d + %d > %d",
				slot_used, size, slot->size);
			goto skip_buffer;
		}

		/* "map" into TILER 1D - will happen after loop */
		oi->ba = slot->phys + (slot_used << PAGE_SHIFT) +
			(oi->ba & ~PAGE_MASK);
		memcpy(slot->page_map + slot_used, pas[i]->mem,
		       sizeof(*slot->page_map) * size);
		slot_used += size;
		goto skip_map1d;

skip_buffer:
		oi->cfg.enabled = false;
skip_map1d:

		if (oi->cfg.enabled)
			ovl_new_use_mask[ch] |= 1 << oi->cfg.ix;
#if defined(CONFIG_DSSCOMP_ADAPT) && (defined(CONFIG_MACH_LGE_COSMO_3D_DISPLAY) || defined(CONFIG_MACH_LGE_CX2_3D_DISPLAY)) //##hwcho_20120522
		{
			struct dss2_ovl_info oi_adapt;
			struct dsscomp_adapt_info* adapt_info_oi;
			adapt_info_oi = dsscomp_adapt_need(adapt_info, oi, d->mgrs+mgr_ix, &oi_adapt);
			if ( adapt_info_oi!=NULL )
			{
//				printk("Adapted ovl info set\n");
				r = dsscomp_set_ovl(comp[ch], &oi_adapt);
				adapt_info = adapt_info_oi;
			}
			else
				r = dsscomp_set_ovl(comp[ch], oi);
		}
#else
		r = dsscomp_set_ovl(comp[ch], oi);
#endif
		if (r)
			dev_err(DEV(cdev), "failed to set ovl%d (%d)\n",
								oi->cfg.ix, r);
		else
			ovl_set_mask |= 1 << oi->cfg.ix;
	}
#if defined(CONFIG_MACH_LGE_COSMO_3D_DISPLAY) || defined(CONFIG_MACH_LGE_CX2_3D_DISPLAY)//##hwcho_20120522
#ifdef CONFIG_DSSCOMP_ADAPT
	if ( adapt_info!=NULL)
	{
		gsync->adapt_info = adapt_info;
		dsscomp_adapt_start_process(adapt_info);
	}
#endif
#endif //##

	if (slot && slot_used) {
		r = tiler_pin_block(slot->slot, slot->page_map,
						slot_used);
		if (r)
			dev_err(DEV(cdev), "failed to pin %d pages into"
				" %d-pg slots (%d)\n", slot_used,
				tiler1d_slot_size(cdev) >> PAGE_SHIFT, r);
	}

#if defined(CONFIG_MACH_LGE_COSMO_3D_DISPLAY) || defined(CONFIG_MACH_LGE_CX2_3D_DISPLAY)//##hwcho_20120522
#ifdef CONFIG_DSSCOMP_ADAPT
	//@todo move another place
	if ( gsync->adapt_info!=NULL )
	{
		int ret = 0;
		r= dsscomp_adapt_wait_complete(gsync->adapt_info, 100, &ret);
		if ( r )
		{
			pr_warning("Waiting dsscomp_adaptation time outed. forcing stop\n");
			dsscomp_adapt_force_stop(gsync->adapt_info);
			pr_warning("Forcing stop completed\n");
			goto skip_comp;
		}
		if ( ret )
		{
			pr_warning("There was error during dsscomp adapt processing\n");
		}
	}
#endif
#endif //##
	for (ch = 0; ch < MAX_MANAGERS; ch++) {
		/* disable all overlays not specifically set from prior frame */
		u32 mask = ovl_use_mask[ch] & ~ovl_set_mask;

		if (!comp[ch])
			continue;

		while (mask) {
			struct dss2_ovl_info oi = {
				.cfg.zonly = true,
				.cfg.enabled = false,
				.cfg.ix = fls(mask) - 1,
			};
			dsscomp_set_ovl(comp[ch], &oi);
			mask &= ~(1 << oi.cfg.ix);
		}

		/* associate dsscomp objects with this gralloc composition */
		comp[ch]->extra_cb = dsscomp_gralloc_cb;
		comp[ch]->extra_cb_data = gsync;
		atomic_inc(&gsync->refs);
		log_event(0, ms, gsync, "++refs=%d for [%p]",
				atomic_read(&gsync->refs), (u32) comp[ch]);
#ifdef CONFIG_DSSCOMP_COPY_FOR_ROT
		dsscomp_rotbuf_mgr_start_dma_copy_of_gralloc(gsync, ch);
		dsscomp_rotbuf_mgr_wait_dma_copy_of_gralloc(gsync, ch);
#endif
		r = dsscomp_delayed_apply(comp[ch]);
		if (r)
			dev_err(DEV(cdev), "failed to apply comp (%d)\n", r);
		else
			ovl_use_mask[ch] = ovl_new_use_mask[ch];
	}
skip_comp:
	/* release sync object ref - this completes unapplied compositions */
	dsscomp_gralloc_cb(gsync, DSS_COMPLETION_RELEASED);

	mutex_unlock(&local_mtx);

	return r;
}
EXPORT_SYMBOL(dsscomp_gralloc_queue);


#ifdef CONFIG_EARLYSUSPEND
static int blank_complete;
static DECLARE_WAIT_QUEUE_HEAD(early_suspend_wq);

static void dsscomp_early_suspend_cb(void *data, int status)
{
	blank_complete = true;
	wake_up(&early_suspend_wq);
}

static void dsscomp_early_suspend(struct early_suspend *h)
{
	struct dsscomp_setup_dispc_data d = {
		.num_mgrs = 0,
	};
	int err;

	pr_info("DSSCOMP: %s\n", __func__);

	/* use gralloc queue as we need to blank all screens */
	blank_complete = false;
	dsscomp_gralloc_queue(&d, NULL, false, dsscomp_early_suspend_cb, NULL);

	/* wait until composition is displayed */
	err = wait_event_timeout(early_suspend_wq, blank_complete,
				 msecs_to_jiffies(500));
	if (err == 0)
		pr_warn("DSSCOMP: timeout blanking screen\n");
	else
		pr_info("DSSCOMP: blanked screen\n");
}

static void dsscomp_late_resume(struct early_suspend *h)
{
	pr_info("DSSCOMP: %s\n", __func__);
	blanked = false;
}

static struct early_suspend early_suspend_info = {
	.suspend = dsscomp_early_suspend,
	.resume = dsscomp_late_resume,
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
};
#endif

void dsscomp_dbg_gralloc(struct seq_file *s)
{
#ifdef CONFIG_DEBUG_FS
	struct dsscomp_gralloc_t *g;
	struct tiler1d_slot *t;
	dsscomp_t c;
	int i;

	mutex_lock(&dbg_mtx);
	seq_printf(s, "ACTIVE GRALLOC FLIPS\n\n");
	list_for_each_entry(g, &flip_queue, q) {
		char *sep = "";
		seq_printf(s, "  [%p] (refs=%d)\n"
			   "    slots=[", g, atomic_read(&g->refs));
		list_for_each_entry(t, &g->slots, q) {
			seq_printf(s, "%s%08x", sep, t->phys);
			sep = ", ";
		}
		seq_printf(s, "]\n    cmdcb=[%08x] ", (u32) g->cb_arg);
		if (g->cb_fn)
			seq_printf(s, "%pf\n\n  ", g->cb_fn);
		else
			seq_printf(s, "(called)\n\n  ");

		list_for_each_entry(c, &dbg_comps, dbg_q) {
			if (c->extra_cb && c->extra_cb_data == g)
				seq_printf(s, "|      %8s      ",
					cdev->mgrs[c->ix]->name);
		}
		seq_printf(s, "\n  ");
		list_for_each_entry(c, &dbg_comps, dbg_q) {
			if (c->extra_cb && c->extra_cb_data == g)
				seq_printf(s, "| [%08x] %7s ", (u32) c,
					   log_state_str(c->state));
		}
#ifdef CONFIG_DSSCOMP_DEBUG_LOG
		for (i = 0; i < ARRAY_SIZE(c->dbg_log); i++) {
			int go = false;
			seq_printf(s, "\n  ");
			list_for_each_entry(c, &dbg_comps, dbg_q) {
				if (!c->extra_cb || c->extra_cb_data != g)
					continue;
				if (i < c->dbg_used) {
					u32 t = c->dbg_log[i].t;
					u32 state = c->dbg_log[i].state;
					seq_printf(s, "| % 6d.%03d %7s ",
						t / 1000, t % 1000,
						log_state_str(state));
					go |= c->dbg_used > i + 1;
				} else {
					seq_printf(s, "%-21s", "|");
				}
			}
			if (!go)
				break;
		}
#endif
		seq_printf(s, "\n\n");
	}
	seq_printf(s, "\n");
	mutex_unlock(&dbg_mtx);
#endif
}

#ifdef CONFIG_DSSCOMP_COPY_FOR_ROT

/**
 * free buffer
 * this function must be called with lock
 */
static bool dsscomp_rotbuf_mgr_free_buf_withlock(int i)
{
	struct tiler_2d_buffer *buf;
	if ( i<0 || i>=MAX_BUF_NUM_FOR_ROT )
		return false;
	buf = & (rot_buf_mgr.buffers[i]);
	if ( buf->in_using )
		return false;
	if ( buf->allocated )
	{
		ion_free(rot_buf_mgr.ion, buf->alloc_data.handle);
		buf->allocated = false;
	}
	return true;
}

/**
 * find empty slot or victim
 * this function must be called with lock
 */
static struct tiler_2d_buffer* dsscomp_rotbuf_mgr_find_avail_buf_withlock(void)
{
	int i;


	//find not allocated buffer
	for(i=0;i<MAX_BUF_NUM_FOR_ROT;i++)
	{
		if ( !rot_buf_mgr.buffers[i].allocated )
			return &(rot_buf_mgr.buffers[i]);
	}
	//find not using buffer
	for(i=0;i<MAX_BUF_NUM_FOR_ROT;i++)
	{
		if ( !rot_buf_mgr.buffers[i].in_using )
		{
			dsscomp_rotbuf_mgr_free_buf_withlock(i);
			return &(rot_buf_mgr.buffers[i]);
		}
	}
	return NULL;
}

/**
 * put buffer when no more use
 */
static void dsscomp_rotbuf_mgr_put_buf(struct tiler_2d_buffer *buf)
{
	mutex_lock(&rot_buf_mgr.lock);
	buf->in_using = false;
	mutex_unlock(&rot_buf_mgr.lock);
}

/**
 * get 2D tiler buffer
 */
static struct tiler_2d_buffer* dsscomp_rotbuf_mgr_get_buf(struct dss2_ovl_info *oi)
{
	int i, bpp;
	int fmt;

	struct tiler_2d_buffer* buf = NULL;
	if ( oi==NULL )
		return NULL;
	switch ( oi->cfg.color_mode )
	{
		case OMAP_DSS_COLOR_CLUT8:              /* BITMAP 8 */
			bpp = 1;
			fmt = TILER_PIXEL_FMT_8BIT;
			break;
		case OMAP_DSS_COLOR_ARGB16:             /* ARGB16-4444 */
		case OMAP_DSS_COLOR_RGB16:              /* RGB16-565 */
			bpp = 2;
			fmt = TILER_PIXEL_FMT_16BIT;
			break;
		case OMAP_DSS_COLOR_ARGB32:             /* ARGB32-8888 */
		case OMAP_DSS_COLOR_RGBA32:             /* RGBA32-8888 */
		case OMAP_DSS_COLOR_RGB24U:             /* xRGB24-8888 */
		case OMAP_DSS_COLOR_RGB24P:             /* RGB24-888 */

		case OMAP_DSS_COLOR_RGBX32:             /* RGBx32-8888 */
			bpp = 4;
			fmt = TILER_PIXEL_FMT_32BIT;
			break;
		case OMAP_DSS_COLOR_YUV2:               /* YUV2 4:2:2 co-sited */
		case OMAP_DSS_COLOR_UYVY:               /* UYVY 4:2:2 co-sited */
		case OMAP_DSS_COLOR_NV12:               /* NV12 format: YUV 4:2:0 */
		case OMAP_DSS_COLOR_RGBA16:             /* RGBA16-4444 */

		case OMAP_DSS_COLOR_RGBX16:             /* RGBx16-4444 */
		case OMAP_DSS_COLOR_ARGB16_1555:        /* ARGB16-1555 */


		default:
			//not supported type
			pr_warn("color mode(%d) is not supported color format in rotation buf manager\n", oi->cfg.color_mode);
			goto Skip_Alloc;
	}
	mutex_lock(&rot_buf_mgr.lock);
	for(i=0;i<MAX_BUF_NUM_FOR_ROT;i++)
	{
		struct tiler_2d_buffer* ith = &(rot_buf_mgr.buffers[i]);
		if ( ith->allocated && !ith->in_using )
		{
			if ( ith->alloc_data.w >=oi->cfg.width
					&& ith->alloc_data.h >=oi->cfg.height
					&& ith->alloc_data.fmt == fmt )
			{
				buf = ith;
				goto Skip_Alloc;
			}
		}
	}
	//find buffer & allocate
	buf = dsscomp_rotbuf_mgr_find_avail_buf_withlock();
	if ( buf!=NULL )
	{
		struct omap_ion_tiler_alloc_data* alloc_data = &(buf->alloc_data);

		alloc_data->w = buf->width = oi->cfg.width;
		alloc_data->h = buf->height = oi->cfg.height;
		alloc_data->fmt = fmt;
		alloc_data->flags = 0;

		//clear manager mask
		buf->mgr_mask = 0;
		//create ion client
		if ( rot_buf_mgr.ion==NULL )
		{
			rot_buf_mgr.ion = ion_client_create(omap_ion_device,
					1 << ION_HEAP_TYPE_CARVEOUT |
					1 << OMAP_ION_HEAP_TYPE_TILER,
					"dsscomp_rotbug_mgr");
			if ( IS_ERR_OR_NULL(rot_buf_mgr.ion) )
			{
				buf = NULL;
				goto Skip_Alloc;
			}
		}
		//alloc ion buffer
		if ( omap_ion_nonsecure_tiler_alloc(rot_buf_mgr.ion, alloc_data) )
		{
			buf = NULL;
			goto Skip_Alloc;
		}
		buf->allocated = true;
		buf->bpp = bpp;
		ion_phys(rot_buf_mgr.ion, alloc_data->handle, &(buf->phy), &(buf->len));
		//get view to get physical stride
		tilview_create(&buf->view, buf->phy, buf->width, buf->height);
#if 0
		{
			//test purpose
			struct tiler_block_t block;
			struct vm_struct *area;
			u32 addr;

			block.phys = buf->phy;
			block.width = alloc_data->w;
			block.height = alloc_data->h;
			area = get_vm_area(buf->size, VM_IOREMAP);
			if ( area==NULL )
				BUG();
			addr = area->addr;
			if ( tiler_ioremap_blk(&block, 0, buf->size, addr, 0) )
			{
				BUG();
			}
			for(i=0;i<buf->alloc_data.h;i++)
			{
				memset(addr+i*buf->alloc_data.stride, (i*16)%0xff, buf->alloc_data.w * bpp);
			}
			vunmap(addr);
		}
#endif
	}
Skip_Alloc:
	if ( buf!=NULL )
	{
		buf->in_using = true;
		buf->last_use = CURRENT_TIME;
		buf->src_phy_addr = oi->ba;
		buf->src_stride = oi->cfg.stride;

		oi->cfg.stride = buf->alloc_data.stride;
		oi->ba = buf->phy;
		oi->uv = 0;
		//printk("Set rot buffer address %p\n", (void*)oi->ba);
	}
	mutex_unlock(&rot_buf_mgr.lock);
	return buf;
}

/**
 * find buffer connected with oi(overlay) & gsync
 */
static struct tiler_2d_buffer* dsscomp_rotbuf_mgr_already_in(struct dss2_ovl_info *oi, struct dsscomp_gralloc_t *gsync)
{
	int i;
	struct tiler_2d_buffer* ret = NULL;
	if ( oi==NULL || gsync==NULL )
		return false;
	mutex_lock(&rot_buf_mgr.lock);
	for(i=0;i<MAX_BUF_NUM_FOR_ROT;i++)
	{
		struct tiler_2d_buffer *ith = &(rot_buf_mgr.buffers[i]);
		if ( ith->gsync==gsync && ith->src_phy_addr==oi->ba && ith->allocated && ith->in_using )
		{
			ret = ith;
			break;
		}
	}
	mutex_unlock(&rot_buf_mgr.lock);
	return ret;
}

/*
 * If rotation & non-tiler alloc tiler
 */
static void dsscomp_rotbuf_mgr_check(struct dss2_ovl_info *oi, struct dsscomp_gralloc_t *gsync, u32 ch)
{
	if ( oi==NULL )
	{
		//printk("oi NULL\n");
		return;
	}
	if ( gsync==NULL )
	{
		//printk("gsync NULL\n");
		return;
	}
	if ( oi->cfg.rotation==0 )              //no rotation
	{
		//printk("no rotation\n");
		return;
	}
	if ( is_tiler_addr(oi->ba) && tiler_fmt(oi->ba)!=TILFMT_PAGE )  //tiler
	{
		//printk("tiler address:0x%x\n", oi->ba);
		return;
	}
	//already exist
	{
		struct tiler_2d_buffer *buf;
		buf = dsscomp_rotbuf_mgr_already_in(oi, gsync);
		if ( buf!=NULL )        //already exist
		{
			//printk("Already exist\n");

			//set manager mask
			mutex_lock(&rot_buf_mgr.lock);
			buf->mgr_mask |= 1 << ch;
			mutex_unlock(&rot_buf_mgr.lock);
			return;
		}
	}
	{
		int i;
		struct tiler_2d_buffer *buf;
		for(i=0;i<MAX_BUF_NUM_FOR_ROT;i++)
			if ( gsync->rot_bufs[i]==NULL )
			{
				buf = dsscomp_rotbuf_mgr_get_buf(oi);
				gsync->rot_bufs[i] = buf;
				break;
			}
		if ( i>=MAX_BUF_NUM_FOR_ROT )
		{
			pr_warning("Too many rotation can't support!!\n");
			return;
		}
		//lock may be not needed. anyway
		mutex_lock(&rot_buf_mgr.lock);
		buf->gsync = gsync;
		buf->mgr_mask |= 1 << ch;
		mutex_unlock(&rot_buf_mgr.lock);
	}
}

/**
 * Callback when DMA complete
 */
static void dsscomp_rotbuf_mgr_dma_cb(int channel, u16 status, void *data)
{
	struct tiler_2d_buffer *buf = (struct tiler_2d_buffer*)data;
	if ( buf==NULL )
	{
		pr_warning("No data in DMA callback\n");
		return;
	}
	//may need status check???
	//locking is needed??? But this can be called in IRQ context
	//mutex lock cause problem in IRQ context
	buf->dma_copying = false;
	//awake
	wake_up_interruptible(&buf->dma_wait_q);
}

/**
 * start DMA copy
 */
static int dsscomp_rotbuf_mgr_start_dma_copy(struct tiler_2d_buffer *buf)
{
	int ret = 0;
	int data_type;
	int src_row_inc, dst_row_inc;
	if ( buf==NULL )
		return -EINVAL;

	mutex_lock(&rot_buf_mgr.lock);

	if ( buf->dma_copying ) //already started
		goto Done;
	ret = omap_request_dma(OMAP_DMA_NO_DEVICE, "dsscomp_rotbuf_dma",
			dsscomp_rotbuf_mgr_dma_cb, buf, &buf->dma_ch);
	if ( ret )
	{
		pr_warning("DMA request failed\n");
		goto Done;
	}
	switch(buf->bpp)
	{
		case 1:
			data_type = OMAP_DMA_DATA_TYPE_S8;
			break;
		case 2:
			data_type = OMAP_DMA_DATA_TYPE_S16;
			break;
		case 4:
			data_type = OMAP_DMA_DATA_TYPE_S32;
			break;
		default:
			ret = -EINVAL;
			goto Done;
	}
	//printk("DMA width:%d, height:%d\n", buf->width, buf->height);
	omap_set_dma_transfer_params(buf->dma_ch, data_type,
			buf->width, buf->height,
			OMAP_DMA_SYNC_BLOCK,    //I don't know exact meaning of SYNC MOD
			OMAP_DMA_NO_DEVICE,
			OMAP_DMA_DST_SYNC);

	src_row_inc = buf->src_stride - buf->width*buf->bpp + 1;
	//printk("DMA src address:%p, row_inc:%d\n", buf->src_phy_addr, src_row_inc);
	//source setting
	omap_set_dma_src_params(buf->dma_ch, 0,
			OMAP_DMA_AMODE_DOUBLE_IDX,      //2D. source can be contiguous, but for safe
			buf->src_phy_addr,                      //address of source
			1,                                                      //column increment
			src_row_inc                                     //row increment
			);
	omap_set_dma_src_data_pack(buf->dma_ch, 1);
	omap_set_dma_src_burst_mode(buf->dma_ch, OMAP_DMA_DATA_BURST_16);

	//destination setting
	dst_row_inc = buf->view.v_inc - buf->width*buf->bpp + 1;
	//printk("DMA dst address:%p, row_inc:%d\n", buf->phy, dst_row_inc);
	omap_set_dma_dest_params(buf->dma_ch, 0,
			OMAP_DMA_AMODE_DOUBLE_IDX,      //2D. Tiler is not contiguous
			buf->phy,                                       //address of target
			1,                                                      //column increment
			dst_row_inc                                     //row increment
			);
	omap_set_dma_dest_data_pack(buf->dma_ch, 1);
	omap_set_dma_dest_burst_mode(buf->dma_ch, OMAP_DMA_DATA_BURST_16);

	//high priority
	omap_dma_set_prio_lch(buf->dma_ch, DMA_CH_PRIO_HIGH, DMA_CH_PRIO_HIGH);
	//what's this????
	omap_dma_set_global_params(DMA_DEFAULT_ARB_RATE, 0xFF, 0);
	//dma start
	buf->dma_copying = true;
	omap_start_dma(buf->dma_ch);
Done:
	mutex_unlock(&rot_buf_mgr.lock);
	return ret;
}

/**
 * Wait DMA copy
 * @param time_out_milli : timeout in milli second
 */
static int dsscomp_rotbuf_mgr_wait_dma_copy(struct tiler_2d_buffer *buf, u32 time_out_milli)
{
	int r;
	r =  wait_event_interruptible_timeout(buf->dma_wait_q,
			!buf->dma_copying, msecs_to_jiffies(time_out_milli));
	mutex_lock(&rot_buf_mgr.lock);
	if ( buf->dma_ch!=-1 )
	{
		omap_stop_dma(buf->dma_ch);
		omap_free_dma(buf->dma_ch);
		buf->dma_ch = -1;
	}
	mutex_unlock(&rot_buf_mgr.lock);
	return 0;
}

/**
 * Start DMA copy of buffers connected with ch(DSS manager channel) in gsync's buffer list
 */
static int dsscomp_rotbuf_mgr_start_dma_copy_of_gralloc(struct dsscomp_gralloc_t *gsync, u32 ch)
{
	int i, r=0;
	u32 ch_flag;

	if ( gsync==NULL )
		return -EINVAL;
	ch_flag = 1 << ch;

	for(i=0;i<MAX_BUF_NUM_FOR_ROT;i++)
	{
		struct tiler_2d_buffer *buf = gsync->rot_bufs[i];
		if ( buf==NULL )
			continue;
		if ( buf->mgr_mask & ch_flag )
		{
			int cr;
			cr = dsscomp_rotbuf_mgr_start_dma_copy(buf);
			if ( cr )
			{
				pr_warning("DMA copy start failed.\n");
				r = -EIO;
				//when one DMA fails, rest DMS should be started
			}
		}

	}
	return r;
}

/**
 * Wait DMA copy buffers connected with ch(DSS manager channel) in gsync's buffer list
 */
static int dsscomp_rotbuf_mgr_wait_dma_copy_of_gralloc(struct dsscomp_gralloc_t *gsync, u32 ch)
{
	int i, r;
	u32 ch_flag;
	if ( gsync==NULL )
		return -EINVAL;
	ch_flag = 1 << ch;

	for(i=0;i<MAX_BUF_NUM_FOR_ROT;i++)
	{
		struct tiler_2d_buffer *buf = gsync->rot_bufs[i];
		if ( buf==NULL )
			continue;
		if ( buf->mgr_mask & ch_flag )
		{
			//wait for 1/60 second
			r = dsscomp_rotbuf_mgr_wait_dma_copy(buf, (1000/60));
			if ( r )
				pr_warning("DMA wait failed\n");
		}

	}
	return 0;
}


void dsscomp_dbg_rotbuf_mgr(struct seq_file *s)
{
#ifdef CONFIG_DEBUG_FS
	int i;
	mutex_lock(&dbg_mtx);
	mutex_lock(&rot_buf_mgr.lock);

	seq_printf(s, "ROTATION BUFFER STATUS\n");
	//             12 12345 123 1234567 1234567 123456 123456789 123456789 12456789
	seq_printf(s, "no Alloc Use   width  height    fmt       len    stride   offset   last\n");
	for(i=0;i<MAX_BUF_NUM_FOR_ROT;i++)
	{
		struct tiler_2d_buffer *buf = &(rot_buf_mgr.buffers[i]);
		seq_printf(s, "%2d ", i);
		if ( buf->allocated )
		{
			char *fmt;
			switch (buf->alloc_data.fmt)
			{
				case TILER_PIXEL_FMT_8BIT:
					fmt = "8";
					break;
				case TILER_PIXEL_FMT_16BIT:
					fmt = "16";
					break;
				case OMAP_DSS_COLOR_RGBX32:
					fmt = "32";
					break;
				default:
					fmt ="Unknown";
					break;
			}
			seq_printf(s, "     y ");
			seq_printf(s, "%3s %7d %7d %6s %9d %9d %9d %d\n",
					buf->in_using ? "Y" : "N",
					buf->alloc_data.w, buf->alloc_data.h,
					fmt, buf->len,
					buf->alloc_data.stride,
					buf->alloc_data.offset,
					(int)buf->last_use.tv_sec);
		}
		else
		{
			seq_printf(s, "    n\n");
		}
	}

	mutex_unlock(&rot_buf_mgr.lock);
	mutex_unlock(&dbg_mtx);
#endif
}

void dsscomp_rotbuf_mgr_init(void)
{
	int i;
	mutex_init(&rot_buf_mgr.lock);
	rot_buf_mgr.ion = NULL;
	for(i=0;i<MAX_BUF_NUM_FOR_ROT;i++)
	{
		rot_buf_mgr.buffers[i].allocated = false;
		init_waitqueue_head( &(rot_buf_mgr.buffers[i].dma_wait_q) );
	}
	//defers ion client creation
}

void dsscomp_rotbuf_mgr_deinit(void)
{
	if ( rot_buf_mgr.ion!=NULL )
	{
		struct ion_client *ion = rot_buf_mgr.ion;
		int i;
		//free tiler buffers
		for(i=0;i<MAX_BUF_NUM_FOR_ROT;i++)
			if ( rot_buf_mgr.buffers[i].allocated )
			{
				ion_free(ion, rot_buf_mgr.buffers[i].alloc_data.handle);
				rot_buf_mgr.buffers[i].allocated = false;
			}
		ion_client_destroy(ion);
		rot_buf_mgr.ion = NULL;
	}
	mutex_destroy(&rot_buf_mgr.lock);
}

#endif

void dsscomp_gralloc_init(struct dsscomp_dev *cdev_)
{
	int i;

	if (!cdev_)
		return;

	/* save at least cdev pointer */
	if (!cdev && cdev_) {
		cdev = cdev_;
#ifdef CONFIG_HAS_EARLYSUSPEND
		register_early_suspend(&early_suspend_info);
#endif
	}

	if (!free_slots.next) {
		INIT_LIST_HEAD(&free_slots);
		for (i = 0; i < NUM_TILER1D_SLOTS; i++) {
			u32 phys;
			tiler_blk_handle slot =
				tiler_alloc_block_area(TILFMT_PAGE,
					tiler1d_slot_size(cdev_), 1, &phys,
					NULL);
			if (IS_ERR_OR_NULL(slot)) {
				pr_err("could not allocate slot");
				break;
			}
			slots[i].slot = slot;
			slots[i].phys = phys;
			slots[i].size = tiler1d_slot_size(cdev_) >> PAGE_SHIFT;
			slots[i].page_map = vmalloc(sizeof(*slots[i].page_map) *
						slots[i].size);
			if (!slots[i].page_map) {
				pr_err("could not allocate page_map");
				tiler_free_block_area(slot);
				break;
			}
			list_add(&slots[i].q, &free_slots);
			up(&free_slots_sem);
		}
		/* reset free_slots if no TILER memory could be reserved */
		if (!i)
			ZERO(free_slots);
	}
}

void dsscomp_gralloc_exit(void)
{
	struct tiler1d_slot *slot;


#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&early_suspend_info);
#endif

	list_for_each_entry(slot, &free_slots, q) {
		vfree(slot->page_map);
		tiler_free_block_area(slot->slot);
	}
	INIT_LIST_HEAD(&free_slots);
}
