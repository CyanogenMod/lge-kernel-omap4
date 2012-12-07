/*
 * dsscomp_adapt.c
 *
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/completion.h>
#include <mach/tiler.h>
#include <video/dsscomp.h>
#include <plat/dsscomp.h>

#include <linux/ion.h>
#include <linux/omap_ion.h>
#include <plat/dma.h>
#include <plat/vram.h>

#include "dsscomp.h"
#include "dsscomp_adapt.h"

#include <linux/pm_runtime.h>
#include <linux/pm_qos_params.h>
#include <plat/omap-pm.h>

#include <linux/delay.h>
//workaround avoiding dma
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/omapfb.h>
#include <plat/vrfb.h>
#include "../omapfb/omapfb.h"

#undef DSSCOMP_ADAPT_DEBUG 

#ifdef DSSCOMP_ADAPT_DEBUG
#define DBG_PRINTK printk
#else
#define DBG_PRINTK(X...) 
#endif

extern struct ion_device *omap_ion_device;

//no access during time, buffer will be free.
#define DSSCOMP_ADAPT_BUF_FREE_TIME	(5 * 60 * 1000)		//5 minutes

#include "dsscomp_adapt.h"

enum adapt_buf_type {
	BEGIN_OF_TYPE = 0,
	TILER_2D_TYPE,
	VRAM_TYPE,
#ifdef CONFIG_MACH_LGE_COSMO
	TILER_1D_DYNAMIC_TYPE,
#endif
	END_OF_TYPE,
};

struct adapt_buf_t {
	bool	allocated;		//allocation flag
	bool	in_using;		//whether dss is using this buffer

	int		bpp;			//byte per pixel
	int		using_width, using_height;	//w,h really used

	enum adapt_buf_type type;	//buffer type

	union {
		//VRAM
		struct {
			void	*virt_addr;		//virtual address
			u32		phy_addr;		//physical address
			u32		alloc_size;		//allocated size
		} vram;
		//TILER
		struct {
			u32		src_phy_addr;	//source physical address
			size_t	src_stride;		//source stride

			struct omap_ion_tiler_alloc_data alloc_data;
			ion_phys_addr_t phy;	//physical address of ION alloc
			size_t	len;
			struct tiler_view_t view;
		} tiler;
#ifdef CONFIG_MACH_LGE_COSMO
		//Dynamic tiler (alloc from non-carve-out memory)
		struct {
			struct tiler_block_t blk;
		} tiler_1d_dyn;
#endif
	} buf;

	struct timespec	last_use;
	struct delayed_work	free_work;	//check & free buffer
	bool	free_scheduled;

};

struct adapt_buf_list_t {
	struct adapt_buf_t *buf;
	struct adapt_buf_list_t *next;
};

#define DSSCOMP_ADAPT_BUF_COUNT 10

//History Logging related
#ifdef DSSCOMP_ADAPT_DEBUG

#define DSSCOMP_ADAPT_ACTION(...) dsscomp_adapt_action_log(__VA_ARGS__)
#define DSSCOMP_ADAPT_HISTORY_COUNT	100
#define DSSCOMP_ADAPT_HISTORY_LEN	256
#define DSSCOMP_ADAPT_HISTORY_NEXT(X)  ( (X+1) % DSSCOMP_ADAPT_HISTORY_COUNT )

#else

#define DSSCOMP_ADAPT_ACTION(...)

#endif

static atomic_t	forced_stop;

inline int is_forces_stop(void)
{
	if ( atomic_read(&forced_stop) )
	{
		//at least one must read forced stop
		atomic_set(&forced_stop, 0);
		pr_warning("dsscomp_adpat forced stop\n");
		return 1;
	}
	else
	{
		DBG_PRINTK("no forced stop\n");
		return 0;
	}
}
struct dsscomp_adapt_mgr_t {
	struct mutex 		lock;
	struct ion_client 	*ion;
	struct adapt_buf_t buffers[DSSCOMP_ADAPT_BUF_COUNT];
	struct workqueue_struct	*workq;
	atomic_t			in_progress_count;
	struct pm_qos_request_list pm_qos_handle;
#ifdef DSSCOMP_ADAPT_DEBUG
	struct mutex		log_lock;
	int					log_tail, log_head;
	struct	{
		char				log[DSSCOMP_ADAPT_HISTORY_LEN];
		struct timespec		time;
	} log_entry[DSSCOMP_ADAPT_HISTORY_COUNT];

#endif
};
static struct dsscomp_dev *cdev;

//DSSCOMP adaptation operand buffer
struct adapt_lambda_opd_t {
	int bpp;				//bytes per pixel
	struct dss2_rect_t	crop;	//crop in buffer
	__u32 paddr;			//physical address
	__u32 uv_paddr;			//physical uv address
	__u32 phy_stride;		//physical stride
	void *vaddr;			//virtual address
	int virt_stride;		//virtual stride
	__u16 buffer_width, buffer_height;	//buffer's width & height
	enum omap_color_mode	color_mode;	//color mode
};

struct lambda_list_t;
//DSSCOMP adaptation operator function proto-type
typedef int (*dsscomp_adapt_op_opt)(struct lambda_list_t *lambda);

//DSCCOMP adaptation lambda(?) list
struct lambda_list_t {
	struct lambda_list_t *next;		//next lambda
	dsscomp_adapt_op_opt opt;					//operator
	struct adapt_lambda_opd_t *src, *dst;		//source & target operand
	const char*	description;					//simple description to operation
	int depth;   //##hw2002.cho@lge.com
#ifdef CONFIG_HRZ_II
	__u8 rotation;
#endif
};

//DSSCOMP adaptation info
struct dsscomp_adapt_info {
	struct adapt_buf_list_t* buf_in_use;	//buffers used in this adaptation
	struct lambda_list_t* lambda_list;	//lambdas used in this adaptation
	struct completion complete;						//all lambda completion
	int		result;									//result of lambda
	volatile int	in_processing;					//in processing
};

//@todo in future
//typdef struct lambda_list_t* (*dsscomp_adapt_lambda_factory)(struct dss2_ovl_info* ovl_info, struct adapt_buf_list_t** buf_list);

//Adaptation Manager
static struct dsscomp_adapt_mgr_t mgr;

unsigned char *black_line;
#define LCD_WIDTH	480
static void dsscomp_adapt_init_deptharea( int depth ,struct adapt_lambda_opd_t *black_backup);


//History
#ifdef DSSCOMP_ADAPT_DEBUG

static void dsscomp_adapt_action_log(const char* fmt, ...)
{
	int ix;
	va_list args;
	mutex_lock(&mgr.log_lock);
	ix = mgr.log_head;
	if ( DSSCOMP_ADAPT_HISTORY_NEXT(mgr.log_head) == mgr.log_tail )
	{
		mgr.log_tail = DSSCOMP_ADAPT_HISTORY_NEXT(mgr.log_tail);
		mgr.log_head = DSSCOMP_ADAPT_HISTORY_NEXT(mgr.log_head);
	}
	else
		mgr.log_head = DSSCOMP_ADAPT_HISTORY_NEXT(mgr.log_head);
	va_start(args, fmt);
	vsnprintf(mgr.log_entry[ix].log, DSSCOMP_ADAPT_HISTORY_LEN-1, fmt, args);
	va_end(args);
	mgr.log_entry[ix].time = CURRENT_TIME;
	mutex_unlock(&mgr.log_lock);
}

#endif

////////////////////////////////////////////////////////////////////////////////////////
// Buffer Management

static bool dsscomp_adapt_mgr_free_buf_with_lock(struct adapt_buf_t *buf)
{
	if ( buf==NULL )
		return false;
	if ( buf->in_using )
		return false;
	if ( buf->free_scheduled )
	{
		//cancel free work
		cancel_delayed_work(&(buf->free_work));
		buf->free_scheduled = false;
	}
	if ( buf->allocated )
	{
		switch ( buf->type )
		{
		case TILER_2D_TYPE:
			ion_free(mgr.ion, buf->buf.tiler.alloc_data.handle);
			buf->allocated = false;
			break;
#ifdef CONFIG_MACH_LGE_COSMO
		case VRAM_TYPE:
#else
		default:
#endif
			omap_vram_free(buf->buf.vram.phy_addr, buf->buf.vram.alloc_size);
			buf->allocated = false;
			break;
#ifdef CONFIG_MACH_LGE_COSMO
		case TILER_1D_DYNAMIC_TYPE:
			tiler_free(&(buf->buf.tiler_1d_dyn.blk));
			buf->allocated = false;
			break;
		default :
			pr_warning("Buffer(0x%x) type(0x%x) is invalid\n", buf, buf->type);
			return false;
#endif
		}
	}
	DSSCOMP_ADAPT_ACTION("Buffer(0x%x) freed", buf);
	return true;
}

/**
 * free buffer
 * this function must be called with lock
 */
static bool dsscomp_adapt_mgr_free_buf_by_idx_withlock(int i)
{
	struct adapt_buf_t *buf;
	if ( i<0 || i>=DSSCOMP_ADAPT_BUF_COUNT )
		return false;
	buf = & (mgr.buffers[i]);
	return dsscomp_adapt_mgr_free_buf_with_lock(buf);
}

/**
 * find empty slot or victim
 * this function must be called with lock
 */
static struct adapt_buf_t* dsscomp_adapt_mgr_find_avail_buf_withlock(void)
{
	int i;


	//find not allocated buffer
	for(i=0;i<DSSCOMP_ADAPT_BUF_COUNT;i++)
	{
		if ( !mgr.buffers[i].allocated )
			return &(mgr.buffers[i]);
	}
	//find not using buffer
	for(i=0;i<DSSCOMP_ADAPT_BUF_COUNT;i++)
	{
		if ( !mgr.buffers[i].in_using )
		{
			dsscomp_adapt_mgr_free_buf_by_idx_withlock(i);
			return &(mgr.buffers[i]);
		}
	}
	return NULL;
}

/**
 * check in-using and free buffer
 */
static void dsscomp_adapt_mgr_delayed_free_buf(struct work_struct *work)
{
	struct adapt_buf_t *buf = container_of(work, struct adapt_buf_t, free_work.work);
	mutex_lock(&mgr.lock);
	buf->free_scheduled = false;
	if ( buf->allocated && !buf->in_using )
		dsscomp_adapt_mgr_free_buf_with_lock(buf);
	mutex_unlock(&mgr.lock);
}

/**
 * put buffer when no more use
 */
static void dsscomp_adapt_mgr_put_buf(struct adapt_buf_t *buf)
{
	if ( buf==NULL )
		return;
	mutex_lock(&mgr.lock);
	buf->in_using = false;

	buf->in_using = false;
	//start delay work for free 2D tiler memory
	INIT_DELAYED_WORK( &(buf->free_work), dsscomp_adapt_mgr_delayed_free_buf);
	buf->free_scheduled = true;
	schedule_delayed_work(&(buf->free_work), msecs_to_jiffies(DSSCOMP_ADAPT_BUF_FREE_TIME) );

	mutex_unlock(&mgr.lock);
	DSSCOMP_ADAPT_ACTION("Buffer(0x%x) put, delayed free scheduled after %d ms", buf, DSSCOMP_ADAPT_BUF_FREE_TIME);
}

/**
 * Get bpp of color mode
 */
static int bpp_from_color_mode(enum omap_color_mode color_mode)
{
	switch ( color_mode )
	{
	case OMAP_DSS_COLOR_CLUT8:		/* BITMAP 8 */
		return 1;
	case OMAP_DSS_COLOR_ARGB16:		/* ARGB16-4444 */
	case OMAP_DSS_COLOR_RGB16:		/* RGB16-565 */
		return 2;
	case OMAP_DSS_COLOR_ARGB32:		/* ARGB32-8888 */
	case OMAP_DSS_COLOR_RGBA32:		/* RGBA32-8888 */
	case OMAP_DSS_COLOR_RGB24U:		/* xRGB24-8888 */
	case OMAP_DSS_COLOR_RGB24P:		/* RGB24-888 */

	case OMAP_DSS_COLOR_RGBX32:		/* RGBx32-8888 */
		return 4;
	case OMAP_DSS_COLOR_NV12:		/* NV12 format: YUV 4:2:0 */
	case OMAP_DSS_COLOR_RGBA16:		/* RGBA16-4444 */

	case OMAP_DSS_COLOR_RGBX16:		/* RGBx16-4444 */
	case OMAP_DSS_COLOR_ARGB16_1555:	/* ARGB16-1555 */
		return 2;
	case OMAP_DSS_COLOR_YUV2:		/* YUV2 4:2:2 co-sited */
	case OMAP_DSS_COLOR_UYVY:		/* UYVY 4:2:2 co-sited */
	default:
		//not supported type
		return 0;
	}

}


/**
 * get buffer
 */
static struct adapt_buf_t* dsscomp_adapt_mgr_get_buf(enum adapt_buf_type type, int bpp, int w, int h)
{
	int i;
	int fmt;
	struct adapt_buf_t* buf = NULL;


	if ( type<=BEGIN_OF_TYPE || END_OF_TYPE<=type)
		return NULL;

	switch ( bpp )
	{
	case 1 :
		fmt = TILER_PIXEL_FMT_8BIT;
		break;
	case 2 :
		fmt = TILER_PIXEL_FMT_16BIT;
		break;
	case 4 :
		fmt = TILER_PIXEL_FMT_32BIT;
		break;
	default:
		//no tiler type
		fmt = -1;
		if ( type==TILER_2D_TYPE )
			return NULL;
		break;
	}

	mutex_lock(&mgr.lock);
	//find available buffer can fit
	for(i=0;i<DSSCOMP_ADAPT_BUF_COUNT;i++)
	{
		struct adapt_buf_t* ith = &(mgr.buffers[i]);
		if ( ith->allocated && !ith->in_using
				&& ith->bpp==bpp && ith->type==type )
		{
			switch ( type )
			{
			case TILER_2D_TYPE:
				if ( ith->buf.tiler.alloc_data.w >=w
					&& ith->buf.tiler.alloc_data.h >=h
					&& ith->buf.tiler.alloc_data.fmt == fmt )
				{
					buf = ith;
					DBG_PRINTK("Found available tiler buffer(%d)\n", i);
					goto Skip_Alloc;
				}
				break;
			case VRAM_TYPE:
				if ( ith->buf.vram.alloc_size>=(w*h*bpp) )
				{
					buf = ith;
					DBG_PRINTK("Found available vram buffer(%d)\n", i);
					goto Skip_Alloc;
				}
				break;
#ifdef CONFIG_MACH_LGE_COSMO
			case TILER_1D_DYNAMIC_TYPE:
				if ( ith->buf.tiler_1d_dyn.blk.width>=(w*h*bpp) )
				{
					buf = ith;
					DBG_PRINTK("Found availabe dynamic tiler 1D buffer(%d)\n", i);
					goto Skip_Alloc;
				}
				break;
#endif
			default:
				goto Skip_Alloc;
			}
		}
	}
	//find buffer & allocate
	buf = dsscomp_adapt_mgr_find_avail_buf_withlock();
	if ( buf!=NULL )
	{
		buf->bpp = bpp;
		switch ( type )
		{
		case TILER_2D_TYPE:
			{
				struct omap_ion_tiler_alloc_data* alloc_data = &(buf->buf.tiler.alloc_data);

				buf->type = TILER_2D_TYPE;
				DBG_PRINTK("Allocating tiler buffer\n");

				alloc_data->w = w;
				alloc_data->h = h;
				alloc_data->fmt = fmt;
				alloc_data->flags = 0;

				//create ion client
				if ( mgr.ion==NULL )
				{
					mgr.ion = ion_client_create(omap_ion_device,
							 1 << ION_HEAP_TYPE_CARVEOUT |
							 1 << OMAP_ION_HEAP_TYPE_TILER,
							 "dsscomp_adapt_mgr_t");
					if ( IS_ERR_OR_NULL(mgr.ion) )
					{
						pr_warn("ion_client_create() failed\n");
						buf = NULL;
						goto Skip_Alloc;
					}
				}
				//alloc ion buffer
				if ( omap_ion_nonsecure_tiler_alloc(mgr.ion, alloc_data) )
				{
					pr_warn("tiler alloc failed\n");
					buf = NULL;
					goto Skip_Alloc;
				}
				buf->allocated = true;
				ion_phys(mgr.ion, alloc_data->handle, &(buf->buf.tiler.phy), &(buf->buf.tiler.len));
				//get view to get physical stride
				tilview_create(&buf->buf.tiler.view, buf->buf.tiler.phy, alloc_data->w, alloc_data->h);
				buf->allocated = true;
				DBG_PRINTK("Alloc Tiler Buffer success\n");
				DSSCOMP_ADAPT_ACTION("Buffer %p allocated(tiler, size:%d)n", buf, buf->buf.tiler.len);
			}
			break;
		case VRAM_TYPE:
			buf->type = VRAM_TYPE;
			DBG_PRINTK("Alloc VRAM Buffer size:%d\n", w*h*bpp);
			buf->buf.vram.phy_addr = 0;
			if ( omap_vram_alloc(OMAP_VRAM_MEMTYPE_SDRAM, w*h*bpp, (unsigned long*)&buf->buf.vram.phy_addr) )
			{
				pr_warn("VRAM alloc for dsscomp_adapt failed\n");
				buf = NULL;
				goto Skip_Alloc;
			}
			buf->buf.vram.virt_addr = NULL;
			buf->buf.vram.alloc_size = w * h * bpp;
			buf->allocated = true;
			DBG_PRINTK("Alloc VRAM Buffer Success\n");
			DSSCOMP_ADAPT_ACTION("Buffer %p allocated(vram, size:%d)", buf, buf->buf.vram.alloc_size);

			break;
#ifdef CONFIG_MACH_LGE_COSMO
		case TILER_1D_DYNAMIC_TYPE:
			buf->type = TILER_1D_DYNAMIC_TYPE;
			DBG_PRINTK("Alloc Dynamic tiler 1D size:%d\n", w*h*bpp);
			{
				struct tiler_block_t *blk = &(buf->buf.tiler_1d_dyn.blk);
				blk->phys = 0;
				blk->width = w*h*bpp;
				blk->height = 1;
				blk->key = 0;
				if ( tiler_alloc(blk, TILFMT_PAGE) )
				{
					pr_warn("Dynamic Tiler 1D alloc for dsscomp_adapt failed\n");
					buf = NULL;
					goto Skip_Alloc;
				}
			}
			buf->allocated = true;
			DBG_PRINTK("Alloc Dynamic 1D Tiler Buffer Success\n");
			DSSCOMP_ADAPT_ACTION("Buffer %p allocated(dynamic 1d tiler, size:%d)", buf, buf->buf.tiler_1d_dyn.blk.width);
			break;
#endif
		default:
			//never happen...
			goto Skip_Alloc;
		}
	}
Skip_Alloc:
	if ( buf!=NULL )
	{
		buf->in_using = true;
		if ( buf->free_scheduled )
		{
			//cancel free work
			cancel_delayed_work(&(buf->free_work));
			buf->free_scheduled = false;
			DSSCOMP_ADAPT_ACTION("Buffer 0x%x delayed free canceled", buf);
		}
		buf->last_use = CURRENT_TIME;
		buf->using_width = w;
		buf->using_height = h;
		DBG_PRINTK("Got dsscomp_adapt buffer\n");
		DSSCOMP_ADAPT_ACTION("Buffer 0x%x get. %dx%d bpp:%d",
				buf, buf->using_width, buf->using_height, buf->bpp);
	}
	mutex_unlock(&mgr.lock);
	return buf;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//	Adaptation Helpers
//

/**
 * Get Buffer form overlay info
 */
static struct adapt_buf_t* dsscomp_adapt_mgr_get_buf_from_ovl(struct dss2_ovl_info *oi, enum adapt_buf_type type)
{
	int bpp, w, h;
	if ( oi==NULL )
		return NULL;
	DBG_PRINTK("Get Buffer type(%d), colodr_mode(%d), w(%d), h(%d)\n", type, oi->cfg.color_mode, oi->cfg.width, oi->cfg.height);
	bpp = bpp_from_color_mode(oi->cfg.color_mode);
	if ( bpp < 1 || bpp > 4 )
	{
		pr_warning("Color mode(%d) is not supported in dsscomp_adapt\n", oi->cfg.color_mode);
		return NULL;
	}
	w = oi->cfg.width;
	h = oi->cfg.height;
	return dsscomp_adapt_mgr_get_buf(type, bpp, w, h);
}


/**
 * Get operand buffer from dsscomp_adapt buffer
 */
static struct adapt_lambda_opd_t* dsscomp_adapt_opd_from_buf(struct adapt_buf_t *buf)
{
	struct adapt_lambda_opd_t *new_alloc = NULL;
	if ( buf==NULL )
		return NULL;
	new_alloc = (struct adapt_lambda_opd_t*) vmalloc(sizeof(struct adapt_lambda_opd_t));
	if ( new_alloc==NULL )
		return NULL;
	memset(new_alloc, sizeof(*new_alloc), 0);
	//common
	new_alloc->bpp = buf->bpp;
	new_alloc->crop.x = 0;
	new_alloc->crop.y = 0;
	new_alloc->crop.w = buf->using_width;
	new_alloc->crop.h = buf->using_height;
	switch ( buf->type)
	{
	case TILER_2D_TYPE:
		new_alloc->paddr = buf->buf.tiler.phy;
		new_alloc->phy_stride = buf->buf.tiler.view.v_inc;
		new_alloc->uv_paddr = 0;
		new_alloc->buffer_width = buf->buf.tiler.alloc_data.w;
		new_alloc->buffer_height = buf->buf.tiler.alloc_data.h;
		break;
	case VRAM_TYPE:
		new_alloc->paddr = buf->buf.vram.phy_addr;
		new_alloc->phy_stride = buf->bpp * buf->using_width;
		new_alloc->uv_paddr = 0;
		new_alloc->buffer_width = buf->using_width;
		//not good
		new_alloc->buffer_height = buf->buf.vram.alloc_size / new_alloc->buffer_width;
		break;
#ifdef CONFIG_MACH_LGE_COSMO
	case TILER_1D_DYNAMIC_TYPE:
		new_alloc->paddr = buf->buf.tiler_1d_dyn.blk.phys;
		new_alloc->phy_stride = buf->bpp * buf->using_width;
		new_alloc->uv_paddr = 0;
		new_alloc->buffer_width = buf->using_width;
		//not good
		new_alloc->buffer_height = buf->buf.tiler_1d_dyn.blk.width / new_alloc->buffer_width;
		break;
#endif
	default:
		vfree(new_alloc);
		return NULL;
	}
	return new_alloc;
}

/**
 * Get Operand buffer from overlay info
 */
static struct adapt_lambda_opd_t* dsscomp_adapt_opd_from_ovl(struct dss2_ovl_info *oi)
{
	struct adapt_lambda_opd_t ret;

	if ( oi==NULL )
		return NULL;

	DBG_PRINTK("Getting dsscomp_adapt operand buffer from ovl_info(addressing:%d, color:%d, %dx%d, ba:0x%x, uv:0x%x\n",
			oi->addressing, oi->cfg.color_mode, oi->cfg.width, oi->cfg.height, oi->ba, oi->uv);

	memset(&ret, sizeof(ret), 0);
	ret.bpp = bpp_from_color_mode(oi->cfg.color_mode);
	if ( ret.bpp < 1 )
	{
		pr_warning("unsupported color mode(%d) in dsscomp_adapt_opd_from_ovl()\n", oi->cfg.color_mode);
		return NULL;
	}
	if ( oi->ba==0 )
	{
		pr_warning("OI's addfress in NULL\n");
		return NULL;
	}
	//oi's ba & uv are already set. So address type can be ignored
#if 0
	switch(oi->addressing)
	{
	case OMAP_DSS_BUFADDR_OVL_IX :	/* using a prior overlay */
		if ( d==NULL )
		{
			pr_warning("dsscomp_setup_dispc_data in NULL in %s\n", __func__);
			return NULL;
		}
		return dsscomp_adapt_opd_from_ovl(&d->ovls[ovl_info->ba], d);
	case OMAP_DSS_BUFADDR_FB :		/* using framebuffer memory */
		//@todo checking uv type
		//in case of FB, assume not using uv. (Now this is true, but in future I don't know)
		{
			int fb_ix = (oi->ba >> 28);
			struct fb_info *fbi = NULL;
			size_t size = oi->cfg.height * oi->cfg.stride;
			if (fb_ix >= num_registered_fb || oi->cfg.color_mode == OMAP_DSS_COLOR_NV12 )
			{
				pr_warning("unsupported frame buffer\n");
				return NULL;
			}
			fbi = registered_fb[fb_ix];
			ret.paddr = oi->ba + fbi->fix.smem_start;
			ret.phy_stride = oi->cfg.stride;
		}
		break;
	case OMAP_DSS_BUFADDR_DIRECT :	/* using direct addresses */
		ret.width	= oi->cfg.width;
		ret.height	= oi->cfg.height;
		ret.paddr	= oi->ba;
		ret.uv_paddr= oi->uv;
		ret.phy_stride = oi->cfg.stride;
		break;
	case OMAP_DSS_BUFADDR_BYTYPE :	/* using buffer types */
	case OMAP_DSS_BUFADDR_ION :		/* using ion handle(s) */
	case OMAP_DSS_BUFADDR_GRALLOC :	/* using gralloc handle */
	case OMAP_DSS_BUFADDR_LAYER_IX :	/* using a Post2 layer */
	default:
		pr_warning("%d is not supported DSS BUFFER ADDRESS type\n",oi->addressing);
		return NULL;
	}
#endif
	ret.crop	= oi->cfg.crop;
	ret.paddr	= oi->ba;
	ret.uv_paddr= oi->uv;
	ret.phy_stride = oi->cfg.stride;
	ret.color_mode = oi->cfg.color_mode;
	ret.virt_stride = 0;
	ret.vaddr = NULL;
	ret.buffer_width	= oi->cfg.width;
	ret.buffer_height	= oi->cfg.height;

	//alloc and fill
	{
		struct adapt_lambda_opd_t *new_alloc = (struct adapt_lambda_opd_t*) vmalloc(sizeof(struct adapt_lambda_opd_t));
		if ( new_alloc )
		{
			memcpy(new_alloc, &ret, sizeof(ret));
			DBG_PRINTK("operand_buffer(%p) : %dx%d, crop(%d,%d)+(%d,%d) paddr:0x%x, uv_paddr:0x%x, phy_stride:%d\n",
					new_alloc, new_alloc->buffer_width, new_alloc->buffer_height,
					new_alloc->crop.x, new_alloc->crop.y, new_alloc->crop.w, new_alloc->crop.h,
					new_alloc->paddr, new_alloc->uv_paddr, new_alloc->phy_stride);
			return new_alloc;
		}
		else
			return NULL;
	}
}

/**
 * queue dsscom_adapt_buffer_list
 */
static struct adapt_buf_list_t *dsscomp_adapt_push_to_buffer_list(struct adapt_buf_list_t *list, struct adapt_buf_t *buf)
{
	struct adapt_buf_list_t *new_tail;
	if ( buf==NULL )
		return list;
	new_tail = vmalloc(sizeof(*new_tail));
	if ( new_tail==NULL )
	{
		pr_warning("vmalloc failed\n");
		return list;
	}
	new_tail->buf = buf;
	new_tail->next = list;
	return new_tail;
}

/**
 * free dsscomp_dapt_buffer_list. buffers in list will be put.
 */
static void dsscomp_adapt_free_buffer_list(struct adapt_buf_list_t *list)
{
	while(list!=NULL)
	{
		struct adapt_buf_list_t *to_free = list;
		if ( list->buf!=NULL )
		{
			dsscomp_adapt_mgr_put_buf(list->buf);
		}
		list = list->next;
		vfree(to_free);
	}
}

/**
 * DMA copy CB
 */
static void dsscomp_adapt_2d_dma_copy_cb(int channel, u16 status, void *data)
{
	if (data!=NULL)
	{
		DBG_PRINTK("DMA copy CB\n");
		complete( (struct completion*)data);
	}
}

struct dsscomp_adapt_dma_wait_work {
	struct work_struct work;
	int dma_ch;
	struct completion *p_completion;
};

static void dsscomp_adapt_dma_wait(struct work_struct *work)
{
	struct dsscomp_adapt_dma_wait_work *wk = container_of(work, typeof(*wk), work);
	int dma_ch;
	struct completion *p_completion;
	dma_ch = wk->dma_ch;
	p_completion = wk->p_completion;
	kfree(wk);
	pr_warning("Waiting uncompleted DMA operation\n");
	if ( !wait_for_completion_timeout(p_completion, msecs_to_jiffies(1000)) )
	{
		extern void omap4_prm_global_warm_sw_reset(void);
		pr_warning("DMA operation not completed in 1 sec. anyway stop and go\n");
		omap4_prm_global_warm_sw_reset();
	}
	else
		pr_warning("DAM operation completed\n");
	while (omap_get_dma_active_status(dma_ch))
		omap_stop_dma(dma_ch);
	omap_free_dma(dma_ch);
	atomic_dec(&mgr.in_progress_count);
}

/**
 * Request 2D DMA copy and waiting completion.
 * 2 buffer must have same type (means bbs is equal)
 * @param src :
 */
static int dsscomp_adapt_2d_dma_copy(u32 src, u32 src_stride, u32 dst, u32 dst_stride, int bpp, int width, int height)
{
	int r = 0;
	int dma_ch;
	int data_type;
//	DECLARE_COMPLETION_ONSTACK(completion);
	//workaround for dma timeout
	static struct completion completion;
#if 1
	const unsigned long dma_wait_time = msecs_to_jiffies(1000/30);
#else
	unsigned long dma_wait_time = msecs_to_jiffies(1000/30);
	static int count = 0;
	if ( (count%33)== 0 )
	{
		dma_wait_time = 1;
		printk("Making DMA timeout for test purpose");
	}
	count++;
#endif

	init_completion(&completion);

	if ( is_forces_stop() )
		return -EINTR;
	DBG_PRINTK("DMA Copy : 0x%x(%d) to 0x%x(%d), bpp:%d, width:%d, height:%d\n",
			src, src_stride, dst, dst_stride, bpp, width, height);
	switch ( bpp )
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
		return -EINVAL;
	}
	r = omap_request_dma(OMAP_DMA_NO_DEVICE, "dsscomp_adapt",
			dsscomp_adapt_2d_dma_copy_cb, &completion, &dma_ch);
	if ( r )
		return r;

	omap_set_dma_transfer_params(dma_ch, data_type,
			width, height,
			OMAP_DMA_SYNC_BLOCK,	//I don't know exact meaning of SYNC MOD
			OMAP_DMA_NO_DEVICE,
			OMAP_DMA_DST_SYNC);

	//source setting
	omap_set_dma_src_params(dma_ch, 0,
			OMAP_DMA_AMODE_DOUBLE_IDX,	//2D. source can be contiguous, but for safe
			src,						//address of source
			1, 							//column increment
			src_stride - width*bpp +1	//row increment
			);
	omap_set_dma_src_data_pack(dma_ch, 1);
	omap_set_dma_src_burst_mode(dma_ch, OMAP_DMA_DATA_BURST_16);

	//destination setting
	omap_set_dma_dest_params(dma_ch, 0,
			OMAP_DMA_AMODE_DOUBLE_IDX,	//2D. Tiler is not contiguous
			dst,						//address of target
			1,							//column increment
			dst_stride - width*bpp +1	//row increment
			);
	omap_set_dma_dest_data_pack(dma_ch, 1);
	omap_set_dma_dest_burst_mode(dma_ch, OMAP_DMA_DATA_BURST_16);

	//high priority
	omap_dma_set_prio_lch(dma_ch, DMA_CH_PRIO_HIGH, DMA_CH_PRIO_HIGH);
	//what's this????
//	omap_dma_set_global_params(DMA_DEFAULT_ARB_RATE, 0xFF, 0);
	//dma start
	omap_start_dma(dma_ch);

	if ( !wait_for_completion_timeout(&completion,  dma_wait_time) )
	{
		struct dsscomp_adapt_dma_wait_work *wk;
		pr_warning("DMA copy time out. queuing completion\n");
		wk = kzalloc(sizeof(*wk), GFP_NOWAIT);
		if ( wk==NULL )
		{
			pr_warning("kalloc fail. nothing to do..go ahead\n");
			r = -EIO;
			while (omap_get_dma_active_status(dma_ch))
				omap_stop_dma(dma_ch);
		}
		else
		{
			wk->dma_ch = dma_ch;
			wk->p_completion = &completion;
			INIT_WORK(&wk->work, dsscomp_adapt_dma_wait);
			pr_warning("Queueing DMA wait & tread DMA as processing in going\n");
			//adapt work and dam wait run on a same work thread.
			//And fail return immediately stop current adaptation.
			//These two can guarantee completion of DMA.
			//during waiting DMA, adaptation work will be skipped by incrementing in_progress_count
			atomic_inc(&mgr.in_progress_count);
			//wake lock may be needed.
			queue_work( mgr.workq, &wk->work);
			return -EIO;
		}
	}
	else
	omap_stop_dma(dma_ch);
	omap_free_dma(dma_ch);
	DBG_PRINTK("DMA Copy Done\n");
	return r;
}
/**
 * Request 2D DMA copy and waiting completion.
 * 2 buffer must have same type (means bbs is equal)
 * @param src :  //##hw2002.cho@lge.com
 */

static int dsscomp_adapt_black_fill(u32 dst, u32 dst_stride, int width, int height)
{
	int r = 0;
	int dma_ch;
//	int data_type;
	DECLARE_COMPLETION_ONSTACK(completion);

	if ( is_forces_stop() )
		return -EINTR;



	r = omap_request_dma
						(
						OMAP_DMA_NO_DEVICE, 
						"dsscomp_adapt",
						dsscomp_adapt_2d_dma_copy_cb, 
						&completion, &dma_ch
						);
	if ( r )
		return r;

	omap_set_dma_transfer_params
						(
						dma_ch, 
						//OMAP_DMA_DATA_TYPE_S32,
						OMAP_DMA_DATA_TYPE_S16,
						width, height,
						OMAP_DMA_SYNC_BLOCK,	//I don't know exact meaning of SYNC MOD
						OMAP_DMA_NO_DEVICE,
						OMAP_DMA_DST_SYNC
						);

#if 1
	omap_set_dma_dest_params(dma_ch, 0,
			OMAP_DMA_AMODE_DOUBLE_IDX,	//2D. Tiler is not contiguous
			dst,						//address of target
			1,							//column increment
			dst_stride - width*2 +1	//row increment
			);
#else
	omap_set_dma_dest_params
						(
						dma_ch, 
						0, 
						OMAP_DMA_AMODE_POST_INC,
						dst, 
						0, 
						0
						);
#endif //##
	omap_set_dma_color_mode(dma_ch, OMAP_DMA_CONSTANT_FILL, 0x00800080);


	omap_start_dma(dma_ch);

	if ( !wait_for_completion_timeout(&completion,  msecs_to_jiffies(1000/30)) )
	{
		pr_warning("DMA fill elapsed 1/30 second\n");
		r = -EIO;
		while (omap_get_dma_active_status(dma_ch))
			omap_stop_dma(dma_ch);
	}
	else
		omap_stop_dma(dma_ch);
	
	omap_free_dma(dma_ch);
	
	DBG_PRINTK("DMA Copy Done\n");
	
	return r;
}


static void dsscomp_adapt_init_deptharea( int depth , struct adapt_lambda_opd_t *black_backup)
{
	int r;
	int s3_depth_abs = depth > 0? depth : -1*depth;

	if(depth == 0 || black_backup == NULL)
	{
		return ;
	}	
		
	r = dsscomp_adapt_black_fill
	(
	//black_addr + opd->crop.x*bpp + s3_stride, 
	black_backup->paddr + black_backup->crop.x*black_backup->bpp + black_backup->crop.y*black_backup->phy_stride,
	black_backup->phy_stride,
	black_backup->buffer_width*2, 
	s3_depth_abs*2
	);
	if ( r )
	{
		pr_warning("black fill failed\n");
	}
	
	
	r = dsscomp_adapt_black_fill
	(
	//black_addr + opd->crop.x*bpp + s3_stride, 
	black_backup->paddr + black_backup->crop.x*black_backup->bpp + black_backup->crop.y*black_backup->phy_stride+
	black_backup->phy_stride*((black_backup->crop.h)-s3_depth_abs*2),
	black_backup->phy_stride,
	black_backup->buffer_width*2, 
	s3_depth_abs*2
	);
	if ( r )
	{
		pr_warning("black fill2 failed\n");
	}
	
	DBG_PRINTK("black_addr:0x%x(%d), {%d,%d}: [%d,%d]->[%d,%d] \n", 
										   black_backup->paddr, black_backup->phy_stride, 
										   black_backup->buffer_width, black_backup->buffer_height,
										   black_backup->crop.x, black_backup->crop.y, black_backup->crop.w, black_backup->crop.h );
	
		   
}


/**
 * Free lambda list
 */
static void dsccomp_adapt_free_lambda_list(struct lambda_list_t* list)
{
	while(list!=NULL)
	{
		struct lambda_list_t *to_del = list;
		if ( list->src!=NULL )
			vfree(list->src);
		if ( list->dst!=NULL )
			vfree(list->dst);
		list = list->next;
		vfree(to_del);
	}
}


/**
 * Fill cropped overlay info from opd.
 * reference set_dss_ovl_info() in base.c
 */
static int dsscomp_adapt_fill_ovlinfo_tiler(struct omap_overlay_info *info, struct adapt_lambda_opd_t *opd, struct dss2_rect_t *crop, u8 rotation)
{
	if ( info==NULL || opd==NULL || crop==NULL )
		return -EINVAL;
	//must be tiler address.
	if ( opd->paddr<0x60000000 || opd->paddr >= 0x78000000 )
	{
		DBG_PRINTK("dsscomp_adapt_fill_ovlinfo_tiler()'s addr(0x%x) is not tiler\n", opd->paddr);
		return -EINVAL;
	}

	info->zorder = 0;
	info->global_alpha = 0;
	info->rotation = rotation;
	info->mirror = 0;
	info->color_mode = opd->color_mode;
	if ( rotation & 1 )
	{
		//rotation 90 or 270, swap width & height
		info->width = crop->h;
		info->height = crop->w;
	}
	else
	{
		info->width = crop->w;
		info->height = crop->h;
	}
	//output info(pos_x/y, out_width.height) will not be set in this function
	info->paddr = opd->paddr;
	info->p_uv_addr = opd->uv_paddr;

	//cropping in tiler
	{
		int bpp = 1 << ((info->paddr >> 27) & 3);
		struct tiler_view_t t;

		if (opd->color_mode &
				(OMAP_DSS_COLOR_YUV2 | OMAP_DSS_COLOR_UYVY))
			bpp = 2;
		else if (opd->color_mode == OMAP_DSS_COLOR_RGB24P)
			bpp = 3;

		tilview_create(&t, info->paddr, opd->buffer_width, opd->buffer_height);
		info->paddr -= t.tsptr;
		tilview_crop(&t, 0, crop->y, crop->w, crop->h);
		info->paddr += t.tsptr + bpp * crop->x;

		info->rotation_type = OMAP_DSS_ROT_TILER;
		info->screen_width = 0;

		/* for NV12 format also crop NV12 */
		if (info->color_mode == OMAP_DSS_COLOR_NV12) 
		{
		   const struct omap_dss_cconv_coefs ctbl_bt601_5 = {298,  409,    0,  298, -208, -100,  298,    0,  517, 0, };
			
			tilview_create(&t, info->p_uv_addr,	opd->buffer_width >> 1, opd->buffer_height >> 1);
			info->p_uv_addr -= t.tsptr;
			tilview_crop(&t, 0, crop->y >> 1, crop->w >> 1,
								crop->h >> 1);
			info->p_uv_addr += t.tsptr + bpp * crop->x;
			info->cconv = ctbl_bt601_5;
		}
	}
	info->max_x_decim = 255;
	info->max_y_decim = 255;
	info->min_x_decim = 1;
	info->min_y_decim = 1;
	DBG_PRINTK("OVL Cropping for rotated IL : paddr(0x%x) uvaddr(0x%x) crop(%d,%d,%d,%d) rotation(%d)=> paddr(0x%x) uvaddr(0x%x) size(%d,%d)\n",
			opd->paddr, opd->uv_paddr, crop->x, crop->y, crop->w, crop->h, rotation, info->paddr, info->p_uv_addr, info->width, info->height);
	return 0;
}

extern int omap_dss_m2m_wb_apply(struct omap_overlay *ovl, struct omap_writeback *wb, int row_inc);
extern int dispc_runtime_get(void);
extern int dispc_runtime_put(void);

#define WB_ISR_IRQ_MASK (0xffffffff)
//#define WB_ISR_IRQ_MAKS	(DISPC_IRQ_FRAMEDONE_WB)
/**
 * ISR for wb done
 */
static void wb_done_isr(void *arg, u32 irqstatus)
{
	struct completion *wb_done = (struct completion*)arg;
	if ( irqstatus & DISPC_IRQ_FRAMEDONE_WB )
	{
		omap_dispc_unregister_isr_nosync(wb_done_isr, arg, WB_ISR_IRQ_MASK);
		if ( wb_done!=NULL )
			complete(wb_done);
	}
//	DBG_PRINTK("WB ISR 0x%x\n", irqstatus);
}

static int dsscomp_adapt_m2m_wb(struct omap_overlay *ovl, struct omap_writeback *wb, int row_inc, bool wait_completion)
{
	int r = 0;
	if ( ovl==NULL || wb==NULL )
		return -EINVAL;
	DBG_PRINTK("Set m2m WB & trigger.\n");
	r = dispc_runtime_get();
	if ( r )
	{
		pr_warning("dispc_rutime_get() failed\n");
		return r;
	}
	if ( wait_completion )
	{
		struct completion wb_done;
		if ( is_forces_stop() )
			return -EINTR;
		init_completion(&wb_done);
		r = omap_dispc_register_isr(wb_done_isr, &wb_done, WB_ISR_IRQ_MASK);
		if ( r )
		{
			pr_warning("WB ISR registration failed\n");
			goto Done;
		}
		r = omap_dss_m2m_wb_apply(ovl, wb, row_inc);
		if ( r )
		{
			omap_dispc_unregister_isr_nosync(wb_done_isr,  &wb_done, WB_ISR_IRQ_MASK);
			pr_warning("WB apply failed\n");
			goto Done;
		}
		DBG_PRINTK("Waiting WB Done\n");
//		wait_for_completion(&wb_done);
		if ( !wait_for_completion_interruptible_timeout(&wb_done,  msecs_to_jiffies(1000/30)) )
		{
			omap_dispc_unregister_isr_nosync(wb_done_isr,  &wb_done, WB_ISR_IRQ_MASK);
			pr_warning("Waiting WB failed\n");
		}
		else
			DBG_PRINTK("WB Done\n");
	}
	else
	{
		//disable apply
		r = omap_dss_m2m_wb_apply(ovl, wb, row_inc);
	}
Done:
	dispc_runtime_put();
	return r;
}


////////////////////////////////////////////////////////////////////////////////////////////
//	Rotated Side By Side to Rotated Interleaved
//

struct lambda_rsbs_to_il_t {
	struct lambda_list_t lambda;
	bool	left_first;
};

#ifdef CONFIG_HRZ_II
extern int hrz_res_conv_mode;
#endif

/**
 * Converting rotated Side-By-Side format to InerLeave.
 */
static int lambda_rsbs_to_il_op(struct lambda_list_t *lambda)
{
	int r;
	bool left_first;
	struct adapt_lambda_opd_t *src, *dst;

	u32 src_addr, dst_addr;
	int copy_w, copy_h;
	int src_stride, dst_stride;
	int bpp;

	if ( lambda==NULL )
		return -EINVAL;

	src = lambda->src;
	dst = lambda->dst;

	if ( src->bpp!=dst->bpp)
		return -EINVAL;
	if ( src->crop.w!=dst->crop.w )
		return -EINVAL;
	if ( src->crop.h!=dst->crop.h )
		return -EINVAL;

	left_first = ((struct lambda_rsbs_to_il_t*)lambda)->left_first;

	bpp = src->bpp;

	src_stride = src->phy_stride;
	dst_stride = dst->phy_stride * 2;

	copy_w = src->crop.w;
	copy_h = src->crop.h / 2;

		if ( left_first )
		{
			//left -> even copy
		src_addr = src->paddr + src->crop.x*bpp;
		dst_addr = dst->paddr + dst->crop.x*bpp;
#ifdef CONFIG_HRZ_II
		if(hrz_res_conv_mode) {
			r = dsscomp_adapt_2d_dma_copy(src_addr, src_stride*2,
					dst_addr, dst_stride*2,
					bpp, copy_w, copy_h/2);
		} else {
			r = dsscomp_adapt_2d_dma_copy(src_addr, src_stride,
					dst_addr, dst_stride,
					bpp, copy_w/2, copy_h/2);
		}
		if ( r )
			return r;
#else
		r = dsscomp_adapt_2d_dma_copy(src_addr, src_stride,
				dst_addr, dst_stride,
				bpp, copy_w, copy_h);
		if ( r )
			return r;
#endif

#ifdef CONFIG_HRZ_II
		if(hrz_res_conv_mode) {
			//right -> odd copy
			src_addr = src->paddr + src->phy_stride*src->crop.h/2 + src->crop.x*bpp;
			dst_addr = dst->paddr + 2*dst->phy_stride +  dst->crop.x*bpp;
			r = dsscomp_adapt_2d_dma_copy(src_addr, src_stride*2,
					dst_addr, dst_stride*2,
					bpp, copy_w, copy_h/2);
		} else {
			//right -> odd copy
			src_addr = src->paddr + src->phy_stride*src->crop.h/4 + src->crop.x*bpp;
			dst_addr = dst->paddr + dst->phy_stride + dst->crop.x*bpp;
			r = dsscomp_adapt_2d_dma_copy(src_addr, src_stride,
					dst_addr, dst_stride,
					bpp, copy_w/2, copy_h/2);
		}
		if ( r )
			return r;
#else
		//right -> odd copy
		src_addr = src->paddr + src->phy_stride*src->crop.h/2 + src->crop.x*bpp;
		dst_addr = dst->paddr + dst->phy_stride + dst->crop.x*bpp;
		r = dsscomp_adapt_2d_dma_copy(src_addr, src_stride,
				dst_addr, dst_stride,
				bpp, copy_w, copy_h);
		if ( r )
			return r;
#endif
	}
	else
	{
#ifdef CONFIG_HRZ_II
		if(hrz_res_conv_mode) {
			//left -> odd copy
			src_addr = src->paddr + src->crop.x*bpp;
			dst_addr = dst->paddr + dst->phy_stride*2 + dst->crop.x*bpp;
			r = dsscomp_adapt_2d_dma_copy(src_addr, src_stride*2,
					dst_addr, dst_stride*2,
					bpp, copy_w, copy_h/2);
		} else {
			//left -> odd copy
			src_addr = src->paddr + src->crop.x*bpp;
			dst_addr = dst->paddr + dst->phy_stride + dst->crop.x*bpp;
			r = dsscomp_adapt_2d_dma_copy(src_addr, src_stride,
					dst_addr, dst_stride,
					bpp, copy_w/2, copy_h/2);
		}
		if ( r )
			return r;
#else
		//left -> odd copy
		src_addr = src->paddr + src->crop.x*bpp;
		dst_addr = dst->paddr + dst->phy_stride + dst->crop.x*bpp;
		r = dsscomp_adapt_2d_dma_copy(src_addr, src_stride,
				dst_addr, dst_stride,
				bpp, copy_w, copy_h);
		if ( r )
			return r;
#endif

#ifdef CONFIG_HRZ_II
		if(hrz_res_conv_mode) {
			//right -> even copy
			src_addr = src->paddr + src->phy_stride*src->crop.h/2 + src->crop.x*bpp;
			dst_addr = dst->paddr + dst->crop.x*bpp ;
			r = dsscomp_adapt_2d_dma_copy(src_addr, src_stride*2,
					dst_addr, dst_stride*2,
					bpp, copy_w, copy_h/2);
		} else {
			//right -> even copy
			src_addr = src->paddr + src->phy_stride*src->crop.h/4 + src->crop.x*bpp;
			dst_addr = dst->paddr + dst->crop.x*bpp;
			r = dsscomp_adapt_2d_dma_copy(src_addr, src_stride,
					dst_addr, dst_stride,
					bpp, copy_w/2, copy_h/2);
		}
		if ( r )
			return r;
#else
		//right -> even copy
		src_addr = src->paddr + src->phy_stride*src->crop.h/2 + src->crop.x*bpp;
		dst_addr = dst->paddr + dst->crop.x*bpp;
		r = dsscomp_adapt_2d_dma_copy(src_addr, src_stride,
				dst_addr, dst_stride,
				bpp, copy_w, copy_h);
		if ( r )
			return r;
#endif
	}

	return 0;
}

/**
 * Making Lambda converting rotated Side-By-Side format to InterLeaving
 */
static struct lambda_list_t* lambda_rsbs_to_il_factory
		(struct dss2_ovl_info *oi, struct adapt_buf_list_t **ret_buf_list,
		 struct dss2_mgr_info *mgr)
{
	struct adapt_lambda_opd_t *src=NULL, *dst=NULL;
	struct adapt_buf_t *dst_buf;
	struct lambda_rsbs_to_il_t* lambda_IL=NULL;
	struct adapt_buf_list_t *buf_list=NULL;

	if ( oi==NULL || ret_buf_list==NULL )
		return NULL;

	DBG_PRINTK("Making same size interleaving lambda\n");

	//preparing source operand (may be frame buffer)
	DBG_PRINTK("Preparing source operand\n");
	src = dsscomp_adapt_opd_from_ovl(oi);
	if ( src==NULL )
		goto Failed;

	//preparing target operand (vram buffer alloc)
	DBG_PRINTK("Preparing destination operand\n");
	//alloc buffer
#ifdef CONFIG_COSMO_ICS_MEM_OPT
	dst_buf = dsscomp_adapt_mgr_get_buf_from_ovl(oi, TILER_1D_DYNAMIC_TYPE);
#else
	dst_buf = dsscomp_adapt_mgr_get_buf_from_ovl(oi, VRAM_TYPE);
#endif
	if ( dst_buf==NULL )
	{
		pr_warning("Alloc buffer for target operand failed\n");
		goto Failed;
	}
	DBG_PRINTK("Getting VRAM buffer success (%p)\n", dst_buf);
	//to maintain mgr buffer, push
	buf_list = dsscomp_adapt_push_to_buffer_list(NULL, dst_buf);
	if ( buf_list==NULL )
	{
		pr_warning("Alloc mgr buffer list failed\n");
		dsscomp_adapt_mgr_put_buf(dst_buf);
		dst_buf = NULL;
		goto Failed;
	}
	DBG_PRINTK("Getting buffer list success(%p)\n", buf_list);
	//making operand from buffer
	dst = dsscomp_adapt_opd_from_buf(dst_buf);
	if ( dst==NULL )
		goto Failed;

	//making & fill lambda
	lambda_IL = (struct lambda_rsbs_to_il_t*)vmalloc(sizeof(*lambda_IL));
	if ( lambda_IL==NULL )
	{
		pr_warning("Alloc lambda list failed\n");
		goto Failed;
	}
	lambda_IL->lambda.next	= NULL;
	lambda_IL->lambda.opt	= lambda_rsbs_to_il_op;
	lambda_IL->lambda.description = "Converting Side-By-Side to Interleaved";
	lambda_IL->lambda.src	= src;
	lambda_IL->lambda.dst	= dst;

	if ( mgr->s3d_disp_info.order==S3D_DISP_ORDER_R)
	{
		if ( oi->cfg.s3d_rotation==1 )	//90
			lambda_IL->left_first = false;
		else
			lambda_IL->left_first = true;
	}
	else
	{
		if ( oi->cfg.s3d_rotation==1 )	//90
			lambda_IL->left_first = true;
		else
			lambda_IL->left_first = false;
	}

	//change oi info
	DBG_PRINTK("oi->ba 0x%x->0x%x\n",oi->ba, dst->paddr);
	DBG_PRINTK("oi->cfg.stride %d->%d\n",oi->cfg.stride, dst->phy_stride);
	oi->ba = dst->paddr;
	oi->cfg.stride = dst->phy_stride;

	*ret_buf_list = buf_list;
	DBG_PRINTK("'%s' lambda creation success\n", lambda_IL->lambda.description);
	return (struct lambda_list_t*)lambda_IL;
Failed:
	DBG_PRINTK("Making same size interleaving lambda failed\n");
	if ( src!=NULL )
		vfree(src);
	if ( dst!=NULL )
		vfree(dst);
	if ( lambda_IL!=NULL )
		vfree(lambda_IL);
	if ( buf_list!=NULL )
		dsscomp_adapt_free_buffer_list(buf_list);
	return NULL;
}


//////////////////////////////////////////////////////////////////////////////
//	Copy to Tiler buffer for rotation


static int lambda_to_tiler_op(struct lambda_list_t *lambda)
{
	int r;
	int bpp;
	int w, h;
	struct adapt_lambda_opd_t *src, *dst;

	if ( lambda==NULL )
		return -EINVAL;

	src = lambda->src;
	dst = lambda->dst;

	if ( src->bpp!=dst->bpp)
		return -EINVAL;
	if ( src->crop.w!=dst->crop.w )
		return -EINVAL;
	if ( src->crop.h!=dst->crop.h )
		return -EINVAL;

	bpp = src->bpp;
	w = src->crop.w;
	h = src->crop.h;
#ifdef CONFIG_HRZ_II
	if(hrz_res_conv_mode) {
		r = dsscomp_adapt_2d_dma_copy(src->paddr + src->crop.x*bpp, src->phy_stride,
				dst->paddr + dst->crop.x*bpp, dst->phy_stride,
				bpp, w, h);
	} else {
            if(lambda->rotation == OMAP_DSS_ROT_270)
				r = dsscomp_adapt_2d_dma_copy(src->paddr + src->crop.x*bpp, src->phy_stride,
					dst->paddr + dst->crop.x*bpp + 480*4, dst->phy_stride,
					bpp, w/2, h/2);
			else
				r = dsscomp_adapt_2d_dma_copy(src->paddr + src->crop.x*bpp, src->phy_stride,
					dst->paddr + dst->crop.x*bpp + 800*dst->phy_stride, dst->phy_stride,
					bpp, w/2, h/2);
	}
#else
	r = dsscomp_adapt_2d_dma_copy(src->paddr + src->crop.x*bpp, src->phy_stride,
			dst->paddr + dst->crop.x*bpp, dst->phy_stride,
			bpp, w, h);
#endif
	return r;
}


/**
 * Making lambda allocating tiler and copy data to it
 */
static struct lambda_list_t* lambda_to_tiler_factory
		(struct dss2_ovl_info *oi, struct adapt_buf_list_t **ret_buf_list)
{
	struct adapt_lambda_opd_t *src=NULL, *dst=NULL;
	struct adapt_buf_t *dst_buf;
	struct adapt_buf_list_t *buf_list=NULL;
	struct lambda_list_t *lambda_tiler = NULL;

	if ( oi==NULL || ret_buf_list==NULL )
		return NULL;
	DBG_PRINTK("Making tiler alloc & copy lambda\n");
	src = dsscomp_adapt_opd_from_ovl(oi);
	if ( src==NULL )
		goto Failed;

	dst_buf = dsscomp_adapt_mgr_get_buf_from_ovl(oi, TILER_2D_TYPE);
	if ( dst_buf==NULL )
	{
		pr_warning("Alloc buffer for target operand failed\n");
		goto Failed;
	}
	DBG_PRINTK("Getting TILER buffer success (%p)\n", dst_buf);
	//to maintain mgr buffer, push
	buf_list = dsscomp_adapt_push_to_buffer_list(NULL, dst_buf);
	if ( buf_list==NULL )
	{
		pr_warning("Alloc mgr buffer list failed\n");
		dsscomp_adapt_mgr_put_buf(dst_buf);
		dst_buf = NULL;
		goto Failed;
	}
	DBG_PRINTK("Getting buffer list success(%p)\n", buf_list);
	//making operand from buffer
	dst = dsscomp_adapt_opd_from_buf(dst_buf);
	if ( dst==NULL )
		goto Failed;

	lambda_tiler = (struct lambda_list_t *)vmalloc(sizeof(*lambda_tiler));
	if ( lambda_tiler==NULL )
		goto Failed;
	lambda_tiler->next = NULL;
	lambda_tiler->description = "Copy to tiler for rotation";
	lambda_tiler->opt = lambda_to_tiler_op;
	lambda_tiler->src = src;
	lambda_tiler->dst = dst;
#ifdef CONFIG_HRZ_II
    lambda_tiler->rotation = oi->cfg.rotation;
#endif

	//change oi info
	DBG_PRINTK("oi->ba 0x%x->0x%x\n",oi->ba, dst->paddr);
	DBG_PRINTK("oi->cfg.stride %d->%d\n",oi->cfg.stride, dst->phy_stride);
	oi->ba = dst->paddr;
	oi->cfg.stride = dst->phy_stride;

	*ret_buf_list = buf_list;
	DBG_PRINTK("'%s' lambda creation success\n", lambda_tiler->description);
	return lambda_tiler;
Failed:
	DBG_PRINTK("Making tiler alloc & copy lambda failed\n");
	if ( src!=NULL )
		vfree(src);
	if ( dst!=NULL )
		vfree(dst);
	if ( buf_list!=NULL )
		dsscomp_adapt_free_buffer_list(buf_list);
	return NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// S3D to rotated interleaved
//

//Lambda converting S3D to Rotated Interleaved(row interleaved)
struct lambda_s3d_to_ril_t
{
	struct lambda_list_t lambda;
	enum s3d_layout_type		s3d_type;
	enum s3d_layout_order		s3d_order;
	bool	left_to_even;
	__u8	rotation;
};


/**
 * Fill WB info
 */
static int lambda_s3d_ro_ril_fill_wbinfo
								(
								struct adapt_lambda_opd_t *opd, 
								enum omap_writeback_source source,
								int depth,
								struct omap_writeback_info *info_even, 
								struct omap_writeback_info *info_odd, 
								u32 *stride
								)
{
	int 	s3_depth, s3_depth_abs;
	u32 	black_addr_backup, black_addr;
	u32		s3_stride, src_stride;
	int 	height;
	int 	i;
	int 	bpp;
	int 	r;
	
	if ( info_even==NULL || info_odd==NULL || opd==NULL || opd->uv_paddr!=0 )
		return -EINVAL;
	if ( opd->crop.x!=0 || opd->crop.y!=0 )
	{
		pr_warning("WB's crop should be (0,0) current value(%d,%d)\n", opd->crop.x, opd->crop.y);
		return -EINVAL;
	}
	
	s3_depth = depth*2;  //##hw2002.cho@lge.com

	if (s3_depth > 0)
	{
		s3_depth = (((opd->crop.h/2 - 1) > s3_depth) ? s3_depth : (opd->crop.h/2 - 1)) & ~(0x1);	
	}
	else
	{
		s3_depth = (((opd->crop.h/2 - 1) > (s3_depth*-1)) ? (s3_depth*-1) : (opd->crop.h/2 - 1)) & ~(0x1);	
		s3_depth = -1 * s3_depth;
	}
	
	s3_stride = opd->phy_stride;//mo2seongjae.jang 20120702 WBT issue modification	

	if(s3_stride > LCD_WIDTH*2 || s3_depth > LCD_WIDTH/4 )
	{
		printk("[S3D] ERR s3_stride = %d s3_depth = %dcroot\n", s3_stride, s3_depth);
		return -EINVAL;
	}
	//initilize depth area 
	dsscomp_adapt_init_deptharea(s3_depth,opd);
	
	if ( is_forces_stop() )
		return -EINTR;
	info_even->source = source;
	info_even->capturemode = OMAP_WB_CAPTURE_ALL;
	info_even->mode = OMAP_WB_MEM2MEM_MODE;
	info_even->dss_mode = opd->color_mode;
	//divide (W,H) buffer to two (W,H/2) buffers
	info_even->width = info_even->out_width = opd->crop.w;
	//output width of WB must be even(in rotated case, height)
	info_even->height = info_even->out_height = (opd->crop.h / 2) & ~(0x1);
	info_even->rotation = 0;	//no rotation
	info_even->p_uv_addr = 0;
	if (opd->paddr >= 0x60000000 && opd->paddr < 0x78000000)
		info_even->rotation_type = OMAP_DSS_ROT_TILER;
	else
		info_even->rotation_type = OMAP_DSS_ROT_DMA;
	//copy
	*info_odd = *info_even;

	//even start address
	info_even->paddr = opd->paddr;

	//odd start address
	info_odd->paddr = opd->paddr + opd->phy_stride;

	//stride
	*stride = opd->phy_stride;
	s3_stride = opd->phy_stride;
	s3_depth_abs = s3_depth > 0? s3_depth : -1*s3_depth;

	if(s3_depth <= 0)		//##hw2002.cho@lge.com
	{
		info_even->paddr += s3_stride*s3_depth_abs;
		info_even->out_height -= s3_depth_abs;
		info_even->height -= s3_depth_abs;
		info_odd->out_height -= s3_depth_abs; 
		info_odd->height -= s3_depth_abs; 
	}
	else
	{
		info_odd->paddr += s3_stride*s3_depth_abs;
		info_odd->out_height -= s3_depth_abs;
		info_odd->height -= s3_depth_abs;
		info_even->out_height -= s3_depth_abs;
		info_even->height -= s3_depth_abs;
	}

	DBG_PRINTK("WB Info filled : mode:%d color:0x%x, width:%d, height:%d, rotation:%d, rotation_type:%d, stride:%d, even_paddr:0x%x, odd_paddr:0x%x\n",
			info_even->mode, info_even->dss_mode, info_even->width, info_even->height, info_even->rotation, info_even->rotation_type,
			*stride,
			info_even->paddr, info_odd->paddr);

	return 0;
}

extern int dsscomp_queue_is_free_overlay(u32 ix);
extern void dispc_set_channel_out(enum omap_plane plane,	enum omap_channel channel);

/**
 * Rotating & Interleave conversion of s3d
 */
static int lambda_s3d_to_ril_op(struct lambda_list_t *lambda)
{
	int r;
	struct omap_overlay *wb_src = NULL;
	struct omap_writeback *wb;
	struct adapt_lambda_opd_t *src, *dst;
	struct lambda_s3d_to_ril_t *lambda_s3d
			= (struct lambda_s3d_to_ril_t *)lambda;
	struct omap_overlay_info infoL, infoR;
	struct dss2_rect_t crop_left, crop_right;
	struct omap_writeback_info wb_info_even, wb_info_odd;
	int		wb_stride;
	int 	depth;
//	struct adapt_lambda_opd_t black_backup;
	
	//backup & restore overlay
	struct omap_overlay_info info_backup;
	enum omap_channel ch_backup;

	if ( lambda==NULL )
		return -EINVAL;
	src = lambda->src;
	dst = lambda->dst;
#if 0
	{
		static int test_count = 0;
		test_count++;
		if ( (test_count%33)==0 )
		{
			printk("Test purpose sleep\n");
			ssleep(10);
		}
	}
#endif

//	memcpy(&black_backup ,dst,sizeof(black_backup));

	depth = lambda_s3d->lambda.depth;  //##hw2002.cho@lge.com
	//get free overlay
	//this will be called after setting of all overlay in this composition
	//so there is no conflict problem. I wish
	{
		int ix;
		for(ix=omap_dss_get_num_overlays()-1;ix>=0;ix--)
		{
			if ( dsscomp_queue_is_free_overlay(ix) )
			{
				wb_src = omap_dss_get_overlay(ix);
				break;
			}
			else
			{
				DBG_PRINTK("ovl (%d) is not free\n", ix);
			}
		}
		//@todo waiting until free
		if ( wb_src==NULL )
		{
			DBG_PRINTK("Getting Free overlay failed\n");
			return -EBUSY;
		}
	}
	DBG_PRINTK("Getting Free overlay success(%d)\n", wb_src->id);
	//save current status for restoring wb_src
	wb_src->get_overlay_info(wb_src, &info_backup);
	if ( info_backup.enabled )
	{
		pr_warning("Free overlay is enabled\n");
		return -EINVAL;
	}
	//unset manager for wb use
	if ( wb_src->manager!=NULL )
	{
		r = wb_src->unset_manager(wb_src);
		if ( r )
		{
			pr_warning("Overlay manager unset failed\n");
			return r;
		}
	}

	//WB out setting
	wb = omap_dss_get_wb(0);
	if ( wb==NULL )
	{
		DBG_PRINTK("Getting WB failed\n");
		return -EBUSY;

	}
	DBG_PRINTK("Getting WB success\n");
	
	r = lambda_s3d_ro_ril_fill_wbinfo
								(
								dst, 
								wb_src->id + OMAP_WB_GFX, 
								depth,				
								&wb_info_even, 
								&wb_info_odd, 
								&wb_stride
								);

	if ( r )
		return r;
	DBG_PRINTK("WB info filled\n");

	//WB Input Setting


	ch_backup = wb_src->manager->id;
	infoR = infoL = info_backup;

	//cut out Left & Right
	switch ( lambda_s3d->s3d_type )
	{
	case S3D_SIDE_BY_SIDE:
		crop_left.w	= crop_right.w	= src->crop.w / 2;
		crop_left.h	= crop_right.h	= src->crop.h;
		crop_left.y	= crop_right.y	= src->crop.y;
		if ( lambda_s3d->s3d_order==S3D_DISP_ORDER_L )
		{
			crop_left.x		= src->crop.x;
			crop_right.x	= crop_left.x  + crop_left.w;
		}
		else
		{
			crop_right.x	= src->crop.x;
			crop_left.x		= crop_right.x + crop_right.w;
		}
		break;
	case S3D_TOP_BOTTOM:
		crop_left.w	= crop_right.w	= src->crop.w;
		crop_left.h	= crop_right.h	= src->crop.h / 2;
		crop_left.x	= crop_right.x	= src->crop.x;
		if ( lambda_s3d->s3d_order==S3D_DISP_ORDER_L )
		{
			crop_left.y		= src->crop.y;
			crop_right.y	= crop_left.y + crop_left.h;
		}
		else
		{
			crop_right.y	= src->crop.y;
			crop_left.y		= crop_right.y + crop_right.h;
		}
		break;
	default:
		return -EINVAL;
	}

	DBG_PRINTK("Src(%d,%d)+(%d,%d)'%s, %s first' => Left(%d,%d)+(%d,%d) Right(%d,%d)+(%d,%d)\n",
			src->crop.x, src->crop.y, src->crop.w, src->crop.h,
			lambda_s3d->s3d_type==S3D_SIDE_BY_SIDE ? "SBS" : "TB",
			lambda_s3d->s3d_order==S3D_DISP_ORDER_L ? "L" : "R",
			crop_left.x, crop_left.y, crop_left.w, crop_left.h,
			crop_right.x, crop_right.y, crop_right.w, crop_right.h);
	//setting overlay info for left & right
	r = dsscomp_adapt_fill_ovlinfo_tiler(&infoL, src, &crop_left, lambda_s3d->rotation);
	if ( r )
	{
		DBG_PRINTK("Left ovl info fill failed\n");
		return r;
	}
	r = dsscomp_adapt_fill_ovlinfo_tiler(&infoR, src, &crop_right, lambda_s3d->rotation);
	if ( r )
	{
		DBG_PRINTK("Right ovl info fill failed\n");
		return r;
	}
	DBG_PRINTK("Fill ovlinfo for left & right\n");


	
	//Rotating Left & Right image separately can make interleaved image.
	//For example, converting 4x3 side-by-side image to 3x4 interleaved image
	// Left  Right
	//  01    ab
	//  23    cd
	//  45    ef
	// rotation to 6x2. Left(0,0)~(1,3) Right(3,0)~(5,1)
	//  420eca
	//  531fdb
	// regarding 6x2 as 3x4 will be
	//  420
	//  eca
	//  531
	//  fdb
	//
	infoL.pos_x			= infoR.pos_x		= 0;
	infoL.pos_y			= infoR.pos_y		= 0;
	infoL.out_width		= infoR.out_width	= wb_info_even.width;
	infoL.out_height	= infoR.out_height	= wb_info_even.height;

	{
		struct omap_overlay_info *info_even, *info_odd;
		if ( lambda_s3d->left_to_even )
		{
			//left to even
			info_even	= &infoL;
			//right to odd
			info_odd	= &infoR;
		}
		else
		{
			//right to even
			info_even	= &infoR;
			//left to odd
			info_odd	= &infoL;
		}
		//WB for even line
		wb_info_even.enabled = true;
		wb->set_wb_info(wb, &wb_info_even);
		info_even->enabled = true;
		wb_src->set_overlay_info(wb_src, info_even);

		r = dsscomp_adapt_m2m_wb(wb_src, wb, wb_stride+1, true);	//row inc = stride + 1
		if ( r )
		{
			pr_warning("Even Line WB Apply failed(%d)\n", r);
			return r;
		}
		//WB for odd line
		wb_info_odd.enabled = true;
		wb->set_wb_info(wb, &wb_info_odd);
		info_odd->enabled = true;
		wb_src->set_overlay_info(wb_src, info_odd);
		r = dsscomp_adapt_m2m_wb(wb_src, wb, wb_stride+1, true);	//row inc = stride + 1
		if ( r )
		{
			pr_warning("Odd Line WB Apply failed(%d)\n", r);
			return r;
		}
	}
	//disable & cleanup
	wb_info_even.enabled = false;
	wb->set_wb_info(wb, &wb_info_even);
	//restore overlay info
	info_backup.enabled = false;
	wb_src->set_overlay_info(wb_src, &info_backup);
	//1 will row_inc in wb
	r = dsscomp_adapt_m2m_wb(wb_src, wb, 1, false);
	if ( r )
	{
		pr_warning("WB disable failed(%d)\n", r);
		return r;
	}

	return 0;
}

extern struct omap_overlay_manager *find_dss_mgr(int display_ix);
/**
 * Making lambda converting S3D to rotated interleaved format
 */
static struct lambda_list_t* lambda_s3d_to_ril_op_factory
	(struct dss2_ovl_info* oi, struct adapt_buf_list_t **ret_buf_list,
			struct dss2_mgr_info *mgr)
{
	struct adapt_lambda_opd_t *src=NULL, *dst=NULL;
	struct adapt_buf_t *dst_buf;
	struct adapt_buf_list_t *buf_list=NULL;
	struct lambda_s3d_to_ril_t *lambda_s3d=NULL;
	int out_w, out_h;

	DBG_PRINTK("Making lamba converting S3D to rotated interleave\n");

	//checking suitable display for s3d
	{
		struct dss2_rect_t win = oi->cfg.win;
		struct omap_overlay_manager *mgr_info;
		//minus position is invalid
		if ( win.x<0 || win.y<0 )
		{
			DBG_PRINTK("Window Position is out of screen(%d,%d), stop converting\n", win.x, win.y);
			return NULL;
		}
		mgr_info = find_dss_mgr(mgr->ix);
		if ( mgr_info==NULL || mgr_info->device==NULL )
		{
			DBG_PRINTK("Magaer's device is not connected, stop converting\n");
			return NULL;
		}
		else
		{
			int x_res, y_res;
			x_res = mgr_info->device->panel.timings.x_res;
			y_res = mgr_info->device->panel.timings.y_res;
			if ( (win.x+win.w) > x_res )
			{
				DBG_PRINTK("Window Position is out of scope x:%d, w:%d, resolution:%d, stop converting\n",
						win.x, win.w, x_res);
				return NULL;
			}
			if ( (win.y+win.h) > y_res )
			{
				DBG_PRINTK("Window Position is out of scope y:%d, h:%d, resolution:%d, stop converting\n",
						win.y, win.h, y_res);
				return NULL;
			}
		}
	}

	//preparing source operand (may be tiler nv12. video data)
	DBG_PRINTK("Preparing source operand\n");
	src = dsscomp_adapt_opd_from_ovl(oi);
	if ( src==NULL )
		goto Failed;

	out_w = oi->cfg.win.w;
	out_h = oi->cfg.win.h;
	//adjust to even size
	out_w = (out_w /2 )*2;
	out_h = (out_h /2 )*2;
#ifdef CONFIG_COSMO_ICS_MEM_OPT

	dst_buf = dsscomp_adapt_mgr_get_buf(TILER_1D_DYNAMIC_TYPE, 2, out_w, out_h);
#else
	dst_buf = dsscomp_adapt_mgr_get_buf(VRAM_TYPE, 2, out_w, out_h);
#endif
	if ( dst_buf==NULL )
	{
		pr_warning("Alloc buffer for target operand failed\n");
		goto Failed;
	}
	DBG_PRINTK("Getting VRAM buffer success(%p)\n", dst_buf);
	//push buffer
	buf_list = dsscomp_adapt_push_to_buffer_list(NULL, dst_buf);
	if ( buf_list==NULL )
	{
		pr_warning("Alloc mgr buffer list failed\n");
		dsscomp_adapt_mgr_put_buf(dst_buf);
		dst_buf = NULL;
		goto Failed;
	}
	DBG_PRINTK("Getting buffer list success(%p)\n", buf_list);
	//making operadn from buffer
	dst = dsscomp_adapt_opd_from_buf(dst_buf);
	dst->color_mode = OMAP_DSS_COLOR_UYVY;

	lambda_s3d = (struct lambda_s3d_to_ril_t*) vmalloc(sizeof(*lambda_s3d));
	if ( lambda_s3d==NULL )
	{
		pr_warning("Alloc lambda list failed\n");
		goto Failed;
	}

	DBG_PRINTK("S3D Input : %s, (%d,%d)+(%d,%d) color:0x%x ==> (%d,%d)+(%d,%d)\n",
			oi->cfg.s3d_input_layout_type==S3D_SIDE_BY_SIDE ? "SBS" : "TB",
			oi->cfg.crop.x, oi->cfg.crop.y, oi->cfg.crop.w, oi->cfg.crop.h,
			oi->cfg.color_mode,
			oi->cfg.win.x, oi->cfg.win.y, oi->cfg.win.w, oi->cfg.win.h);
	lambda_s3d->lambda.next	= NULL;
	lambda_s3d->lambda.opt	= lambda_s3d_to_ril_op;
	lambda_s3d->lambda.description = "Converting S3D to rotated and scaled interleaved";
	lambda_s3d->lambda.src	= src;
	lambda_s3d->lambda.dst	= dst;
	lambda_s3d->lambda.depth = mgr->s3d_disp_info.gap;	////##hw2002.cho@lge.com
	lambda_s3d->rotation	= oi->cfg.rotation;
	lambda_s3d->s3d_type	= oi->cfg.s3d_input_layout_type;
//	lambda_s3d->s3d_type	= S3D_TOP_BOTTOM;
	lambda_s3d->s3d_order	= oi->cfg.s3d_input_layout_order;

	if ( mgr->s3d_disp_info.order==S3D_DISP_ORDER_R)
	{
		if ( oi->cfg.rotation==1 )	//90
			lambda_s3d->left_to_even = false;
		else
			lambda_s3d->left_to_even = true;
	}
	else
	{
		if ( oi->cfg.rotation==1 )	//90
			lambda_s3d->left_to_even = true;
		else
			lambda_s3d->left_to_even = false;
	}
	//if first line odd then inverse
	if ( oi->cfg.win.x & 0x01 )
	{
		DBG_PRINTK("lambda_s3d_to_ril_op_factory first line odd then inverse \n");
	//	lambda_s3d->left_to_even = !lambda_s3d->left_to_even;
	}


	//changing OI
	DBG_PRINTK("oi->ba:0x%x->0x%x, oi->uv:0x%x->0x%x, oi->cfg.stride:%d->%d\n",
			oi->ba, dst->paddr, oi->uv, dst->uv_paddr, oi->cfg.stride, dst->phy_stride);
	oi->ba = dst->paddr;
	oi->uv = dst->uv_paddr;
//	oi->cfg.stride = dst->bpp * dst->crop.w;
	oi->cfg.stride = dst->phy_stride;
	DBG_PRINTK("oi->cfg.rotation:%d->%d, oi->cfg.mirror:%d->%d, oi->cfg.color_mode:0x%x->0x%x\n",
			oi->cfg.rotation, 0, oi->cfg.mirror, 0, oi->cfg.color_mode, dst->color_mode);
	oi->cfg.rotation = 0;
	oi->cfg.mirror = 0;
	oi->cfg.color_mode = dst->color_mode;
	DBG_PRINTK("oi->cfg.crop:(%d,%d,%d,%d)->(%d,%d,%d,%d)\n",
			oi->cfg.crop.x, oi->cfg.crop.y, oi->cfg.crop.w, oi->cfg.crop.h,
			0, 0, out_w, out_h);
	oi->cfg.crop.x = 0;
	oi->cfg.crop.y = 0;
	oi->cfg.crop.w = out_w;
	oi->cfg.crop.h = out_h;
	DBG_PRINTK("oi->cfg: width(%d->%d) height(%d->%d)\n",
			oi->cfg.width, dst->crop.w, oi->cfg.height, dst->crop.h);
	oi->cfg.width = dst->crop.w;
	oi->cfg.height = dst->crop.h;

	*ret_buf_list = buf_list;
	DBG_PRINTK("'%s' lambda creation success\n", lambda_s3d->lambda.description);
	return &(lambda_s3d->lambda);
Failed:
	DBG_PRINTK("Making lamba failed\n");
	if ( src!=NULL )
		vfree(src);
	if ( dst!=NULL )
		vfree(dst);
	if ( lambda_s3d!=NULL )
		vfree(lambda_s3d);
	if ( buf_list!=NULL )
		dsscomp_adapt_free_buffer_list(buf_list);
	return NULL;
}

/////////////////////////////////////////////////////////////////////////////////////////
//	Rotated Side-by-Side to Rotated Top Bottom
//   (no reverse)
//    LLLL
//    LLLL  ==> RRRRLLLL
//    RRRR      RRRRLLLL
//    RRRR
//
//   (reverse)
//    RRRR      RRRRLLLL
//    RRRR  ==> RRRRLLLL
//    LLLL
//    LLLL

struct lambda_rsbs_to_rtb_t
{
	struct lambda_list_t lambda;
	bool	reverse;
};

static int lambda_rsbs_to_rtb_op(struct lambda_list_t *lambda)
{
	int r;
	int bpp;
	int w, h;
	struct lambda_rsbs_to_rtb_t *lambda_rtb = (struct lambda_rsbs_to_rtb_t*)lambda;
	struct adapt_lambda_opd_t *src, *dst;
	u32	src_addr1, src_addr2, dst_addr1, dst_addr2;

	if ( lambda==NULL )
		return -EINVAL;

	src = lambda->src;
	dst = lambda->dst;

	if ( src->bpp!=dst->bpp)
		return -EINVAL;
	if ( (src->crop.w*2) != dst->crop.w )
		return -EINVAL;
	if ( (src->crop.h/2) != dst->crop.h )
		return -EINVAL;

	bpp = src->bpp;
	w = src->crop.w;
	h = src->crop.h / 2;

	//Rotated side by side.
	//So difference of start address of L/R is (src's h)/2 * stride
#ifdef CONFIG_HRZ_II
	src_addr1 = src->paddr;
	src_addr2 = src_addr1
			+ src->phy_stride * 400;
#else
	src_addr1 = src->paddr
			+ src->phy_stride * src->crop.y
			+ bpp * src->crop.x;
	src_addr2 = src_addr1
			+ src->phy_stride * h;
#endif

	//Rotated side by side.
	//So difference of start addfress of L/R is (src's w) * bpp
#ifdef CONFIG_HRZ_II
	dst_addr1 = dst->paddr;
	dst_addr2 = dst_addr1
			+ bpp * 480;
#else
	dst_addr1 = dst->paddr
			+ dst->phy_stride * dst->crop.y
			+ bpp * dst->crop.x;
	dst_addr2 = dst_addr1
			+ bpp * w;
#endif

	if ( lambda_rtb->reverse)	//swap
	{
		u32 temp;
		temp = src_addr1;
		src_addr1 = src_addr2;
		src_addr2 = temp;
	}

#ifdef CONFIG_HRZ_II
//DOLCOM tb
	//printk("DOLCOM : rtb %x %x %x %x %x\n", 
	//	src_addr1, src->phy_stride, dst_addr1, dst->phy_stride);
	w = 480;
	h = 800;
	r = dsscomp_adapt_2d_dma_copy(src_addr1, src->phy_stride,
			dst_addr1, dst->phy_stride,
			bpp, w, h/2);
	if ( r )
		return r;
	r = dsscomp_adapt_2d_dma_copy(src_addr2, src->phy_stride,
			dst_addr2, dst->phy_stride,
			bpp, w, h/2);
#else
	r = dsscomp_adapt_2d_dma_copy(src_addr1, src->phy_stride,
			dst_addr1, dst->phy_stride,
			bpp, w, h);
	if ( r )
		return r;
	r = dsscomp_adapt_2d_dma_copy(src_addr2, src->phy_stride,
			dst_addr2, dst->phy_stride,
			bpp, w, h);
#endif
	return r;
}

/**
 * Allocating tiler and making lambda copying in rotated top bottom
 */
static struct lambda_list_t* lambda_rsbs_to_rtb_factory
		(struct dss2_ovl_info *oi, struct adapt_buf_list_t **ret_buf_list, struct dss2_mgr_info *mgr)
{
	struct adapt_lambda_opd_t *src=NULL, *dst=NULL;
	struct adapt_buf_t *dst_buf;
	struct adapt_buf_list_t *buf_list=NULL;
	struct lambda_rsbs_to_rtb_t *lambda_rtb = NULL;

	if ( oi==NULL || ret_buf_list==NULL )
		return NULL;
	DBG_PRINTK("Making lambda converting rotated SbS to rotated TB\n");
	src = dsscomp_adapt_opd_from_ovl(oi);
	if ( src==NULL )
		goto Failed;

#ifdef CONFIG_HRZ_II
//DOLCOM tb
	dst_buf = dsscomp_adapt_mgr_get_buf(TILER_2D_TYPE, src->bpp, 480*2, 400);
	if ( dst_buf==NULL )
	{
		pr_warning("Alloc buffer for target operand failed\n");
		goto Failed;
	}
#else
	dst_buf = dsscomp_adapt_mgr_get_buf(TILER_2D_TYPE, src->bpp, src->crop.w * 2, src->crop.h /2);
	if ( dst_buf==NULL )
	{
		pr_warning("Alloc buffer for target operand failed\n");
		goto Failed;
	}
#endif
	DBG_PRINTK("Getting TILER buffer success (%p)\n", dst_buf);
	//to maintain mgr buffer, push
	buf_list = dsscomp_adapt_push_to_buffer_list(NULL, dst_buf);
	if ( buf_list==NULL )
	{
		pr_warning("Alloc mgr buffer list failed\n");
		dsscomp_adapt_mgr_put_buf(dst_buf);
		dst_buf = NULL;
		goto Failed;
	}
	DBG_PRINTK("Getting buffer list success(%p)\n", buf_list);
	//making operand from buffer
	dst = dsscomp_adapt_opd_from_buf(dst_buf);
	if ( dst==NULL )
		goto Failed;

	lambda_rtb = (struct lambda_rsbs_to_rtb_t *)vmalloc(sizeof(*lambda_rtb));
	if ( lambda_rtb==NULL )
		goto Failed;

#ifdef CONFIG_HRZ_II
	src->crop.x = 0;
	src->crop.y = 0;
	src->crop.w = 480;
	src->crop.h = 800;

	dst->crop.x = 0;
	dst->crop.y = 0;
	dst->crop.w = 480*2;
	dst->crop.h = 800/2;
#endif
	lambda_rtb->lambda.next = NULL;
	lambda_rtb->lambda.description = "Converting rotated side-by-side to rotated top-bottom";
	lambda_rtb->lambda.opt = lambda_rsbs_to_rtb_op;
	lambda_rtb->lambda.src = src;
	lambda_rtb->lambda.dst = dst;
	lambda_rtb->reverse = oi->cfg.s3d_input_layout_order != mgr->s3d_disp_info.order;
	DBG_PRINTK("Reverse:%d, input s3d order:%d, display s3d order:%d\n",
			lambda_rtb->reverse, oi->cfg.s3d_input_layout_order, mgr->s3d_disp_info.order);

	//change oi info
	DBG_PRINTK("oi->ba 0x%x->0x%x\n",oi->ba, dst->paddr);
	DBG_PRINTK("oi->cfg.stride %d->%d\n",oi->cfg.stride, dst->phy_stride);
	oi->ba = dst->paddr;
	oi->cfg.stride = dst->phy_stride;
	DBG_PRINTK("oi->cfg.crop (%d,%d)+(%d,%d) ==> (%d,%d)+(%d,%d)\n",
			oi->cfg.crop.x, oi->cfg.crop.y, oi->cfg.crop.w, oi->cfg.crop.h,
			dst->crop.x, dst->crop.y, dst->crop.w, dst->crop.h);

#ifdef CONFIG_HRZ_II
	oi->cfg.crop.w = 480*2;
	oi->cfg.crop.h = 800/2;
	
	oi->cfg.win.w = oi->cfg.win.w / 2;
	oi->cfg.win.h = oi->cfg.win.h / 2;
	
	oi->cfg.win.x = 0;
	oi->cfg.win.y = 0;
#else
	oi->cfg.crop = dst->crop;
#endif

	*ret_buf_list = buf_list;
	DBG_PRINTK("'%s' lambda creation success\n", lambda_rtb->lambda.description);
	return &lambda_rtb->lambda;
Failed:
	DBG_PRINTK("Making lambda converting rotated sbs to rotated failed\n");
	if ( src!=NULL )
		vfree(src);
	if ( dst!=NULL )
		vfree(dst);
	if ( buf_list!=NULL )
		dsscomp_adapt_free_buffer_list(buf_list);
	return NULL;
}
/////////////////////////////////////////////////////////////////////////////////////////
//	DSSCOMP ADAPT Processing functions
//

static void dsscomp_adapt_dump_info_buf_list(struct adapt_buf_list_t *buf_list)
{
	DBG_PRINTK("Dumping Buffer List\n");
	while(buf_list!=NULL)
	{
		DBG_PRINTK("Buffer %p\n", buf_list->buf);
		buf_list = buf_list->next;
	}
}

static void dsscomp_adapt_dump_info_lambda_list(struct lambda_list_t *list)
{
	DBG_PRINTK("Dumping lambda list\n");
	while(list!=NULL)
	{
		DBG_PRINTK("Lambda (%p) : %s\n", list->opt, list->description);
		list = list->next;
	}
}

static void dsscomp_adapt_dump_info(struct dsscomp_adapt_info *info)
{
	DBG_PRINTK("Dump dsscomp_adapt_info %p\n", info);
	if ( info==NULL )
		return;
	dsscomp_adapt_dump_info_buf_list(info->buf_in_use);
	dsscomp_adapt_dump_info_lambda_list(info->lambda_list);
	DBG_PRINTK("Dump End\n");
}

struct dsscomp_adapt_work {
	struct work_struct work;
	struct dsscomp_adapt_info* adapt_info;
};

static void dsscomp_adapt_processing(struct work_struct *work)
{
	struct dsscomp_adapt_work *wk = container_of(work, typeof(*wk), work);
	struct dsscomp_adapt_info *adapt_info = wk->adapt_info;
	kfree(wk);

	//processing adaptation
	if ( adapt_info!=NULL )
	{
		int result = 0;
		struct lambda_list_t *process = adapt_info->lambda_list;
		DSSCOMP_ADAPT_ACTION("Adaptation(0x%x) will be processed", adapt_info);
#ifdef DSSCOMP_ADAPT_DEBUG
		dsscomp_adapt_dump_info(adapt_info);
#endif
		//set min memory bandwidth
		pm_qos_update_request(&mgr.pm_qos_handle, 10);
		if ( omap_pm_set_min_bus_tput(cdev->pdev,OCP_INITIATOR_AGENT, 800000) )
			pr_warning("To L3 max failed\n");
		atomic_inc(&mgr.in_progress_count);

		//process all lambda
		while(process!=NULL)
		{
			DBG_PRINTK("Process %p(func:%p, %s) \n", process, process->opt, process->description);
			if ( is_forces_stop() )
				break;
			if ( process->opt!=NULL )
			{
				DSSCOMP_ADAPT_ACTION(" lambda'%s' process", process->description);
				result = process->opt(process);
				if ( result )
				{
					DSSCOMP_ADAPT_ACTION(" lambda'%s' processing faield(0x%x)", process->description, result);
					break;
				}
			}
			process = process->next;
		}
		adapt_info->in_processing = 0;
		adapt_info->result = result;
		complete(&adapt_info->complete);
		atomic_dec(&mgr.in_progress_count);
		//to L3 Default
		pm_qos_update_request(&mgr.pm_qos_handle, PM_QOS_DEFAULT_VALUE);
		omap_pm_set_min_bus_tput(cdev->pdev,OCP_INITIATOR_AGENT, -1);
		DSSCOMP_ADAPT_ACTION("Adaptation Complete");
	}
}

/**
 * Force stop processing & wait
 */
void dsscomp_adapt_force_stop(struct dsscomp_adapt_info *adapt_info)
{
	atomic_set(&forced_stop, 1);
	//forced_stop will be cleared at first is_foced_stop() call
	if ( adapt_info!=NULL )
	{
		pr_warning("dscomp_adapt(%p) is forced stop(%d)\n", adapt_info, atomic_read(&forced_stop));
		if ( dsscomp_adapt_wait_complete(adapt_info, 500, NULL) )
		{
			pr_warning("dsscomp_adapt force stop also timeout\n");
			return;
		}
	}
	pr_warning("force stop succeeded in waiting completion. force_stop(%d)\n", atomic_read(&forced_stop));
}


/**
 * Waiting completion of adaptation processing
 * @return no error : 0, error : non-zero
 */
int dsscomp_adapt_wait_complete(struct dsscomp_adapt_info *adapt_info, unsigned long msec, int *ret)
{
	if ( adapt_info==NULL )
		return 0;
	DBG_PRINTK("dsscomp_adapt_wait_complete(%p,%lu)\n", adapt_info, msec);
	DSSCOMP_ADAPT_ACTION("Adaptation(0x%x) waiting completion", adapt_info);
	if ( adapt_info->in_processing )
	{
		if ( !wait_for_completion_timeout(&adapt_info->complete,  msecs_to_jiffies(msec)) )
		{
			pr_warning("dsscomp_adapt(%p) waiting time out\n", adapt_info);
			return -EIO;
		}
	}
	DBG_PRINTK("dsscomp_adapt_wait_complete() success\n");
	DSSCOMP_ADAPT_ACTION("Adaptation waiting completed");
	if ( ret!=NULL )
		*ret = adapt_info->result;
	return 0;
}


static void dsscomp_adapt_deffered_free(struct work_struct *work)
{
	struct dsscomp_adapt_work *wk = container_of(work, typeof(*wk), work);
	struct dsscomp_adapt_info *adapt_info = wk->adapt_info;

	pr_warning("deferred dsscomp_adapt free\n");
	kfree(wk);
	if ( adapt_info->in_processing  )
	{
		pr_err("dsscomp_adapt is still in processing crazy\n");
		adapt_info->in_processing = 0;
	}
	dsscomp_adapt_free(adapt_info);
}


/**
 * Free Adaptation Info
 */
void dsscomp_adapt_free(struct dsscomp_adapt_info *adapt_info)
{
	if ( adapt_info==NULL )
		return;
	DBG_PRINTK("dsscomp_adapt_free(%p)\n", adapt_info);
	if ( adapt_info->in_processing  )
	{
		struct dsscomp_adapt_work *wk;
		pr_warning("dsscomp_adapt(%p) is not completed, waiting completion and free\n", adapt_info);
		wk = kzalloc(sizeof(*wk), GFP_NOWAIT);
		if ( wk==NULL )
		{
			pr_err("kzalloc failed in dsscomp_adapt_free()\n");
		}
		else
		{
			wk->adapt_info = adapt_info;
			INIT_WORK(&wk->work, dsscomp_adapt_deffered_free);
			queue_work(mgr.workq, &wk->work);
			return;
		}
	}
	if ( adapt_info->buf_in_use!=NULL )
		dsscomp_adapt_free_buffer_list(adapt_info->buf_in_use);
	if ( adapt_info->lambda_list!=NULL )
		dsccomp_adapt_free_lambda_list(adapt_info->lambda_list);
	vfree(adapt_info);
}


/**
 * Check adaptation is needed and accumulate
 */
struct dsscomp_adapt_info* dsscomp_adapt_need(struct dsscomp_adapt_info* adapt_info,
		struct dss2_ovl_info *oi, struct dss2_mgr_info *mgr_info,
		struct dss2_ovl_info *oi_ret)
{
	struct adapt_buf_list_t* new_buffer = NULL;
	struct lambda_list_t* lambda = NULL;
	DBG_PRINTK("dsscomp_adapt_need() for %p\n", oi);
	//oi must be enabled
	if ( oi->cfg.enabled)
	{
		if ( atomic_read(&mgr.in_progress_count) > 0 )
		{
			pr_warning("Adaptation is in progress, give up adaptation\n");
			return NULL;
		}

		//@todo interleave condition check
		//no nv type
		//rotated side by side
		DBG_PRINTK("s3d type :%d, color mode:%d, ovl_ix:%d, mgr_ix=%d  mgr_type:%d\n",
				oi->cfg.s3d_input_layout_type, oi->cfg.color_mode, oi->cfg.ix,
				oi->cfg.mgr_ix,mgr_info->s3d_disp_info.type);
		//rotated Side-by-Side to Rotated top bottom
		if ( (oi->cfg.s3d_rotation==1 || oi->cfg.s3d_rotation==3) && oi->cfg.s3d_input_layout_type==S3D_SIDE_BY_SIDE
				&& mgr_info->s3d_disp_info.type==S3D_TOP_BOTTOM
				)
		{
			*oi_ret = *oi;
			lambda = lambda_rsbs_to_rtb_factory(oi_ret, &new_buffer, mgr_info);
		}
		//Rotated Side-by-Side to ROW Interleaved
		else if ( oi->cfg.s3d_input_layout_type==S3D_SIDE_BY_SIDE					//Side By Side
				&& oi->cfg.ix==0 && oi->cfg.color_mode!= OMAP_DSS_COLOR_NV12	//GFX Layer output
				&& (oi->cfg.s3d_rotation==1 || oi->cfg.s3d_rotation==3)			//90 or 270
				&& mgr_info->s3d_disp_info.type==S3D_ROW_INTERLEAVED					//output is interleaved
			)
		{
			//making IL conversion lambda
			*oi_ret = *oi;
			lambda = lambda_rsbs_to_il_factory(oi_ret, &new_buffer, mgr_info);
		}


		//rotation for non tiler
		else if ( oi->cfg.rotation!=0 && (!is_tiler_addr(oi->ba) || tiler_fmt(oi->ba)==TILFMT_PAGE)
					&& oi->cfg.color_mode!= OMAP_DSS_COLOR_NV12 )
		{
			*oi_ret = *oi;
			lambda = lambda_to_tiler_factory(oi_ret, &new_buffer);
		}
		//rotate & interleave
		else if ( (oi->cfg.s3d_input_layout_type==S3D_SIDE_BY_SIDE || oi->cfg.s3d_input_layout_type==S3D_TOP_BOTTOM) //S3D
					&& oi->cfg.ix!=0			//not gfx layer
					&& (oi->cfg.rotation==1 || oi->cfg.rotation==3)	//90 or 270 degree
					&& mgr_info->s3d_disp_info.type==S3D_ROW_INTERLEAVED	//output is interleaved
				)
		{
			//making rotated IL lambda
			*oi_ret = *oi;
			lambda = lambda_s3d_to_ril_op_factory(oi_ret, &new_buffer, mgr_info);
		}

		if ( lambda!=NULL )
		{
			if ( adapt_info==NULL )
			{
				adapt_info = vmalloc(sizeof(*adapt_info));
				if ( adapt_info==NULL )
					goto Failed;
				adapt_info->buf_in_use = NULL;
				adapt_info->lambda_list = NULL;
				adapt_info->result = 0;
				init_completion(&adapt_info->complete);
				adapt_info->in_processing = 0;
			}
			DSSCOMP_ADAPT_ACTION("Adaptation(0x%x) is needed for oi(idx:%d, ba:0x%x)",
					adapt_info, oi->cfg.ix, oi->ba);
			//dump buffers to action history
#ifdef DSSCOMP_ADAPT_DEBUG
			if ( new_buffer!=NULL )
			{
				struct adapt_buf_list_t* t = new_buffer;
				while(t!=NULL)
				{
					DSSCOMP_ADAPT_ACTION(" using buffer 0x%x", t->buf);
					t = t->next;
				}
			}
#endif
			//add buffers
			if ( adapt_info->buf_in_use==NULL )
				adapt_info->buf_in_use = new_buffer;
			else
			{
				struct adapt_buf_list_t* tail = adapt_info->buf_in_use;
				while(tail->next!=NULL)
					tail = tail->next;
				tail->next = new_buffer;
			}
			//dump lambda
#ifdef DSSCOMP_ADAPT_DEBUG
			if ( lambda!=NULL )
			{
				struct lambda_list_t* t = lambda;
				while(t!=NULL)
				{
					DSSCOMP_ADAPT_ACTION(" using lambda 0x%x'%s'", t, t->description);
					t = t->next;
				}
			}
#endif
			//add lambda
			if ( adapt_info->lambda_list==NULL )
				adapt_info->lambda_list = lambda;
			else
			{
				struct lambda_list_t* tail = adapt_info->lambda_list;
				while(tail->next!=NULL)
					tail = tail->next;
				tail->next = lambda;
			}
			return adapt_info;
		}

	}
	DBG_PRINTK("No Adaptation needed\n");
	return NULL;
Failed:
	if ( lambda!=NULL )
		dsccomp_adapt_free_lambda_list(lambda);
	if ( new_buffer!=NULL )
		dsscomp_adapt_free_buffer_list(new_buffer);
	pr_warning("Making adapt failed\n");
	return NULL;
}

void dsscomp_adapt_start_process(struct dsscomp_adapt_info* adapt_info)
{
	struct dsscomp_adapt_work *wk;
	if ( adapt_info==NULL )
		return;
	DBG_PRINTK("dsscomp_adapt_start_process(%p) will be queued\n", adapt_info);
	wk = kzalloc(sizeof(*wk), GFP_NOWAIT);
	if ( wk==NULL )
	{
		//treat as completed
		adapt_info->result = -ENOMEM;
		adapt_info->in_processing = 0;
		complete(&adapt_info->complete);
		return;
	}
	wk->adapt_info = adapt_info;
	INIT_WORK(&wk->work, dsscomp_adapt_processing);
	DSSCOMP_ADAPT_ACTION("Adaptation(0x%x) will be queued", adapt_info);
	adapt_info->in_processing = 1;
	queue_work( mgr.workq, &wk->work);
}


int dsscomp_adapt_init(struct dsscomp_dev *cdev_)
{
	int i;
	cdev = cdev_;
	mutex_init(&mgr.lock);
	mgr.ion = NULL;
	for(i=0;i<DSSCOMP_ADAPT_BUF_COUNT;i++)
		mgr.buffers[i].allocated = false;
	mgr.workq = create_singlethread_workqueue("dsscomp_adapt");
	if ( !mgr.workq )
		return -ENOMEM;
	atomic_set(&mgr.in_progress_count,0);
	atomic_set(&forced_stop, 0);
	//Reques Power Management Quality Service
	pm_qos_add_request(&mgr.pm_qos_handle, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
#ifdef DSSCOMP_ADAPT_DEBUG
	mutex_init(&mgr.log_lock);
	mgr.log_head = mgr.log_tail = 0;
	memset(mgr.log_entry, 0, sizeof(mgr.log_entry));
#endif
	return 0;
}

void dsscomp_adapt_deinit(void)
{
	int i;

	pm_qos_remove_request(&mgr.pm_qos_handle);
	//free buffer
	for(i=0;i<DSSCOMP_ADAPT_BUF_COUNT;i++)
		dsscomp_adapt_mgr_free_buf_by_idx_withlock(i);
	//ion manager free
	ion_client_destroy(mgr.ion);

	//workqueue free
	if ( mgr.workq )
	{
		//@todo cancel delayed work
		destroy_workqueue(mgr.workq);
		mgr.workq = NULL;
	}
	mutex_destroy(&mgr.lock);
#ifdef DSSCOMP_ADAPT_DEBUG
	mutex_destroy(&mgr.log_lock);
#endif
}

///////////////////////////////////////////////////////////////////////////
//debug fs

void dsscomp_adapt_dbg_buffer(struct seq_file *s)
{
#ifdef CONFIG_DEBUG_FS
	int i;
	struct timespec now;
	mutex_lock(&mgr.lock);
	now = CURRENT_TIME;
	//format
	seq_printf(s, "[ix] (Alloc|Free) (Used|Not-used) WxH BPP Type Alloc_size last access(ago)\n");
	for(i=0;i<DSSCOMP_ADAPT_BUF_COUNT;i++)
	{
		seq_printf(s, "[%d] ", i);
		if ( mgr.buffers[i].allocated )
		{
			struct adapt_buf_t *buf = &(mgr.buffers[i]);
			seq_printf(s, "%s %dx%d %d ",
				buf->in_using ? "U" : "N",
				buf->using_width, buf->using_height, buf->bpp);
			switch ( buf->type )
			{
			case VRAM_TYPE:
				seq_printf(s, "VRAM %uB ", buf->buf.vram.alloc_size);
				break;
			case TILER_2D_TYPE:
				seq_printf(s, "TILER %uB ", buf->buf.tiler.len);
				break;
#ifdef CONFIG_MACH_LGE_COSMO
			case TILER_1D_DYNAMIC_TYPE:
				seq_printf(s, "DYN_TILER_1D %uB ", buf->buf.tiler_1d_dyn.blk.width);
				break;				
#endif
			default:
				seq_printf(s, "Invalid Type ");
				break;
			}
			{
				struct timespec diff = timespec_sub(now, buf->last_use);
				seq_printf(s, "%ld.%09ld", diff.tv_sec, diff.tv_nsec);
			}
		}
		else
			seq_printf(s, "F");
		seq_printf(s, "\n");
	}
	mutex_unlock(&mgr.lock);
#endif
}

void dsscomp_adapt_dbg_action_history(struct seq_file *s)
{
#ifdef CONFIG_DEBUG_FS

#ifdef DSSCOMP_ADAPT_DEBUG
	int h, t;
	mutex_lock(&mgr.log_lock);
	h = mgr.log_head;
	t = mgr.log_tail;
	while(t!=h)
	{
		seq_printf(s, "%ld.%09ld : %s\n",
			mgr.log_entry[t].time.tv_sec, mgr.log_entry[t].time.tv_nsec,
			mgr.log_entry[t].log);
		t = DSSCOMP_ADAPT_HISTORY_NEXT(t);
	}
	{
		struct timespec now = CURRENT_TIME;
		seq_printf(s, "Current Time is %ld.%09ld\n", now.tv_sec, now.tv_nsec);
	}
	mutex_unlock(&mgr.log_lock);
#else
	seq_puts(s,"'DSSCOMP_ADAPT_DEBUG' is not defined. define this and rebuild\n");
#endif

#endif
}



