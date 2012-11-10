/*
 * xmd_hsi_mem.c
 *
 * Copyright (C) 2011 Intel Mobile Communications. All rights reserved.
 *
 * Author: Chaitanya <Chaitanya.Khened@intel.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*****************************************************************************/
/* INCLUDES                                                                  */
/*****************************************************************************/

#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include "xmd_hsi_mem.h"

#if defined (HSI_PRE_ALLOC_BUFFERS)
unsigned int *hsi_mem_blk0_ptr[HSI_MEM_NUM_OF_BLK0];
unsigned int *hsi_mem_blk1_ptr[HSI_MEM_NUM_OF_BLK1];
unsigned int *hsi_mem_blk2_ptr[HSI_MEM_NUM_OF_BLK2];
unsigned int *hsi_mem_blk3_ptr[HSI_MEM_NUM_OF_BLK3];

#if !defined (HSI_MEM_USE_DMA_BUFS)
/*Declared as int to make sure that buffers are word aligned */
unsigned int hsi_mem_blk0[HSI_MEM_NUM_OF_BLK0][HSI_MEM_BLOCK0_SIZE/4];
unsigned int hsi_mem_blk1[HSI_MEM_NUM_OF_BLK1][HSI_MEM_BLOCK1_SIZE/4];
unsigned int hsi_mem_blk2[HSI_MEM_NUM_OF_BLK2][HSI_MEM_BLOCK2_SIZE/4];
unsigned int hsi_mem_blk3[HSI_MEM_NUM_OF_BLK3][HSI_MEM_BLOCK3_SIZE/4];
#else
unsigned int *hsi_mem_blk0[HSI_MEM_NUM_OF_BLK0];
unsigned int *hsi_mem_blk1[HSI_MEM_NUM_OF_BLK1];
unsigned int *hsi_mem_blk2[HSI_MEM_NUM_OF_BLK2];
unsigned int *hsi_mem_blk3[HSI_MEM_NUM_OF_BLK3];
#endif

unsigned char *hsi_mem_fb_block[HSI_MEM_NUM_OF_FB_BLK] = {NULL};
#endif
static spinlock_t hsi_mem_lock;

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined (HSI_MEM_DEBUG)
int mem_dbg_blk0_max;			/* maximum number of allocated blocks from blk0 */
int mem_dbg_blk1_max;			/* maximum number of allocated blocks from blk1 */
int mem_dbg_blk2_max;			/* maximum number of allocated blocks from blk2 */
int mem_dbg_blk3_max;			/* maximum number of allocated blocks from blk3 */
int mem_dbg_fb_max;				/* maximum number of allocated blocks from fb */

/*****************************************************************************/
/* Function:... hsi_dbg_mem_init                                               */
/*****************************************************************************/
static void hsi_dbg_mem_init(void)
{
	mem_dbg_blk0_max = 0;
	mem_dbg_blk1_max = 0;
	mem_dbg_blk2_max = 0;
	mem_dbg_blk3_max = 0;
	mem_dbg_fb_max = 0;
}

/*****************************************************************************/
/* Function:... hsi_dbg_mem_log                                               */
/*****************************************************************************/
static void hsi_dbg_mem_log(char* txt, int* pre_max, int cur_index)
{
	if((*pre_max) < cur_index)
	{
		printk("hsi_dbg_mem: [%s] pre_max[%d] cur_index [%d]\n", txt, (*pre_max), cur_index);
		(*pre_max) = cur_index;
	}
}
#endif /* HSI_MEM_DEBUG */
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

/*****************************************************************************/
/* Function:... hsi_mem_mem_init                                             */
/* Description: Initialization of the memory pools.                          */
/*****************************************************************************/
int hsi_mem_init(void)
{
	static int is_hsi_mem_init;

	if(!is_hsi_mem_init) {
#if defined (HSI_PRE_ALLOC_BUFFERS) && defined (HSI_MEM_USE_DMA_BUFS)
	unsigned int i;

	for(i=0; i < HSI_MEM_NUM_OF_BLK0; i++) {
		hsi_mem_blk0[i] =  (unsigned int*) kmalloc(HSI_MEM_BLOCK0_SIZE,
												GFP_DMA|GFP_KERNEL|GFP_ATOMIC);
#if defined (HSI_MEM_ENABLE_LOGS)
		if(hsi_mem_blk0[i] == NULL)
			printk("\nhsi_mem: failed to alloc HSI_MEM_BLOCK0  memory.i=%d",i);
#endif
	}

	for(i=0; i < HSI_MEM_NUM_OF_BLK1; i++) {
		hsi_mem_blk1[i] =  (unsigned int*) kmalloc(HSI_MEM_BLOCK1_SIZE,
												GFP_DMA|GFP_KERNEL|GFP_ATOMIC);
#if defined (HSI_MEM_ENABLE_LOGS)
		if(hsi_mem_blk1[i] == NULL)
			printk("\nhsi_mem: failed to alloc HSI_MEM_BLOCK1  memory.i=%d",i);
#endif
	}

	for(i=0; i < HSI_MEM_NUM_OF_BLK2; i++) {
		hsi_mem_blk2[i] =  (unsigned int*) kmalloc(HSI_MEM_BLOCK2_SIZE,
												GFP_DMA|GFP_KERNEL|GFP_ATOMIC);
#if defined (HSI_MEM_ENABLE_LOGS)
		if(hsi_mem_blk2[i] == NULL)
			printk("\nhsi_mem: failed to alloc HSI_MEM_BLOCK2  memory.i=%d",i);
#endif
	}

	for(i=0; i < HSI_MEM_NUM_OF_BLK3; i++) {
		hsi_mem_blk3[i] =  (unsigned int*) kmalloc(HSI_MEM_BLOCK3_SIZE,
												GFP_DMA|GFP_KERNEL|GFP_ATOMIC);
#if defined (HSI_MEM_ENABLE_LOGS)
		if(hsi_mem_blk3[i] == NULL)
			printk("\nhsi_mem: failed to alloc HSI_MEM_BLOCK3  memory.i=%d",i);
#endif
	}
#endif
	spin_lock_init(&hsi_mem_lock);

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined (HSI_MEM_DEBUG)
	hsi_dbg_mem_init();
#endif /* HSI_MEM_DEBUG */
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

	is_hsi_mem_init = 1;
	}
	return 0;
}

/*****************************************************************************/
/* Function:... hsi_mem_uninit                                               */
/* Description: Freeing of memory pools                                      */
/*****************************************************************************/
int hsi_mem_uninit(void)
{
	return 0;
}

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
/*****************************************************************************/
/* Function:... hsi_mem_reinit                                               */
/* Description: Initialization of the memory pools for RIL recovery                                     */
/*****************************************************************************/
int hsi_mem_reinit(void)
{
#if defined (HSI_PRE_ALLOC_BUFFERS)
	unsigned int i;

	spin_lock_bh(&hsi_mem_lock);

	for(i=0; i < HSI_MEM_NUM_OF_BLK0;i++) {
		hsi_mem_blk0_ptr[i] = NULL;
	}

	for(i=0; i < HSI_MEM_NUM_OF_BLK1;i++) {
		hsi_mem_blk1_ptr[i] = NULL;
	}

	for(i=0; i < HSI_MEM_NUM_OF_BLK2; i++) {
		hsi_mem_blk2_ptr[i] = NULL;
	}

	for(i=0; i < HSI_MEM_NUM_OF_BLK3; i++) {
		hsi_mem_blk3_ptr[i] = NULL;
	}

	/* free fallback memory */
	for (i=0; i < HSI_MEM_NUM_OF_FB_BLK; i++) {
		if (hsi_mem_fb_block[i] != NULL) {
			kfree(hsi_mem_fb_block[i]);
			hsi_mem_fb_block[i] = NULL;
		}
	}

	/*TODO : How to free allocated buffer from BUF retry WQ.*/

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined (HSI_MEM_DEBUG)
	hsi_dbg_mem_init();
#endif /* HSI_MEM_DEBUG */
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

	spin_unlock_bh(&hsi_mem_lock);
#endif
	return 0;
}
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

/*****************************************************************************/
/* Function:... hsi_mem_alloc                                                */
/* Description: allocates a block with the requested size. If no memory      */
/* block of the requested size available, NULL will be returned. The         */
/* returned pointer will be word aligned                                     */
/*****************************************************************************/
void *hsi_mem_alloc(int size)
{
	void *buf = NULL;
#if defined (HSI_PRE_ALLOC_BUFFERS)
	unsigned int i;
	spin_lock_bh(&hsi_mem_lock);

	if(size == 0) {
		goto quit_mem_alloc;
	}

	if(size <= HSI_MEM_BLOCK0_SIZE) {
		for(i=0;i<HSI_MEM_NUM_OF_BLK0;i++) {
			if (hsi_mem_blk0_ptr[i] == NULL) {
				hsi_mem_blk0_ptr[i] = hsi_mem_blk0[i];
				buf = (void *)hsi_mem_blk0_ptr[i];
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined (HSI_MEM_DEBUG)
			hsi_dbg_mem_log("HSI_MEM_BLOCK0_SIZE(512B)", &mem_dbg_blk0_max, i);
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]
				goto quit_mem_alloc;
			}
		}
#if defined (HSI_MEM_ENABLE_LOGS)
		printk("\nhsi_mem: Running out of HSI_MEM_BLOCK0 memory.\n");
#endif
	}

	if((size > HSI_MEM_BLOCK0_SIZE) && (size <= HSI_MEM_BLOCK1_SIZE)) {
		for(i=0; i < HSI_MEM_NUM_OF_BLK1;i++) {
			if (hsi_mem_blk1_ptr[i] == NULL) {
				hsi_mem_blk1_ptr[i] = hsi_mem_blk1[i];
				buf = (void *)hsi_mem_blk1_ptr[i];
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined (HSI_MEM_DEBUG)
				hsi_dbg_mem_log("HSI_MEM_BLOCK1_SIZE(2KB)", &mem_dbg_blk1_max, i);
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]	
				goto quit_mem_alloc;
			}
		}
#if defined (HSI_MEM_ENABLE_LOGS)
		printk("\nhsi_mem: Running out of HSI_MEM_BLOCK1 memory.\n");
#endif
	}

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
/* Uplink Throughput issue : share pre-allocated buffer (2KB & 8KB) */
#if 1
	if((size > HSI_MEM_BLOCK0_SIZE) && (size <= HSI_MEM_BLOCK2_SIZE)) {
#else
	if((size > HSI_MEM_BLOCK1_SIZE) && (size <= HSI_MEM_BLOCK2_SIZE)) {
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

		for(i=0; i < HSI_MEM_NUM_OF_BLK2;i++) {
			if (hsi_mem_blk2_ptr[i] == NULL) {
				hsi_mem_blk2_ptr[i] = hsi_mem_blk2[i];
				buf = (void *)hsi_mem_blk2_ptr[i];
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined (HSI_MEM_DEBUG)
				hsi_dbg_mem_log("HSI_MEM_BLOCK2_SIZE(8KB)", &mem_dbg_blk2_max, i);
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]	
				
				goto quit_mem_alloc;
			}
		}
#if defined (HSI_MEM_ENABLE_LOGS)
		printk("\nhsi_mem: Running out of HSI_MEM_BLOCK2 memory.\n");
#endif
	}

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
/* Uplink Throughput issue : share pre-allocated buffer (8KB & 30KB) */
#if 1
	if((size > HSI_MEM_BLOCK1_SIZE) && (size <= HSI_MEM_BLOCK3_SIZE)) {
#else
	if((size > HSI_MEM_BLOCK2_SIZE) && (size <= HSI_MEM_BLOCK3_SIZE)) {
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

		for(i=0; i < HSI_MEM_NUM_OF_BLK3;i++) {
			if (hsi_mem_blk3_ptr[i] == NULL) {
				hsi_mem_blk3_ptr[i] = hsi_mem_blk3[i];
				buf = (void *)hsi_mem_blk3_ptr[i];
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined (HSI_MEM_DEBUG)
				hsi_dbg_mem_log("HSI_MEM_BLOCK3_SIZE(30KB)", &mem_dbg_blk3_max, i);
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]				
				goto quit_mem_alloc;
			}
		}
#if defined (HSI_MEM_ENABLE_LOGS)
		printk("\nhsi_mem: Running out of HSI_MEM_BLOCK3 memory.\n");
#endif
	}

#if defined (HSI_MEM_ENABLE_LOGS)
	printk("\nhsi_mem: Requesting Fall back memory of size %d.\n",size);
#endif

	/* fallback to kernel for memory */
	for (i=0; i < HSI_MEM_NUM_OF_FB_BLK; i++) {
		if (hsi_mem_fb_block[i] == NULL) {
			hsi_mem_fb_block[i] = (unsigned char*) kmalloc(size,
													GFP_DMA|GFP_ATOMIC);
			if(hsi_mem_fb_block[i] == NULL) {
#if defined (HSI_MEM_ENABLE_LOGS)
				printk("\nhsi_mem:Failed to alloc fall-back mem, returning NULL");
#endif
				buf = NULL;
			} else {
				buf = (void *) hsi_mem_fb_block[i];
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined (HSI_MEM_DEBUG)
			hsi_dbg_mem_log("HSI_MEM_NUM_OF_FB_BLK", &mem_dbg_fb_max, i);
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]				
			}
			goto quit_mem_alloc;
		}
	}
quit_mem_alloc:
#else
	spin_lock_bh(&hsi_mem_lock);
	buf = kmalloc(size, GFP_DMA | GFP_ATOMIC);
#endif
#if defined (HSI_MEM_ENABLE_LOGS)
	if(buf == NULL)
		printk("\nhsi_mem:Failed to alloc mem returning NULL. Mem exhausted");
#endif
	spin_unlock_bh(&hsi_mem_lock);
	return buf;
}

/*****************************************************************************/
/* Function:... hsi_mem_free                                                 */
/* Description: Frees a memory block, which was allocated before with        */
/*              hsi_mem_alloc.                                               */
/*****************************************************************************/
void hsi_mem_free(void* buf)
{
#if defined (HSI_PRE_ALLOC_BUFFERS)
	unsigned int i;
	unsigned int *mem = (unsigned int *) buf;
	unsigned char *fb_mem = (unsigned char *) buf;

	spin_lock_bh(&hsi_mem_lock);

	if(mem == NULL) {
		goto quit_mem_free;
	}

	for(i=0; i < HSI_MEM_NUM_OF_BLK0;i++) {
		if (mem == hsi_mem_blk0_ptr[i]) {
			hsi_mem_blk0_ptr[i] = NULL;
			goto quit_mem_free;
		}
	}

	for(i=0; i < HSI_MEM_NUM_OF_BLK1;i++) {
		if (mem == hsi_mem_blk1_ptr[i]) {
			hsi_mem_blk1_ptr[i] = NULL;
			goto quit_mem_free;
		}
	}

	for(i=0; i < HSI_MEM_NUM_OF_BLK2; i++) {
		if (mem == hsi_mem_blk2_ptr[i]) {
			hsi_mem_blk2_ptr[i] = NULL;
			goto quit_mem_free;
		}
	}

	for(i=0; i < HSI_MEM_NUM_OF_BLK3; i++) {
		if (mem == hsi_mem_blk3_ptr[i]) {
			hsi_mem_blk3_ptr[i] = NULL;
			goto quit_mem_free;
		}
	}

	/* free fallback memory */
	for (i=0; i < HSI_MEM_NUM_OF_FB_BLK; i++) {
		if (hsi_mem_fb_block[i] == fb_mem) {
			kfree(fb_mem);
			hsi_mem_fb_block[i] = NULL;
			goto quit_mem_free;
		}
	}

	/*Reached here ? Then this should be allocated from BUF retry WQ.*/
	kfree(buf);
quit_mem_free:
#else
	spin_lock_bh(&hsi_mem_lock);
	if(buf != NULL) {
		kfree(buf);
	}
#endif
	spin_unlock_bh(&hsi_mem_lock);
}

