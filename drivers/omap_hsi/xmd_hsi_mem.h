/*
 * xmd_hsi_mem.h.h
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

#if !defined(XMD_HSI_MEM__H)
#define XMD_HSI_MEM__H

/*****************************************************************************/
/* INCLUDES                                                                  */
/*****************************************************************************/

/*****************************************************************************/
/* DEFINES                                                                   */
/*****************************************************************************/
/*  If this is defined, preallocated buffers are used else memory is allocated
	only when required by kmalloc.*/
#define HSI_PRE_ALLOC_BUFFERS

/*  If this is defined, preallocated mem is obtained from dma_alloc_coherent
	else static pools are used. */
#define HSI_MEM_USE_DMA_BUFS

//                                                  
#define HSI_MEM_ENABLE_LOGS

/* #define HSI_MEM_DEBUG */
//                                                

/*TODO: Fine tune below memory configuration as per requirement. */

#define HSI_MEM_BLOCK0_SIZE    512
#define HSI_MEM_BLOCK1_SIZE	 (1024*2)
#define HSI_MEM_BLOCK2_SIZE	 (1024*8)
#define HSI_MEM_BLOCK3_SIZE	 (1024*30)

#define HSI_MEM_LARGE_BLOCK_SIZE  HSI_MEM_BLOCK3_SIZE

//                                                  
#define HSI_MEM_NUM_OF_BLK0		80
#define HSI_MEM_NUM_OF_BLK1	 	80
#define HSI_MEM_NUM_OF_BLK2	 	10
#define HSI_MEM_NUM_OF_BLK3 	8
#define HSI_MEM_NUM_OF_FB_BLK	120
//                                                

/*****************************************************************************/
/* TYPE DEFINITIONS                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* PROTOTYPES                                                                */
/*****************************************************************************/
int hsi_mem_init(void);
int hsi_mem_uninit(void);
//                                                  
int hsi_mem_reinit(void);
//                                                
void* hsi_mem_alloc(int size);
void hsi_mem_free(void* ptr);
#endif /* XMD_HSI_MEM__H */

