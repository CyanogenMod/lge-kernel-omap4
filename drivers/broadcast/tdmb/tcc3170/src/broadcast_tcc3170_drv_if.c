#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/uaccess.h>        /* copy_to_user */
#include <linux/slab.h>

#include "../../broadcast_tdmb_typedef.h"
#include "../../broadcast_tdmb_drv_ifdef.h"
#include "tdmb_tunerbbdrv_tcc3170def.h"

#include "tcpal_debug.h"
#include "tcpal_os.h"

#include "tcpal_queue.h"

#define TDMB_MSC_BUF_CHUNK_NUM 10
#define TDMB_MSC_BUFFER_SIZE (16*1024)

static TcbdQueue_t MscQueue;

static int g_ch_setting_done = ERROR;
static unsigned int ServiceType = 0;
static I08U MscBuffer[TDMB_MSC_BUFFER_SIZE * (TDMB_MSC_BUF_CHUNK_NUM+2)];
int broadcast_drv_if_power_on(void)
{
	int8 rc = ERROR;
	boolean retval = FALSE;
	
    TcbdInitQueue(&MscQueue, MscBuffer, TDMB_MSC_BUFFER_SIZE * TDMB_MSC_BUF_CHUNK_NUM );

	if(tunerbb_drv_tcc3170_is_on())
	{
		TcbdDebug(DEBUG_INFO,"tdmb_tcc317_power_on state true\n");
		tunerbb_drv_tcc3170_stop();

		tunerbb_drv_tcc3170_power_off();
	}
	
	retval = tunerbb_drv_tcc3170_power_on( );

	if(retval == TRUE)
	{
		rc = OK;
	}
	tunerbb_drv_tcc3170_set_userstop();

	return rc;
}


int broadcast_drv_if_power_off(void)
{
	int8 rc = ERROR;
	boolean retval = FALSE;

	TcbdDeinitQueue(&MscQueue);//20120110 mo2hyungmin.kim 
	retval = tunerbb_drv_tcc3170_power_off( );

	TcbdDebug(DEBUG_INFO,"broadcast_drv_if_power_off  result = (%d)\n", retval);

	if(retval == TRUE)
	{
		rc = OK;
	}
	tunerbb_drv_tcc3170_set_userstop();

	return rc;
}


int broadcast_drv_if_open(void) 
{
	int8 rc = ERROR;
	boolean retval = FALSE;

	TcbdDebug(DEBUG_INFO,"broadcast_drv_if_open\n");
	
	retval = tunerbb_drv_tcc3170_init( );

	if(retval == TRUE)
	{
		rc = OK;
	}

	return rc;	
}


int broadcast_drv_if_close(void)
{
	int8 rc = ERROR;
	boolean retval = FALSE;

    if(tunerbb_drv_tcc3170_is_on() == TRUE)
    {
	    retval = tunerbb_drv_tcc3170_stop( );
    }
    
	if(retval == TRUE)
	{
		rc = OK;
	}

	return rc;
}


int broadcast_drv_if_set_channel(unsigned int freq_num, unsigned int subch_id, unsigned int op_mode)
{
    int8 rc = ERROR;
    boolean retval = FALSE;

    TcbdDebug(DEBUG_INFO,"broadcast_drv_if_set_channel IN( )\n");

    retval = tunerbb_drv_tcc3170_set_channel(freq_num, subch_id, op_mode);

    TcbdDebug(DEBUG_INFO,"broadcast_drv_if_set_channel OUT( ) result = (%d)\n", retval);

    if(retval == TRUE)
    {
        rc = OK;
    }

    g_ch_setting_done = rc;
    ServiceType = op_mode;
    return rc;	
}

int broadcast_drv_if_resync(void)
{
	return ERROR;
}

int broadcast_drv_if_detect_sync(int op_mode)
{
	int8 rc = ERROR;
	boolean retval = FALSE;

	if(g_ch_setting_done == OK)
	{
		TcbdDebug(DEBUG_INFO,"broadcast_drv_if_detect_sync. channel_set_ok = (%d)\n", g_ch_setting_done);
		
		return OK;
	}
	
	retval = tunerbb_drv_tcc3170_re_syncdetector(op_mode);

	if(retval == TRUE)
	{
		rc = OK;
	}

	g_ch_setting_done = rc;

	return rc;
}

int broadcast_drv_if_get_sig_info(struct broadcast_tdmb_sig_info *dmb_bb_info)
{
	int8 rc = ERROR;
	boolean retval = FALSE;

	retval = tunerbb_drv_tcc3170_get_ber(dmb_bb_info);

	if(retval == TRUE)
	{
		rc = OK;
	}
//20111116 mo2hyungmin.kim block for TDMB AV Test mode 
/*
	if(g_ch_setting_done == ERROR)
	{
		dmb_bb_info->cir = 0;
		dmb_bb_info->msc_ber = 20000;
	}
	else
	{
		dmb_bb_info->cir = 1;
	}
*/
	//TcbdDebug(DEBUG_INFO,"broadcast_drv_if_get_sig_info ber = (%d)\n", dmb_bb_info->msc_ber);

	return rc;
}


int broadcast_drv_if_get_ch_info(char* buffer, unsigned int* buffer_size)
{
	int8 rc = ERROR;
	boolean retval = FALSE;

	if(buffer == NULL || buffer_size == NULL)
	{
		TcbdDebug(DEBUG_ERROR,"broadcast_drv_if_get_ch_info argument error\n");

		return rc;
	}

	retval = tunerbb_drv_tcc3170_get_fic(buffer, buffer_size);

	if(retval == TRUE)
	{
		rc = OK;
	}

	return rc;	
}


int broadcast_drv_if_get_dmb_data(char** buffer_ptr, unsigned int* buffer_size, unsigned int user_buffer_size)
{
    I32S type;

    if(buffer_ptr == NULL || buffer_size == NULL)
    {
        TcbdDebug(DEBUG_ERROR,"input arg is null\n");
        return ERROR;
    }
    
    if(TcbdGetFirstQueuePtr(&MscQueue, (I08U**)buffer_ptr, (I32S*)buffer_size, &type) < 0)
        return ERROR;

    if(user_buffer_size < *buffer_size)
    {
        TcbdDebug(DEBUG_ERROR,"user buffer is not enough user:%d, buffer_size:%d\n", user_buffer_size, *buffer_size);
        return ERROR;
    }

    if(TcbdDequeuePtr(&MscQueue, (I08U**)buffer_ptr, (I32S*)buffer_size, &type) < 0)
        return ERROR;

    return OK;
}

int broadcast_drv_if_push_msc_data(char *buffer_ptr, unsigned int buffer_size)
{
    TDMB_BB_HEADER_TYPE *lgeHeader;
    
    lgeHeader = (TDMB_BB_HEADER_TYPE*)buffer_ptr;
    switch(ServiceType)
    {
        case TCBD_DMB:
        case TCBD_VISUAL:
            lgeHeader->data_type = TDMB_BB_DATA_TS;
            break;
        case TCBD_DAB:
            lgeHeader->data_type = TDMB_BB_DATA_DAB;
            break;
        case TCBD_DATA:
            lgeHeader->data_type = TDMB_BB_DATA_PACK;
            break;
    }

    TcbdEnqueue(&MscQueue, (I08U*)buffer_ptr,  (I32S)buffer_size, 1);
    
    return OK;
}

int broadcast_drv_if_reset_ch(void)
{
    int8 rc = ERROR;
    boolean retval = FALSE;

    TcbdResetQueue(&MscQueue);

    retval = tunerbb_drv_tcc3170_reset_ch( );

    if(retval == TRUE)
    {
        rc = OK;
    }
       
    return rc;
}

int broadcast_drv_if_user_stop(int mode)
{
    TcbdResetQueue(&MscQueue);
    tunerbb_drv_tcc3170_set_userstop();

    return OK ;
}


int broadcast_drv_if_select_antenna(unsigned int sel)
{
	tunerbb_drv_tcc3170_select_antenna(sel);
	
	return OK;
}


int broadcast_drv_if_isr(void)
{
	return ERROR;
}

