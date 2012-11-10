/*
 * hdcp_top.c
 *
 * HDCP support functions for TI OMAP processors.
 *
 * Copyright (C) 2011 Texas Instruments
 *
 * Author: Fabrice Olivero <f-olivero@ti.com>
 *
 * Use of this software is controlled by the terms and conditions found
 * in the license agreement under which this software has been supplied.
 *
 */

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <plat/display.h>
#include <linux/completion.h>
#include <../drivers/video/omap2/dss/hdmi.h>
#include <../drivers/video/omap2/dss/dss.h>
#include <plat/hdmi_lib.h>

#include <linux/delay.h>

#include "hdcp.h"

// wooho47.jung@lge.com 2011.11.02
// ADD : for HDCP Debug
#if 0
#define HDCP_LOG_FUNCTION_NAME_ENTRY             printk(KERN_INFO "[HDCP]## %s() ++ ##\n",  __func__);
#define HDCP_LOG_FUNCTION_NAME_EXIT              printk(KERN_INFO "[HDCP]## %s() -- ##\n",  __func__);
#define HDMI_LOG_FUNCTION_NAME_EXIT_ERROR        printk(KERN_ERR "[HDCP]## %s() ERROR Exit -- ##\n",  __func__);
#else
#define HDCP_LOG_FUNCTION_NAME_ENTRY             
#define HDCP_LOG_FUNCTION_NAME_EXIT              
#define HDMI_LOG_FUNCTION_NAME_EXIT_ERROR        
#endif

struct hdcp hdcp;

#if 1
/* State machine / workqueue */
static void hdcp_wq_disable(void);
static void hdcp_wq_start_authentication(void);
static void hdcp_wq_restart_hdmi(void);
static void hdcp_wq_check_r0(void);
static void hdcp_wq_step2_authentication(void);
static void hdcp_wq_authentication_failure(void);
static void hdcp_work_queue(struct work_struct *work);
static struct delayed_work *hdcp_submit_work(int event, int delay, int irq_context);
static void hdcp_cancel_work(struct delayed_work **work);

/* Callbacks */
static void hdcp_start_frame_cb(void);
static void hdcp_stop_frame_cb(void);
static void hdcp_irq_cb(int hpd_low);

/* Control */
static long hdcp_enable_ctl(void __user *argp);
static long hdcp_disable_ctl(void);
static long hdcp_query_status_ctl(void __user *argp);
static long hdcp_encrypt_key_ctl(void __user *argp);
long hdcp_ioctl(struct file *fd, unsigned int cmd, unsigned long arg);

/* Driver */
static int __init hdcp_init(void);
static void __exit hdcp_exit(void);

// LGE_CHANGE_S [jh.koo kibum.lee] 2011-09-08, hdmi and hdcp lockup issue WA for P940
static int hdcp_mainclk_state=false;
static int backup_retry_count=3;
// LGE_CHANGE_E [jh.koo kibum.lee] 2011-09-08, hdmi and hdcp lockup issue WA for P940
// LGE_CHANGE_S [jh.koo kibum.lee] 2011-11-01, uevent for HDCP completion timing
extern void hdcp_send_uevent(u8 on);
// LGE_CHANGE_E [jh.koo kibum.lee] 2011-11-01, uevent for HDCP completion timing
#endif

struct completion hdcp_comp;
#define DSS_POWER


// by Joshua
void load_hdcp_key(void);

/*-----------------------------------------------------------------------------
 * Function: hdcp_request_dss
 *-----------------------------------------------------------------------------
 */
static void hdcp_request_dss(void)
{
#ifdef DSS_POWER
	//hdcp.dss_state = dss_mainclk_enable();
	if (hdcp_mainclk_state == false) { 
		hdcp.dss_state = dss_mainclk_enableEx();
		if(hdcp.dss_state >= 0)
		{
			hdcp_mainclk_state = true;
		}
	}
#endif
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_release_dss
 *-----------------------------------------------------------------------------
 */
static void hdcp_release_dss(void)
{
#ifdef DSS_POWER
//	if (hdcp.dss_state == 0)
//		dss_mainclk_disable();
	if (hdcp_mainclk_state == true) { 
		dss_mainclk_disableEx();
		hdcp_mainclk_state = false;
	}
		
#endif
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_disable
 *-----------------------------------------------------------------------------
 */
static void hdcp_wq_disable(void)
{
    HDCP_LOG_FUNCTION_NAME_ENTRY
    
    if(&hdcp.pending_wq_event && hdcp.pending_wq_event)
	    hdcp_cancel_work(&hdcp.pending_wq_event);

	hdcp_lib_disable();
	hdcp.pending_disable = 0;
    HDCP_LOG_FUNCTION_NAME_EXIT
}

// by Joshua
void load_hdcp_key(void)
{

	/* Load 3DES key */
    if(hdcp.en_ctrl)
	{
	    hdcp_3des_load_key(hdcp.en_ctrl->key);
    }
    else
    {
        DBG_ERROR("hdcp.en_ctrl\n");
    }
	return;
	
}


/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_start_authentication
 *-----------------------------------------------------------------------------
 */
static void hdcp_wq_start_authentication(void)
{
	int status = HDCP_OK;
    HDCP_LOG_FUNCTION_NAME_ENTRY

	hdcp.hdcp_state = HDCP_AUTHENTICATION_START;

// by Joshua
#if 1 //0 // sungho.jung
	/* Load 3DES key */
	if (hdcp_3des_load_key(hdcp.en_ctrl->key) != HDCP_OK) {
		hdcp_wq_authentication_failure();
		return;
	}
#endif

	/* Step 1 part 1 (until R0 calc delay) */
	status = hdcp_lib_step1_start();

	if (status == -HDCP_AKSV_ERROR) {
		hdcp_wq_restart_hdmi();
	}
	else if (status == -HDCP_CANCELLED_AUTH) {
		DBG_ERROR("Authentication step 1 cancelled.\n");
		return;
	}
	else if (status != HDCP_OK) {
		hdcp_wq_authentication_failure();
	}
	else {
		// by Joshua
		//hdcp.retry_cnt = hdcp.en_ctrl->nb_retry;	// kibum.lee@lge.com //comment
		DBG("retry_cnt=%d\n", hdcp.retry_cnt);

		hdcp.hdcp_state = HDCP_WAIT_R0_DELAY;
		hdcp.auth_state = HDCP_STATE_AUTH_1ST_STEP;
		hdcp.pending_wq_event = hdcp_submit_work(HDCP_R0_EXP_EVENT,
							 HDCP_R0_DELAY, 0);
	}
    HDCP_LOG_FUNCTION_NAME_EXIT
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_restart_hdmi
 *-----------------------------------------------------------------------------
 */
static void hdcp_wq_restart_hdmi(void)
{
    HDCP_LOG_FUNCTION_NAME_ENTRY;
    
    if(&hdcp.pending_wq_event && hdcp.pending_wq_event )
        hdcp_cancel_work(&hdcp.pending_wq_event);
    
	hdcp_lib_disable();
	hdcp.pending_disable = 0;

	if (hdcp.retry_cnt) {
		hdcp.retry_cnt--;
        printk("====================================================\n");
		printk("authentication failed - restarting HDMI, attempts=%d\n", hdcp.retry_cnt);
        printk("====================================================\n");
        /*
        // wooho47.jung@lge.com 2011.11.02
        // ADD : for HDCP Auth retry
		//hdmi_restart();
		hdcp.hdcp_state = HDCP_ENABLE_PENDING;
		hdcp.auth_state = HDCP_STATE_DISABLED; //HDCP_STATE_AUTH_FAIL_RESTARTING;
		hdcp_start_frame_cb();		
        */		
		//hdcp.hdmi_restart = 1;
        //hdmi_restart();
		//hdcp.hdmi_restart = 0;
		hdcp_start_frame_cb();		
        // wooho47.jung@lge.com 2011.11.04
        // MOD : for HDCP Auth Retry
		hdcp.hdcp_state = HDCP_ENABLE_PENDING;
		hdcp.auth_state = HDCP_STATE_AUTH_FAIL_RESTARTING;
	}
	else 
    {
        printk("=============================\n");
		printk("=== authentication failed !!!\n");
        printk("=============================\n");
		//hdcp.hdcp_state = HDCP_DISABLED;	// HDCP_DISABLE ?? 
		hdcp.hdcp_state = HDCP_ENABLE_PENDING;	// kibum.lee@lge.com
		hdcp.auth_state = HDCP_STATE_AUTH_FAILURE;
	}
    HDCP_LOG_FUNCTION_NAME_EXIT
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_check_r0
 *-----------------------------------------------------------------------------
 */
static void hdcp_wq_check_r0(void)
{
	int status = 0;
    HDCP_LOG_FUNCTION_NAME_ENTRY
    status = hdcp_lib_step1_r0_check();

	if (status == -HDCP_CANCELLED_AUTH)
    {
		DBG_ERROR("Authentication step 1/R0 cancelled.\n");
		return;
	}
	else if (status < 0)
	{
	    hdcp_wq_authentication_failure();
    }
	else
	{
	    if (hdcp_lib_check_repeater_bit_in_tx()) 
        {
			/* Repeater */
			hdcp.hdcp_state = HDCP_WAIT_KSV_LIST;
			hdcp.auth_state = HDCP_STATE_AUTH_2ND_STEP;
			/* TODO: 5s: value to confirm for polling Bcaps RDY */
			hdcp.pending_wq_event = hdcp_submit_work(HDCP_KSV_TIMEOUT_EVENT,
								 HDCP_KSV_TIMEOUT_DELAY,
								 0);
		}
		else 
        {
			/* Receiver */
			hdcp.hdcp_state = HDCP_LINK_INTEGRITY_CHECK;
			hdcp.auth_state = HDCP_STATE_AUTH_3RD_STEP;
		}
    }
    HDCP_LOG_FUNCTION_NAME_EXIT        
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_step2_authentication
 *-----------------------------------------------------------------------------
 */
static void hdcp_wq_step2_authentication(void)
{
	int status = HDCP_OK;
    HDCP_LOG_FUNCTION_NAME_ENTRY;

	/* KSV list timeout is running and should be canceled */
    if(&hdcp.pending_wq_event && hdcp.pending_wq_event )
	    hdcp_cancel_work(&hdcp.pending_wq_event);

	status = hdcp_lib_step2();

	if (status == -HDCP_CANCELLED_AUTH) {
		DBG_ERROR("Authentication step 2 cancelled.\n");
		return;
	}
	else if (status < 0)
		hdcp_wq_authentication_failure();
	else {
		hdcp.hdcp_state = HDCP_LINK_INTEGRITY_CHECK;
		hdcp.auth_state = HDCP_STATE_AUTH_3RD_STEP;
	}
    HDCP_LOG_FUNCTION_NAME_EXIT
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_authentication_failure
 *-----------------------------------------------------------------------------
 */
static void hdcp_wq_authentication_failure(void)
{
    HDCP_LOG_FUNCTION_NAME_ENTRY
	hdcp_lib_auto_ri_check(false);
	hdcp_lib_auto_bcaps_rdy_check(false);
	hdcp_lib_set_av_mute(AV_MUTE_SET);
	hdcp_lib_set_encryption(HDCP_ENC_OFF);

    if(&hdcp.pending_wq_event && hdcp.pending_wq_event )
	    hdcp_cancel_work(&hdcp.pending_wq_event);

	hdcp_lib_disable();
	hdcp.pending_disable = 0;

// 1A-04 and 1A-07a spec
/*
	if (hdcp.retry_cnt) {
			hdcp.retry_cnt--;
			printk(KERN_INFO "HDCP: authentication failed - "
					 "retrying, attempts=%d\n",
							hdcp.retry_cnt);
*/
	hdcp.hdcp_state = HDCP_AUTHENTICATION_START;
	hdcp.auth_state = HDCP_STATE_AUTH_FAIL_RESTARTING;

	hdcp.pending_wq_event = hdcp_submit_work(HDCP_AUTH_REATT_EVENT,
						 HDCP_REAUTH_DELAY, 0);
/*	
	}
	else {
		printk(KERN_INFO "HDCP: authentication failed - "
				 "HDCP disabled\n");
		hdcp.hdcp_state = HDCP_ENABLE_PENDING;
		hdcp.auth_state = HDCP_STATE_AUTH_FAILURE;
	}
*/	
    HDCP_LOG_FUNCTION_NAME_EXIT    
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_work_queue
 *-----------------------------------------------------------------------------
 */
static void hdcp_work_queue(struct work_struct *work)
{
	struct hdcp_delayed_work *hdcp_w =
		container_of(work, struct hdcp_delayed_work, work.work);
	int event = hdcp_w->event;
	int hdcp_state = 1;
    HDCP_LOG_FUNCTION_NAME_ENTRY

	mutex_lock(&hdcp.lock);

// LGE_CHANGE_S [jh.koo kibum.lee] 2011-09-08, hdmi and hdcp lockup issue WA for P940
	hdcp_request_dss();
// LGE_CHANGE_E [jh.koo kibum.lee] 2011-09-08, hdmi and hdcp lockup issue WA for P940

	DBG("%u hdmi=%d hdcp=%d auth=%d evt= %02x %04x\n",
		jiffies_to_msecs(jiffies),
		hdcp.hdmi_state,
		hdcp.hdcp_state,
		hdcp.auth_state,
		(event & 0xFF00) >> 8,
		event & 0xFF);

	/* Clear pending_wq_event
	 * In case a delayed work is scheduled from the state machine "pending_wq_event"
	 * is used to memorize pointer on the event to be able to cancel any pending work in
	 * case HDCP is disabled
	 */
	if (event & HDCP_WORKQUEUE_SRC)
		hdcp.pending_wq_event = 0;

	/* First handle HDMI state */
	if (event == HDCP_START_FRAME_EVENT) {
		hdcp.pending_start = 0;
		hdcp.hdmi_state = HDMI_STARTED;
	}
	else if (event == HDCP_STOP_FRAME_EVENT)
	{
	    hdcp.hdmi_state = HDMI_STOPPED;
    }

	/**********************/
	/* HDCP state machine */
	/**********************/


	/* Handle HDCP disable (from any state) */
	if ((event == HDCP_DISABLE_CTL) ||
	    (event == HDCP_STOP_FRAME_EVENT) ||
	    (event == HDCP_HPD_LOW_EVENT)) {
		if (hdcp.hdcp_state != HDCP_DISABLED) {
			//hdcp_request_dss();		// kibum.lee@lge.com 2011.09.08 comment
			hdcp_wq_disable();
			//hdcp_release_dss();		// kibum.lee@lge.com 2011.09.08 comment

			if (event == HDCP_DISABLE_CTL) {
				if (hdcp.en_ctrl) {
					backup_retry_count = hdcp.en_ctrl->nb_retry;	// kibum.lee@lge.com
					kfree(hdcp.en_ctrl);
					hdcp.en_ctrl = 0;
				}

				hdcp.hdcp_state = HDCP_DISABLED;
			}
			else
				hdcp.hdcp_state = HDCP_ENABLE_PENDING;
            if (event == HDCP_STOP_FRAME_EVENT) {
				complete(&hdcp_comp);
            }

			hdcp.auth_state = HDCP_STATE_DISABLED;
		}

		hdcp.pending_disable = 0;
	}

	switch (hdcp.hdcp_state) {

	/* State */
	/*********/
	case HDCP_DISABLED:
        DBG("HDCP_DISABLED\n");
		/* HDCP enable control or re-authentication event */
		if (event == HDCP_ENABLE_CTL) {
			hdcp.retry_cnt = hdcp.en_ctrl->nb_retry;
			DBG("-2army hdcp.retry_cnt =%d\n", hdcp.retry_cnt);
			if (hdcp.hdmi_state == HDMI_STARTED) {
			//	hdcp_request_dss();		// kibum.lee@lge.com 2011.09.08 comment
				hdcp_wq_start_authentication();
			//	hdcp_release_dss();		// kibum.lee@lge.com 2011.09.08 comment
			}
			else
				hdcp.hdcp_state = HDCP_ENABLE_PENDING;
		}

		break;

	/* State */
	/*********/
	case HDCP_ENABLE_PENDING:
        DBG("HDCP_ENABLE_PENDING\n");
		/* HDMI start frame event */
		if (event == HDCP_START_FRAME_EVENT) {
		//	hdcp_request_dss();		// kibum.lee@lge.com 2011.09.08 comment
			hdcp_wq_start_authentication();
		//	hdcp_release_dss();		// kibum.lee@lge.com 2011.09.08 comment
		}

		break;

	/* State */
	/*********/
	case HDCP_AUTHENTICATION_START:
        DBG("HDCP_AUTHENTICATION_START\n");
		/* Re-authentication */
		if (event == HDCP_AUTH_REATT_EVENT)
        {
			//hdcp_request_dss();		// kibum.lee@lge.com 2011.09.08 comment
			hdcp_wq_start_authentication();
			//hdcp_release_dss();		// kibum.lee@lge.com 2011.09.08 comment
		}
		break;
		
	/* State */
	/*********/
	case HDCP_WAIT_R0_DELAY:
        DBG("HDCP_WAIT_R0_DELAY\n");
		/* R0 timer elapsed */
		if (event == HDCP_R0_EXP_EVENT) {
			//hdcp_request_dss();		// kibum.lee@lge.com 2011.09.08 comment
			hdcp_wq_check_r0();
			//hdcp_release_dss();		// kibum.lee@lge.com 2011.09.08 comment
		}
		break;

	/* State */
	/*********/
	case HDCP_WAIT_KSV_LIST:
        DBG("HDCP_WAIT_KSV_LIST\n");
		/* Ri failure */
		if (event == HDCP_RI_FAIL_EVENT) {
			/* KSV list timeout is running and should be canceled */
            // wooho47.jung@lge.com 2011.11.05
            // DEL : no need 
			//hdcp_cancel_work(&hdcp.pending_wq_event);

			//hdcp_request_dss();		// kibum.lee@lge.com 2011.09.08 comment
			hdcp_wq_authentication_failure();
			//hdcp_release_dss();		// kibum.lee@lge.com 2011.09.08 comment
		}
		/* KSV list ready event */
		else if (event == HDCP_KSV_LIST_RDY_EVENT) {
			//hdcp_request_dss();		// kibum.lee@lge.com 2011.09.08 comment
			hdcp_wq_step2_authentication();
			//hdcp_release_dss();		// kibum.lee@lge.com 2011.09.08 comment
		}
		/* Timeout */
		else if (event == HDCP_KSV_TIMEOUT_EVENT) {
			//hdcp_request_dss();		// kibum.lee@lge.com 2011.09.08 comment
			DBG("event == HDCP_KSV_TIMEOUT_EVENT\n");
			hdcp_wq_authentication_failure();
			//hdcp_release_dss();		// kibum.lee@lge.com 2011.09.08 comment
		}
		break;

	/* State */
	/*********/
	case HDCP_LINK_INTEGRITY_CHECK:
        DBG("HDCP_LINK_INTEGRITY_CHECK\n");
		/* Ri failure */
		if (event == HDCP_RI_FAIL_EVENT) {
            DBG("event == HDCP_RI_FAIL_EVENT\n");
			//hdcp_request_dss();		// kibum.lee@lge.com 2011.09.08 comment
			hdcp_wq_authentication_failure();
			//hdcp_release_dss();		// kibum.lee@lge.com 2011.09.08 comment
		}
		break;

	default:
        
		DBG_ERROR("unknow HDCP state %d\n",hdcp.hdcp_state);
		break;
	}

	kfree(hdcp_w);

	hdcp_w = NULL;						// kibum.lee@lge.com 2011.09.08
	DBG("%u hdmi=%d hdcp=%d auth=%d evt=%x %d\n",
		jiffies_to_msecs(jiffies),
		hdcp.hdmi_state,
		hdcp.hdcp_state,
		hdcp.auth_state,
		(event & 0xFF00) >> 8,
		event & 0xFF);

    #if 1
	if(hdcp.auth_state == HDCP_STATE_AUTH_3RD_STEP ){
		printk("====================================\n\n");
		printk("auth_state = HDCP_STATE_AUTH_3RD_STEP hdmi_state[%d] hdcp_state[%d]\n", hdcp.hdmi_state, hdcp.hdcp_state);
	}
	else
    if(hdcp.auth_state == HDCP_STATE_AUTH_FAILURE )
    {
		printk("====================================\n");
		printk("auth_state =HDCP_STATE_AUTH_FAILURE\n\n\n\n\n\n");
	}
    #endif

    // wooho47.jung@lge.com 2011.11.02
    // ADD : for HDCP Uevent
	hdcp_release_dss();
    
    if((event & 0xFF00) >> 8 != 2 
        && (event & 0xFF ) != 2 
        && hdcp.auth_state == HDCP_STATE_AUTH_3RD_STEP)
	{
	    hdcp_state = 1;
        hdcp_send_uevent(hdcp_state);
    }    
    else 
    if(hdcp.auth_state == 0 
        && hdcp.hdmi_state == 1 
        && hdcp.hdcp_state == 0
        && ((event & 0xFF00) >> 8) == 2 
        && (event & 0xFF) == 2
        )
    {
	    hdcp_state = 1;
        hdcp_send_uevent(hdcp_state);        
    }

	mutex_unlock(&hdcp.lock);
    HDCP_LOG_FUNCTION_NAME_EXIT
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_submit_work
 *-----------------------------------------------------------------------------
 */
static struct delayed_work *hdcp_submit_work(int event, int delay, int irq_context)
{
	struct hdcp_delayed_work *work;
    HDCP_LOG_FUNCTION_NAME_ENTRY
    DBG("event %04x\n",event);
    
	if (irq_context)
		work = kmalloc(sizeof(struct hdcp_delayed_work), GFP_ATOMIC);
	else
		work = kmalloc(sizeof(struct hdcp_delayed_work), GFP_KERNEL);

	if (work) {
		work->event = event;
		INIT_DELAYED_WORK(&work->work, hdcp_work_queue);
		schedule_delayed_work(&work->work, msecs_to_jiffies(delay));
	}
	else {
		DBG_ERROR("HDCP: Cannot allocate memory to create work");
		return 0;
	}
    HDCP_LOG_FUNCTION_NAME_EXIT

	return &work->work;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_cancel_work
 *-----------------------------------------------------------------------------
 */
static void hdcp_cancel_work(struct delayed_work **work)
{
// LGE_CHANGE_S [jh.koo kibum.lee] 2011-09-08, hdmi and hdcp lockup issue WA for P940
/*
	if (*work) {
		cancel_delayed_work(*work);
		kfree(*work);
		*work = 0;
	}
*/
	int ret = 0;
    HDCP_LOG_FUNCTION_NAME_ENTRY
    
    // wooho47.jung@lge.com 2011.11.05
    // ADD : cancel work queue backtrace patch for HDCP    
    mutex_lock(&hdcp.event_lock);

	if (*work) {
		ret = cancel_delayed_work(*work);
		if (ret == 1)
		{
			kfree(*work);
			*work = NULL;
			
		}
		else {
			ret = cancel_delayed_work_sync(*work);
            // wooho47.jung@lge.com 2011.11.05
            // ADD : cancel work queue backtrace patch for HDCP
			kfree(*work);
			*work = NULL;
			DBG("result of the hdcp_cancel_work %d\n", ret);
		}
	}
    // wooho47.jung@lge.com 2011.11.05
    // ADD : cancel work queue backtrace patch for HDCP
    mutex_unlock(&hdcp.event_lock);        
    HDCP_LOG_FUNCTION_NAME_EXIT
// LGE_CHANGE_E [jh.koo kibum.lee] 2011-09-08, hdmi and hdcp lockup issue WA for P940
}


/******************************************************************************
 * HDCP callbacks
 *****************************************************************************/


/*-----------------------------------------------------------------------------
 * Function: hdcp_start_frame_cb
 *-----------------------------------------------------------------------------
 */
static void hdcp_start_frame_cb(void)
{
	//DBG("hdcp_start_frame_cb() %u", jiffies_to_msecs(jiffies));
    HDCP_LOG_FUNCTION_NAME_ENTRY
	/* Cancel any pending work */
//	mutex_lock(&hdcp.lock);

    
    if(&hdcp.pending_start && hdcp.pending_start )
	    hdcp_cancel_work(&hdcp.pending_start);

    
    if(&hdcp.pending_wq_event  && hdcp.pending_wq_event )
	    hdcp_cancel_work(&hdcp.pending_wq_event);


	if(hdcp.en_ctrl != NULL && hdcp.retry_cnt > 0)
		hdcp.hdcp_state = HDCP_ENABLE_PENDING;	// kibum.lee@lge.com	// kibum.lee@lge.com force settings 
		
	hdcp.pending_start = hdcp_submit_work(HDCP_START_FRAME_EVENT,
					      HDCP_ENABLE_DELAY,
					      0);
//	mutex_unlock(&hdcp.lock);
    HDCP_LOG_FUNCTION_NAME_EXIT


}

/*-----------------------------------------------------------------------------
 * Function: hdcp_stop_frame_cb
 *-----------------------------------------------------------------------------
 */
static void hdcp_stop_frame_cb(void)
{
	int i = 10;
    HDCP_LOG_FUNCTION_NAME_ENTRY

	/* Cancel any pending work */
//	mutex_lock(&hdcp.lock);

    if(&hdcp.pending_start && hdcp.pending_start )
		hdcp_cancel_work(&hdcp.pending_start);

    if(&hdcp.pending_wq_event && hdcp.pending_wq_event )
		hdcp_cancel_work(&hdcp.pending_wq_event);

	hdcp.pending_disable = 1;
	while(hdcp.hdcp_state == HDCP_KEY_ENCRYPTION_ONGOING && i--)
		mdelay(5);

// kibum.lee@lge.com ...
	if ((hdcp.hdcp_state != HDCP_DISABLED) && (hdcp.hdmi_restart == 0))
		INIT_COMPLETION(hdcp_comp);
	hdcp_submit_work(HDCP_STOP_FRAME_EVENT, 0, 0);

	/* Function is blocking until stop frame event fully processed
	 * in HDCP workqueue to avoid HDMI driver powering down DSS/HDMI
	 */
	if ((hdcp.hdcp_state != HDCP_DISABLED) && (hdcp.hdmi_restart == 0)) {
		DBG("HDCP stop callback blocked %u",
					jiffies_to_msecs(jiffies));

		if (wait_for_completion_timeout(&hdcp_comp,
			msecs_to_jiffies(HDCP_STOP_FRAME_BLOCKING_TIMEOUT)))
			printk("HDCP stop callback unblocked");
		else
			printk(KERN_ERR "HDCP: stop frame callback blocking "
					"timeout %u\n",
						jiffies_to_msecs(jiffies));
	}
// kibum.lee@lge.com ...

//	mutex_unlock(&hdcp.lock);
    HDCP_LOG_FUNCTION_NAME_EXIT

}

/*-----------------------------------------------------------------------------
 * Function: hdcp_irq_cb
 *-----------------------------------------------------------------------------
 */
static void hdcp_irq_cb(int status)
{
    HDCP_LOG_FUNCTION_NAME_ENTRY
	DBG("status=%04x\n", status);
// by Joshua
	if(status & HDMI_CONNECT)
		load_hdcp_key();

	/* Disable auto Ri/BCAPS immediately */
	if ((status & HDMI_RI_ERR) ||
	    (status & HDMI_BCAP) ||
	    (status & HDMI_HPD_LOW)) {
		hdcp_lib_auto_ri_check(false);
		hdcp_lib_auto_bcaps_rdy_check(false);
	}

	/* Work queue execution not required if HDCP is disabled */
	/* TODO: ignore interrupts if they are masked (cannnot access UMASK here
	 * so should use global variable
	 */
	if ((hdcp.hdcp_state != HDCP_DISABLED) &&
	    (hdcp.hdcp_state != HDCP_ENABLE_PENDING)) {
		/* HPD low event provided by HDMI IRQ since already handled by HDMI driver */
		if (status & HDMI_HPD_LOW) {
			hdcp_lib_set_encryption(HDCP_ENC_OFF);
			hdcp.pending_disable = 1;

			hdcp_ddc_abort();

            // wooho47.jung@lge.com 2011.11.09
            // ADD : for HDMI Certificate (Case HDMI)
            if(&hdcp.pending_start && hdcp.pending_start )
                hdcp_cancel_work(&hdcp.pending_start);
            if(&hdcp.pending_wq_event && hdcp.pending_wq_event )
                hdcp_cancel_work(&hdcp.pending_wq_event);            
            hdcp_submit_work(HDCP_HPD_LOW_EVENT, 0, 1);

		}
		
		if (status & HDMI_RI_ERR) {
			hdcp_lib_set_av_mute(AV_MUTE_SET);
			hdcp_lib_set_encryption(HDCP_ENC_OFF);
			hdcp_submit_work(HDCP_RI_FAIL_EVENT, 0, 1);
		}
		/* RI error takes precedence over BCAP */
		else if (status & HDMI_BCAP) {
			hdcp_submit_work(HDCP_KSV_LIST_RDY_EVENT, 0, 1);
		}
	}

	// kibum.lee@lge.com
	if(status & HDMI_DISCONNECT)
	{
	    DBG("status=hdcp HDMI_DISCONNECT\n");
        
		if(hdcp.en_ctrl == NULL)
			hdcp.retry_cnt =backup_retry_count;
		else
			hdcp.retry_cnt = hdcp.en_ctrl->nb_retry;
	}
    HDCP_LOG_FUNCTION_NAME_EXIT
}

/******************************************************************************
 * HDCP control from ioctl
 *****************************************************************************/


/*-----------------------------------------------------------------------------
 * Function: hdcp_enable_ctl
 *-----------------------------------------------------------------------------
 */
static long hdcp_enable_ctl(void __user *argp)
{
    HDCP_LOG_FUNCTION_NAME_ENTRY
	DBG("ENABLE %u\n", jiffies_to_msecs(jiffies));

	if (hdcp.en_ctrl == 0) {
		hdcp.en_ctrl =
			kmalloc(sizeof(struct hdcp_enable_control),
							GFP_KERNEL);

		if (hdcp.en_ctrl == 0) {
			DBG_ERROR("Cannot allocate memory for HDCP enable control struct\n");
			return -EFAULT;
		}
	}
	
	if (copy_from_user(hdcp.en_ctrl, argp,
			   sizeof(struct hdcp_enable_control))) {
		DBG_ERROR("Error copying from user space - enable ioctl");
		return -EFAULT;
	}

	/* Post event to workqueue */
	if (hdcp_submit_work(HDCP_ENABLE_CTL, 0, 0) == 0)
	{
	    DBG_ERROR("HDCP_ENABLE_CTL Error");
	    return -EFAULT;
    }
    
    HDCP_LOG_FUNCTION_NAME_EXIT

	return 0;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_disable_ctl
 *-----------------------------------------------------------------------------
 */
static long hdcp_disable_ctl(void)
{
	//DBG("hdcp_ioctl() - DISABLE %u", jiffies_to_msecs(jiffies));
    HDCP_LOG_FUNCTION_NAME_ENTRY
//	mutex_lock(&hdcp.lock);

    if(&hdcp.pending_start && hdcp.pending_start )
		hdcp_cancel_work(&hdcp.pending_start);

    if(&hdcp.pending_wq_event && hdcp.pending_wq_event )
		hdcp_cancel_work(&hdcp.pending_wq_event);

//	mutex_unlock(&hdcp.lock);

	hdcp.pending_disable = 1;
	/* Post event to workqueue */
	if (hdcp_submit_work(HDCP_DISABLE_CTL, 0, 0) == 0)
	{
	    DBG_ERROR("HDCP_DISABLE_CTL Error");
	    return -EFAULT;
    }

    HDCP_LOG_FUNCTION_NAME_EXIT

	return 0;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_query_status_ctl
 *-----------------------------------------------------------------------------
 */
static long hdcp_query_status_ctl(void __user *argp)
{
	uint32_t *status = (uint32_t *)argp;
    HDCP_LOG_FUNCTION_NAME_ENTRY

	//DBG("hdcp_ioctl() - QUERY %u", jiffies_to_msecs(jiffies));

	mutex_lock(&hdcp.lock);

	*status = hdcp.auth_state;

	/* Read SW authentication status (used to bypass HW status) */
	mutex_unlock(&hdcp.lock);
    HDCP_LOG_FUNCTION_NAME_EXIT

	return 0;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_encrypt_key_ctl
 *-----------------------------------------------------------------------------
 */
static long hdcp_encrypt_key_ctl(void __user *argp)
{
	struct hdcp_encrypt_control *ctrl;
	uint32_t *out_key;

    HDCP_LOG_FUNCTION_NAME_ENTRY
	//DBG("hdcp_ioctl() - ENCRYPT KEY %u", jiffies_to_msecs(jiffies));

	mutex_lock(&hdcp.lock);

	if (hdcp.hdcp_state != HDCP_DISABLED) {
		DBG_ERROR("Cannot encrypt keys while HDCP is enabled\n");
		mutex_unlock(&hdcp.lock);
		return -EFAULT;
	}

	hdcp.hdcp_state = HDCP_KEY_ENCRYPTION_ONGOING;
//	mutex_unlock(&hdcp.lock);

	/* Encryption happens in ioctl / user context */
	ctrl = kmalloc(sizeof(struct hdcp_encrypt_control),
		       GFP_KERNEL);

	if (ctrl == 0) {
		DBG_ERROR("Cannot allocate memory for HDCP encryption control struct\n");
		mutex_unlock(&hdcp.lock);
		return -EFAULT;
	}

	out_key = kmalloc(sizeof(uint32_t) *
					DESHDCP_KEY_SIZE, GFP_KERNEL);

	if (out_key == 0) {
		DBG_ERROR("HDCP: Cannot allocate memory for HDCP encryption output key\n");
		kfree(ctrl);	
		mutex_unlock(&hdcp.lock);
		return -EFAULT;
	}

	if (copy_from_user(ctrl, argp,
				sizeof(struct hdcp_encrypt_control))) {
		DBG_ERROR("Error copying from user space - encrypt ioctl\n");
		kfree(ctrl);
		kfree(out_key);
		mutex_unlock(&hdcp.lock);
		return -EFAULT;
	}

	hdcp_request_dss();

	/* Call encrypt function */
	hdcp_3des_encrypt_key(ctrl, out_key);

	hdcp_release_dss();

	hdcp.hdcp_state = HDCP_DISABLED;
	mutex_unlock(&hdcp.lock);
	/* Store output data to output pointer */
	if (copy_to_user(ctrl->out_key, out_key,
				sizeof(uint32_t)*DESHDCP_KEY_SIZE)) {
		DBG_ERROR("Error copying to user space - encrypt ioctl\n");
		goto hdcp_encrypt_error;
	}

	kfree(ctrl);
	kfree(out_key);
    HDCP_LOG_FUNCTION_NAME_EXIT
	return 0;

hdcp_encrypt_error:
	kfree(ctrl);
	kfree(out_key);
    HDMI_LOG_FUNCTION_NAME_EXIT_ERROR
	return -EFAULT;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_ioctl
 *-----------------------------------------------------------------------------
 */
long hdcp_ioctl(struct file *fd, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case HDCP_ENABLE:
		return hdcp_enable_ctl(argp);

	case HDCP_DISABLE:
		return hdcp_disable_ctl();

	case HDCP_ENCRYPT_KEY:
		return hdcp_encrypt_key_ctl(argp);

	case HDCP_QUERY_STATUS:
		return hdcp_query_status_ctl(argp);

	default:
		return -ENOTTY;
	} /* End switch */
}


/******************************************************************************
 * HDCP driver init/exit
 *****************************************************************************/


static struct file_operations hdcp_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = hdcp_ioctl,
};

static struct cdev hdcp_cdev;

/*-----------------------------------------------------------------------------
 * Function: hdcp_init
 *-----------------------------------------------------------------------------
 */
static int __init hdcp_init(void)
{
	//DBG("hdcp_init() %u", jiffies_to_msecs(jiffies));
    HDCP_LOG_FUNCTION_NAME_ENTRY
	/* Map HDMI WP address */
	hdcp.hdmi_wp_base_addr = ioremap(HDMI_WP, 0x1000);

	if (!hdcp.hdmi_wp_base_addr) {
		DBG_ERROR("HDMI WP IOremap error\n");
		return -EFAULT;
	}

	/* Map DESHDCP in kernel address space */
	hdcp.deshdcp_base_addr = ioremap(DSS_SS_FROM_L3__DESHDCP, 0x34);

	if (!hdcp.deshdcp_base_addr) {
		DBG_ERROR("DESHDCP IOremap error\n");
		goto err_map_deshdcp;
	}

	mutex_init(&hdcp.lock);
    // wooho47.jung@lge.com 2011.11.05
    // ADD : cancel work queue backtrace patch for HDCP
    mutex_init(&hdcp.event_lock);

	/* Get the major number for this module */
	if (alloc_chrdev_region(&hdcp.dev_id, 0, 1, "hdcp")) {
		DBG_ERROR("Cound not register character device\n");
		goto err_register;
	}

	/* Initialize character device */
	cdev_init(&hdcp_cdev, &hdcp_fops);
	hdcp_cdev.owner = THIS_MODULE;
	hdcp_cdev.ops = &hdcp_fops;

	/* add char driver */
	if (cdev_add(&hdcp_cdev, hdcp.dev_id, 1)) {
		DBG_ERROR("Could not add character driver\n");
		goto err_add_driver;
	}

	mutex_lock(&hdcp.lock);

	/* Variable init */
	hdcp.en_ctrl  = 0;
	hdcp.hdcp_state = HDCP_DISABLED;
	hdcp.pending_start = 0;
	hdcp.pending_wq_event = 0;
	hdcp.retry_cnt = 0;
	hdcp.auth_state = HDCP_STATE_DISABLED;
	hdcp.pending_disable = 0;
	hdcp.hdmi_restart = 0;
	spin_lock_init(&hdcp.spinlock);
	
	init_completion(&hdcp_comp);
	hdcp_request_dss();

	/* Register HDCP callbacks to HDMI library */
	if (hdmi_register_hdcp_callbacks(&hdcp_start_frame_cb,
					 &hdcp_stop_frame_cb,
					 &hdcp_irq_cb))
		hdcp.hdmi_state = HDMI_STARTED;
	else
		hdcp.hdmi_state = HDMI_STOPPED;

	hdcp_release_dss();

	mutex_unlock(&hdcp.lock);
    HDCP_LOG_FUNCTION_NAME_EXIT
	return 0;

err_add_driver:
	cdev_del(&hdcp_cdev);

	unregister_chrdev_region(hdcp.dev_id, 1);

err_register:
	mutex_destroy(&hdcp.lock);
    
    // wooho47.jung@lge.com 2011.11.05
    // ADD : cancel work queue backtrace patch for HDCP
    mutex_destroy(&hdcp.event_lock);

	iounmap(hdcp.deshdcp_base_addr);

err_map_deshdcp:
	iounmap(hdcp.hdmi_wp_base_addr);
    HDMI_LOG_FUNCTION_NAME_EXIT_ERROR

	return -EFAULT;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_exit
 *-----------------------------------------------------------------------------
 */
static void __exit hdcp_exit(void)
{
	//DBG("hdcp_exit() %u", jiffies_to_msecs(jiffies));
    HDCP_LOG_FUNCTION_NAME_ENTRY

	mutex_lock(&hdcp.lock);

	if (hdcp.en_ctrl)
		kfree(hdcp.en_ctrl);

	hdcp_request_dss();

	/* Un-register HDCP callbacks to HDMI library */
	hdmi_register_hdcp_callbacks(0, 0, 0);

	hdcp_release_dss();

	/* Unmap HDMI WP / DESHDCP */
	iounmap(hdcp.hdmi_wp_base_addr);
	iounmap(hdcp.deshdcp_base_addr);

	/* Unregister char device */
	cdev_del(&hdcp_cdev);
	unregister_chrdev_region(hdcp.dev_id, 1);

	mutex_unlock(&hdcp.lock);

	mutex_destroy(&hdcp.lock);
    
    // wooho47.jung@lge.com 2011.11.05
    // ADD : cancel work queue backtrace patch for HDCP
    mutex_destroy(&hdcp.event_lock);
    
    HDCP_LOG_FUNCTION_NAME_EXIT
}

/*-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 */
module_init(hdcp_init);
module_exit(hdcp_exit);
MODULE_LICENSE("BSD");
MODULE_DESCRIPTION("OMAP HDCP kernel module");
MODULE_AUTHOR("Fabrice Olivero");

