/*
 *  felica_common.c
 *  
 */
 
/*
 *    INCLUDE FILES FOR MODULE
 */
#include <linux/wakelock.h>
 
#include "felica_common.h"

/*
 *  DEFINE
 */

/*
 *    INTERNAL DEFINITION
 */
 
static struct wake_lock felica_wake_lock;

#ifdef FELICA_LED_SUPPORT
static struct wake_lock felica_rfs_wake_lock;
#endif

/*
 *    FUNCTION DEFINITION
 */

/*
* Description : 
* Input : None
* Output :
*/
void lock_felica_wake_lock(void)
{
  wake_lock(&felica_wake_lock);
}
/*
* Description : 
* Input : None
* Output :
*/
void unlock_felica_wake_lock(void)
{
  wake_unlock(&felica_wake_lock);
}
/*
* Description : 
* Input : None
* Output :
*/
void init_felica_wake_lock(void)
{
  wake_lock_init(&felica_wake_lock, WAKE_LOCK_SUSPEND, "felica");
}
/*
* Description : 
* Input : None
* Output :
*/
void destroy_felica_wake_lock(void)
{
  wake_lock_destroy(&felica_wake_lock);
}

#ifdef FELICA_LED_SUPPORT
/*
* Description :
* Input : None
* Output :
*/
void lock_felica_rfs_wake_lock(void)
{
  wake_lock(&felica_rfs_wake_lock);
}
/*
* Description :
* Input : None
* Output :
*/
void unlock_felica_rfs_wake_lock(void)
{
  wake_unlock(&felica_rfs_wake_lock);
}
/*
* Description :
* Input : None
* Output :
*/
void init_felica_rfs_wake_lock(void)
{
  wake_lock_init(&felica_rfs_wake_lock, WAKE_LOCK_SUSPEND, "felica_rfs");
}
/*
* Description :
* Input : None
* Output :
*/
void destroy_felica_rfs_wake_lock(void)
{
  wake_lock_destroy(&felica_rfs_wake_lock);
}
#endif
