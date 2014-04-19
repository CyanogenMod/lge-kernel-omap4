/*
 *  felicauart.c
 *  
 */
 
/*
 *    INCLUDE FILES FOR MODULE
 */
#include <linux/syscalls.h>
#include <asm/ioctls.h>
#include <linux/termios.h>

#include "felica_uart.h"
#include "felica_gpio.h"
#include <plat/omap-serial.h>
#include <plat/serial.h>

static int fd = -1;

static struct termios console_settings;

//char tx_buf[4096 + 4];
//int tx_count;

/*
* Description : open uart
* Input : None
* Output : success : 0 fail : others
*/
int felica_uart_open(void)
{
//  struct termios newtio;
  int rc;
  mm_segment_t old_fs = get_fs();

  #ifdef FEATURE_DEBUG_LOW
  FELICA_DEBUG_MSG("[FELICA_UART] open_hs_uart - start \n");
  #endif

  if(0 <= fd)
  {
    FELICA_DEBUG_MSG("[FELICA_UART] felica_uart is already opened fd : %d\n",fd);
    return 0;
  }

  set_fs(KERNEL_DS);

    fd = sys_open("/dev/ttyO3", O_RDWR | O_NOCTTY | O_NONBLOCK, 0);
    
  FELICA_DEBUG_MSG("[FELICA_UART] open UART - fd : %d \n",fd);
    
    
    if (fd < 0)
    {
    FELICA_DEBUG_MSG("[FELICA_UART] ERROR - can not sys_open \n");
      set_fs(old_fs);
      return fd;
    }

    {
      /*  Set speed */
      struct termios newtio;

      // yjchoi; 120412 temporary block due to build error
      serial_omap_disable_console_port();
      
      // yjchoi; 120412 temporary block due to build error
      rc = felica_gpio_open(GPIO_FELICA_UART_SW, GPIO_DIRECTION_OUT, GPIO_LOW_VALUE);

      felica_gpio_write(GPIO_FELICA_UART_SW, 0);
      sys_ioctl(fd, TCGETS, (unsigned long)&newtio);
      memcpy(&console_settings, &newtio, sizeof(struct termios));

      memset(&newtio, 0, sizeof(newtio));
      newtio.c_cflag = B460800 | CS8 | CLOCAL | CREAD;
      newtio.c_cc[VMIN] = 1;
      newtio.c_cc[VTIME] = 5;
      sys_ioctl(fd, TCFLSH, TCIOFLUSH);
      sys_ioctl(fd, TCSETSF, (unsigned long)&newtio);
    }
    set_fs(old_fs);

  #ifdef FEATURE_DEBUG_LOW
  FELICA_DEBUG_MSG("[FELICA_UART] open_hs_uart - end \n");
  #endif

  return 0;
}

/*
* Description : close uart
* Input : None
* Output : success : 0
*/
int felica_uart_close(void)
{

  mm_segment_t old_fs = get_fs();

  #ifdef FEATURE_DEBUG_LOW
  FELICA_DEBUG_MSG("[FELICA_UART] close_hs_uart - start \n");
  #endif

  if(0 > fd)
  {
    FELICA_DEBUG_MSG("[FELICA_UART] felica_uart is not opened\n");
    return 0;
  }

    set_fs(KERNEL_DS);
    {
      /*  Set speed */
      struct termios settings;
  
      sys_ioctl(fd, TCGETS, (unsigned long)&settings);
#if 1
      memcpy(&settings, &console_settings, sizeof(struct termios));
      FELICA_DEBUG_MSG("[FELICA_UART] Set BAUD rate to 115200\n");
#else
      if((settings.c_cflag & CBAUD) == B460800)
      {
        settings.c_cflag &= ~(CBAUD);
        settings.c_cflag |= B115200;
        FELICA_DEBUG_MSG("[FELICA_UART] Set BAUD rate to 115200\n");
      }
#endif
      sys_ioctl(fd, TCSETS, (unsigned long)&settings);
    }

  sys_close(fd);
  fd = -1;

  felica_gpio_write(GPIO_FELICA_UART_SW, 1);

  // yjchoi; 120412 temporary block due to build error
  serial_omap_enable_console_port();

  set_fs(old_fs);

  #ifdef FEATURE_DEBUG_LOW
  FELICA_DEBUG_MSG("[FELICA_UART] close_hs_uart - end \n");
  #endif

  return 0;

}

/*
* Description : write data to uart
* Input : buf : data count : data length
* Output : success : data length fail : 0
*/
int felica_uart_write(char *buf, size_t count)
{
  mm_segment_t old_fs = get_fs();
  ssize_t n = 0;

  #ifdef FEATURE_DEBUG_LOW
  FELICA_DEBUG_MSG("[FELICA_UART] write_hs_uart - start \n");
  #endif

  if(0 > fd)
  {
    FELICA_DEBUG_MSG("[FELICA_UART] felica_uart is not opened\n");
    return n;
  }

  set_fs(KERNEL_DS);
  
  n = sys_write(fd, buf, count);

  FELICA_DEBUG_MSG("[FELICA_UART] write_hs_uart - sys_write (%d)\n", n);
  
  set_fs(old_fs);

  #ifdef FEATURE_DEBUG_LOW
  FELICA_DEBUG_MSG("[FELICA_UART] write_hs_uart - end \n");
  #endif

  // uart test
  //tx_count = 0;
  //memset(tx_buf, 0x00, 4096+4);
  
  //tx_count = count;
  //memcpy(tx_buf, buf, tx_count);

  return n;
}

/*
* Description : read data from uart
* Input : buf : data count : data length
* Output : success : data length fail : 0
*/
int felica_uart_read(char *buf, size_t count)
{
  mm_segment_t old_fs = get_fs();
  ssize_t n = 0;
  int retry = 10; // uart test 5 -> 10
  
  #ifdef FEATURE_DEBUG_LOW
  FELICA_DEBUG_MSG("[FELICA_UART] read_hs_uart - start \n");
  #endif

  if(0 > fd)
  {
    FELICA_DEBUG_MSG("[FELICA_UART] felica_uart is not opened\n");
    return n;
  }

  set_fs(KERNEL_DS);

  while ((n = sys_read(fd, buf, count)) == -EAGAIN && retry > 0)
  {
    mdelay(50); // Change delay 100 -> 10 -> 50, daehwan.kim, 20110925
    FELICA_DEBUG_MSG("[FELICA_UART] felica_uart_read - retry : %d \n", retry);
    retry--;	
  }

  #ifdef FEATURE_DEBUG_LOW
  FELICA_DEBUG_MSG("[FELICA_UART] read_hs_uart - count: %d, n: %d\n",count ,n);
  #endif

/*
  // uart test
  if ( 0 >= n ){
  	  int i;	  
//	  mdelay(100);

	  felica_uart_close();

	  felica_uart_open();

      sys_write(fd, tx_buf, tx_count);
	  FELICA_DEBUG_MSG("[FELICA_UART] sys_write - retry\n");
	  if(NULL != tx_buf)
	  {
		FELICA_DEBUG_MSG("===== WRITE FELICA COMMAND =====\n");
	   
		for(i=0; i<tx_count; i++)
		{
		  FELICA_DEBUG_MSG(" %02x", tx_buf[i]);
		  if(0 == (i+1)%10)
		  {
			FELICA_DEBUG_MSG("\n");
		  }
		}
		FELICA_DEBUG_MSG("\n\n");
	  }   

		retry = 10;
	  while ((n = sys_read(fd, buf, count)) == -EAGAIN && retry > 0)
	  {
	    mdelay(50);
	    FELICA_DEBUG_MSG("[FELICA_UART] felica_uart_read - retry : %d \n", retry);
	    retry--;	
	  }
	  
  }
*/

  set_fs(old_fs);

  #ifdef FEATURE_DEBUG_LOW
  FELICA_DEBUG_MSG("[FELICA_UART] read_hs_uart - end \n");
  #endif

  return n;
}
/*
* Description : get size of remaing data
* Input : none
* Output : success : data length fail : 0
*/
int felica_uart_ioctrl(int *count)
{
  mm_segment_t old_fs = get_fs();
  int rc = 0;

  #ifdef FEATURE_DEBUG_LOW
  FELICA_DEBUG_MSG("[FELICA_UART] felica_uart_ioctrl - start \n");
  #endif

  if(0 > fd)
  {
    FELICA_DEBUG_MSG("[FELICA_UART] felica_uart is not opened\n");
    return rc;
  }

  set_fs(KERNEL_DS);
  
  rc = sys_ioctl(fd, TIOCINQ, (unsigned long)count);

  set_fs(old_fs);

  #ifdef FEATURE_DEBUG_LOW
  FELICA_DEBUG_MSG("[FELICA_UART] felica_uart_ioctrl - end \n");
  #endif

  return rc;
}
