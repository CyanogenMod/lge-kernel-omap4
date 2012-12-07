#ifndef __TCPAL_DEBUG_H__
#define __TCPAL_DEBUG_H__

#define DEBUG_ERROR         0x00000001
#define DEBUG_INFO          0x00000002
#define DEBUG_DRV_IO        0x00000004
#define DEBUG_API_COMMON    0x00000008
#define DEBUG_TCPAL_OS      0x00000010
#define DEBUG_TCHAL         0x00000020
#define DEBUG_TCPAL_CSPI    0x00000040
#define DEBUG_DRV_COMP      0x00000080
#define DEBUG_DRV_RF        0x00000100
#define DEBUG_TCPAL_I2C     0x00000200
#define DEBUG_DRV_PERI      0x00000400
#define DEBUG_STREAM_PARSER 0x00000800
#define DEBUG_PARSING_TIME  0x00001000
#define DEBUG_PARSING_PROC  0x00002000
#define DEBUG_INTRRUPT      0x00004000
#define DEBUG_STREAM_READ   0x00008000
#define DEBUG_LGE           0x00010000

int printk(const char *fmt, ...);

#define TcbdDebug(__class, __msg, ...) \
do{\
    if(__class&TcbdDebugClass)\
        printk("[%s:%d] " __msg, __func__, __LINE__, ##__VA_ARGS__); \
}while(0)

extern unsigned int TcbdDebugClass;

#endif //__TCPAL_DEBUG_H_
