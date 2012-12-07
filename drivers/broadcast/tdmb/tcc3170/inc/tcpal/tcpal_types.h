
#ifndef __TCPAL_TYPES_H__
#define __TCPAL_TYPES_H__

#ifdef __cplusplus
    #ifdef NULL
        #undef NULL
        #define NULL 0
    #endif
    #define TCBB_FUNC extern "C"
#else
    #ifdef NULL
        #undef NULL
        #define NULL (void*)0
    #else
        #define NULL (void*)0
    #endif
    #define TCBB_FUNC 
#endif

#define SWAP16(x) \
    ((I16U)( \
    (((I16U)(x) & (I16U)0x00ffU) << 8) | \
    (((I16U)(x) & (I16U)0xff00U) >> 8) ))

#define SWAP32(x) \
    ((I32U)( \
    (((I32U)(x) & (I32U)0x000000ffUL) << 24) | \
    (((I32U)(x) & (I32U)0x0000ff00UL) <<  8) | \
    (((I32U)(x) & (I32U)0x00ff0000UL) >>  8) | \
    (((I32U)(x) & (I32U)0xff000000UL) >> 24) ))

#define MIN(x,y)            ( (x) < (y) ? (x) : (y) )
#define MAX(x,y)            ( (x) > (y) ? (x) : (y) )

#define TCBD_TRUE  1 
#define TCBD_FALSE 0

typedef unsigned char I08U; /* 1 byte */
typedef signed char I08S; /* 1 byte */
typedef unsigned short I16U; /* 2 bytes */
typedef signed short I16S; /* 2 bytes */
typedef unsigned long I32U; /* 4 bytes */
typedef signed long I32S; /* 4 bytes */
typedef signed long long I64S; /* 8 bytes */
typedef unsigned long long I64U; /* 8 bytes */

typedef I64U TcpalTime_t;
typedef I32U TcpalSem_t;

#endif
