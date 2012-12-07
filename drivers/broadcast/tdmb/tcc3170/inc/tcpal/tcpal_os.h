#ifndef __TCPAL_OS_H__
#define __TCPAL_OS_H__
#include "tcpal_types.h"

/* For TimeCheck */
#define TCPAL_MAX_TIMECNT 0xFFFFFFFFFFFFFFFFULL
TCBB_FUNC TcpalTime_t TcpalGetCurrentTimeMs (void);
TCBB_FUNC TcpalTime_t TcpalGetTimeIntervalMs (TcpalTime_t _startTimeCount);

/* for sleep */
TCBB_FUNC void TcpalSleep(I32S _ms);
TCBB_FUNC void TcpalSleepUs(I32S _us);

/* for memory allocation, free, set */
TCBB_FUNC void* TcpalMemoryAllocation(I32U _size);
TCBB_FUNC void  TcpalMemoryFree(void *_ptr);
TCBB_FUNC void* TcpalMemorySet(void* _dest, I32U _data, I32U _cnt);
TCBB_FUNC void* TcpalMemoryCopy(void* _dest, const void* _src, I32U _cnt);
TCBB_FUNC I32S  TcpalMemoryCompare(void* _s1, const void* _s2, I32U _cnt);

/* For Semaphore */
#define TCPAL_INFINITE_SEMAPHORE  0xFFFFFFFFUL

TCBB_FUNC I32S TcpalCreateSemaphore(TcpalSem_t *_semaphore, I08S *_name, I32U _initialCount);
TCBB_FUNC I32S TcpalDeleteSemaphore(TcpalSem_t *_semaphore);
TCBB_FUNC I32S TcpalSemaphoreLock(TcpalSem_t *_semaphore);
TCBB_FUNC I32S TcpalSemaphoreUnLock(TcpalSem_t *_semaphore);

#endif
