#ifndef __TCBD_QUEUE_H__
#define __TCBD_QUEUE_H__

#define TCBD_QUEUE_SIZE 50

typedef struct _TcbdQueueItem_t
{
    I08U *buffer;
    I32S size;
    I32S type;
} TcbdQueueItem_t;

typedef struct _TcbdQueue_t
{
    I32S front;
    I32S rear;
    I32S qsize;

    I32S pointer;
    I32S buffSize;
    I08U *globalBuffer;

    TcpalSem_t sem;
    TcbdQueueItem_t q[TCBD_QUEUE_SIZE];
} TcbdQueue_t;

TCBB_FUNC void TcbdInitQueue(TcbdQueue_t *_queue, I08U* buffer, I32S _buffSize);
TCBB_FUNC void TcbdDeinitQueue(TcbdQueue_t *_queue);

TCBB_FUNC I32S TcbdEnqueue(TcbdQueue_t *_queue, I08U *_chunk, I32S _size, I32S _type);
TCBB_FUNC I32S TcbdDequeue(TcbdQueue_t *_queue, I08U *_chunk, I32S *_size, I32S *_type);
TCBB_FUNC I32S TcbdDequeuePtr(TcbdQueue_t *_queue, I08U **_chunk, I32S *_size, I32S *_type);

TCBB_FUNC I32S TcbdGetFirstQueuePtr(TcbdQueue_t *_queue, I08U **_chunk, I32S *_size, I32S *_type);

TCBB_FUNC I32S TcbdQueueIsEmpty(TcbdQueue_t *_queue);
TCBB_FUNC I32S TcbdQueueIsFull(TcbdQueue_t *_queue);
TCBB_FUNC void TcbdResetQueue(TcbdQueue_t *_queue);
#endif //__TCBD_QUEUE_H__
