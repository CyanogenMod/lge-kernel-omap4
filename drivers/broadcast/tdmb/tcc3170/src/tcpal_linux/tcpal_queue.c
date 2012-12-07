#include "tcpal_os.h"
#include "tcpal_debug.h"

#include "tcbd_feature.h"
#include "tcpal_queue.h"

I32S TcbdQueueIsFull(TcbdQueue_t *_queue)
{
    if( _queue->front == ((_queue->rear+1)%_queue->qsize) )
    {
        return 1;
    }

    return 0;
}

I32S TcbdQueueIsEmpty(TcbdQueue_t *_queue)
{
    if( _queue->front == _queue->rear)
    {
        return 1;
    }
    return 0;
}

void TcbdInitQueue(TcbdQueue_t *_queue, I08U* _buffer, I32S _buffSize)
{
    TcpalMemorySet((void*)_queue->q, 0, sizeof(sizeof(TcbdQueueItem_t) * TCBD_QUEUE_SIZE));
    _queue->front        = 0;
    _queue->rear         = 0;
    _queue->qsize        = TCBD_QUEUE_SIZE;
    _queue->buffSize     = _buffSize;
    _queue->globalBuffer = _buffer;
    _queue->pointer      = 0;

    TcpalCreateSemaphore(&_queue->sem, "TcbdQueue", 0);

    //TcbdDebug(1, "buff :0x%X, size:%d\n", _buffer, _buffSize);
}

void TcbdDeinitQueue(TcbdQueue_t *_queue)
{
    _queue->front    = 0;
    _queue->rear     = 0;
    _queue->qsize    = 0;
    _queue->buffSize = 0;

    TcpalDeleteSemaphore(&_queue->sem);
}

void TcbdResetQueue(TcbdQueue_t *_queue)
{
    _queue->front   = 0;
    _queue->rear    = 0;
    _queue->pointer = 0;
    TcpalMemorySet(_queue->q, 0, sizeof(sizeof(TcbdQueueItem_t)) * _queue->qsize);
}

I32S TcbdEnqueue(TcbdQueue_t *_queue, I08U *_chunk, I32S _size, I32S _type)
{
    //I32U wrapSize;
    if(_chunk == NULL || _size <= 0)
    {
        TcbdDebug(DEBUG_ERROR, "Invalid argument!! \n");
        return -1;
    }

    TcpalSemaphoreLock(&_queue->sem);

    if(TcbdQueueIsFull(_queue))
    {
        TcbdDebug(DEBUG_ERROR, "Queue Full!! \n");
        _queue->pointer = 0;
    }

    if(_queue->q[_queue->rear].buffer < _queue->q[_queue->front].buffer)
    {
        I32U nextPosOfRear = (I32U)_queue->q[_queue->rear].buffer + _size;
        I32U currPosOfFront = (I32U)_queue->q[_queue->front].buffer;

        if(nextPosOfRear > currPosOfFront)
        {
            TcbdDebug(DEBUG_ERROR, "Buffer overflow!!\n");
            TcbdResetQueue(_queue);
            TcpalSemaphoreUnLock(&_queue->sem);
            return -1;
        }
    }

    _queue->q[_queue->rear].buffer = _queue->globalBuffer + _queue->pointer;

    if(_queue->pointer + _size >= _queue->buffSize)
        _queue->pointer = 0;
    else
        _queue->pointer += _size;

    TcpalMemoryCopy(_queue->q[_queue->rear].buffer, _chunk, _size);
    //TcbdDebug(DEBUG_ERROR, "rear :%d, pos:0x%08X\n", 
    //    _queue->rear, _queue->q[_queue->rear].buffer);

    _queue->q[_queue->rear].size   = _size;
    _queue->q[_queue->rear].type   = _type;
    _queue->rear = (_queue->rear + 1) % _queue->qsize;


    TcpalSemaphoreUnLock(&_queue->sem);

    return 0;
}

I32S TcbdDequeue(TcbdQueue_t *_queue, I08U *_chunk, I32S *_size, I32S *_type)
{
    TcpalSemaphoreLock(&_queue->sem);
    if(TcbdQueueIsEmpty(_queue))
    {
        TcbdDebug(0, "Queue Empty!! \n");
        TcpalSemaphoreUnLock(&_queue->sem);        
        return -1;
    }

    if(_queue->q[_queue->front].size > *_size)
    {
        TcbdDebug(DEBUG_ERROR, "insufficient buffer!! size:%d, qsize:%d\n",
                *_size, _queue->q[_queue->front].size);

        TcpalSemaphoreUnLock(&_queue->sem);
        return -1;
    }

    //TcbdDebug(DEBUG_ERROR,"front :%d, pos:0x%08X\n", 
    //    _queue->front, _queue->q[_queue->front].buffer);
    TcpalMemoryCopy(_chunk, _queue->q[_queue->front].buffer, _queue->q[_queue->front].size);

    *_size = _queue->q[_queue->front].size;
    if(_type) *_type = _queue->q[_queue->front].type;
    _queue->front = (_queue->front + 1) % _queue->qsize;

    TcbdDebug(0, "pos:%d, size:%d\n", _queue->pointer, *_size);
    TcpalSemaphoreUnLock(&_queue->sem);

    return 0;
}

I32S TcbdDequeuePtr(TcbdQueue_t *_queue, I08U **_chunk, I32S *_size, I32S *_type)
{
    TcpalSemaphoreLock(&_queue->sem);

    if(TcbdQueueIsEmpty(_queue))
    {
        //TcbdDebug(DEBUG_ERROR, "Queue Empty!! \n");
        TcpalSemaphoreUnLock(&_queue->sem);
        return -1;
    }

    if(_queue->q[_queue->front].size > *_size)
    {
        TcbdDebug(DEBUG_ERROR, "insufficient buffer!! size:%d, qsize:%d\n",
                *_size, _queue->q[_queue->front].size);

        TcpalSemaphoreUnLock(&_queue->sem);
        return -1;
    }

    //TcbdDebug(DEBUG_ERROR, "front :%d, pos:0x%08X\n", 
    //    _queue->front, _queue->q[_queue->front].buffer);
    *_chunk = _queue->q[_queue->front].buffer;
    *_size = _queue->q[_queue->front].size;
    if(_type) *_type = _queue->q[_queue->front].type;
    
    _queue->front = (_queue->front + 1) % _queue->qsize;

    TcbdDebug(0, "pos:%d, size:%d\n", _queue->pointer, *_size);
    TcpalSemaphoreUnLock(&_queue->sem);

    return 0;
}

I32S TcbdGetFirstQueuePtr(TcbdQueue_t *_queue, I08U **_chunk, I32S *_size, I32S *_type)
{
    TcpalSemaphoreLock(&_queue->sem);

    if(TcbdQueueIsEmpty(_queue))
    {
        TcbdDebug(0, "Queue Empty!! \n");
        TcpalSemaphoreUnLock(&_queue->sem);
        return -1;
    }
    *_size = _queue->q[_queue->front].size;
    *_chunk = _queue->q[_queue->front].buffer;
    if(_type) *_type = _queue->q[_queue->front].type;

    TcbdDebug(0, "pos:%d, size:%d\n", _queue->pointer, *_size);
    TcpalSemaphoreUnLock(&_queue->sem);
    
    return 0;
}

