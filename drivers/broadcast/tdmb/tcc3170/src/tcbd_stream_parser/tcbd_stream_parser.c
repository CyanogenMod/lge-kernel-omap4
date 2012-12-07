#include "tcpal_os.h"
#include "tcpal_debug.h"

#include "tcbd_feature.h"
#include "tcbd_stream_parser.h"

#define SIZE_MSC_CACHE_BUFF    (188*20)
#define SIZE_FIC_CACHE_BUFF    (388*3)
#define SIZE_STATUS_CACHE_BUFF (32*3)

#define SIZE_MSC_STACK_BUFF    (1024*16)
#define SIZE_FIC_STACK_BUFF    (388*10)
#define SIZE_STATUS_STACK_BUFF (32*10)

typedef enum _ParserState_t
{
    StateNoSync = 0,
    StateReadNext,
    StateOther,
    StateStatus,
    StateMsc,
    StateFic,
    StateGarbage,
} ParserState_t;

typedef enum _DataType_t
{
    DataTypeMsc = 0,
    DataTypeFic,
    DataTypeStatus,
    DataTypeOther,
    MaxDataType
} DataType_t;

#define HEADER_SIZE 4
#define SYNC_BYTE 0x5C
typedef struct _StreamHeader_t
{
    I08U sync;        /**< sync byte. must be 0x5C */
    I08U subch;       /**< sub channel id */
    I16U dataSize;
    I08U parity;
    DataType_t type;
} StreamHeader_t;

#if defined(__MERGE_EACH_TYPEOF_STREAM__)
#define MAX_NUM_CHUNKS 10
typedef struct _MergedStream_t
{
    I32S numChunk; /**< The number of chunk. For debugging purpose*/
    I32S currPos;  /**< Current position of buffer of merged stream */
    I08U *buffer;  /**< Buffer for stacking stream chunk */
    I08U subchId;
} MergedStream_t;
#endif //__MERGE_EACH_TYPEOF_STREAM__

typedef struct _ParserData_t
{
    ParserState_t state;
    I32S remain;
    I32S nextRead;
    I08U *buffer;
    StreamHeader_t header;
#if defined(__MERGE_EACH_TYPEOF_STREAM__)
    MergedStream_t mergedStream[(I32S)MaxDataType];
#endif //__MERGE_EACH_TYPEOF_STREAM__
    StreamCallback streamCallback;
} ParserData_t;

static struct _ParserData_t Parser;

inline static I08U TcbdParserParityCheck(I08U *parity)
{
    I32U i, k;
    I08U p = 0;

    for (i = 0; i < 4; i++)
    {
        for (k = 0; k < 8; k++)
        {
            p = (p + ((parity[i] >> k) & 1)) & 1;
        }
    }

    if (p == 0)
        TcbdDebug(DEBUG_ERROR, "Odd parity error\n");

    return p;
}

static I32S TcbdFindSync(ParserData_t *_parser)
{
    I32S i;
    I32S size = _parser->remain;
    I08U *stream = _parser->buffer;

#if defined(__FIND_HEADER_MORE__)
    I32S j, first = 0, numFound = 0, dataSize;
#endif //__FIND_HEADER_MORE__

    /* 1. SYNC_BYTE must exist at stream[3] */
    if( (stream[3] == SYNC_BYTE) && (TcbdParserParityCheck(stream) == 1 ) )
    {
        return 0;
    }
    /* 2. if SYNC_BYTE doesn't exist at stream[3] then search SYNC_BYTE at whole stream*/
    for(i = 0; i < size; i++)
    {
        if( (i < 3) || (stream[i] != SYNC_BYTE) ) continue;

        if(TcbdParserParityCheck(&stream[i - 3]) != 1)
        {
            TcbdDebug(DEBUG_ERROR, "parity error!! find next byte, offset:%d, header:0x%08X\n",
                i, SWAP32(*((I32U*)&stream[i-3])) );
            continue;
        }
#if !defined(__FIND_HEADER_MORE__)
        TcbdDebug(DEBUG_STREAM_PARSER, "found header offset %d\n", i-3);
        return i - 3;
#else //__FIND_HEADER_MORE__
        numFound = 1;
        first = j = i - 3;
        do
        {
            dataSize = (stream[j + 1] << 7 | (stream[j+0] >> 1)) << 2;
            j += (dataSize + 4);
            if(j >= size) break;

            if( (stream[j + 3] == SYNC_BYTE) && (TcbdParserParityCheck(stream + j) == 1))
            {
                TcbdDebug(DEBUG_STREAM_PARSER, "header ok pos:%d, remain:%d\n", j, size);
                numFound++;
            }
            else
            {
                TcbdDebug(DEBUG_ERROR, "Header ERR!! [%02X] j = %d, size=%d, dataSize:%d\n",
                        SWAP32(*((I32U*)(stream+j))), j, size, dataSize);
                numFound--;
                break;
            }
        }
        while(j < size);

        if(numFound > 1)
        {
            TcbdDebug(DEBUG_STREAM_PARSER, "offset:%d, header ok count : %d, 0x%08X\n",
                first, numFound, SWAP32(*((I32U*)(stream+first))));
            return first;
        }
#endif //__FIND_HEADER_MORE__
    }
    return -i;
}

inline static I32S TcbdParseHeader(ParserData_t *_parser)
{
    I08U *stream = _parser->buffer;
    StreamHeader_t *header = &_parser->header;

    if( (stream[3] != SYNC_BYTE) || (TcbdParserParityCheck(stream) != 1 ) )
    {
        TcbdDebug(DEBUG_ERROR, "wrong header! header:0x%08X\n", SWAP32(*(I32U*)stream) );
        TcpalMemorySet(header, 0, sizeof(StreamHeader_t));
        return HEADER_SIZE;
    }
    header->sync = stream[3];
    header->type = (stream[2] & 0xC0) >> 6;
    header->dataSize = ((stream[1]<<7) | (stream[0]>>1)) << 2;
    header->subch = stream[2] & 0x3F;

    _parser->state = StateNoSync;
    switch((DataType_t)header->type)
    {
        case DataTypeMsc:
            if(header->dataSize != TCBD_FIC_SIZE)
                _parser->state = StateMsc;
            break;
        case DataTypeFic:
            if(header->subch)
                _parser->state = StateGarbage;
            else
                _parser->state = StateFic;
            break;
        case DataTypeStatus: _parser->state = StateStatus; break;
        case DataTypeOther: _parser->state = StateOther; break;
        default: break;
    }
    return HEADER_SIZE;
}

inline static void TcbdStackChunk(ParserData_t *_parser, I08U *_buffer)
{
    I08U *streamChunk = (_buffer) ? _buffer : _parser->buffer;
    StreamHeader_t *header = &_parser->header;
#if defined(__MERGE_EACH_TYPEOF_STREAM__)
    static I08U MscBuffer[SIZE_MSC_STACK_BUFF];
    static I08U FicBuffer[SIZE_FIC_STACK_BUFF];
    static I08U StatusBuffer[SIZE_STATUS_STACK_BUFF];

    I08U *bufferForMerging[] = {MscBuffer + sizeof(TDMB_BB_HEADER_TYPE), FicBuffer, StatusBuffer};
    MergedStream_t *merged = &_parser->mergedStream[header->type];

    if(header->type > DataTypeStatus)
    {
        TcbdDebug(DEBUG_ERROR, "unknown stream type! \n");
        return;
    }
    merged->numChunk++; //useless, just for debugging
    if(!merged->buffer)
    {
        merged->buffer = bufferForMerging[header->type];
    }
    merged->subchId = header->subch;

    TcbdDebug(DEBUG_PARSING_PROC, "type:%d, buffer:0x%X currpos:%d, size:%d\n",
            header->type, merged->buffer, merged->currPos, header->dataSize);
    TcpalMemoryCopy(merged->buffer + merged->currPos, streamChunk, header->dataSize);
    merged->currPos += header->dataSize;

#else //__MERGE_EACH_TYPEOF_STREAM__
    if(_parser->streamCallback)
    {
        _parser->streamCallback(streamChunk, header->dataSize, header->subch, header->type);
    }
#endif //!__MERGE_EACH_TYPEOF_STREAM__
}

static I32S TcbdPushChunk(ParserData_t *_parser, I08U *_cachedBuffer)
{
    I32S move, preCopied;
    I08U *buffer = _parser->buffer;
    char *type[] = {"msc", "fic", "status", "other"};
    I32S sizeCacheBuff[] = {SIZE_MSC_CACHE_BUFF, SIZE_FIC_CACHE_BUFF, SIZE_STATUS_CACHE_BUFF};
    StreamHeader_t *header = &_parser->header;

    if(_parser->nextRead)
    {
        if(_parser->state != StateGarbage)
        {
            preCopied = header->dataSize - _parser->nextRead;

            TcbdDebug(DEBUG_PARSING_PROC, "send %s data %d bytes, pre:%d, curr:%d, buffer:0x%X\n",
                type[header->type], header->dataSize, preCopied, _parser->nextRead, _cachedBuffer);

            if(header->dataSize > sizeCacheBuff[header->type])
            {
                TcbdDebug(DEBUG_ERROR, "overflow %s cache buffer!!\n", type[header->type]);
            }
            TcpalMemoryCopy(_cachedBuffer + preCopied, buffer, _parser->nextRead);
            TcbdStackChunk(_parser, _cachedBuffer);
        }
        move = _parser->nextRead;
        _parser->state = StateNoSync;
        _parser->nextRead = 0;
    }
    else if(_parser->remain >= header->dataSize)
    {
        if(_parser->state != StateGarbage)
        {
            TcbdDebug(DEBUG_PARSING_PROC, "send %s data %d bytes \n", type[header->type], header->dataSize);
            TcbdStackChunk(_parser, NULL);
        }
        _parser->state = StateNoSync;
        move = header->dataSize;
    }
    else
    {
        if(_parser->state != StateGarbage)
        {
            TcbdDebug(DEBUG_PARSING_PROC, "keep %s data %d bytes buff:0x%X\n",
                type[header->type], _parser->remain, _cachedBuffer);

            if(header->dataSize > sizeCacheBuff[header->type])
            {
                TcbdDebug(DEBUG_ERROR, "overflow %s cache buffer!!\n", type[header->type]);
            }
            TcpalMemoryCopy(_cachedBuffer, buffer, _parser->remain);
        }
        _parser->nextRead = header->dataSize - _parser->remain;
        move = _parser->remain;
    }
    return move;
}

#if defined(__MERGE_EACH_TYPEOF_STREAM__)
static I32S TcbdPushStream(ParserData_t *_parser)
{
    register I32S i;
    MergedStream_t *merged = NULL;

    for(i = 0; i < (I32S)MaxDataType; i++)
    {
        merged = &_parser->mergedStream[i];
        if(!merged->buffer || !merged->currPos) continue;

        /* send merged data to user space */
        if(_parser->streamCallback)
        {
            _parser->streamCallback(merged->buffer, merged->currPos, merged->subchId, i);
            merged->buffer = NULL; merged->currPos = 0; merged->numChunk = 0;
        }
    }
    return 0;
}
#endif //__MERGE_EACH_TYPEOF_STREAM__

I32S TcbdParser(I08U *_stream, I32S _size)
{
    I32S ret = 0;
    register I32S point, move;

    /* buffer for un-handled spare data of each interrupt */
    static I08U MscBufferCache[SIZE_MSC_CACHE_BUFF];
    static I08U FicBufferCache[SIZE_FIC_CACHE_BUFF];
    static I08U StatusBufferCache[SIZE_STATUS_CACHE_BUFF];

    TcpalTime_t time;

    point = move = 0;
    Parser.remain = _size;

    time = TcpalGetCurrentTimeMs();
    while(Parser.remain > 0)
    {
        Parser.buffer = _stream + point;
        switch(Parser.state)
        {
            case StateNoSync:
                ret = TcbdFindSync(&Parser);
                if(ret < 0)
                {
                    TcbdDebug(DEBUG_STREAM_PARSER, "could not find sync byte!! %d\n", ret);
                    TcbdInitParser(NULL);
                    return ret;
                }
                else if(ret > 0)
                {
                    point += ret;
                    Parser.buffer += ret;
                    Parser.remain -= ret;
                }
                move = TcbdParseHeader(&Parser);
                break;
            case StateOther:
                move = TcbdParseHeader(&Parser);
                TcbdDebug(DEBUG_ERROR, "State Other!! size:%d\n", Parser.header.dataSize);
                break;
            case StateMsc:
                move = TcbdPushChunk(&Parser, MscBufferCache);
                break;
            case StateFic:
                move = TcbdPushChunk(&Parser, FicBufferCache);
                break;
            case StateStatus:
                move = TcbdPushChunk(&Parser, StatusBufferCache);
                break;
            case StateGarbage:
                move = TcbdPushChunk(&Parser, NULL);
                TcbdDebug(DEBUG_STREAM_PARSER, "State Garbage!:%d\n", Parser.header.dataSize);
                break;
            default:
                move = 0; point = 0;
                TcbdDebug(DEBUG_ERROR, "something wrong!\n");
                break;
        }

        Parser.remain -= move;
        point += move;
        TcbdDebug(0, "remain:%d, point:%d, move:%d\n", Parser.remain, point, move);
    }
#if defined(__MERGE_EACH_TYPEOF_STREAM__)
    ret = TcbdPushStream(&Parser);
#endif //__MERGE_EACH_TYPEOF_STREAM__
    TcbdDebug(DEBUG_PARSING_TIME, "%Ldms elapsed!\n", TcpalGetTimeIntervalMs(time));
    return ret;
}

void TcbdInitParser(StreamCallback _streamCallback)
{
    StreamCallback bak = NULL;

    if(Parser.streamCallback)
        bak = Parser.streamCallback;

    TcpalMemorySet(&Parser, 0, sizeof(Parser));

    if(_streamCallback)
        Parser.streamCallback = _streamCallback;
    else if(bak)
        Parser.streamCallback = bak;

    TcbdDebug(DEBUG_STREAM_PARSER, "callback:0x%X\n", Parser.streamCallback);
}
