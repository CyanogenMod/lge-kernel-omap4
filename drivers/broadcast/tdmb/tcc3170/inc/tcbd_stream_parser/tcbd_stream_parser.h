#ifndef __TCBD_STREAM_PARSER_H__
#define __TCBD_STREAM_PARSER_H__

#define __MERGE_EACH_TYPEOF_STREAM__
#define __FIND_HEADER_MORE__


typedef struct
{
	I16U   reserved;
	I08U   subch_id;
	I16U   size;
	I08U   data_type:7;
	I08U   ack_bit:1;
} TDMB_BB_HEADER_TYPE;

typedef I32S (*StreamCallback)(I08U *_stream, I32S _size, I08U _subchId, I08U _type);

TCBB_FUNC void TcbdInitParser(StreamCallback _streamCallback);
TCBB_FUNC I32S TcbdParser(I08U *_stream, I32S _size);

#endif //__TCBD_STREAM_PARSER_H__
