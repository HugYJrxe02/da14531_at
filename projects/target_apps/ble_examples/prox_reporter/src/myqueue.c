
/*******************************************************************************
 * 文件名称: myqueue.c 
 * 功能: 队列
 * 形参: 无
 * 返回: 无
 * 时间：2021-07-07
*******************************************************************************/


#include "myqueue.h"

#include "ke_mem.h"
#include "MyPrintf.h"	
#include "userAPP.h"



#ifdef CHAIN_QUEUE

pQueue queueCreate(void) 
{	
	pQueue q;
	q=(pQueue)malloc(sizeof(struct Queue));
//	q=(pQueue)ke_malloc(sizeof(struct Queue));
	q->front.pData=NULL;
	q->front.next=NULL;
	q->tail=&q->front;
	return q;
}

int queueIsEmpty(pQueue q) 
{
	return &q->front==q->tail;
}

void* queueEnqueue(pQueue q, void* Data, unsigned short bytes) 
{
//	q->tail->next=(pQueueNode)malloc(sizeof(struct QueueNode));
	q->tail->next=(pQueueNode)ke_malloc(sizeof(struct QueueNode), KE_MEM_BLOCK_MAX);
	if(!q->tail->next)
	{
		return NULL;
	}
	
//	q->tail->next->pData = (void *)malloc(bytes);
	q->tail->next->pData = (void *)ke_malloc(bytes, KE_MEM_BLOCK_MAX);
	if(!q->tail->next->pData)
	{
		return NULL;
	}

	memcpy(q->tail->next->pData, Data, bytes);
	q->tail->next->length = bytes;
	q->tail->next->next=NULL;
	q->tail = q->tail->next;
	return q->tail->pData;
}
 
unsigned short queueDequeue(pQueue q, void **pData) 
{
	pQueueNode tmp = q->front.next;
	unsigned short len;
	
	if(tmp==NULL) {
		return NULL;
	}
	len = tmp->length;
	*pData=tmp->pData;
	q->front.next=tmp->next;
//	free(tmp);	
	ke_free(tmp);
	if(q->front.next==NULL)
	{
		q->tail=&q->front;
	}

	return len;	
}

void queueDestroy(pQueue q)
{
	pQueueNode tmp, p = q->front.next;
	while(p!=NULL) 
	{
		tmp = p;
		p = p->next;
		free(tmp);
//		ke_msg_free(tmp);
	}
	free(q);
//	ke_msg_free(q); 
}


#else	
/**********************order queue**********************/
volatile Queue	UartRxQueue = {0};
//Queue	UartTxQueue = {0};

int queueIsEmpty(pQueue q) 
{
//	if ( (0 == q->count || q->count < 0) )
	if (  q->count <= 0 )
	{
//		MyLog("Queue is Empty\n");
        return true;
	}
	else
		return false;
}

bool isFullQueue(pQueue q, unsigned short length)
{
//	if( (q->tail + length)%QUEUE_MAX_LENGTH == q->front || ( (q->count + length) >= QUEUE_MAX_LENGTH) )  //队列满
	if( (q->count + length) >= QUEUE_MAX_LENGTH )
	{
//		MyLog("\n\n+++full q->count = %d, q->tail = %d, length=%d \n", q->count, q->tail, length);

//		MyLog("Queue is Full\n");
        return true;
	}
	if ((q->tail + length)>QUEUE_MAX_LENGTH)
	{
		if ( (q->tail + length)%QUEUE_MAX_LENGTH >= q->front )
		{
			return true;
		}
	}
	
    return false;
}

bool isQueueTail(pQueue q, unsigned short length)
{
	if( (q->tail + length) > QUEUE_MAX_LENGTH  )  //到达队列尾部
    {
//		MyLog("reached the end of the queue\n");
        return true;
	}

    return false;
}

void* queueEnqueue(pQueue q, void* Data, unsigned short bytes) 
{	
	unsigned char static len = sizeof(bytes);
	wdg_reload(WATCHDOG_DEFAULT_PERIOD);
	{
		if(isFullQueue(q, bytes+len))
		{		
			return NULL;
		}

		if ( isQueueTail(q, bytes+len) && ((bytes+len) < q->front))
		{
			q->tail = 0;
		}
		
//		if( (QUEUE_MAX_LENGTH - q->count) >= (bytes+len) )
		if(!isQueueTail(q, bytes+len))
		{
//			MyLog("\n\n+++>1 q->tail = %d, q->count=%d, bytes=%d \n",  q->tail, q->count, bytes);
			memcpy(&(q->data[q->tail]), (char*)&bytes, len);
			memcpy(&(q->data[q->tail+len]), Data, bytes);
			q->tail = (q->tail+bytes+len) % QUEUE_MAX_LENGTH;
					
//			MyLog("+++>3 q->count = %d \n", q->count);
			q->count = (q->count + len + bytes) % QUEUE_MAX_LENGTH;
			
//			MyLog("+++>4 q->count = %d \n", q->count);
//			MyLog("+++>2 count=%d, tail=%d, bytes=%d \n", q->count, q->tail, bytes);
		}
		
	}	
	
	return q;
}

unsigned short queueDequeue(pQueue q, void *pData) 
{
	unsigned short static len = sizeof(unsigned short);
	unsigned short validDataLen = 0;
	static unsigned short count = 0;
	uint16_t Temp = 0xaa;
	wdg_reload(WATCHDOG_DEFAULT_PERIOD);
#if 1
	memcpy(&validDataLen, &(q->data[q->front]), len);

//	MyLog("--->1 q->front=%d, q->tail=%d, q->count=%d, validDataLen=%d\n", q->front, q->tail, q->count, validDataLen);	
//	if (validDataLen <= 0 || validDataLen > NodeSize || 0 == q->count)
//	{		
//		q->front = (q->front+1)%(QUEUE_MAX_LENGTH-3);//2字节表示数据长度+最小1字节数据域=3
//
//		if(count++ > NodeSize)
//		{
//			q->count = 0;
//			q->front = 0;
//			q->tail = 0;
////			memset(q->data, 0, QUEUE_MAX_LENGTH);
//		}
//		
//		return 0;
////		MyLog("%s, (q->front+1)=%d, validDataLen=%d\n", __FUNCTION__, q->front, validDataLen);	
//	}

	Temp = q->front;
	while(validDataLen <= 0 || validDataLen > QUEUE_NODE_SIZE)
	{		
		q->front = (q->front+1)%(QUEUE_MAX_LENGTH-3);//2字节表示数据长度+最小1字节数据域=3
		memcpy(&validDataLen, &(q->data[q->front]), len);
		if(count++ > (QUEUE_MAX_LENGTH - Temp))
		{
			q->front = 0;
//			count = 0;
//			MyLog("--->2 q->front=%d, q->tail=%d, q->count=%d, validDataLen=%d\n", 
//					q->front, q->tail, q->count, validDataLen);
//			break;
		}

		if(Temp == q->front)
		{	
			goto exit;
		}
		if( validDataLen > 0 && validDataLen < QUEUE_NODE_SIZE )
		{	
			break;
		}
	}
//	else
	{
		count = 0;
//		MyLog("\n\n---->3 q->count=%d, q->front = %d, validDataLen=%d\n", q->count, q->front, validDataLen);
		if ( (q->front + len + validDataLen) > QUEUE_MAX_LENGTH)
		{
			q->front = 0;
		}
		else if(queueIsEmpty(&UartRxQueue))
		{
			goto exit;
		}
//		MyLog("--->4 q->count=%d, q->front = %d, validDataLen=%d\n", q->count, q->front, validDataLen);
		memcpy(pData, &(q->data[q->front + len]), validDataLen);
		memset(&(q->data[q->front]), 0, len+validDataLen);

//		MyLog("--->5 q->count=%d, q->front = %d, validDataLen=%d\n", q->count, q->front, validDataLen);
		q->count = (q->count - len - validDataLen)%QUEUE_MAX_LENGTH;
		q->front = (q->front + len + validDataLen)%(QUEUE_MAX_LENGTH-3);	
//		MyLog("--->6 count=%d, front=%d, validDataLen=%d\n", q->count, q->front, validDataLen);
		
	}
	
	if(queueIsEmpty(&UartRxQueue))
	{
exit:		
//		MyLog("%d\n", count++);
		q->front = 0;
		q->tail = 0;
//		wdg_reload(WATCHDOG_DEFAULT_PERIOD);
//		memset(q->data, 0, QUEUE_MAX_LENGTH);
//		wdg_reload(WATCHDOG_DEFAULT_PERIOD);
	}
#endif
//	MyLog("%s, q->count = %d, q->front=%d, return validDataLen=%d\n", __FUNCTION__, q->count, q->front, validDataLen);	
	return validDataLen;
}

void EmptyTheQueue(pQueue q)
{
	q->front = 0;
	q->tail = 0;
	q->count = 0;
	memset(q->data, 0, QUEUE_MAX_LENGTH);
}

#endif


