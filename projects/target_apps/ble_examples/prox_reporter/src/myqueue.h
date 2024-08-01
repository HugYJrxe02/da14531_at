
/*******************************************************************************
 * 文件名称: myqueue.c 
 * 功能: 队列
 * 形参: 无
 * 返回: 无
 * 编辑: Luo
 * 时间：2020-03-25
*******************************************************************************/



#ifndef __QUEUE_H__
#define	__QUEUE_H__
 


#include <stdlib.h>



//#define	CHAIN_QUEUE
#ifdef CHAIN_QUEUE 


typedef struct QueueNode {
	unsigned char length;
	void *pData;
	struct QueueNode *next;
}*pQueueNode;

typedef struct Queue {
	struct QueueNode front;
	struct QueueNode *tail;
}*pQueue;


extern pQueue queueCreate(void);
extern unsigned short queueDequeue(pQueue q, void **pData);

//#define	QUEUE_MAX_LENGTH	50
//typedef struct TestQueueNode {
//	//data[QUEUE_MAX_LENGTH]结构：2Byte长度+数据域
//	uint8_t	 data[QUEUE_MAX_LENGTH];
//	uint16_t count;
//	uint16_t front;
//	uint16_t tail;
//}*TestpQueue, TestQueue;


#else	
/**********************order queue**********************/

#define	QUEUE_MAX_LENGTH	1024 //3072
 
typedef struct queueNode {
	//data[QUEUE_MAX_LENGTH]结构：2Byte长度+数据域
	uint8_t	 data[QUEUE_MAX_LENGTH]; 
	uint16_t count;
	uint16_t front;
	uint16_t tail;
}*pQueue, Queue;


extern volatile Queue	UartRxQueue;
//extern Queue	UartTxQueue;

extern unsigned short queueDequeue(pQueue q, void *pData);
extern void EmptyTheQueue(pQueue q);

#endif

//extern TestQueue UartRxQueueTest;
//extern TestpQueue pTestUartRxQueueTest;

//extern pQueue pUart1RxQueue;
extern void* queueEnqueue(pQueue q, void* Data, unsigned short bytes);
int queueIsEmpty(pQueue q);

extern void queueDestroy(pQueue q);

 
#endif	// __QUEUE_H__


