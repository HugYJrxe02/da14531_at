
#ifndef _USER_APP_H_

#define	_USER_APP_H_


#define  QUEUE_NODE_SIZE   (256-3)

enum{
	PASSTHROUGH_MODE,
	AT_MODE,
};

extern void userAPPInit(void);

#endif
