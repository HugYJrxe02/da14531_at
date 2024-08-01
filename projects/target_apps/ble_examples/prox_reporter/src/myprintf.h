


#ifndef	_MYPRINTF_H_

#define	_MYPRINTF_H_


#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "arch_api.h"
#include "ke_mem.h"
#include "uart.h"





int MyPrintf(const char *fmt, ...);

	#define LOG
	#if defined(LOG)
		int MyLog(const char *fmt, ...);
	#else
		#define MyLog(fmt, args...) {}
	
	#endif









#endif


