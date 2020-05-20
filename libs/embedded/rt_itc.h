#ifndef _RT_ITC_H_
#define _RT_ITC_H_

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
/*****************************************************************************/
/* RT_TASKS */
/*****************************************************************************/
#include "embdCOMMON.h"
#include "rt_tasks.h"

/* we focus on Mutex first */
#ifdef _XENOMAI_TASKS_
	#include <alchemy/mutex.h> // mutex
#else
	#include <rt_posix_mutex.h>
	#define RT_MUTEX PT_MUTEX
#endif
/*****************************************************************************/
/* Real-time ITCs - Mutex */
/*****************************************************************************/
int create_rt_mutex(RT_MUTEX *mutex, char *name);
int delete_rt_mutex(RT_MUTEX *mutex);
int acquire_rt_mutex(RT_MUTEX *mutex);
int release_rt_mutex(RT_MUTEX *mutex);
#endif // _RT_ITC_H_