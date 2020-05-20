#include <rt_posix_mutex.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
/*****************************************************************************/
int pt_mutex_acquire(PT_MUTEX *mutex)
{
	 return pthread_mutex_lock(&mutex->lock);

}
/*****************************************************************************/
int pt_mutex_create(PT_MUTEX *mutex, char* name)
{
	pthread_mutexattr_t mtx_attr;
	int ret = 0;
	
	mutex->name = name;

	ret = pthread_mutexattr_init(&mtx_attr);
	if (ret != 0)
	{
		fprintf(stderr,"cannot init mutex attribute\n");
		return ret;
	}

	ret = pthread_mutexattr_setprotocol(&mtx_attr,PTHREAD_PRIO_INHERIT);
	if (ret != 0)
	{
		fprintf(stderr,"cannot set mutex prioirity inheritance\n");
		return ret;
	}

	ret = pthread_mutex_init(&mutex->lock,&mtx_attr);
	if (ret != 0)
	{
		fprintf(stderr,"cannot int mutex\n");
		return ret;
	}
	
	ret = pthread_mutexattr_destroy(&mtx_attr);
	if (ret != 0)
	{
		fprintf(stderr,"cannot destroy mutex attribute\n");
		return ret;
	}
	return 0;
}
/*****************************************************************************/
int pt_mutex_delete(PT_MUTEX *mutex)
{
	return pthread_mutex_destroy(&mutex->lock);

}
/*****************************************************************************/
int pt_mutex_release(PT_MUTEX *mutex)
{
	return pthread_mutex_unlock(&mutex->lock);

}
/*****************************************************************************/