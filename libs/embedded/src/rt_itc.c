/*****************************************************************************/
#include <rt_itc.h>
/*****************************************************************************/
#define MUTEX_MODE TM_INFINITE
/*****************************************************************************/
int create_rt_mutex(RT_MUTEX *mutex, char *name)
{
#ifdef _XENOMAI_TASKS_
	return rt_mutex_create(mutex, name);
#else
	return pt_mutex_create(mutex, name);
#endif
}
/*****************************************************************************/
int delete_rt_mutex(RT_MUTEX *mutex)
{
#ifdef _XENOMAI_TASKS_
	return rt_mutex_delete(mutex);
#else
	return pt_mutex_delete(mutex);
#endif
}
/*****************************************************************************/
int acquire_rt_mutex(RT_MUTEX *mutex)
{
#ifdef _XENOMAI_TASKS_
	return rt_mutex_acquire(mutex, MUTEX_MODE);
#else
	return pt_mutex_acquire(mutex);
#endif
}
/*****************************************************************************/
int release_rt_mutex(RT_MUTEX *mutex)
{
#ifdef _XENOMAI_TASKS_
	return rt_mutex_release(mutex);
#else
	return pt_mutex_release(mutex);
#endif
}
/*****************************************************************************/
