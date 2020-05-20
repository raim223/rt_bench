/*
 *  This file is owned by the Embedded Systems Laboratory of Seoul National University of Science and Technology
 *  to test EtherCAT master protocol using the IgH EtherCAT master userspace library.	
 *  
 *  
 *
 *
 *  2020 Raimarius Delgado
*/
/****************************************************************************/
#include <rt_posix_task.h>
/****************************************************************************/
struct timespec NS2TIMESPEC(uint64_t nanosecs);
char* _mode_name(PT_MODE mode);
/*****************************************************************************/
int pt_task_create(PT_TASK* task, char* name, int stksize, int prio, PT_MODE mode)
{
	task->name = name;
	task->mode = mode;
	task->s_mode = _mode_name(task->mode);

	int err = pthread_attr_init(&task->thread_attributes);
	if (err)
	{
		TASK_DBG(task->s_mode,"attr_init() failed for %s with err=%d\n", task->name, err);
		return -EATTR;
	}

	if(mode == RT)
	{
		err = pthread_attr_setinheritsched(&task->thread_attributes, PTHREAD_EXPLICIT_SCHED);
		if (err)
		{
			TASK_DBG(task->s_mode,"set explicit sched failed for %s with err=%d\n", task->name, err);
			return -EINHRTSCHD;
		}
	}

	err = pthread_attr_setdetachstate(&task->thread_attributes, PTHREAD_CREATE_DETACHED /*PTHREAD_CREATE_JOINABLE*/);
	if (err)
	{
		TASK_DBG(task->s_mode,"set detach state failed for thread %s with err=%d\n", task->name, err);
		return -EDTCHSTAT;
	}
	
	if(mode == RT)
	{
		err = pthread_attr_setschedpolicy(&task->thread_attributes, SCHED_FIFO);
		if (err)
		{
			TASK_DBG(task->s_mode,"set scheduling policy failed for thread '%s' with err=%d\n", task->name, err);
			return -ESCHEDPOL;
		}
		struct sched_param paramA = { .sched_priority = prio};
		err = pthread_attr_setschedparam(&task->thread_attributes, &paramA);
		if (err)
		{
			TASK_DBG(task->s_mode,"set priority failed for thread '%s' with err=%d\n", task->name, err);
			return -ESETPRIO;
		}
		task->prio = prio;
	}

	if (stksize == 0)
		stksize = 256;
	else if (stksize < 0)
		stksize = -stksize;

	err = pthread_attr_setstacksize(&task->thread_attributes, stksize*1024);
	if (err)
	{
		TASK_DBG(task->s_mode,"set stack size failed for thread '%s' with err=%d\n", task->name, err);
		return -ESETSTKSZ;
	}

	task->stksize = stksize;
	return 0;
}
/*****************************************************************************/
int pt_task_set_periodic(PT_TASK* task,PRTIME idate, PRTIME period)
{
	
	task->fd_timer = -1; 
	/* calc start time of the periodic thread */
	struct timespec start_time;
	if (idate == TMR_NOW){
		if (clock_gettime(CLOCK_TO_USE, &start_time))
		{
			TASK_DBG(task->s_mode,"Failed to call clock_gettime\n" );
			return -ECLKGETTM;
		}
	} else
		start_time = NS2TIMESPEC(idate);


	/* Start one second later from now. */
	start_time.tv_sec += START_DELAY_SECS;
	
	/* if a timerfd is used to make thread periodic (Linux / Xenomai 3),
	 * initialize it before launching thread (timer is read in the loop)
	*/
	struct itimerspec period_timer_conf;
	task->fd_timer = timerfd_create(CLOCK_TO_USE, 0);
	if ( task->fd_timer ==-1 )
	{
		TASK_DBG(task->s_mode,"Failed to create timerfd for thread '%s'\n", task->name);
		return -ETMRFD;
	}
	period_timer_conf.it_value = start_time;
	period_timer_conf.it_interval.tv_sec = period/NANOSEC_PER_SEC;
	period_timer_conf.it_interval.tv_nsec = (period%NANOSEC_PER_SEC) ;

	if ( timerfd_settime(task->fd_timer, TFD_TIMER_ABSTIME, &period_timer_conf, NULL) )
	{
		TASK_DBG(task->s_mode,"Failed to set periodic tor thread '%s' with errno=%d\n", task->name, errno);
		return -ESETPRD;
	}
	task->period = period;
	task->overruns = 0;
	return 0;
}
/*****************************************************************************/
int pt_task_start(PT_TASK* task,void (*entry)(void *arg), void * arg)
{
	int err;

	err = pthread_create(&task->thread, &task->thread_attributes, (void *) entry, arg);
	if (err)
	{
		TASK_DBG(task->s_mode,"Failed to create thread '%s' with err=%d !!!!!\n", task->name, err);
		return -EPTHCREATE;
	}
	else
	{
		pthread_attr_destroy(&task->thread_attributes);
		err = pthread_setname_np(task->thread, task->name);
		if (err)
		{
			TASK_DBG(task->s_mode,"set name failed for thread '%s', err=%d\n", task->name, err);
			return -EPTHNAME;
		}
		TASK_DBG(task->s_mode,"Created thread '%s' period=%lu ns ok.\n", task->name, task->period);
	}
		return 0;
}
/*****************************************************************************/
void pt_task_wait_period(PT_TASK *task)
{
	int err = 0;
	uint64_t ticks;
	
	err = read(task->fd_timer, &ticks, sizeof(ticks));
	if ( err<0 )
	{
		TASK_DBG(task->name,"TimerFd wait period failed with errno=%d\n", errno);
	}
	if ( ticks>1 )
	{
		task->overruns += ticks;
//		TASK_DBG(task->name,"TimerFd wait period missed for thread: overruns=%lu\n", (long unsigned int)ticks);
	}

}
/*****************************************************************************/
PRTIME pt_timer_read(void){
	struct timespec probe;
	PRTIME ret;
	if (clock_gettime(CLOCK_TO_USE,&probe))
		{
			printf("Failed to clock_gettime probe\n" );
			return -ECLKGETTM;
		}
	else
	{
		return (TIMESPEC2NS(probe));
	}
}
/*****************************************************************************/
void pt_timer_spin(PRTIME spintime)
{
	PRTIME end;
	end = pt_timer_read() + spintime;
	while (pt_timer_read() < end)
		cpu_relax();
}
/*****************************************************************************/
struct timespec NS2TIMESPEC(uint64_t nanosecs)
{
	struct timespec ret; 
	ret.tv_sec = nanosecs / NANOSEC_PER_SEC;
	ret.tv_nsec = nanosecs % NANOSEC_PER_SEC;

	return ret;
}
int pt_timer_ns2ticks(PRTIME ticks)
{

}
/*****************************************************************************/
void pt_task_delete(void)
{
	pthread_exit(NULL);
}
/*****************************************************************************/
char* _mode_name(PT_MODE mode)
{
	switch(mode)
	{
		case RT:
			return "RT";
			break;
		case NRT:
			return "NRT";
			break;
		default:
			return "Unknown Mode";
			break;
	}
}
/*****************************************************************************/
