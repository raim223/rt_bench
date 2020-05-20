#ifndef _RT_TASK_H_
#define _RT_TASK_H_

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
/*****************************************************************************/
/* RT_TASKS */
/*****************************************************************************/
#include "embdCOMMON.h"


#ifdef _XENOMAI_TASKS_
	#include <alchemy/task.h> //native -> alchemy
	#include <alchemy/timer.h> //native -> alchemy
	#define printf rt_printf
#else
	#include "rt_posix_task.h"
	#define RT_TASK PT_TASK
	#define SRTIME PRTIME
	#define RTIME PRTIME
	#define TM_NOW TMR_NOW
	#define rt_timer_read pt_timer_read
	#define rt_timer_spin pt_timer_spin
	#define rt_timer_ns2ticks pt_timer_ns2ticks
#endif
/*****************************************************************************/
/* Real-time Task */
/*****************************************************************************/
int create_rt_task(RT_TASK *task, char *name, int prio);
int set_rt_task_period(RT_TASK *task, RTIME period);
int start_rt_task(int enable, RT_TASK *task, void (*fun)(void *cookie));
void wait_rt_period(RT_TASK *task);
void delete_rt_task(void);
void print_xeno_skin(void);
#endif //_RT_TASK_H_