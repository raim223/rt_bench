#ifndef _RT_POSIX_TASK_H_
#define _RT_POSIX_TASK_H_
#define _GNU_SOURCE //for pthread_setname_np
/*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/timerfd.h>
/*****************************************************************************/
#include <embdCOMMON.h>
/*****************************************************************************/
#define START_DELAY_SECS 1 //1sec
#define NANOSEC_PER_SEC 1000000000
#define CLOCK_TO_USE CLOCK_MONOTONIC
#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NANOSEC_PER_SEC + (T).tv_nsec)
#define TMR_NOW (-99)
#define PREDEFINED_STKSIZE (32) //for 32 kb

/* error code */
typedef enum{
	EATTR = 10,
	EINHRTSCHD,
	EDTCHSTAT,
	ESCHEDPOL,
	ESETPRIO,
	ESETSTKSZ,
	ECLKGETTM,
	ETMRFD,
	ESETPRD,
	EPTHCREATE,
	EPTHNAME
}ERROR_CODE;

typedef int FDTIMER; //for fd timer
typedef uint64_t PRTIME; //for timer probe
typedef TASK_TYPE PT_MODE;
/*****************************************************************************/
typedef struct{
	pthread_t thread;
	struct timespec deadline;
	pthread_attr_t thread_attributes;
	PT_MODE mode;
	char* s_mode;
	uint64_t overruns;
	int prio;
	int stksize;
	PRTIME period;
	char* name;
	pid_t pid;

}PT_TASK;
/*****************************************************************************/
/* Creation of real-time periodic task using Xenomai Posix Skin 
 * MyPosixThread -> address of pthread descriptor
 * TimerFdForThread -> FD timer for periodicity (FDTIMER should be set globally to -1 at the beginning 
 *                     e.g., FDTIMER fdMotor = -1; )
 * TaskName -> Desired name of task 
 * Priority -> 0~99 with 99 as the highest priority
 * StackSizeInKo -> Defines the size of stack the system will allocate for the thread in kB
 * Period -> Period / Deadline of the task in nanoseconds
 * *pTaskFunction -> address of the task's body routine
 *****************************************************************************/ 
int pt_task_create(PT_TASK* task,char* name, int stksize, int prio, PT_MODE mode);
/*****************************************************************************/
int pt_task_set_periodic(PT_TASK* task,PRTIME idate, PRTIME period);
/*****************************************************************************/
int pt_task_start(PT_TASK* task,void (*entry)(void *arg) , void* arg);
/*****************************************************************************/
void pt_task_wait_period(PT_TASK *task);
/*****************************************************************************/
void pt_task_delete(void);
/*****************************************************************************/
/* Returns the current system time expressed in nanoseconds
 *****************************************************************************/ 
PRTIME pt_timer_read(void);
/*****************************************************************************/
void pt_timer_spin(PRTIME spintime);
PRTIME pt_timer_ns2ticks(PRTIME ticks);

#define TASK_DBG(mode,format, args...) printf("[%s Task] "format"\n", mode, ##args) 

#endif //_RT_POSIX_TASK_H_