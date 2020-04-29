/*
 *  This file is owned by the Embedded Systems Laboratory of Seoul National University of Science and Technology
 *  
 *  2016 Raimarius Delgado
*/
/****************************************************************************/
#ifndef EMBD_COMMON_H
#define EMBD_COMMON_H
/****************************************************************************/
/* Scaling Macros */
#define SCALE_1K		(1000)
#define SCALE_10K		(10000)
#define SCALE_100K		(100000)
#define SCALE_1M		(1000000)
#define SCALE_1G		(1000000000)

#define PI (float)3.14159265359
#define TWO_PI (float)(2*PI)


typedef enum{OFF = 0, ON, off = 0, on, }FLAG;
typedef enum {FALSE = 0, TRUE,false = 0, true,False = 0, True,} bool;

#define NSEC_PER_USEC (SCALE_1K)
#define NSEC_PER_MSEC (SCALE_1M)
#define NSEC_PER_SEC  (SCALE_1G)

/* assuming that one tick is 1 nanosecond */
#define TICKS_PER_SEC(x)	(NSEC_PER_SEC/(x))


#ifndef cpu_relax
#define cpu_relax() __sync_synchronize()
#endif

#if defined (__COBALT__)
#define _XENOMAI_TASKS_
#endif

typedef enum 
{
	RT=0,
	NRT
} TASK_TYPE;

/****************************************************************************/
#endif // EMBD_COMMON_H
