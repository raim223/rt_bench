/*
 *  This file is owned by the Embedded Systems Laboratory of Seoul National University of Science and Technology
 *  to benchmark Xenomai and RT_PREEMPT
 *  
 *  2020 Raimarius Delgado
*/

/* This is a prototype */

/****************************************************************************/
#include <embdCOMMON.h>  //./libs/embedded
#include <embdMATH.h>  //./libs/embedded
/*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>
#include <pthread.h>
#include <ctype.h>
/*****************************************************************************/
/* RT_TASKS */
/*****************************************************************************/
#include <rt_tasks.h>
#include <rt_itc.h> // for mutex

/* comment out to run periodicity test */
#define _PREEMPTION_TEST_

/* conversion of selected jiffy to actual clock ticks in nanoseconds */
#define JIFFY_TO_USE (NSEC_PER_MSEC) // ms (1M)
// #define JIFFY_TO_USE (NSEC_PER_USEC) // μs (1k)
// #define JIFFY_TO_USE (NSEC_PER_SEC) // s (1G)

/* convert jiffies to nanoseconds */
/* we define our "virtual" jiffy/clock tick as defined by JIFFY_TO_USE, 
 * for easier computation. 
 * Note that actual clock ticks are in nanoseconds */
/* should use enum in future release */
#define CLOCKTICKS(x) (int)(((float)x * JIFFY_TO_USE))

/* task creation */
RT_TASK TskTest1;
RT_TASK TskTest2;
RT_TASK TskTest3;

#define TASK_1_PRIO		(99) // xeno: 99 
#define TASK_1_PRD		(100)
#define TASK_1_EXE		(3)
char *sTaskName1 = "task_1";

#define TASK_2_PRIO		(80)
#define TASK_2_PRD		(20)
#define TASK_2_EXE		(5)
char *sTaskName2 = "task_2";

#ifdef _PREEMPTION_TEST_
#define TEST_NAME "_prmpt_test"
#define TASK_3_PRIO		(50)
#define TASK_3_PRD		(40)
#define TASK_3_EXE		(10)
char *sTaskName3 = "task_3";
#else
#define TEST_NAME "_sched_test"
#endif

#define TASK_TIMESLICE (0.1) //timeslice of 1 cpu spin 

/* data acquisition */
#define SEC_TO_BUF(x,y) (x*TICKS_PER_SEC(CLOCKTICKS(y)))
#define MAX_BUF (3600000) 
#define FILE_EXT ".dat"
#define FILE_PATH "./results/"

/* task duration should be in seconds and an integer. 
 * we can define as a constant, but we need a variable in future releases */
int test_duration = 100;  //1 hour

/* end the whole process when the buffer of TASK_1 is full */
#define FULL_BUF (int)(SEC_TO_BUF(test_duration, TASK_1_PRD)) 

/* TASK_1 Buffers */
int BufPrd1[MAX_BUF] 	= {0,}; 
int BufResp1[MAX_BUF] 	= {0,};
int BufJtr1[MAX_BUF]	= {0,};
int iBufCnt1 = 0; 

/* TASK_2 Buffers */
int BufPrd2[MAX_BUF] 	= {0,}; 
int BufResp2[MAX_BUF] 	= {0,};
int BufJtr2[MAX_BUF]	= {0,};
int iBufCnt2 = 0;

#ifdef _PREEMPTION_TEST_
/* TASK_3 Buffers */
int BufPrd3[MAX_BUF] 	= {0,}; 
int BufResp3[MAX_BUF] 	= {0,};
int BufJtr3[MAX_BUF]	= {0,};
int iBufCnt3 = 0;
#endif

FLAG bQuitFlag = off;

/* mutex */
RT_MUTEX lock;

/*****************************************************************************/
/* function macros */
/*****************************************************************************/
void XenoInit();
void XenoStart();
void SignalHandler(int signum);
void FilePrintEval(char *task_name,int BufPrd[], int BufExe[], int BufJit[], int ArraySize);
/****************************************************************************/
void TestTask1(void *arg){
	
	int iTaskTick = 0;
	int task_runtime;

	RTIME rtmPrdCurr=0, rtmPrdPrev=0, rtmRespStart=0, rtmResp=0; 
	int tmPrd=0, tmResp=0, tmJtr=0;
	
	RTIME TaskSpinTime = CLOCKTICKS(TASK_TIMESLICE);
	RTIME TaskExeTime = CLOCKTICKS(TASK_1_EXE);

	rtmPrdPrev = rt_timer_read();
	while (1) {
		rtmPrdCurr = rt_timer_read();

		/* spin the CPU doing nothing until target execution time is reached */
		task_runtime = 0;
		while(task_runtime < TaskExeTime){
			acquire_rt_mutex(&lock);
			rt_timer_spin(TaskSpinTime);
			task_runtime += TaskSpinTime;
			release_rt_mutex(&lock);
			// rt_printf("1\n");
		}
		rtmResp = rt_timer_read();


			tmPrd = ((int)rtmPrdCurr - (int)rtmPrdPrev);
			tmResp = ((int)rtmResp - (int)rtmPrdCurr); ; 
			tmJtr = MathAbsValI(CLOCKTICKS(TASK_1_PRD) - tmPrd);	

		if(iTaskTick > 1) // omit "irregular" data at start-up
		{
			BufPrd1[iBufCnt1] = tmPrd;
			BufResp1[iBufCnt1] = tmResp;
			BufJtr1[iBufCnt1] = tmJtr;
			++iBufCnt1;
			
			if(iBufCnt1 == FULL_BUF)
				bQuitFlag = ON;
		}

		/* print a dot every one second to check if program is still running */
		if (!(iTaskTick % TICKS_PER_SEC(CLOCKTICKS(TASK_1_PRD))))
			printf(".\n");

		rtmPrdPrev = rtmPrdCurr;
		++iTaskTick;

		if (bQuitFlag == on){
			delete_rt_task();
			break;
		}else
			wait_rt_period(&TskTest1);
	}
}
/****************************************************************************/
void TestTask2(void *arg){
	int iTaskTick = 0;
	int task_runtime;

	RTIME rtmPrdCurr=0, rtmPrdPrev=0, rtmRespStart=0, rtmResp=0; 
	int tmPrd=0, tmResp=0, tmJtr=0;
	int tmMsg = 0;

	RTIME TaskSpinTime = CLOCKTICKS(TASK_TIMESLICE);
	RTIME TaskExeTime = CLOCKTICKS(TASK_2_EXE);

	rtmPrdPrev = rt_timer_read();
	while (1)
	{
		rtmPrdCurr = rt_timer_read(); // start of current iteration
		
		task_runtime = 0;
		while(task_runtime < TaskExeTime){
#ifdef _PREEMPTION_TEST_
			acquire_rt_mutex(&lock);
#endif
			rt_timer_spin(TaskSpinTime);
			task_runtime += TaskSpinTime;
#ifdef _PREEMPTION_TEST_
			release_rt_mutex(&lock);
#endif
			// rt_printf("2\n");
		}
		rtmResp = rt_timer_read(); // end of execution 

		tmPrd = ((int)rtmPrdCurr - (int)rtmPrdPrev);
		tmResp = ((int)rtmResp - (int)rtmPrdCurr);
		tmJtr = MathAbsValI(CLOCKTICKS(TASK_2_PRD) - tmPrd);

		if(iTaskTick > 1) // omit "irregular" data at start-up
		{
			BufPrd2[iBufCnt2] = tmPrd;
			BufResp2[iBufCnt2] = tmResp;
			BufJtr2[iBufCnt2] = tmJtr;
			++iBufCnt2;
		}
		rtmPrdPrev = rtmPrdCurr;
		iTaskTick++;

		if (bQuitFlag == on)
		{
			delete_rt_task();
			break;
		}
		else
			wait_rt_period(&TskTest2);
   	}
}
/****************************************************************************/
#ifdef _PREEMPTION_TEST_
void TestTask3(void *arg){
	int iTaskTick = 0;
	int task_runtime;

	RTIME rtmPrdCurr=0, rtmPrdPrev=0, rtmRespStart=0, rtmResp=0; 
	int tmPrd=0, tmResp=0, tmJtr=0;
	int tmMsg = 0;

	RTIME TaskSpinTime = CLOCKTICKS(TASK_TIMESLICE);
	RTIME TaskExeTime = CLOCKTICKS(TASK_3_EXE);

	rtmPrdPrev = rt_timer_read();
	while (1)
	{
		rtmPrdCurr = rt_timer_read(); // start of current iteration
		
		task_runtime = 0;
		while(task_runtime < TaskExeTime){
#ifdef _PREEMPTION_TEST_
			acquire_rt_mutex(&lock);
#endif
			rt_timer_spin(TaskSpinTime);
			task_runtime += TaskSpinTime;
#ifdef _PREEMPTION_TEST_
			release_rt_mutex(&lock);
#endif
			// rt_printf("3\n");
		}
		
		rtmResp = rt_timer_read(); // end of execution 

		tmPrd = ((int)rtmPrdCurr - (int)rtmPrdPrev);
		tmResp = ((int)rtmResp - (int)rtmPrdCurr);
		tmJtr = MathAbsValI(CLOCKTICKS(TASK_3_PRD) - tmPrd);

		if(iTaskTick > 1) // omit "irregular" data at start-up
		{
			BufPrd3[iBufCnt3] = tmPrd;
			BufResp3[iBufCnt3] = tmResp;
			BufJtr3[iBufCnt3] = tmJtr;
			++iBufCnt3;
		}
		rtmPrdPrev = rtmPrdCurr;
		iTaskTick++;

		if (bQuitFlag == on)
		{
			delete_rt_task();
			break;
		}
		else
			wait_rt_period(&TskTest3);
   	}
}
#endif
/****************************************************************************/
int main(int argc, char **argv){
	int c, err;

	/* Interrupt Handler "ctrl+c"  */
	signal(SIGTERM, SignalHandler);
	signal(SIGINT, SignalHandler);

	/* init mutex */
	if (create_rt_mutex(&lock, NULL) != 0)
    {
        printf("\n mutex init failed\n");
        return 1;
    }

	/* RT-tasks */
	mlockall(MCL_CURRENT|MCL_FUTURE); 
	XenoInit();
	XenoStart();

	while (1) {
		usleep(1);
		if (bQuitFlag==ON) break;
	}

	FilePrintEval(sTaskName1,BufPrd1,BufResp1,BufJtr1,iBufCnt1);
	FilePrintEval(sTaskName2,BufPrd2,BufResp2,BufJtr2,iBufCnt2);
#ifdef _PREEMPTION_TEST_
	FilePrintEval(sTaskName3,BufPrd3,BufResp3,BufJtr3,iBufCnt3);
#endif
	delete_rt_mutex(&lock);
	return 0;
}
/****************************************************************************/
void SignalHandler(int signum){
		bQuitFlag=on;
}
/****************************************************************************/
void XenoInit(){
	printf("Creating Real-time task(s)...");
	create_rt_task(&TskTest1,sTaskName1, TASK_1_PRIO);
	create_rt_task(&TskTest2,sTaskName2, TASK_2_PRIO);
#ifdef _PREEMPTION_TEST_
	create_rt_task(&TskTest3,sTaskName3, TASK_3_PRIO);
#endif
	printf("OK!\n");

	printf("Making Real-time task(s) Periodic...");
	set_rt_task_period(&TskTest1,CLOCKTICKS(TASK_1_PRD));
	set_rt_task_period(&TskTest2,CLOCKTICKS(TASK_2_PRD));
#ifdef _PREEMPTION_TEST_
	set_rt_task_period(&TskTest3,CLOCKTICKS(TASK_3_PRD));
#endif
	printf("OK!\n");
}
/****************************************************************************/
void XenoStart(){
	printf("Starting Xenomai Real-time Task(s)...");
	start_rt_task(1,&TskTest1,&TestTask1);
	start_rt_task(1,&TskTest2,&TestTask2);
#ifdef _PREEMPTION_TEST_
	start_rt_task(1,&TskTest3,&TestTask3);
#endif
	printf("OK!\n");
}
/****************************************************************************/
int _file_existence(char* filenames)
{
	return access(filenames,F_OK); 
}
/****************************************************************************/
void _filePrintEval(char *FileName,int BufPrd[], int BufExe[], int BufJit[], int ArraySize){
	FILE *fptemp;
	int iCnt;

	fptemp = fopen(FileName, "w");

	for(iCnt=0; iCnt < ArraySize; ++iCnt)
	{
		fprintf(fptemp,"%d.%06d,%d.%06d,%d.%06d\n",
				BufPrd[iCnt]/CLOCKTICKS(1),
				BufPrd[iCnt]%CLOCKTICKS(1),
				BufExe[iCnt]/CLOCKTICKS(1),
				BufExe[iCnt]%CLOCKTICKS(1),
				BufJit[iCnt]/CLOCKTICKS(1),
				BufJit[iCnt]%CLOCKTICKS(1));
	}
	fclose(fptemp);
}
/****************************************************************************/
void FilePrintEval(char *task_name,int BufPrd[], int BufExe[], int BufJit[],int ArraySize)
{
	char task_filename[100];
	char number_buffer[32];
	int k = 1;

	sprintf(number_buffer,"_%dsec_%d",test_duration,k);
	strcpy(task_filename,FILE_PATH);
	strcat(task_filename,task_name);
	strcat(task_filename,TEST_NAME);
	strcat(task_filename,number_buffer);
	strcat(task_filename,FILE_EXT);	

	while(1)
	{
		if(_file_existence(task_filename) == -1){
			_filePrintEval(task_filename,BufPrd,BufExe,BufJit,ArraySize);
			break;
		}
		else{
			sprintf(number_buffer,"_%dsec_%d",test_duration,k++);
			strcpy(task_filename,FILE_PATH);
			strcat(task_filename,task_name);
			strcat(task_filename,TEST_NAME);
			strcat(task_filename,number_buffer);
			strcat(task_filename,FILE_EXT);
		}
	}
	printf("Performance analysis datafile is generated at:%s\n",task_filename);
}
/***************************************************************************/