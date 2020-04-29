/*
 *  This file is owned by the Embedded Systems Laboratory of Seoul National University of Science and Technology
 *  to benchmark Xenomai and the EtherCAT master protocol using the IgH EtherCAT master userspace library.	
 *  
 *  
 *  IgH EtherCAT master library for Linux is found at the following URL: 
 *  <http://www.etherlab.org/en/ethercat>
 *
 *
 *
 *
 *  2018 Raimarius Delgado
*/
/****************************************************************************/
#include <embdCONIO.h>  //./libs/embedded
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
#include <rtdm/rtdm.h>
/*****************************************************************************/
/* Xenomai */
/*****************************************************************************/
#include <xeno_task.h>
#include <native/event.h>
#include <native/mutex.h>
#include <native/sem.h>
#include <native/queue.h>

/* task creation */
#define TEST1_TASK_PRIORITY	(99) // xeno: 99 , preempt: 80
#define TEST1_TASK_PERIOD	(1000000L)
#define TEST2_TASK_PRIORITY	(50)
#define TEST2_TASK_PERIOD	(5000000L)

RT_TASK TskTest1;
RT_TASK TskTest2;

/* data acquisition */
RTIME rtmProcStart = 0;
RTIME rtmMsgStart = 0; 

#define SEC_TO_BUF(x,y) (x*FREQ_PER_SEC(y))
#define SEC_IN_HR (3600)

#define MAX_BUF (3600000) // maximum one-hour (1 ms task)
#define DEF_DURATION	(3)
#define DEF_BUF_EXIT	SEC_TO_BUF(DEF_DURATION,TEST1_TASK_PERIOD);
//#define BUF_SIZE	(DEF_DURATION*FREQ_PER_SEC(TEST1_TASK_PERIOD)) // buff size depends on runtime in seconds
#define OUTPUT 1
#define INPUT 0

#define RTDM_GPIO1 45
#define RTDM_GPIO2 66

int BufPrd1[MAX_BUF] = {0,}; 
int BufComp1[MAX_BUF] 	= {0,};
int BufJtr1[MAX_BUF]	= {0,};

int BufPrd2[MAX_BUF] = {0,}; 
int BufComp2[MAX_BUF] 	= {0,};
int BufJtr2[MAX_BUF]	= {0,};

int BufMsgPerf[MAX_BUF]	= {0,};

long long BufRising1[MAX_BUF] = {0,};
long long BufFalling1[MAX_BUF] = {0,};

long long BufRising2[MAX_BUF] = {0,};
long long BufFalling2[MAX_BUF] = {0,};

bool bTimingFlag = FALSE;
bool bOscilikeFlag = FALSE;

int iBufCnt1 = 0;
int iBufCnt2 = 0;

int iBufOscCnt1=0;
int iBufOscCnt2=0;

/* task management */
#define FILE_EXT ".dat"
#define FILE_PATH "./results/"

#define BOARD_SELECT_TEXT "BOARD SELECTION: \n" \
						  "1. BeagleBone Black \n" \
						  "2. i.MX6Q \n" \
						  "3. Raspberry Pi 3 \n" \
						  "4. Zynq-7000 \n" \
						  "5. Exit \n" 

#define HELP_TEXT 	"Usage: test_perf [options]\n" \
						"  [-T <test_duration in seconds>] # default= 3 seconds, maximum of 3600 seconds.\n" \
						"  [-p <1 or 2>]              	  # evaluate periodicity of 1 or 2 task(s).\n" \
						"  [-m <mechanism>]                # evaluate periodicity with real-time mechanisms\n" \
						"  [-o]                            # enable oscilloscope-like periodicity results!\n" \
						"  				  #    mtx = Mutex, \n" \
						"		  		  #    sem = Semaphore, \n" \
						"				  #    evf = Event Flag\n" \
						"				  #    msg = Message Queue\n" \
						"  [-h]                            # displays this message!\n" 
typedef enum {
	no_test = 0,
	single_periodic,
	two_periodic,
	ev_mutex,
	ev_sem,
	ev_eflag,
	ev_msgq,
} test_mode_e;

typedef struct{
    uint8_t gpio_num;
    uint8_t value;
} gpio_set;

bool bQuitFlag = FALSE;

test_mode_e eval_mode = no_test;
int test_duration = 0;
int fd;
int task1_init = 0;
int task2_init = 0;
int no_of_tasks = 0;
char* task1_name;
char* task2_name;
char* board_name;
/*****************************************************************************/
/* Mechanisms */
/*****************************************************************************/
RT_MUTEX	mutex_desc;
RT_QUEUE 	msgq_desc;
RT_EVENT	eflag_desc;
RT_SEM		sem_desc;

/*****************************************************************************/
/* function macros */
/*****************************************************************************/
void XenoInit();
void XenoStart();
void XenoQuit();
void SignalHandler(int signum);
void FilePrintEval(char *task_name,int BufPrd[], int BufExe[], int BufJit[],long long BufRis[], long long BufFal[], int ArraySize, int ArraySize2);
char* _check_test_mode(test_mode_e mode);
void _task_init(int n);
void _errnexit(int err, char* mech);
/****************************************************************************/
void TestTask1(void *arg){
	int iTaskTick = 0;
	int err;
	gpio_set task1_gpio;

	RTIME rtmPrdStart1=0, rtmPrdEnd1=0, rtmPrdComp1=0; 
	int tmPrd1=0, tmComp1=0, tmJtr1=0;
	
	RTIME task_runtime = 0;
	RTIME oscStart, oscEnd;
	RTIME TaskExeTime = 500000;

	task1_gpio.gpio_num = RTDM_GPIO1;
	if(rt_dev_ioctl(fd, OUTPUT, &task1_gpio) < 0){
        rt_printf("not support num %d\n",RTDM_GPIO1);
    }

	rtmPrdEnd1 = rt_timer_read();
	while (1) {
		task1_gpio.value = 1;		
		rt_dev_write(fd, &task1_gpio, sizeof(task1_gpio));
		rtmPrdStart1 = rt_timer_read();
		
		if(eval_mode != ev_msgq && eval_mode != ev_eflag)
		{
			if (eval_mode == ev_mutex)
				rt_mutex_acquire(&mutex_desc, TM_INFINITE);
			else if (eval_mode == ev_sem)
				rt_sem_p(&sem_desc, TM_INFINITE);

			rt_timer_spin(TaskExeTime);

			if (eval_mode == ev_mutex)
				rt_mutex_release(&mutex_desc);
			else if (eval_mode == ev_sem)
				rt_sem_v(&sem_desc);
		}

		tmPrd1 = ((int)rtmPrdStart1 - (int)rtmPrdEnd1);
		tmComp1 = ((int)rtmPrdComp1 - (int)rtmPrdEnd1); ; 
		tmJtr1 = MathAbsValI((int)TEST1_TASK_PERIOD - tmPrd1);

		if(iTaskTick > 1) //omit "irregular" data at start-up
		{
			BufPrd1[iBufCnt1] = tmPrd1;
			BufComp1[iBufCnt1] = tmComp1;
			BufJtr1[iBufCnt1] = tmJtr1;
			++iBufCnt1;
			if(iBufCnt1 == SEC_TO_BUF(test_duration,TEST1_TASK_PERIOD)){
				bQuitFlag = TRUE;
			}
		}

		rtmPrdComp1 = rt_timer_read();
		rtmPrdEnd1 = rtmPrdStart1;

		task1_gpio.value = 0;		
        rt_dev_write(fd, &task1_gpio, sizeof(task1_gpio));

		if(iTaskTick < 16 && bOscilikeFlag == TRUE) // acquire 16 samples when -O option is enabled
		{
			BufRising1[iBufCnt1] = ((long long)rtmPrdStart1 - (long long)rtmProcStart);
			BufFalling1[iBufCnt1] = ((long long)rtmPrdComp1 - (long long)rtmProcStart);

			iBufOscCnt1++;
		}
		
		iTaskTick++;

		rtmMsgStart = rt_timer_read();
		if(eval_mode == ev_msgq){
			err = rt_queue_write(&msgq_desc,"Trivial",7*sizeof(unsigned char),Q_NORMAL);
		}
		else if (eval_mode == ev_eflag){
			err = rt_event_signal(&eflag_desc,0x1);
		}

		rt_task_wait_period(NULL);

	}
}
/****************************************************************************/
void TestTask2(void *arg){
	int iTaskTick2 = 0;
	int task_runtime;
	int err;
	gpio_set task2_gpio;

	RTIME rtmPrdStart2=0, rtmPrdEnd2=0, rtmPrdComp2=0; 
	int tmPrd2=0, tmComp2=0, tmJtr2=0;
	int tmMsg = 0;

	RTIME oscStart;
	RTIME oscEnd;

	RTIME TaskSpinTime = 100000;
	RTIME TaskExeTime = 1500000;

	/* event flag or message queue */
	unsigned char *msgBuf;
	msgBuf = (unsigned char*) malloc (sizeof(int)*sizeof(unsigned char));
	unsigned long eflag_mask;
	task2_gpio.gpio_num = RTDM_GPIO2;
	if(rt_dev_ioctl(fd, OUTPUT, &task2_gpio) < 0){
        //rt_printf("not support num %d\n",RTDM_GPIO2);
    }

	rtmPrdEnd2 = rt_timer_read();
	while (1) {
		/*
		task2_gpio.value = 1;		
		rt_dev_write(fd, &task2_gpio, sizeof(task2_gpio));
		*/
		if(eval_mode == ev_msgq){
			err = rt_queue_read(&msgq_desc,msgBuf,sizeof(msgBuf),TM_INFINITE);
			if (err < 0) rt_printf("Receiving Message Queue Error \n");
		}
		else if(eval_mode == ev_eflag){
			err = rt_event_wait(&eflag_desc,0x1,&eflag_mask,EV_ALL,TM_INFINITE);
			if (err < 0) rt_printf("Receiving Event Flag Error \n");
		}
		rtmPrdStart2 = rt_timer_read();

		if(eval_mode != ev_msgq && eval_mode != ev_eflag){
			task_runtime = 0;
			while(task_runtime < TaskExeTime){
				//task2_gpio.value = 1;		
				//rt_dev_write(fd, &task2_gpio, sizeof(task2_gpio));
				oscStart = rt_timer_read();
				if (eval_mode == ev_mutex)
					rt_mutex_acquire(&mutex_desc, TM_INFINITE);
				else if (eval_mode == ev_sem)
					rt_sem_p(&sem_desc, TM_INFINITE);

				rt_timer_spin(TaskSpinTime);
				
				if (eval_mode == ev_mutex)
					rt_mutex_release(&mutex_desc);
				else if (eval_mode == ev_sem)
					rt_sem_v(&sem_desc);

				oscEnd	= rt_timer_read();
				//task2_gpio.value = 0;		
				//rt_dev_write(fd, &task2_gpio, sizeof(task2_gpio));
				task_runtime = task_runtime	+ TaskSpinTime;

				if(iTaskTick2 < 3 && bOscilikeFlag == TRUE) //acquire 3 samples when -O option is enabled
				{
					BufRising2[iBufOscCnt2] = ((long long)oscStart - (long long)rtmProcStart);
					BufFalling2[iBufOscCnt2] = ((long long)oscEnd - (long long)rtmProcStart);
					iBufOscCnt2++;
				}
			}
		}else{
			tmMsg = ((int)rtmPrdStart2 - (int)rtmMsgStart);
			BufMsgPerf[iBufCnt2] = tmMsg;
		}


		tmPrd2 = ((int)rtmPrdStart2 - (int)rtmPrdEnd2);
		tmComp2 = ((int)rtmPrdComp2 - (int)rtmPrdEnd2);

		if (eval_mode != ev_msgq && eval_mode != ev_eflag) tmJtr2 = MathAbsValI((int)TEST2_TASK_PERIOD - tmPrd2);
		else tmJtr2 = MathAbsValI((int)TEST1_TASK_PERIOD - tmPrd2);

		if(iTaskTick2 > 1) // omit "irregular" data at start-up
		{
			BufPrd2[iBufCnt2] = tmPrd2;
			BufComp2[iBufCnt2] = tmComp2;
			BufJtr2[iBufCnt2] = tmJtr2;
			
			++iBufCnt2;
		}

		rtmPrdComp2 = rt_timer_read();
		rtmPrdEnd2 = rtmPrdStart2;
		/*
		task2_gpio.value = 0;		
		rt_dev_write(fd, &task2_gpio, sizeof(task2_gpio));
		*/
		if(eval_mode != ev_msgq && eval_mode != ev_eflag)
		{
			rt_task_wait_period(NULL);
		}
		else
		{
			if(iTaskTick2 < 3 && bOscilikeFlag == TRUE) //acquire 3 samples when -O option is enabled
			{
				BufRising2[iBufOscCnt2] = ((long long)rtmPrdStart2 - (long long)rtmProcStart);
				BufFalling2[iBufOscCnt2] = ((long long)rtmPrdComp2 - (long long)rtmProcStart);
				iBufOscCnt2++;
			}
		}
		iTaskTick2++;
		if(bQuitFlag == TRUE) break; 
   	}
}
/****************************************************************************/
int main(int argc, char **argv){
	int c, err;
	int period_test;
	char *cargs = NULL;
	int board_select = 0;

	/* Xenomai RTDK to enable rt_printf 
	 * all calls to rt_printf are wrapped with printf in ./libs/embedded/EMBDcommon.h
	*/
	rt_print_auto_init(1); 
	
	/* Interrupt Handler "ctrl+c"  */
	signal(SIGTERM, SignalHandler);
	signal(SIGINT, SignalHandler);
/*
	if((fd = rt_dev_open("gpio_rtdm", 0)) < 0) {
        perror("rt_open");
    	exit(-1);
    }*/
	/* defaults */
	test_duration = (int)DEF_DURATION;
	_task_init(1);
	task1_name = "default";

	while((c = getopt(argc, argv, "T:p:m:oh")) != -1)
		switch(c){
			case 'T':
					test_duration = atoi(optarg);
					if(test_duration >= SEC_IN_HR){
						printf("Maximum test duration is %d seconds for 1ms periodic task\n",(int)SEC_IN_HR);
						printf("Configuring %d seconds as test_duration\n",(int)SEC_IN_HR);
						test_duration = (int)SEC_IN_HR;
					}
					break;
			case 'p':
					if(atoi(optarg) == 1){
						_task_init(1);
						task1_name = "1periodic";
						eval_mode = single_periodic;
					}
					else if(atoi(optarg) == 2){
						_task_init(2);
						task1_name = "2periodic_h";
						task2_name = "2periodic_l";
						eval_mode = two_periodic;
					}
					else{
						printf("Choose between 1 and 2 only!\n");
						exit(2);
					}
					break;
			case 'm':
					cargs = optarg;										
					_task_init(2);
					if(strcmp(cargs,"mtx") == 0){
						task1_name = "mutex_h";
						task2_name = "mutex_l";
						eval_mode = ev_mutex;
						if ((err = rt_mutex_create(&mutex_desc,"Test Mutex")))
							_errnexit(err,"Mutex");
					}else if(strcmp(cargs,"sem") == 0){
						task1_name = "sempahore_h";
						task2_name = "semaphore_l";
						eval_mode = ev_sem;
						if((err = rt_sem_create(&sem_desc,"Test Semaphore",1,S_FIFO))) //starting value: 1, mode: FIFO
							_errnexit(err,"Semaphore");
					}else if(strcmp(cargs,"evf") == 0){
						task1_name = "eventflag_h";
						task2_name = "eventflag_l";
						eval_mode = ev_eflag;
						if((err = rt_event_create(&eflag_desc,"Test Event Flag",0x0,EV_FIFO))) //starting value 0, mode: PRIORITY
							_errnexit(err,"EVENT FLAG");
					}else if(strcmp(cargs,"msg") == 0){
						task1_name = "msgq_h";
						task2_name = "msgq_l";			
						eval_mode = ev_msgq;
						if((err = rt_queue_create(&msgq_desc, "Test Queue",255,Q_UNLIMITED,Q_FIFO))) //Size: 255, limit: unlimited mode: FIFO
							_errnexit(err,"Message Queue");
					}else{
						fprintf(stderr,HELP_TEXT);
						exit(2);
					}
					break;
			case 'o':
					bOscilikeFlag = TRUE;
					break;
			case 'h':
			default:
					fprintf(stderr,HELP_TEXT);
					exit(2);
					break;
		}

		system("clear");
		printf(BOARD_SELECT_TEXT);
		scanf("%d",&board_select);
		switch(board_select){
			case 1:
					board_name = "bbb_";
					break;
			case 2:
					board_name = "imx6_";
					break;
			case 3:
					board_name = "rpi3_";
					break;
			case 4:
					board_name = "zynq_";
					break;
			case 5:
					printf("Exiting without performing test!\n");
					exit(2);
					break;
			default:
					printf("Invalid Selection!...Exiting!\n");
					exit(2);
					break;
		}


	/* Check test type and duration */
	printf("\nBoard name: %s\n",board_name);
	printf("Type of Test: %s\n",_check_test_mode(eval_mode));
	printf("Test Duration: %d seconds\n",test_duration);
	printf("Oscilloscope-like data acquisition is %s!\n\n", bOscilikeFlag ? "enabled" : "disabled");

	/* RT-task */
	mlockall(MCL_CURRENT|MCL_FUTURE); 
	XenoInit();
	XenoStart();
	rtmProcStart = rt_timer_read();

	while (1) {
		usleep(1);
		if (bQuitFlag==TRUE) break;
	}

	printf("Total: %d\n",iBufCnt2);
	FilePrintEval(task1_name,BufPrd1,BufComp1,BufJtr1,BufRising1,BufFalling1,iBufCnt1,iBufOscCnt1);
	if(no_of_tasks > 1) {
		sleep(1);
		FilePrintEval(task2_name,BufPrd2,BufComp2,BufJtr2,BufRising2,BufFalling2,iBufCnt2,iBufOscCnt2);
	}

	XenoQuit();
	return 0;
}
/****************************************************************************/
void SignalHandler(int signum){
		bQuitFlag=TRUE;
}
/****************************************************************************/
void XenoInit(){
	
	printf("Creating Real-time task(s)...");
		create_rt_task(&TskTest1,task1_name, TEST1_TASK_PRIORITY);
	if(no_of_tasks > 1)
		create_rt_task(&TskTest2,task2_name, TEST2_TASK_PRIORITY);
	printf("OK!\n");

	printf("Making Real-time task(s) Periodic...");
		set_rt_task_period(&TskTest1, TEST1_TASK_PERIOD);
		if (eval_mode != ev_msgq && eval_mode != ev_eflag)
			set_rt_task_period(&TskTest2, TEST2_TASK_PERIOD);
	printf("OK!\n");
}
/****************************************************************************/
void XenoStart(){
	printf("Starting Xenomai Real-time Task(s)...");
		start_rt_task(task1_init,&TskTest1,&TestTask1);
	if(no_of_tasks > 1)
		start_rt_task(task2_init,&TskTest2,&TestTask2);
	printf("OK!\n");
}
/****************************************************************************/
void XenoQuit(void){
	rt_dev_close(fd);

	rt_event_delete(&eflag_desc);
	rt_queue_delete(&msgq_desc);
	rt_mutex_delete(&mutex_desc);
	rt_sem_delete(&sem_desc);

	rt_task_suspend(&TskTest2);
	rt_task_suspend(&TskTest1);
	
	rt_task_delete(&TskTest2);
	rt_task_delete(&TskTest1);
	printf("\033[%dm%s\033[0m",95,"Xenomai Task(s) Deleted!\n");
}
/****************************************************************************/
char* _check_test_mode(test_mode_e mode)
{
	switch(mode & 0xff){
		case no_test:
			return "Default!";
			break;
		case single_periodic:
			return "Single Periodic Test";
			break;
		case two_periodic:
			return "Two-task Periodic Test";
			break;
		case ev_mutex:
			return "Mutex Test";
			break;
		case ev_sem:
			return "Semaphore Test";
			break;
		case ev_eflag:
			return "Event Flag Test";
			break;
		case ev_msgq:
			return "Message Queue Test";
			break;
		default:
			return "No Test Case!";
			break;
	}
}
/****************************************************************************/
void _task_init(int n)
{
	no_of_tasks = n;
	if(n == 1){
		task1_init  = 1;
		task2_init  = 0;
	}
	else if (n == 2){
		task1_init  = 1;
		task2_init  = 1;
	} else
	{
		printf("Invalid task number[%d]\n",n);
		exit(2);
	}	
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
		if (eval_mode == ev_msgq || eval_mode == ev_eflag)
		{
			fprintf(fptemp,"%d.%06d,%d.%03d,%d.%03d,%d.%03d\n",
				BufPrd[iCnt]/SCALE_1M,
				BufPrd[iCnt]%SCALE_1M,
				BufExe[iCnt]/SCALE_1K,
				BufExe[iCnt]%SCALE_1K,
				BufJit[iCnt]/SCALE_1K,
				BufJit[iCnt]%SCALE_1K,
				BufMsgPerf[iCnt]/SCALE_1K,
				BufMsgPerf[iCnt]%SCALE_1K);			
		} else
		{
		fprintf(fptemp,"%d.%06d,%d.%03d,%d.%03d\n",
				BufPrd[iCnt]/SCALE_1M,
				BufPrd[iCnt]%SCALE_1M,
				BufExe[iCnt]/SCALE_1K,
				BufExe[iCnt]%SCALE_1K,
				BufJit[iCnt]/SCALE_1K,
				BufJit[iCnt]%SCALE_1K);
		}
	}
	fclose(fptemp);
}
/****************************************************************************/
void _filePrintOscillike(char *FileName,long long BufRising[], long long BufFalling[],int ArraySize){
	FILE *fptemp;
	int iCnt;

	fptemp = fopen(FileName, "w");

	for(iCnt=0; iCnt < ArraySize; ++iCnt){
		fprintf(fptemp,"%lld 1\n",(long long)BufRising[iCnt]);
		fprintf(fptemp,"%lld 0\n",(long long)BufFalling[iCnt]);
	}
	fclose(fptemp);
}
/****************************************************************************/
void FilePrintEval(char *task_name,int BufPrd[], int BufExe[], int BufJit[],long long BufRis[], long long BufFal[], int ArraySize, int ArraySize2)
{
	char task_filename[100];
	char number_buffer[32];
	int k = 1;

	sprintf(number_buffer,"_%dsec_%d",test_duration,k);
	strcpy(task_filename,FILE_PATH);
	strcat(task_filename,board_name);
	strcat(task_filename,task_name);
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
			strcat(task_filename,board_name);
			strcat(task_filename,task_name);
			strcat(task_filename,number_buffer);
			strcat(task_filename,FILE_EXT);
		}
	}
	printf("Performance analysis datafile is generated at:%s\n",task_filename);
	
	if (bOscilikeFlag == TRUE)
	{
		memset(task_filename,0,sizeof(task_filename));
		memset(number_buffer,0,sizeof(number_buffer));
		k = 1;
	
		sprintf(number_buffer,"_%dsec_osc_%d",test_duration,k);
		strcpy(task_filename,FILE_PATH);
		strcat(task_filename,board_name);
		strcat(task_filename,task_name);
		strcat(task_filename,number_buffer);
		strcat(task_filename,FILE_EXT);	

		while(1)
		{
			if(_file_existence(task_filename) == -1){
				_filePrintOscillike(task_filename,BufRis,BufFal,ArraySize2);
				break;
			}
			else{
				sprintf(number_buffer,"_%dsec_osc_%d",test_duration,k++);
				strcpy(task_filename,FILE_PATH);
				strcat(task_filename,board_name);
				strcat(task_filename,task_name);
				strcat(task_filename,number_buffer);
				strcat(task_filename,FILE_EXT);
			}
		}
		printf("Oscilloscope-like datafile is generated at:%s\n",task_filename);
	}
}
/****************************************************************************/
void _errnexit(int err, char* mech)
{
	printf("[%d]Failed to create %s!\n", err, mech);
	exit(2);
}
/****************************************************************************/
