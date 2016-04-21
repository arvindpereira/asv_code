// Program Architecture:
/*
	1. gps.cpp
	2. imu.cpp
	3. ax1500.cpp
	4. autopilot.cpp
	5. guidance.cpp
	6. kalman.cpp
	7. camCap.cpp
	7. stereo.cpp
	8. opticFlow.cpp
	9. obsAvoid.cpp

	All shared data is global for now... ( Use Publish/Subscribe in future version )

	Communications with the shore computer should be handled by another process! To do that I will have to use shared memory!
	DbeX will be re-used to do data-communications with the NAMOS-GUI.
*/
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/msg.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "pid.h"
#include "../common/ms_timer.h"
#include "do_share.h"

// The most important Global variables...
char dir_name[200],full_name[300],data_directory[100]="/home/arvind/";
time_t t;
struct itimerval timer;
struct tm *tm;
int len;
int quiet = 0;
struct timeval tv;
struct sigaction sa1,sa2;
FILE *logFile;
PID *headingPID;
PID *speedPID;

extern int fd_gps;
// global settings

int OpenAX1500(int);
int setAX1500(int leftMotor, int rightMotor);
void WayPtGuidance();
void DoCarrotStickGuidance();
void stationKeep(int);
int applyThrustMapping(float , float , int *, int *);

void quitApp(int signum)
{
	fprintf(stderr,"\nQuitting application.");

	// close file pointers etc...
	fclose(logFile);
	exit(0);
}

void captureQuitSignal()
{
	memset(&sa1, 0, sizeof(sa1));
	sa1. sa_handler = &quitApp;
	sigaction(SIGINT, &sa1, NULL);
	sigaction(SIGTERM, &sa1, NULL);
	sigaction(SIGKILL, &sa1, NULL);
}

//===============================shared memory ================

void *shared_memory = (void *)0;
Q_SHARED	*shared;
DIAG_SHARED	*sharedDIAG;
int shmid,msgid;

void * attach_memory(int key, int size)
{
   shmid = shmget((key_t)key,size,0666|IPC_CREAT);

   if(shmid==-1){
    printf("shmget failure\n");
    exit(EXIT_FAILURE);
   }
  shared_memory=shmat(shmid, (void *)0,0);
  if(shared_memory==(void *)-1){
    printf("shmat failure");
    exit(EXIT_FAILURE);
  }
  printf("Memory attached at %p\n",shared_memory);
  //shared=(Q_SHARED*)shared_memory;
  sleep(1);
   	
  return shared_memory;
}

void createDirectory()
{
 	sprintf(full_name, "%s%%Y%%m%%d/", data_directory);
  	time(&t);
  	tm = localtime(&t);
  	len = strftime(dir_name, sizeof(dir_name) - 1, full_name, tm);
  	if(-1 == mkdir(dir_name, 0700))
	{
		if(errno != EEXIST)
		{
			perror("roboduck unable to create directory");
		}
  	}
}


void timer_handler(int signum)
{
	static int isr_count,isr_err;
	int dif;
 	float rudder_out, fin_out,T1,T2, diff_mode,comm_mode; unsigned long my_timer;
 	isr_count++;

	dif= get_delay();
	if(dif<90 || dif >110) isr_err++;

	int LeftThr=0, RightThr=0;
	if(shared->qboatData.qboat_mode!=QBOAT_DIRECT && shared->qboatData.qboat_mode!=QBOAT_SINUSOID)
	{	
		diff_mode = headingPID->doPID(shared->navData.des_heading, shared->SensorData.cYaw, sharedDIAG->xbowData.yawRate * 3.1415926535/180.0 , 1);	
		comm_mode=shared->navData.des_speed; // Can be output of the speed PID...

		if(comm_mode > 1.3) comm_mode = 1.3;
		if(comm_mode < -0.7) comm_mode = -0.7;
		T1 = comm_mode + diff_mode;
		T2 = comm_mode - diff_mode;
		applyThrustMapping(T1,T2,&LeftThr,&RightThr);
		gettimeofday(&tv, NULL);
		fprintf(logFile,"\n$THROUT_PID,%ld.%06ld,%f,%f,%d,%d",
			tv.tv_sec,tv.tv_usec, T1,T2, LeftThr, RightThr);
		shared->qboatData.T1=LeftThr; 
		shared->qboatData.T2=RightThr;
		// 	Apply Obstacle avoidance if available...
		if(shared->obsData.status != NO_ACTION )
		{	if(shared->obsData.status == TURN_LEFT) { shared->qboatData.T1 = T1 - 100; shared->qboatData.T2 = T2 +100; }
			else if( shared->obsData.status == TURN_RIGHT) { shared->qboatData.T1 = T1 + 100; shared->qboatData.T2 = T2 + 100; }
			else if( shared->obsData.status == BACKUP ) { shared->qboatData.T1 = T1 -100; shared->qboatData.T2 = T2 -100; }
			printf("\nObAvoid.");
		}
	}
	if(shared->qboatData.qboat_mode==QBOAT_DIRECT){ // Joystick Control
	//printf("\t DIRECT ");
	T1 = shared->qboatData.T1; T2 = shared->qboatData.T2;
	gettimeofday(&tv,NULL);
	fprintf(logFile,"\n$THRJOY,%ld.%06ld,%f,%f",
		tv.tv_sec,tv.tv_usec, T1, T2);
	}
	else if(shared->qboatData.qboat_mode == QBOAT_SINUSOID) {
		T1 = shared->qboatData.T1; T2 = shared->qboatData.T2;
		gettimeofday(&tv,NULL);
		fprintf(logFile,"\n$SYSIDENT,%ld.%06ld,%f,%f",
			tv.tv_sec,tv.tv_usec, T1,T2);
		}
	// setAX1500(T1,T2);
	setAX1500(shared->qboatData.T1,shared->qboatData.T2);
	if(shared->qboatData.qboat_mode==QBOAT_GUIDANCE){
		//printf("\t GUIDANCE");
 	   // WayPtGuidance();
 	   stationKeep(1);
	   if(shared->qboatData.qboat_mode == QBOAT_STATION_KEEP) {
	//    DoCarrotStickGuidance();	   
	   }
 }
}

bool openRoboducklogfile()
{
	char file_name[100];
	gettimeofday(&tv,NULL);
	strncpy(full_name, dir_name, sizeof(full_name)-1);
	sprintf(file_name,"QBOAT_%ld",tv.tv_sec);
	strcat(full_name,file_name);
	if ((logFile = fopen(full_name, "wb")) == NULL)
	{
		fprintf(stderr, "cannot open %s for binary file", full_name);
		perror("and the reason is");
                return false;
	}
	else return true;
}


void set_timer(long usec_value)
{
	timer.it_value.tv_sec=0;
	timer.it_value.tv_usec=usec_value;
	timer.it_interval.tv_sec=0;
	timer.it_interval.tv_usec=usec_value;
	setitimer(ITIMER_REAL, &timer, NULL);


	// 10 Hz signal
   	memset(&sa2,0,sizeof (sa2));
   	sa2.sa_handler=&timer_handler;
   	sigaction(SIGALRM, &sa2,NULL);
}

void initializeRoboduck()
{
	createDirectory();
	openRoboducklogfile();
	

//	process_arguments(argc, argv);
	OpenAX1500(0);
	captureQuitSignal();
	shared = (Q_SHARED*)attach_memory(QBOATShareKey,sizeof(Q_SHARED));
	sharedDIAG = (DIAG_SHARED *)attach_memory(DIAGShareKey,sizeof(DIAG_SHARED));
	
	sleep(2);
	headingPID = new PID();	
	headingPID->setLimits(0.7,-0.7);
	headingPID->setName("HEADING");
//	headingPID->setGains(0.017,0.2,0);

	speedPID = new PID();
	speedPID->setLimits(0.8,-0.7);
	speedPID->setName("SPEED");
//	speedPID->setGains(0.1,0.15,0);	
	headingPID->setSHMPointers(&(sharedDIAG->pidParams.Kp),&(sharedDIAG->pidParams.Ki),&(sharedDIAG->pidParams.Kd),
			&(sharedDIAG->pidParams.pTerm),&(sharedDIAG->pidParams.iTerm),&(sharedDIAG->pidParams.dTerm));
	speedPID->setSHMPointers(&(sharedDIAG->SpdPidParams.Kp),&(sharedDIAG->SpdPidParams.Ki),&(sharedDIAG->SpdPidParams.Kd),
			&(sharedDIAG->SpdPidParams.pTerm),&(sharedDIAG->SpdPidParams.iTerm),&(sharedDIAG->SpdPidParams.dTerm));
		
	headingPID->setGains(0.9,20,0);
	speedPID->setGains(0.1,3,0);	
	sleep(2);
	shared->navData.des_heading=0;
	shared->navData.des_speed = 0;
	shared->navData.AcceptRad = 3;	// If we are within 3 meters... we're almost there! Yay!
	shared->obsData.status = NO_ACTION;
	set_timer(100000);	// Lets run the controllers at 10Hz...

}

int main(int argc, char *argv[])
{
	int sid,i=0;
	// Setup a loop
	initializeRoboduck();	

	if(argc>1) if(!strcmp(argv[1],"-q")) quiet=1;

	// We utilize a timer to generate commands, so we basically sleep the rest of the time.
	while(1)
	{	
		//if(!quiet)
		//printf("%d\n",++i);
		usleep(80000);
	}
	return 0;
}
