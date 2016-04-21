// Description: A simple Data Exchange Server
// Author : Arvind Antonio de Menezes Pereira
// 
// Databank.cpp contains code that starts up the listener thread for client connections.
// This is the main file... server implementation is going to take place in server.cpp

#include <time.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <pthread.h>
#include <math.h>
#include <string.h>
#include <signal.h>
#include "do_share.h"
#include "qboatVars.h"
#include "boat.h"
#include "gps.h"
#include "databank.h"
#include "server.h"
#include "CommProtS.h"
#include <sys/ipc.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/time.h>
#include <sys/stat.h>
// #include "mission.h"
//#include "serial.c"

#define OS_CYGWIN 2
#define OS_LINUX  1

extern int OS_TYPE;
Monitor monitor;
Stat test;
// #include "batteryvoltage.c"
int quiet = 0;
char dir_name[200],full_name[300],data_directory[100]="/home/arvind/";
struct timeval tv;
int server_sockfd, client_sockfd;
socklen_t server_len, client_len;
extern struct sockaddr_in server_address, client_address;
int result,bye;
fd_set readfds,testfds;	
unsigned char buf[MAX_BUFSIZE];
int choice=0,fd,nread,client_present=0,write_client=0;
int client_fds[10]; 
int display_once=0;
extern Stat test;
FILE *logFile;
int shmid;
int shmidDIAG;
int msgid;
struct sigaction sa1;
void *shared_memory;
void *shared_memoryDIAG;
Q_SHARED *shared;
DIAG_SHARED *sharedDIAG;
extern void attach_mem();
extern void detach_mem();
extern void do_mission();
extern int OpenMissionFile(char *MissFileName);
extern int AnalyzeMissionCommand(void);
extern void doSinusoid();
extern int setupWinch();
// Assuming a Maximum of 10 clients will connect up... (Static Connection Table)
// 

Client_info ClientTable[20];
int last_entry=0;

void quitApp(int signum)
{
	fprintf(stderr,"\nQuitting application.");
	fprintf(stderr,"\nClosing Shared Memory...");
	detach_mem();
	// close file pointers etc...
	fclose(logFile);
	close(server_sockfd);

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

void createDirectory()
{
	time_t t;
	struct tm *tm;
	int len;
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

bool openRoboduckServerlogfile()
{
	char file_name[100];
	gettimeofday(&tv,NULL);
	strncpy(full_name, dir_name, sizeof(full_name)-1);
	sprintf(file_name,"QSERV_%ld",tv.tv_sec);
	strcat(full_name,file_name);
	if ((logFile = fopen(full_name, "wb")) == NULL)
	{
		fprintf(stderr, "cannot open %s for binary file", full_name);
		perror("and the reason is");
		return false;
	}
	else return true;
}

int main(int argc, char *argv[])
{
	pthread_t listener, listener1;
	void *thread_result;
	int i,res;

	if(argc>1)
	{
		if(!strcmp(argv[1],"-q"))
			quiet=1;	// Run without sending feedback...
	}
setupWinch();
	
	fprintf(stderr,"\nNAMOS data Exchange Stub\n");
	fprintf(stderr,"\nCapturing Quit Signal...\n");
	captureQuitSignal();

	fprintf(stderr,"Creating Shared Memory...\n");
	 attach_mem();
	if(OS_TYPE == OS_CYGWIN) strcpy(data_directory,"/home/JD/");
	createDirectory();
    if(openRoboduckServerlogfile()!=true) exit(-1);



	sleep(2);
	shared->missData.missionLoaded=NO_MISS;
	shared->missData.ExecutionStatus = NO_MISS; 	


	fprintf(stderr,"Creating a socket.\n");
	create_socket();
	fprintf(stderr,"Server waiting...\n");
	fprintf(stderr,"Creating Listener thread...\n");
	res=pthread_create(&listener,NULL,listen_thread,NULL);
	if(res!=0)
	{
	  perror("Thread creation failed.");
	  exit(EXIT_FAILURE);
	}

	unsigned long incr=0;
	shared->navData.des_heading=83;
	while(1)
	{
		choice=0;

		gettimeofday(&tv,NULL); // tv.tv_sec, tv.tv_usec
		if(!quiet)
		fprintf(stderr,"\r$IMU %.3f,%.3f,%.3f, %.2f, GPS %.7f,%.7f, %.2f,%.2f, %.2f, %.2f",
			shared->SensorData.roll,shared->SensorData.cYaw,shared->SensorData.yaw,
			shared->SensorData.yawRate, 
			shared->SensorData.lat, shared->SensorData.lon, shared->SensorData.speed, shared->SensorData.magVar, shared->qboatData.T1, shared->qboatData.T2);  
		
	if(shared->missData.missionLoaded!=NO_MISS)
		{	
			do_mission();
		}
			
		if(shared->qboatData.qboat_mode == QBOAT_SINUSOID)
			doSinusoid();
		if(bye)
		{
	  	  res=pthread_join(listener, &thread_result);
		  if(res!=0)
		  {
			perror("Thread join failed");
			exit(EXIT_FAILURE);
		  }
		  printf("Thread joined...\n");

		  monitor.battery=0;
		  monitor.gps=0;
		  monitor.imu=0;

	  	  res=pthread_join(listener1, &thread_result);
		  if(res!=0)
		  {
			perror("Thread join failed");
			exit(EXIT_FAILURE);
		  }
		  printf("Battery Monitor Thread joined...\n");

		  for(i=0;i<client_present;i++)
			remove_client(client_fds[i]);
		  close(server_sockfd);
		  break;
		}
		usleep(50);
	}
	detach_mem();
	exit(EXIT_SUCCESS);
}
