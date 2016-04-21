#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdlib.h>
#include <sys/msg.h>
#include "do_share.h"

#define OS_CYGWIN 2
#define OS_LINUX  1

int OS_TYPE = OS_LINUX;
extern void *shared_memory;
extern void *shared_memoryDIAG;
extern Q_SHARED *shared;
extern DIAG_SHARED *sharedDIAG;
extern int shmid;
extern int shmidDIAG;
extern int msgid;
extern int msgidDIAG;

extern char **environ;
// extern ControllerSettings OldControllerSettings;

void init_shared_mem()
{

}

void attach_mem()	// Attach to the shared memory.
{

	printf("OS type = %s",getenv("TERM"));
	

	if(strcmp(getenv("TERM"),"cygwin"))
	{
	 shmid=shmget((key_t)QBOATShareKey,sizeof(Q_SHARED),0666|IPC_CREAT);
	 shmidDIAG=shmget((key_t)DIAGShareKey,sizeof(DIAG_SHARED),0666|IPC_CREAT);
	 if(shmid==-1)
	 {	printf("shmget failure\n");
	 	exit(EXIT_FAILURE);
	 }
	 shared_memory=shmat(shmid, (void*)0,0);
	 shared_memoryDIAG=shmat(shmidDIAG, (void*)0,0);
	 if(shared_memory==(void*)-1)
	 {
	  	printf("shmat failed\n");
		exit(EXIT_FAILURE);
	 }
	
	 printf("Memory attached at %p\n",shared_memory);
	 shared=(Q_SHARED*)shared_memory;
	 sharedDIAG=(DIAG_SHARED*)shared_memoryDIAG;
	}else
	{ 
		/*printf("\nOS type = CYGWIN");
		OS_TYPE = OS_CYGWIN;
		shared_memory = malloc(sizeof(Q_SHARED));
		shared = (Q_SHARED*)shared_memory;*/
	}
	sleep(1);
}

void detach_mem()	// Detach from the Shared memory..
{
	if(OS_TYPE == OS_LINUX)
	{
	 if(shmdt(shared_memory)==-1)
	 {	printf("shmdt failure\n");
	 	exit(EXIT_FAILURE);
	 }
	
	 if(shmctl(shmid, IPC_RMID, 0) == -1)
	 {	printf("shmctl(IPC_RMID) failed\n");
		exit(EXIT_FAILURE);
	 }
	}
	else { 
		free(shared_memory); 
		free(shared_memoryDIAG); 
		shared = 0; 
		sharedDIAG = 0; 
		}
}

void attach_shmq(int key)
{
	msgid=msgget((key_t)key,0666|IPC_CREAT);
	if(msgid==-1)
	{
		perror("msgget failed...Error.");
		exit(EXIT_FAILURE);
	}
}

void remove_shmq()
{
	if(msgctl(msgid, IPC_RMID,0) ==-1)
	{	perror("msgctl(IPC_RMID) failed.");
		exit(EXIT_FAILURE);
	}
}
