#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <time.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <errno.h>
#include <signal.h>

#include <do_share.h>
#include <ms_timer.h>
#include <sscControl.h>



// These globals are required to perform other peripheral tasks such as logging data
// and so on.
FILE *logFile;
int xbowCount = 0;

void *shared_memory = (void *)0;
Q_SHARED	*shared;
DIAG_SHARED	*sharedDIAG;
int shmid;

time_t t;
char dir_name[200], full_name[300], data_directory[100]="/home/arvind/";
struct tm *tm;
int len;
struct timeval tv;
struct sigaction sa1;

int RudderControl::open_serial_port() {
       fd=open(PortName, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if(fd < 0) 
	{ 
		fprintf(stderr,"Error opening serial port.");
		exit(fd);
	}
       printf("SSC-II serial port opened. File Descriptor =%d\n",fd);
		
	tcgetattr(fd,&oldtio);
	bzero(&newtio,sizeof(newtio));
        newtio.c_cflag=B2400 | CS8 | CLOCAL |CREAD;
	newtio.c_iflag=IGNPAR;
	newtio.c_iflag=IGNBRK;
	newtio.c_oflag=0;
  	newtio.c_lflag= 0;
   	newtio.c_cc[VTIME] 	=0;
	newtio.c_cc[VMIN] 	=1;
   	tcflush(fd,TCIFLUSH);
   	tcsetattr(fd,TCSANOW,&newtio);
   	return fd;
}


int RudderControl::InitRudder() 
{
  	tcflush(fd,TCIFLUSH);
	tcflush(fd,TCOFLUSH);
}

int RudderControl::SendAngle(double ang)
{
	int  servoNum = 0;
	char txBuf[5]={ 0xff, servoNum, 0,0, 0} ;
	enum txBufFields { SyncMark, ServoNum, Position };
	const double angLow  = -90;
	const double angHigh = +90;
	txBuf[ServoNum] = servoNum;
	int offset = 158;
	double ang2send = (char)(ang*2);
	if(ang2send < angLow) ang2send = (char)(angLow +offset);
	if(ang2send >  angHigh) ang2send = (char)(angHigh+offset);

	txBuf[Position] = (char)ang2send;
	fprintf(stderr,"   - %x, %x, %02x.\n", txBuf[0], txBuf[1], txBuf[2]);
	write(fd, txBuf, 3);
}
//-----------------------------------------------------------------------------	
// Example usage as well as logging code...
//
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
			perror("Xbow Reader unable to create directory");
		}
	}
}


bool openRudderlogFile()
{
	char file_name[100];
	gettimeofday(&tv,NULL);
	strncpy(full_name, dir_name, sizeof(full_name)-1);
	sprintf(file_name,"RUD_%ld",tv.tv_sec);
	strcat(full_name,file_name);
	if ((logFile = fopen(full_name, "wb")) == NULL)
	{
		fprintf(stderr, "cannot open %s for binary file", full_name);

		perror("and the reason is");
                return false;
	}
	else return true;
}

//-----------------------------------------------------------------------------
main()
{
	RudderControl *RUD;
	double lastRuddertime;
	
	fprintf(stderr,"Hello... This program writes Rudder Commands to the SSC-II.");
	RUD = new RudderControl("/dev/ttyUSB7");
	RUD->open_serial_port();	
	RUD->InitRudder();

	// Let us open files and do other stuff that needs to be done...
	shared = (Q_SHARED*)attach_memory(QBOATShareKey,sizeof(Q_SHARED));
	sharedDIAG = (DIAG_SHARED *)attach_memory(DIAGShareKey,sizeof(DIAG_SHARED));
	sleep(2);
	
	captureQuitSignal();
	createDirectory();
	openRudderlogFile();

	sharedDIAG->rudderData.rudAng = 0;
	lastRuddertime = get_time();
	while(1) {
		usleep(250000);
	  // if(sharedDIAG->rudderData.lastRudTime>lastRuddertime) {
		RUD->SendAngle(sharedDIAG->rudderData.rudAng);	
		lastRuddertime = get_time();
		fprintf(stderr,"\nRud = %f",sharedDIAG->rudderData.rudAng);
//	  }
	  usleep(500);
	}
}	


