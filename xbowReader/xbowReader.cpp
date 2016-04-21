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
#include <xbowReader.h>



// A few globals which are not part of the real crossbow class.
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



int xbow::open_serial_port() {
       fd=open(PortName, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if(fd < 0) 
	{ 
		fprintf(stderr,"Error opening serial port.");
		exit(fd);
	}
       printf("Xbow DMU serial port opened. File Descriptor =%d\n",fd);
		
	tcgetattr(fd,&oldtio);
	bzero(&newtio,sizeof(newtio));
        newtio.c_cflag=B38400 ^CRTSCTS| CS8 | CLOCAL |CREAD;
	newtio.c_iflag=IGNPAR;
	newtio.c_iflag=IGNBRK;
	newtio.c_oflag=0;
  	newtio.c_lflag= 0;
   	newtio.c_cc[VTIME] 	=0;
	newtio.c_cc[VMIN] 	=22;
   	tcflush(fd,TCIFLUSH);
   	tcsetattr(fd,TCSANOW,&newtio);
   	return fd;
}


int xbow::init_xbow() 
{
	unsigned char buf[20];
	int i;

	readPtr = 0;
	circBuf[0] = 0;
	totXbowBuf= 0;	

  	tcflush(fd,TCIFLUSH);
	tcflush(fd,TCOFLUSH);
//	write(fd,"P",1); sleep(1);
//	write(fd,"R",1); sleep(1);
//	read(fd,buf,5);
	write(fd,"a",1); sleep(1);
	read(fd,buf,5);
	tcflush(fd,TCIFLUSH);
	tcflush(fd,TCOFLUSH);
// 	write(fd,"G",1); // sleep(1);
	write(fd,"C",1);
}

//-----------------------------------------------------------------------------	
float xbow::tofloat(unsigned char *buf) 
{
	short s;
	s=((*buf)*256)+*(buf+1);
	return((float)s);
}	
//-----------------------------------------------------------------------------		
void xbow::poll_xbow()	
{
	tcflush(fd,TCIFLUSH);
	write(fd,"G",1);
	usleep(10);
}

//-----------------------------------------------------------------------------	
unsigned char xbow::checksum_xbow(unsigned char *buf,int res) 
{
	int i;
	unsigned char chsum=0;
   	for(i=1;i<=20;i++) chsum+=buf[i];
	chsum=chsum%256;
	
	return chsum;
}
//-----------------------------------------------------------
int xbow::check_xbow(unsigned char *temp,int res) 
{
	int i;
	// unsigned char chksum,buf[128];
    	if(*temp==0xff)  {
//		fprintf(stderr,"\n\n Chksum = %d, Frm Packet = %d", checksum_xbow(temp,21),*(temp+21));
	  if(*(temp+21)==checksum_xbow(temp,21)) 
	  {
		   tokenize_xbow_data(temp);
		   return checksum_xbow(temp,21);
		 }
	 }
}
//-----------------------------------------------------------------------------
void xbow::tokenize_xbow_data(unsigned char *buf) 
{
	
	// uint temp,time1;
	
	roll=tofloat(&buf[1])*(0.0054931640625);
	pitch=tofloat(&buf[3])*(0.0054931640625);

	rollRate=tofloat(&buf[5])*0.004577637;
	pitchRate=tofloat(&buf[7])*0.004577637;
	yawRate=tofloat(&buf[9])*0.004577637;

	xAccel=tofloat(&buf[11])*0.0000915527;
	yAccel=tofloat(&buf[13])*0.0000915527;
	zAccel=tofloat(&buf[15])*0.0000915527;

	// temp=44*((tofloat(&buf[17]) * 0.0001220703)-1.375); 
	temp =  ((buf[17])*256+buf[18]) *0.054248-61.105;
}
//----------------------------------------------------------------------------
// Continuous mode read... 
int xbow::read_xbow_cont(unsigned char *temp, int res)
{
	int i,j,k,search=0,sync_loc,chksum_loc;
	unsigned char compare_buffer[22],chksum_debug;
	for(i=0;i<res;i++,totXbowBuf++)	// Copy data to the circular buffer
  	{    circBuf[readPtr++]=*(temp+i); 
	     if(readPtr==(500)) 
		readPtr=0;
	     if(totXbowBuf>500) totXbowBuf=500;
	}	
  	k=readPtr; // After copying contents, check for sync
	
  	if(totXbowBuf>=22)
  	{ // Search for sync
  	   while(search<(43))	
  	   {
  	   	search++; if(search>=(totXbowBuf- 22)) break;
 	   	sync_loc=k-22; if(sync_loc<0) sync_loc+=500;
  	   	if(circBuf[sync_loc]==255)
  	   	{
		  for(i=21,j=k-1;i>=0;)
  	   	  { 	if(j<0) j=499;
			compare_buffer[i--]=circBuf[j--]; 	}
  	   	  chksum_debug=check_xbow(compare_buffer,20);
  	   	  if(chksum_debug==compare_buffer[21])
  	   	  {	// tokenize_xbow_data(compare_buffer); 
			lastDMUupdate = get_time();
			display_data_xbow();
			xbowCount ++;
			return 0; } // Success...
  	   	}
  	   	k--;
	    }	
  	}
  	return -1; // Unsuccessful

}
//-----------------------------------------------------------------------------	
void xbow::read_xbow_polled() 
{
	static unsigned char buf[128],chsum;
	int res,i;
	res=0;
	for(i=0;i<22;i++) buf[i]=0;
	res=read(fd,buf,22);
	if(res==22)  check_xbow(buf,res);
//	fprintf(stderr,"\n");
//	for ( i = 0;i<22; i++) 
//		fprintf(stderr, " %0x,",buf[i]);
	buf[res]=0; 
	lastDMUupdate=get_time();
	// tcflush(fd,TCIFLUSH);
   	// tcflush(fd,TCOFLUSH);
}
//----------------------------------------------------------------------------
void xbow::display_data_xbow()
{
	sharedDIAG->xbowData.roll = roll;
	sharedDIAG->xbowData.pitch = pitch;
	sharedDIAG->xbowData.rollRate = rollRate;
	sharedDIAG->xbowData.pitchRate = pitchRate;
	sharedDIAG->xbowData.yawRate = yawRate;
	sharedDIAG->xbowData.xAccel = xAccel;
	sharedDIAG->xbowData.yAccel = yAccel;
	sharedDIAG->xbowData.zAccel = zAccel;
	sharedDIAG->xbowData.lastXbowTime = lastDMUupdate;
	sharedDIAG->xbowData.temp = temp;
	fprintf(logFile,"\n%.6lf,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f, %8.3f, %8.3f",
			lastDMUupdate,	
			roll,pitch, rollRate, pitchRate, yawRate,
			xAccel, yAccel, zAccel, temp);
}
//-----------------------------------------------------------------------------
int xbow::peek_xbow()
{
	int result;
	Timeout.tv_sec=0;
	Timeout.tv_usec=5000;
	FD_SET(fd, &readfs);
      	result=select(FD_SETSIZE,&readfs, NULL, NULL,&Timeout);   

	unsigned char buf[128];

	if(result>0)
	{
	   if (FD_ISSET(fd, &readfs))  
	   { 	// read_xbow_polled();
		int res = read(fd,buf,22);
		read_xbow_cont(buf,res);
		return 0;
	   }
	   else return -1;
	}
	return result;
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


bool openXbowlogFile()
{
	char file_name[100];
	gettimeofday(&tv,NULL);
	strncpy(full_name, dir_name, sizeof(full_name)-1);
	sprintf(file_name,"XBOW_%ld",tv.tv_sec);
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
	xbow *DMU;
	double lastIMUtime;
	
	fprintf(stderr,"Hello... This program reads the Crossbow DMU Vertical Gyro.");
	DMU = new xbow("/dev/ttyUSB0");
	DMU->open_serial_port();	
	DMU->init_xbow();

	// Let us open files and do other stuff that needs to be done...
	shared = (Q_SHARED*)attach_memory(QBOATShareKey,sizeof(Q_SHARED));
	sharedDIAG = (DIAG_SHARED *)attach_memory(DIAGShareKey,sizeof(DIAG_SHARED));
	sleep(2);
	
	captureQuitSignal();
	createDirectory();
	openXbowlogFile();

	lastIMUtime = get_time();
//	DMU->poll_xbow();
	while(1) {
	  !DMU->peek_xbow();
	  // DMU->poll_xbow();
//	  usleep(1000);
	  if(get_time() - lastIMUtime >= 1) {
		fprintf(stderr,"\n\n\nData rate = %d", xbowCount);
		xbowCount = 0;
		lastIMUtime = get_time();
	  }
	  usleep(500);
	}
}	


