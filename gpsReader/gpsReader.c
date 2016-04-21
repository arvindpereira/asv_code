#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <sys/time.h>
#include <stdio.h>
#include <termios.h>
#include <gps.h>
#include <sys/shm.h>
#include <sys/ipc.h>
#include "do_share.h"
#include "Utm.h"
#include "ms_timer.h"
//#include "Constants.h"

//#define _QUIET_

#define PATH_MAX 255


int quiet = 0;
char dir_name[200],full_name[300],log_directory[100]="/home/arvind/";
char raw_full_name[300];
time_t t;
struct tm *tm;
int len;
struct timeval tv;
struct sigaction sa;
FILE *logFile,*rawLogFile;

struct termios oldtio,newtio;

int fd_gps;
// global settings
char serial_port[PATH_MAX] = "/dev/ttyUSB2";
char* dgps_server = NULL;
tcflag_t baudrate = B19200;

void *shared_memory = (void *) 0;
Q_SHARED *shared;
int shmid;

void createDirectory()
{
 	sprintf(full_name, "%s%%Y%%m%%d/", log_directory);
  	time(&t);
  	tm = localtime(&t);
  	len = strftime(dir_name, sizeof(dir_name) - 1, full_name, tm);
  	if(-1 == mkdir(dir_name, 0700))
	{
		if(errno != EEXIST)
		{
			perror("imu reader unable to create directory");
		}
  	}
}

void quitApp(int signum)
{
	fprintf(stderr,"Quitting application.");
	// close file pointers etc...
	exit(0);
}

void captureQuitSignal()
{
	memset(&sa, 0, sizeof(sa));
	sa. sa_handler = &quitApp;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGKILL, &sa, NULL);
}

void writeGPSreading(GPS* gps)
{
	static long old_sec, time_diff;
	char lineOfData[1024];
	gettimeofday(&tv, NULL);
  	sprintf(lineOfData,"%ld.%06ld,%.7f,%.7f,%.2f,%d,%.1f,%.3f,%d,%d\r\n", 
		tv.tv_sec, tv.tv_usec, gps->latitude, gps->longitude, gps->altitude,
		gps->nsat_used,  gps->Course, gps->GndSpd, gps->mode, gps->rmc_mode
	 );
	shared->SensorData.lat = gps->latitude;
	shared->SensorData.lon = gps->longitude;
	shared->SensorData.speed = gps->GndSpd;
	shared->SensorData.course = gps->Course;	
	shared->SensorData.gpsUpdateTime = get_time();
	shared->SensorData.altitude = gps->altitude;
	shared->SensorData.magVar = (gps->mag_ew=='W') ? -gps->mag_var : gps->mag_var;
	shared->SensorData.sats = gps->nsat_used;
	shared->SensorData.dgps_age = gps->dgps_age;
	shared->SensorData.dop = gps->dop;
	shared->SensorData.horiz_error = gps->dop_horiz;
	shared->SensorData.vert_error = gps->dop_vert;
	shared->SensorData.pos_error = gps->dop_pos;
	shared->SensorData.vel_X = gps->vel_east;
	shared->SensorData.vel_Y = gps->vel_north;
	shared->SensorData.vel_Z = gps->vel_up;

	double UTMN, UTME; char zone[5];
	LLtoUTM(23, shared->SensorData.lat, shared->SensorData.lon,
		&UTMN, &UTME, zone);
	strcpy(shared->navData.UTMzone, zone); 
	shared->navData.UTME = UTME;
	shared->navData.UTMN = UTMN;
	time_diff=tv.tv_sec-old_sec;
	if(time_diff>1)
	{
#ifndef _QUIET_
		if(!quiet)
		printf("Writing : %s\n",lineOfData);
#endif
		old_sec=tv.tv_sec;
	}
        fwrite((const char *)lineOfData,1,strlen(lineOfData), logFile);
	fflush(logFile);
}

void attach_memory(int key)
{
	shmid = shmget ((key_t)key, sizeof(Q_SHARED), 0666| IPC_CREAT);

	if(shmid==-1) {
		printf("shmget failure\n");
		exit(EXIT_FAILURE);
	}

	shared_memory = shmat(shmid, (void*)0,0);
	if(shared_memory == (void*)-1) {
		printf("shmat failure");
		exit(EXIT_FAILURE); }
	
		printf("Memory attached at %p\n",shared_memory);
	shared = (Q_SHARED*)shared_memory;
	sleep(1);
}

int openGPSlogfile()
{
	char file_name[100], raw_fileName[100];
	gettimeofday(&tv,NULL);
	strncpy(full_name, dir_name, sizeof(full_name)-1);
	strncpy(raw_full_name, dir_name, sizeof(full_name)-1);
	sprintf(file_name,"GPS%ld",tv.tv_sec);
	sprintf(raw_fileName,"RAWGPS%ld",tv.tv_sec);
	strcat(full_name,file_name);
	strcat(raw_full_name,raw_fileName);
	if ((logFile = fopen(full_name, "wb")) == NULL)
	{
		fprintf(stderr, "cannot open %s for binary file", full_name);
		perror("and the reason is");
		// Close port and quit...	
                return 0;
	}		
	if ((rawLogFile = fopen(raw_full_name, "wb")) == NULL){
		fprintf(stderr, "cannot open %s for binary file", raw_full_name);
		perror("and the reason is");
		// Close port and quit...	
                return 0;
	}

	
	else return 1;
}

// process the command-line arguments
void process_arguments(int argc, char* argv[])
{
    int c;
    while ((c=getopt(argc,argv,"d:b:q")) != -1)
	switch(c)
	{
	    // DGPS server
	    case 'd':
		dgps_server = strdup(optarg);
		break;

	    // baud rate
	    case 'b':
		if (!strcmp(optarg, "4800"))
		    baudrate = B4800;
		else if(!strcmp(optarg, "9600"))
		    baudrate = B9600;
		else if(!strcmp(optarg, "19200"))
		    baudrate = B19200;
		else if(!strcmp(optarg, "38400"))
		    baudrate = B38400;
		else
		    cerr << "[Error] invalid baud rate (" << optarg << ")" << endl;
		break;
	    case 'q':
		printf("\nSo you want me to be quiet??? I'll keep quiet!");
		quiet=1;
		break;

	    // print help message
	    default:
		cerr << "[usage] gps [-d dgps_server] [serial_port]" << endl;
		abort();
	}

    // serial port
    if (optind < argc) {
	strcpy(serial_port, argv[optind]);
    }
}


int main(int argc, char **argv)
{
	char enableRMC[]="$PGRMO,GPRMC,1\r\n";
	char enableRMV[]="$PGRMO,PGRMV,1\r\n";
	char enableRME[]="$PGRMO,PGRME,1\r\n";
	char enableGGA[]="$PGRMO,GPGGA,1\r\n";
	char enableGSV[]="$PGRMO,GPGSV,1\r\n";

	printf("Garmin GPS reader v.0.1\n");	
	// process command line arguments and capture quit signals
	process_arguments(argc, argv);
    	captureQuitSignal();
	// initialize the gps and begin logging...

	attach_memory(QBOATShareKey);
    	// initialize a GPS device
    	GPS* gps;
    	if (dgps_server) gps = new DGPS(dgps_server, serial_port, baudrate);
    	else gps = new GPS(serial_port, baudrate);

    	// Create a directory to store all the data...
    	createDirectory();
    	openGPSlogfile();

	// Initialize our GPS to send us PGRMV strings...
	gps->command(enableRMC,strlen(enableRMC));
	gps->command(enableRMV,strlen(enableRMV));
	gps->command(enableGGA,strlen(enableGGA));
	gps->command(enableRME,strlen(enableRME));
	gps->command(enableGSV,strlen(enableGSV));
	printf("\nAttempted enabling GPRMC and PGRMV sentences.");

	// Just begin reading the port...
	while (true)
    	{
		// retrieve the next sentence
		int sid = gps->update();

		// update the information
		switch (sid)
		{
		    case GPRMC: 
			    writeGPSreading(gps);
			    break;
		    case PGRMC: fprintf(stderr,"\nvel\n"); break;
	/*	    case GPGGA: updateGGA(); break;
		    case GPGSA: updateGSA(); break;
		    case GPGSV: updateGSV(); break;
		    case PGRME: updatePGRME(); break;
		    case PGRMV: updatePGRMV(); break;	*/
		}
	
		// be nice to other threads
		usleep(5000);
    	}
}
