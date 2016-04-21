#ifndef __LOGGER_H___
	#define __LOGGER_H__
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
#include <fcntl.h>
#include <ms_timer.h>
#include <do_share.h>

class Logger
{
	private:
	char dir_name[200],full_name[300],data_directory[300];
	time_t t;
	struct timeval tv;
	struct tm *tm;
	int len;

	public:
	FILE *logFile;
	
	Logger(); 
	Logger(char *);
	void createDirectory();
	bool openRoboducklogfile(char *prefix);
	bool closeLogFile();
};	

#endif
