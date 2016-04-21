#include <sys/time.h>
#include <stdio.h>
#include <time.h>

unsigned long last_ms_timer,last_full_timer; // For overflow detection
int ms_overflow=0, full_overflow=0;

unsigned long get_timer()
{
	struct timeval tv;
	unsigned long ms_timer;
	gettimeofday(&tv,NULL);
	// Return the millisecs...
	ms_timer=tv.tv_sec*1000 +tv.tv_usec/1000.0;
	ms_timer%=1000000;
	ms_overflow=(ms_timer<last_ms_timer)?1:0;
	last_ms_timer=ms_timer;
	return ms_timer;
}

unsigned long get_full_timer()
{
	struct timeval tv;
	unsigned long full_timer;
	gettimeofday(&tv,NULL);
	// Return the millisecs...
	full_timer=tv.tv_sec*1000 +tv.tv_usec/1000.0;
	full_overflow=(full_timer<last_full_timer)?1:0;
	last_full_timer=full_timer;
	return full_timer;
}	

char *get_current()
{
	time_t timeval;
	(void)time(&timeval);
	return (ctime(&timeval));
}

double get_time()
{
	struct	timeval tv; double sec;
   	gettimeofday(&tv,NULL);
	sec=(double)tv.tv_sec+(double)tv.tv_usec/1e6;
	return sec;
}


int get_delay()
{
	struct timeval tv;
	static int first_time;
	static unsigned long this_time,last_time;
	int dif;

   	gettimeofday(&tv,NULL);
	this_time=tv.tv_sec*1000+tv.tv_usec/1000.0;
	if(first_time++) 
		 dif = this_time - last_time; 
	else dif=100;	
	last_time = this_time;
	return dif;
}
