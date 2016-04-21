#ifndef _MS_TIMER_H_
	#define _MS_TIMER_H_

//extern unsigned long last_ms_timer,last_full_timer; // For overflow detection
//extern int ms_overflow, full_overflow;

double get_time();
unsigned long get_timer();
unsigned long get_full_timer();
char *get_current();
int get_delay();
#endif
