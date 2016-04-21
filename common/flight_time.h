//---------------------------------------------------------------------------
// flight_time.h : utility functions for time information
//---------------------------------------------------------------------------
#ifndef __FLIGHT_TIME_H
#define __FLIGHT_TIME_H

#include <sys/time.h>

#define time_current(x) gettimeofday((x), NULL)


//---------------------------------------------------------------------------
// return the current time in timeval structure format
//
// 	- parameters: none
//
// 	- return value : the current time
//
struct timeval time_current_tv(void);


//---------------------------------------------------------------------------
// return the current time in floating-point format
//
// 	- parameters: none
//
// 	- return value : the current time
//
double time_current_fp(void);


#endif	// __FLIGHT_TIME_H
