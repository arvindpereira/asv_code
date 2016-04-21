//---------------------------------------------------------------------------
// flight_time.c : implementation of time-related functions
//---------------------------------------------------------------------------
#include "flight_time.h"

#include <stdio.h>		// for NULL


//---------------------------------------------------------------------------
// return the current time in floating-point format
//
struct timeval time_current_tv(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv;
}


//---------------------------------------------------------------------------
// return the current time in floating-point format
//
double time_current_fp(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec / 1000000.0;
}
