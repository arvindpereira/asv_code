// This program generates a sinusoiodal wave 
#include <stdio.h>
#include <math.h>
#include <do_share.h>
#include <ms_timer.h>

extern Q_SHARED *shared;


extern double amp1, freq1, amp2, freq2;
double begin_time;

void doSinusoid()
{
	double t,T1,T2;
	t = get_time() - begin_time;
	T1 = amp1 * sin(2*M_PI*freq1*t);
	T2 = amp2 * sin(2*M_PI*freq2*t);
	shared->qboatData.T1 = T1;
	shared->qboatData.T2 = T2;
}

