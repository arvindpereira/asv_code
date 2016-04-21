// PID implementation.
// @Author: Arvind Pereira
#include "pid.h"
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <do_share.h>
#include <string.h>
#include <stdlib.h>
#include "../common/ms_timer.h"

extern Q_SHARED *shared;
extern struct timeval tv;
extern FILE *logFile;
extern int quiet;
extern DIAG_SHARED *sharedDIAG;

PID::PID()
{
	strcpy(PIDName,"NONAME");
	Iteration=0;
	LoopState=CLOSEDLOOP;
	RateType=RATE_FB;
	Integral=0; Derivative=1; // PD control
	Last_Err=0;
	Acc_Err=0;
	Ref=0;
	Meas=0;
	Out = 0;
	Ki=0;
	Kp=.9;		// Check for appropriate gains...
	Kd=1.5 * 180/3.1415926535;	
	update4mSharedMem = 0;
	lastTime = get_time();
}

void PID::setName(char *name)
{
	if(strlen(name)<20)
		strcpy(PIDName,name);
	else strcpy(PIDName,"BADNAME");
}

void PID::setGains(double Prop, double Diff, double Integ)
{
	Kp=Prop; Kd=Diff; Ki=Integ;
	if(update4mSharedMem == 1) {
		*Ki4mSharedMem = Ki;
		*Kd4mSharedMem = Kd;
		*Kp4mSharedMem = Kp;
	}	
}

void PID::setLimits(double max, double min)
{
	Max=max; Min=min;
}

void PID::setSHMPointers(double *KpPtr, double *KiPtr, double *KdPtr, double *pTerm, double *iTerm, double *dTerm)
{
	Kp4mSharedMem = KpPtr;
	Ki4mSharedMem = KiPtr;
	Kd4mSharedMem = KdPtr;

	pTermInSharedMem = pTerm;
	iTermInSharedMem = iTerm;
	dTermInSharedMem = dTerm;

	update4mSharedMem = 1;
}

double PID::doPID(double newRef, double newMeas, double Rate, int pidNum)
{
	double err,prop_t=0,int_t=0,der_t=0;
	const int yaw = 1;

	delta = get_time() - lastTime;
	if(delta > 1.0 ) delta = 0;	// This is the min-loop time... All PIDs must run faster than once a second!!!

	
	if(update4mSharedMem) {
		Kp = *Kp4mSharedMem; // sharedDIAG->pidParams.Kp;
		Ki = *Ki4mSharedMem; // sharedDIAG->pidParams.Ki;
		Kd = *Kd4mSharedMem; // sharedDIAG->pidParams.Kd;
	}

	Ref=newRef; Meas=newMeas; 
	if(LoopState==CLOSEDLOOP)
	{
		Iteration++;
		err = Ref-Meas;
		if(!quiet)
		printf (" M %f",Meas);
		if(pidNum == yaw)	// For heading control, we want to choose the smaller angle for rotation...
		{	// if(Meas<0) Meas=360-fabs(Meas); // CHANGED ON Jun 7, 10pm since the new IMU code is -180 to +180.
			err = Ref-Meas;
			if(err>180) err=err-360;
			else if(err< -180) err=360+err;
			if(!quiet)
			printf(" R %f E %f ",Ref,err);
		}
		
		prop_t=err * Kp; // Calculate the Proportional term.
		
		if(Integral)	 // Calculate the Integral term.
		{
			Acc_Err+=err * delta;
			int_t = Ki * Acc_Err;
			if (int_t > Max) 
			{ int_t = Max; Acc_Err=Max/Ki; }
 			if (int_t < Min) 
			{ int_t = Min; Acc_Err=Min/Ki; }
		}
		
		if(Derivative)	 // Calculate the Derivative term.
		{
		  	if(RateType==RATE_FB)	der_t= -Kd * (Rate);
			else der_t = Kd * (err-Last_Err) / delta;
			
			// Removing the Kd saturation...
			//if(der_t > Max) der_t = Max;
			//if(der_t < Min) der_t = Min;	
		}
		

		Last_Err = err;

		Out = prop_t + int_t + der_t;
		if(update4mSharedMem) {	
			*pTermInSharedMem = prop_t;	// sharedDIAG->pidParams.pTerm = prop_t;
			*dTermInSharedMem = der_t;	// sharedDIAG->pidParams.dTerm = der_t;
			*iTermInSharedMem = int_t;	// sharedDIAG->pidParams.iTerm = int_t;
		}
		if(Out > Max) Out = Max;
		if(Out < Min) Out = Min;
		gettimeofday(&tv, NULL);
		fprintf(logFile,"\n$PID_%s,%ld.%06ld,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
			PIDName,tv.tv_sec,tv.tv_usec, Meas, Ref, prop_t, int_t, der_t, Out);		
		return Out;
	}
	else return Ref;
}
