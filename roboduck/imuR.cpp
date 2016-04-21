// Inertial Measurement Unit Reader code for the Q-boat.
// Author: Arvind Pereira
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include "imu.h"
#include <sys/time.h>
#include <math.h>
#include "do_share.h"

extern FILE *logFile;
extern Q_SHARED	*shared;
extern MicroStrain3DMG* imu;
extern struct timeval tv;
// This code relies on code in imu.cc and imu.h

void printIMUreading(imu_vector_t reading)
{

//	gettimeofday(&tv, NULL);
/* 	sprintf(lineOfData,"$IMU,%ld.%06ld,%.3f,%.3f,%.2f,%d,%.1f,%.3f,%c,%c\r\n", 
		tv.tv_sec, tv.tv_usec, gps->latitude, gps->longitude, gps->altitude,
		gps->nsat_used,  gps->Course, gps->GndSpd, gps->mode, gps->rmc_mode
	 );
*/

	char lineOfData[300];
	sprintf(lineOfData,"$IMUVEC,%ld.%06ld,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", 
		tv.tv_sec,tv.tv_usec,
		reading.mx,reading.my,reading.mz,
		reading.ax,reading.ay,reading.az,
		reading.gx,reading.gy,reading.gz);
//	fprintf(stdout,"%s",lineOfData);
	fprintf(logFile,"%s",lineOfData);	
}

void printEulerAngles(imu_euler_t &euler_angles)
{
	char lineOfData[300];
	sprintf(lineOfData,"$IMUANG,%ld.%06ld,%f,%f,%f",tv.tv_sec,tv.tv_usec, 
	shared->SensorData.roll, shared->SensorData.pitch, shared->SensorData.yaw);

	// euler_angles.roll*180.0/M_PI, euler_angles.pitch*180.0/M_PI, euler_angles.yaw*180.0/M_PI);
	fprintf(logFile,"%s\n",lineOfData);
}

bool getIMUreadingPolled()
{
	imu_vector_t reading;
	imu_euler_t  euler_angles;
	if(imu->readStablized(reading))
	{
		gettimeofday(&tv, NULL);
		// Transfer to Shared memory...
		shared->SensorData.rollRate = reading.gx;
		shared->SensorData.pitchRate = reading.gy;
		shared->SensorData.yawRate = reading.gz;
		// We have to store these readings with the time-stamp...
		//writeIMUreading(reading); 
		printIMUreading(reading);
	}else return false;
	
	if(imu->readStablized(euler_angles))
	{
		gettimeofday(&tv, NULL);
		shared->SensorData.roll = euler_angles.roll*180.0/3.1415926535;
		shared->SensorData.pitch = euler_angles.pitch*180.0/3.1415926535;
		shared->SensorData.yaw = euler_angles.yaw*180.0/3.1415926535;

		printEulerAngles(euler_angles);
	}else return false;
	return true;
}
