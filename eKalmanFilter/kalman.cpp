/* 
	Author: Arvind Pereira.
	Desc  : This program invokes calls into the 6DOF Extended Direct Feed Forward Kalman Filter written by Srikanth Sarrippalli.
		Updates Shared memory variables with new states. These updated states can be used by the Controllers to perform additional
		control operations.
*/
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <signal.h>
#include <sys/ipc.h>
#include <sys/time.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "ms_timer.h"
#include "do_share.h"
#include "gpsins.h"

#define C_DEG2RAD M_PI/180
#define C_RAD2DEG 180/M_PI

// #define __USEXBOW_DATA__
#define __USE3DMG_DATA__

// Globals
char dir_name[300], full_name[300], data_directory[100]="/home/arvind/";
struct itimerval timer;
struct sigaction sa1,sa2;
FILE *logFile;
struct timeval tv;
struct tm *tm;
time_t t;
// Globals used to initialize the filter...
static double ax_avg, ay_avg, az_avg, p_avg, q_avg, r_avg, heading_avg, roll_avg, pitch_avg;
static double northing_avg, easting_avg, height_avg;
static double northing_vel_avg, easting_vel_avg, height_vel_avg;
double pos[3],vel[3],rate[3],euler[3],accel[3];
int init_kf = 0 , init_imu = 0, init_compass = 0, init_gps;			
int count_compass, count_imu, count_gps;
static double current_time = 0;
static double prev_time = 0;

void quitApp(int signum)
{
	fprintf(stderr,"\nQuitting application.");
	fclose(logFile);
}

void captureQuitSignal()
{
	memset(&sa1, 0, sizeof(sa1));
	sa1.sa_handler = &quitApp;
	sigaction(SIGINT, &sa1, NULL);
	sigaction(SIGTERM, &sa1, NULL);
	sigaction(SIGKILL, &sa1, NULL);
}

//======================= shared memory =====================
void *shared_memory = (void*)0;
Q_SHARED *shared;
DIAG_SHARED *sharedDIAG;
int shmid;

void *attach_memory(int key, int size)
{
  shmid = shmget((key_t)key, sizeof(Q_SHARED),0666|IPC_CREAT);

  if(shmid==-1) {
			printf("shmget failure\n");
    			exit(EXIT_FAILURE);
  }
  shared_memory=shmat(shmid, (void *)0,0);
  if(shared_memory==(void *)-1) {
    printf("shmat failure");
    exit(EXIT_FAILURE);
  }
  printf("Memory attached at %X\n",(int)shared_memory);
  // shared=(Q_SHARED*)shared_memory;
  sleep(1);
 
  return shared_memory;
}

void createDirectory()
{
	int len;

        sprintf(full_name, "%s%%Y%%m%%d/", data_directory);
        time(&t);
        tm = localtime(&t);
        len = strftime(dir_name, sizeof(dir_name) - 1, full_name, tm);
        if(-1 == mkdir(dir_name, 0700))
        {
                if(errno != EEXIST)
                {
                        perror("roboduck unable to create directory");
                }
        }
}

bool openKalmanlogfile()
{
        char file_name[100],full_file_name[300];
        gettimeofday(&tv,NULL);
        strncpy(full_file_name, dir_name, sizeof(full_name)-1);
        sprintf(file_name,"KAL%ld",tv.tv_sec);
        strcat(full_file_name,file_name);
        if ((logFile = fopen(full_file_name, "w")) == NULL)
        {
                fprintf(stderr, "cannot open %s for binary file", full_name);
                perror("and the reason is");
                //imu->close();
                return false;
        }
        else return true;
}

void timer_handler(int signum) {


}

void set_timer(long usec_value)
{
         timer.it_value.tv_sec=0;
	 timer.it_value.tv_usec=usec_value;
	 timer.it_interval.tv_sec=0;
	 timer.it_interval.tv_usec=usec_value;
	 setitimer(ITIMER_REAL, &timer, NULL);
	 // 10 Hz signal
	 memset(&sa2,0,sizeof (sa2));
	 sa2.sa_handler=&timer_handler;
	 sigaction(SIGALRM, &sa2,NULL);
}
									

void initEverything()
{
	int i;
	// Set all our times to zero... so that we know when we need to start initializing the filter.
	// Also zero out our counts on the averages.
	shared->kalmanData.lastKalmanUpdateTime = 0;
	shared->SensorData.lastEulerKalmanTime = 0;
	shared->SensorData.lastImuKalmanTime = 0;
	shared->SensorData.lastGpsKalmanTime = 0;
	shared->SensorData.lastTcmKalmanTime = 0;
	shared->ax1500Data.lastAx1500KalmanTime = 0;
	heading_avg = 0; roll_avg = 0; pitch_avg = 0;

	count_compass = 0;
	count_imu = 0;
	count_gps = 0;

	init_compass = 0;
	init_gps = 0;
	init_kf = 0;
	init_imu = 0;
	
	// Set all our averages to zero.
	p_avg = 0; q_avg = 0; r_avg = 0;
	ax_avg = 0; ay_avg = 0; az_avg = 0;
	heading_avg = 0; roll_avg = 0; pitch_avg = 0;
	for ( i = 0; i<3 ; i++ ) pos[i]=0;
	for ( i = 0; i<3 ; i++ ) vel[i]=0;
	for ( i = 0; i<3 ; i++ ) euler[i]=0;
	
}

// Run the updates for the sensors...
// 3DMG data...
void compassInit()
{
	heading_avg += shared->SensorData.cYaw;
	roll_avg += shared->SensorData.roll;
	pitch_avg += shared->SensorData.pitch;
	count_compass++;
	
	// These angles are in degrees... 
	// the filter requires them to be in radians.
	if(count_compass == 100) {
		heading_avg/=100;
		roll_avg/=100;
		pitch_avg/=100;
		init_compass=1;
	}
	shared->SensorData.lastEulerKalmanTime = get_time();
}

void imuInit()
{
	#ifdef __USE3DMG_DATA__
	 // 3DMG data
	if( fabs(shared->SensorData.xAccel) < 1.5 )  
		ax_avg += shared->SensorData.xAccel;
	if( fabs(shared->SensorData.yAccel) < 1.5 )
		ay_avg += shared->SensorData.yAccel;
	if( fabs(shared->SensorData.zAccel) < 1.5 )
		az_avg += shared->SensorData.zAccel;	
	p_avg += shared->SensorData.rollRate;
	q_avg += shared->SensorData.pitchRate;
	r_avg += shared->SensorData.yawRate;
	#endif
	
	// XBOW data
	#ifdef __USEXBOW_DATA__
	if( fabs(sharedDIAG->xbowData.xAccel) < 1.5 )
		ax_avg += sharedDIAG->xbowData.xAccel; else return;
	if( fabs(sharedDIAG->xbowData.yAccel) < 1.5 )	
		ay_avg += sharedDIAG->xbowData.yAccel; else return;
	if( fabs(sharedDIAG->xbowData.zAccel) < 1.5)
		az_avg += sharedDIAG->xbowData.zAccel; else return;

	p_avg += sharedDIAG->xbowData.rollRate;
	q_avg += sharedDIAG->xbowData.pitchRate;
	r_avg += sharedDIAG->xbowData.yawRate;

	#endif



	count_imu++;
	
	if(count_imu == 100) {
		ax_avg/=100;
		ay_avg/=100;
		az_avg/=100;
		p_avg/=100;
		q_avg/=100;
		r_avg/=100;
		init_imu = 1;
	}
	shared->SensorData.lastImuKalmanTime = get_time();
}

void gpsInit()
{
	if( shared->SensorData.pos_error < 15 ) {
		northing_avg += shared->navData.UTMN;
		easting_avg  += shared->navData.UTME;
		height_avg   += shared->SensorData.altitude;
		count_gps++;
	}

	if(count_gps == 10) {
		northing_avg/=10;
		easting_avg/=10;
		height_avg/=10;
		init_gps = 1;
	}
	shared->SensorData.lastGpsKalmanTime = get_time();
}

void kalmanInit()
{
	pos[0] = northing_avg;
	pos[1] = easting_avg;
	pos[2] = height_avg;

	vel[0] = 0;
	vel[1] = 0;
	vel[2] = 0;

	euler[0] = roll_avg * C_DEG2RAD;
	euler[1] = pitch_avg * C_DEG2RAD;
	euler[2] = heading_avg * C_DEG2RAD;

	if(euler[2] > M_PI)
		euler[2] = euler[2] - 2 * M_PI;

	fprintf(stderr, " initial euler angles %8.4f %8.4f %8.4f\r",euler[0]*C_RAD2DEG, euler[1]*C_RAD2DEG, euler[2]*C_RAD2DEG);
	gpsins_init(euler, pos, vel, rate, 0);
	init_kf = 1;

}

void updateCompass()
{
	euler[0] = shared->SensorData.roll * C_DEG2RAD;
	euler[1] = shared->SensorData.pitch * C_DEG2RAD;
	euler[2] = shared->SensorData.cYaw * C_DEG2RAD;

	if(euler[2]> M_PI)
		euler[2] = euler[2] - 2 * M_PI;
	gpsins_compass_update ( euler[0], euler[1], euler[2]);
	shared->SensorData.lastEulerKalmanTime = get_time();

	fprintf(logFile,"\n$CMPUPDT,%.6lf,%f,%f,%f",shared->SensorData.lastEulerKalmanTime,euler[0],euler[1],euler[2]);
}

void updateImu()
{
	double gyro_sd = 0.01;  // 0.7 degrees/sec (Microstrain manual)
	double yaw_sd = 0.0008; // 0.5 degrees
	double timeDiff;


	// The rates should be in rad/sec while the accels shud be m/sec^2.
	
	#ifdef __USE3DMG_DATA__

	rate[0] = shared->SensorData.rollRate; //* C_DEG2RAD;
	rate[1] = shared->SensorData.pitchRate; // * C_DEG2RAD;
	rate[2] = shared->SensorData.yawRate; // * C_DEG2RAD;

	accel[0] = shared->SensorData.xAccel * 9.8;
	accel[1] = shared->SensorData.yAccel * 9.8;
	accel[2] = shared->SensorData.zAccel * 9.8;
	timeDiff = get_time() - shared->SensorData.lastImuKalmanTime;
	#endif

	#ifdef __USEXBOW_DATA__
	rate[0] = sharedDIAG->xbowData.rollRate * C_DEG2RAD;
	rate[1] = sharedDIAG->xbowData.pitchRate * C_DEG2RAD;
	rate[2] = sharedDIAG->xbowData.yawRate * C_DEG2RAD;

	accel[0] = sharedDIAG->xbowData.xAccel * 9.8;
	accel[1] = sharedDIAG->xbowData.yAccel * 9.8;
	accel[2] = sharedDIAG->xbowData.zAccel * 9.8;
	
	timeDiff = get_time() - shared->SensorData.lastImuKalmanTime;
	#endif

	gpsins_state_update(rate, accel, gyro_sd, yaw_sd, timeDiff, 0);

	shared->SensorData.lastImuKalmanTime = get_time();
	fprintf(logFile,"\n$IMUUPDT,%.6lf,%f,%f,%f,%f,%f,%f,%f,%f,%f",shared->SensorData.lastImuKalmanTime,rate[0],rate[1],rate[2],
			accel[0],accel[1],accel[2],timeDiff,yaw_sd,gyro_sd);
}

	
void updateGPS()
{
	double r_vel = 0.1;
	
	pos[0] = shared->navData.UTMN;
	pos[1] = shared->navData.UTME;
	pos[2] = shared->SensorData.altitude;

	gpsins_gps_pos_update( pos, shared->SensorData.horiz_error, shared->SensorData.horiz_error, shared->SensorData.vert_error, 0 );

	vel[0] = shared->SensorData.vel_Y;
	vel[1] = shared->SensorData.vel_X;
	vel[2] = shared->SensorData.vel_Z;

	gpsins_gps_vel_update( vel, r_vel, r_vel, r_vel, 0 );

	shared->SensorData.lastGpsKalmanTime = get_time();
	fprintf(logFile,"\n$GPSUPDT,%.6lf,%f,%f,%f,%f,%f,%f,%f,%f,%f",shared->SensorData.lastGpsKalmanTime,
		pos[0],pos[1],pos[2],vel[0],vel[1],vel[2],
		shared->SensorData.horiz_error, shared->SensorData.vert_error,r_vel);
}


void updateState()
{
	double att[3], b[3], ab[3];
	double trace;
	int i;
	gpsins_get_state(pos, vel, att, b, ab, &trace);

	gettimeofday(&tv, NULL); 
	fprintf(logFile,"\n$KALMAN,%ld.%06ld,",tv.tv_sec, tv.tv_usec);
	// Update the bias...
	shared->kalmanData.Xpos = pos[1];
	shared->kalmanData.Ypos = pos[0];
	shared->kalmanData.Zpos = pos[2];
	fprintf(logFile,"%f,%f,%f,",shared->kalmanData.Xpos, shared->kalmanData.Ypos,shared->kalmanData.Zpos);
	
	shared->kalmanData.Yaw = att[2];
	shared->kalmanData.Roll = att[0];
	shared->kalmanData.Pitch = att[1];
	
	if(shared->kalmanData.Yaw < 0) shared->kalmanData.Yaw+=360;

	fprintf(logFile,"%f,%f,%f",shared->kalmanData.Yaw, shared->kalmanData.Roll, shared->kalmanData.Pitch);

	for (i = 0 ; i<3 ; i++) {
		shared->kalmanData.ImuBias[i] = b[i];
		shared->kalmanData.AccelBias[i] = ab[i];
	}

	for (i = 0 ; i < 3 ; i++) {
		fprintf(logFile,",%f",shared->kalmanData.ImuBias[i]);
	}
	for( i = 0 ; i < 3 ; i++) {
		fprintf(logFile,",%f",shared->kalmanData.AccelBias[i]);
	}

	// The daft idiot, Arvind... had forgotten to log velocities!!!
	// 08/22/2007
	fprintf(logFile,",%f,%f,%f",vel[0],vel[1],vel[2]);

	shared->kalmanData.Xvel = vel[1];
	shared->kalmanData.Yvel = vel[0];
	shared->kalmanData.Zvel = vel[2];

	shared->kalmanData.lastKalmanUpdateTime = get_time();
	// Done with state update.
	fflush(logFile);
}


int main(int argc, char *argv[])
{
	int done=0; double gpsUpdateRate, imuUpdateRate, eulerUpdateRate;
	long int gpsUpdateCount, imuUpdateCount, eulerUpdateCount, kalmanUpdateCount;
	double lastTime;
	
	// Program begins with the usual logging of data and so on...
	captureQuitSignal();
	shared = (Q_SHARED * )attach_memory(QBOATShareKey, sizeof(Q_SHARED));
	sharedDIAG = (DIAG_SHARED *)attach_memory(DIAGShareKey, sizeof(DIAG_SHARED));

	createDirectory();
	sleep(1);
	openKalmanlogfile();
		
	fprintf(stderr,"\nExtended Kalman Filter v 1.0 for the Q-boat (based on the Heli INS by Srik)");
	
	initEverything();
	lastTime = get_time();	
	// Now begin main loop... 
	while (!done) {
		// Run a loop to check if we've  received new data from the sensors...				
	if((get_time()-lastTime)>=1.0) {
		fprintf(stderr,"\nGPS %dHz, IMU %d Hz, EUL %d Hz, KAL %dHz,  LastTime %f",gpsUpdateCount, imuUpdateCount, eulerUpdateCount,kalmanUpdateCount,lastTime);	

		gpsUpdateCount = 0; imuUpdateCount = 0; eulerUpdateCount = 0; kalmanUpdateCount = 0; lastTime = get_time();
	}

	if(shared->SensorData.lastEulerKalmanTime<shared->SensorData.eulerUpdateTime) {
			++eulerUpdateCount;
			if(!init_compass)
				compassInit();
			else updateCompass();
		}
		#ifdef __USE3DMG_DATA__
		if(shared->SensorData.lastImuKalmanTime<shared->SensorData.imuUpdateTime) {
			if(!init_imu)
				imuInit();
			else updateImu();
			++imuUpdateCount;
		}
		#endif
		#ifdef __USEXBOW_DATA__
		if(shared->SensorData.lastImuKalmanTime<sharedDIAG->xbowData.lastXbowTime) {
			if(!init_imu)
				imuInit();
			else updateImu();
			++imuUpdateCount;
		}
		#endif
		if(shared->SensorData.lastGpsKalmanTime<shared->SensorData.gpsUpdateTime) {
			if(!init_gps)
				gpsInit();
			else updateGPS();
			++gpsUpdateCount;
		}
		if(!init_kf) {
			if(init_gps && init_compass && init_imu)
				kalmanInit();
		}
		// Update the State...
		if(init_kf && (get_time() - shared->kalmanData.lastKalmanUpdateTime)>=0.01) {
			updateState();
			++kalmanUpdateCount;
		}
		usleep(50);
	}
}

