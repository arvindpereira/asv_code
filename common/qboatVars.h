/*
	Author: Arvind A de Menezes Pereira
	Desc. : Q-Boat Variable declarations. Contains the basic data structures that are being shared via Shared Memory.
*/

#ifndef _QBOAT_VARS_H_
	#define _QBOAT_VARS_H_

#define WAYPT_NOTACHV	0
#define WAYPT_ACHV	1

#define NO_WAIT		0
#define WAIT_WAYPT	1
#define WAIT_TIMEOUT	2

#define MISS_LOADED	99
#define NO_MISS		0
#define EXEC_MISS	1
#define STOP_MISS	2
#define MISS_COMPLETE	3

#define QBOAT_GUIDANCE	0
#define QBOAT_AUTOPILOT	1
#define QBOAT_DIRECT	2
#define QBOAT_OBSAVOID	3
#define QBOAT_SINUSOID	4
#define QBOAT_STATION_KEEP 5

#define NO_ACTION	0
#define TURN_LEFT	1
#define TURN_RIGHT	2
#define BACKUP		3



typedef struct
{
	int 	lineNum;
	int	numCmds;
	int	WaitOnStatus;
	int	ExecutionStatus;
	int     missionLoaded;
}MISSION_STATUS;


typedef struct
{
	double time;
	double T1,T2;
	double winchDepth;
	int    qboat_mode;	// Autonomous, Joystick
	int    WayPtAchv;
}QBOAT_STATUS;

typedef struct
{
	// 3DMG
	double roll, pitch, yaw,cYaw;
	double eulerUpdateTime;
	double lastEulerKalmanTime;
	double rollRate, pitchRate, yawRate;
	double xAccel, yAccel, zAccel;
	double imuUpdateTime;
	double lastImuKalmanTime;
	
	// 3DMG-X1 
	int sn;
    	float temp, ticks;
    	float mag[3];       /*  magetic */
    	float quat[4];      /*  quaternions */
    	float xform[3][3];  /*  transformation matrix */

	// Garmin GPS 16/A
	double lat, lon, speed, magVar, course;
	int dgps,sats,qof,dgps_age;
	double dop,altitude;
	double horiz_error, vert_error, pos_error;
	double gpsUpdateTime;
	double lastGpsKalmanTime;
	double vel_X, vel_Y, vel_Z;
	
	// TCM2
	double TCMroll, TCMpitch, TCMyaw;
	double tcmUpdateTime;
	double lastTcmKalmanTime;
}NAVSENSOR;

typedef struct
{
	double AcceptRad;
	double des_heading, des_speed;
	double desUpdateTime;
	double UTME, UTMN;
	double UTME_ref, UTMN_ref; 	// Origin in X,Y co-ods
//	double StationE, StationN;	// References for station-keeping...
	double lastUTME,lastUTMN;
	char UTMzone[5];
}NAVPARAMS;

typedef struct
{
	// Everything Kalman...
	// The State Consists of:
	// Position(3), Velocity(3), Quaternions(4), G, IMU_bias(3), Accel_bias(3) 
	double Xpos, Ypos, Zpos;
	double Xvel, Yvel, Zvel;
	double Roll, Pitch, Yaw;
	double ImuBias[3];
	double AccelBias[3];
	double kalmanUpdateTime;
	double lastKalmanUpdateTime;
}KALMANOUTPUTS;

typedef struct{
	int leftMotor;
	int rightMotor;
	int mode;
	float batteryVoltage;
	double ax1500UpdateTime;
	double lastAx1500KalmanTime;
}AX1500;

typedef struct
{
	double sonde_time, sonde_flo, sonde_temp, sonde_do, sonde_ph, sonde_batt_voltage, boat_batt_voltage, winch_depth;
}SONDE_DATA;

typedef struct
{
	int status;
	double obsUpdateTime;
	double lastObsKalmanTime;
}OBSTACLE_DATA;

typedef struct
{
	double lat,lon,range;
	double sonarUpdateTime;
	double lastSonarKalmanTime;
}SONAR_DATA;

typedef struct
{
	double Kp;
	double Kd;
	double Ki;
	double pTerm;
	double dTerm;
	double iTerm;
	double yRateBiasComp;
}PIDPARAMS;

typedef struct
{
	double leftThComp;
	double rightThComp;
}MOTOR_PARAMS;

typedef struct
{
	double roll, pitch;
	double rollRate, pitchRate, yawRate;
	double xAccel, yAccel, zAccel;
	double temp;
	double lastXbowTime;
}DMU_DATA;

typedef struct
{
	double rudAng;
	double lastRudTime;
}RUDDER_DATA;

typedef struct
{
	double rawWindSpd;
	double rawWindDirn;
	double windDirnLocal;
	double windDirnGlobal;
	double windSpeed;
	double battVoltage;
	double lastWindUpdateTime;
}WIND_DATA;

#endif
