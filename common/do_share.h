#ifndef _DO_SHARE_H_
	#define _DO_SHARE_H_
#include "qboatVars.h"

#define QBOATShareKey 1234
#define DIAGShareKey 5678

typedef struct
{
	QBOAT_STATUS	qboatData;
	NAVSENSOR 	SensorData;
	NAVPARAMS 	navData;
	SONDE_DATA 	sondeData;
	MISSION_STATUS  missData;
	OBSTACLE_DATA obsData;
	SONAR_DATA	sonarData;
	AX1500		ax1500Data;
	KALMANOUTPUTS	kalmanData;
}Q_SHARED;


typedef struct 
{
 	PIDPARAMS pidParams;
	PIDPARAMS SpdPidParams;
	MOTOR_PARAMS motorParams;
	DMU_DATA    xbowData;
	RUDDER_DATA	rudderData;
	WIND_DATA	windData;
}DIAG_SHARED;

struct MakeDir
{
	long int message_type;
	char dir_name[50];
};

void init_shared_mem();
void attach_mem();
void detach_mem();

#endif
