#ifndef __TCM_H
#define __TCM_H

#include "flight_serial.h"

#include <math.h>
#include <limits.h>
#include <signal.h>

#define DEFAULT_COMPASS_SPEED B9600

typedef struct
{
	char device[PATH_MAX];
	speed_t speed;
} tcm_config_t;

typedef struct
{
	float heading, roll, pitch;
	float x, y, z;
	float t, e;
	double timestamp;
	int new_data;
} tcm_compass_t;


int init_tcm_compass (tcm_config_t *cfg);

int close_tcm_compass(void);

int read_tcm_packet(unsigned char packet[]);

#endif
