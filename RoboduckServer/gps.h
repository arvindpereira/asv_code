#ifndef _GPS_H__
 #define _GPS_H__

#include <sys/poll.h>

#define false 0
#define true 1

#define DEFAULT_GPS_BAUD 	19200            /* Baud Rate of GPS Unit. Use 19200. Can use 9600 if needed */
#define GPS_STARTUP_CYCLE_USEC  100000           /* Time that we will wait to get the first round of data from the unit. */
                                                 /* Currently set to 1 second */
#define GPS_STARTUP_CYCLES      10               /* Number of cycles to skip before assuming valid data has arrived */
#define NMEA_MAX_SENTENCE_LEN   83               /* spec says 82, but that done mean squat. Make it big */
#define NMEA_START_CHAR         '$'              /* NMEA start character. Appears at start of all sequences */
#define NMEA_END_CHAR           '\n'             /* NMEA end character. Appears as the last character of all sequences */
#define NMEA_CHKSUM_CHAR        '*'              
#define NUMFIELDS               15               /* The number of fields to display */
#define FIELDLENGTH             10               /* The maximum number of messages in the field */
#define MAXMESSAGES             10               /* The maximum number of GPS messages to display */
#define LAT_FT		        6076 		 /* since there are 6076 feet in a nautical mile (1min latitude at equator) */

char    gps_buf[NMEA_MAX_SENTENCE_LEN];
char    rtcm_buf[1024];
char    nmea_buf[NMEA_MAX_SENTENCE_LEN+1];

size_t nmea_buf_len;

// typedef unsigned int bool;

bool	PGRMC = false;	/* if true, then send the PGRMCE signal to the gps unit once. This signal is toggled */
bool	PGRMC1 = false;	/* if true, then send the PGRMC1E signal to the gps unit once. This signal is toggled */
bool	write_PGRMC = false;	/* if true, then write the formatted pgrmc message to the gps unit */
bool	write_PGRMC1 = false;	/* if true, then write the formatted pgrmc1 message to the gps unit */
bool    gps_fd_blocking;

struct  pollfd fds[2];
struct  field {
    char    name[FIELDLENGTH];                    /* Name of the message */
    char    fields[NUMFIELDS][FIELDLENGTH];       /* the data*/
    char    fieldlabels[NUMFIELDS][FIELDLENGTH];  /* The data labels*/
};

typedef struct field Field;
Field    message[MAXMESSAGES];

int gps_fd;
int fd_count;
int nummessages;
double global_gps_time;

extern int FillBufferGPS();

#endif
