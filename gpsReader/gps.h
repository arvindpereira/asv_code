//-----------------------------------------------------------------------------
// gps.h : a device class for standard GPS devices
//
//	- implements NMEA-0183, version 2.0
//
//	- programmed by Boyoon Jung (boyoon@robotics.usc.edu)
//  - further modified by Arvind Pereira (menezesp@usc.edu)
//-----------------------------------------------------------------------------
#ifndef __GPS_H_
#define __GPS_H_

// header files for a serial port
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termio.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>

// header files for string manipulation
#include <cstring>
#include <cstdlib>
#include <cmath>

// header files for error handling
#include <cstdio>
#include <cerrno>
#include <iostream>
using std::cerr;
using std::endl;

extern FILE *rawLogFile;


// NMEA-0183 sentences [GPS receiver]
#define GPALM	0	// GPS almanac data
#define GPAPB	1	// Autopilot format B
#define GPBOD	2	// bearing & origin to destination waypoint
#define GPBWC	3	// bearing & distance to waypoint [great circle]
#define GPBWR	4	// bearing & distance to waypoint [rhumb line]
#define GPDBT	5	// depth below transducer
#define GPGGA	6	// GPS fix data
#define GPGLL	7	// geographic position, latitude & longitude
#define GPGSA	8	// GPS DOP and active satellites
#define GPGSV	9	// GPS satellites in view
#define GPHDM	10	// heading [magnetic]
#define GPHSC	11	// command heading to steer
#define GPMTW	12	// water temperature [celcious]
#define GPR00	13	// list of waypoint IDs in currently active route
#define GPRMB	14	// recommended minimum navigation info
#define GPRMC	15	// recommended minimum specific GPS/Transit data
#define GPRTE	16	// waypoints in active route
#define GPVHW	17	// water speed and heading
#define GPVWR	18	// relative wind direction and speed
#define GPVTG	19	// track made good and ground speed with GPS talker ID
#define GPWCV	20	// waypoint closure velocity
#define GPWDC	21	// distance to waypoint
#define GPWDR	22	// waypoint distance [rhumb line]
#define GPWPL	23	// waypoint location
#define GPXTE	24	// cross track error [measured]
#define GPXTR	25	// cross track error [dead reckoning]

// NMEA sentences [Loran-C receiver]
#define LCGLL	100	// geographic position with LORAN talker ID
#define LCVTG	101	// track made good and ground speed with LORAN talker ID

// proprietary sentences [Garmin]
#define PGRMB	200	// DGPS beacon info
#define PGRMC	201	// sensor configuration info
#define PGRMCE  202	// sensor configuration info retrieval
#define PGRMC1  203	// additional sensor configuration info
#define PGRMC1E 204	// additional sensor configuration info retrieval
#define PGRME	205	// estimated error info
#define PGRMF	206	// GPS fix data sentence
#define PGRMI	207	// sensor initialization info
#define PGRMIE	208	// sensor initialization info retrieval
#define PGRMO	209	// output senstence enable/disable
#define PGRMT	210	// sensor status info
#define PGRMV	211	// 3D velocity info
#define PGRMZ	212	// Altitude info

// proprietary sentences [Starlink]
#define PSLIB	300	// tune DGPS beacon receiver

// proprietary sentences [Magellan]
#define PMGNST	400	// waypoint and route maintenance

// for error handling
#define GPPND	-1	// pending for full data
#define GPERR	-2	// error
#define GPNON	-3	// no data is available


// WGS84 Parameters
#define WGS84_A		6378137.0		// major axis
#define WGS84_B		6356752.31424518	// minor axis
#define WGS84_F		0.0033528107		// ellipsoid flattening
#define WGS84_E		0.0818191908		// first eccentricity
#define WGS84_EP	0.0820944379		// second eccentricity


// UTM Parameters
#define UTM_K0		0.9996			// scale factor
#define UTM_FE		500000.0		// false easting
#define UTM_FN_N	0.0			// false northing on north hemisphere
#define UTM_FN_S	10000000.0		// false northing on south hemisphere
#define UTM_E2		(WGS84_E*WGS84_E)	// e^2
#define UTM_E4		(UTM_E2*UTM_E2)		// e^4
#define UTM_E6		(UTM_E4*UTM_E2)		// e^6
#define UTM_EP2		(UTM_E2/(1-UTM_E2))	// e'^2


// constants
#define DCS	'$'	// sentence start delimiter character
#define DCE	'\r'	// termination delimiter character
#define DSE	"\r\f"	// termination delimiter string
#define CSF	'*'	// checksum field character

#define MAX_NMEA	82		// maximum length of a NMEA sentence
#define MAX_RTCM	1024		// maximum length of a RTCM datum
#define DGPS_PORT	7777		// server port number


// structures
struct satinfo_t
{
    int prn;		// PRN number
    float elevation;	// elevation in degree
    float azimuth;	// azimuth in degree
    int snr;		// signal strength
};



// class definition : tokenizer for NMEA sentences
class NMEAParser
{
    protected:
	int len;			// length of a sentence
	char buffer[MAX_NMEA];		// copy of a sentence
	int ptr;			// poniter

	// convert a string to a sentence ID
	int token2id(char token[]);

    public:
	// initialize the parser
	int init(char sentence[], int len=-1);

	// retrive the next token
	char* next(void);

	// retrive the next token as a character
	char next_char(void);

	// retrive the next token as an integer
	int next_int(void);

	// retrive the next token as a floating point
	float next_float(void);
};



// class definition : GPS interface
class GPS
{

 char someData[1024];

struct timeval tv_;

    protected:
	int serial;			// serial port
	termios old_conf;		// old configuration of the serial port
	pthread_mutex_t mutex;		// for multi-threading

	char buffer[MAX_NMEA];		// internal message buffer
	NMEAParser parser;		// sentence parser

	fd_set	readfs;			// Added by Arvind for select...
	
	// verify a sentence using a checksum
	bool verify(char* msg, int len, char* checksum);

	// read a sentence from a GPS (blocking opr)
	int sread(char* msg);

	// convert a latitude string to a floating point number
	double str2lat(char* lat, char ns);

	// convert a longitude string to a floating point number
	double str2lon(char* lon, char ew);

	// set default values
	void set_defaults(void);

    public:
	// global positioning system fix data
	char fix_time[6];		// UTC time of position fix [hhmmss]
	char fix_date[6];
	double latitude;			// latitude
	double longitude;		// longitude
	int fix_quality;		// GPS quality indication [0/1/2/6]
	int nsat_used;			// number of satellites in use
	float dop;			// horizontal dilution of precision
	float altitude;			// altitude in meters above mean sea level
	float geoid;			// height of geoid above WGS84 ellipsoid
	int dgps_age;			// number of secs since last valid RTCM transmission
	int drs_id;			// differenctial reference ID

	// GPS DOP and active satellites
	char mode;			// mode
	int fix_type;			// fix type [1/2/3]
	int prns_used[12];		// PRN numbers
	float dop_pos;			// position dilution of precision
	float dop_horiz;		// horizontal dilution of precision
	float dop_vert;			// vertical dilution of precision

	// satellites in view
	int nsat_view;			// number of satellites in view
	satinfo_t sat_view[12];		// satellite info

	// estimated error information
	float horiz_error;		// horizontal error
	float vert_error;		// vertical error
	float pos_error;		// overall position error

	// 3D velocity information
	float vel_east;			// velocity on x-axis
	float vel_north;		// velocity on y-axis
	float vel_up;			// velocity on z-axis

	// GPRMC information
	float Course;			// Course of the Vehicle in degrees
	float GndSpd;			// Velocity in Knots
	float mag_var;			// Magnetic Variation in degrees
	float mag_ew;			// Magnetic direction.
	char  rmc_mode;			// Mode in RMC ( A/D/E/N )
    public:
	// constructor
	GPS(char* device=0, tcflag_t baudrate=B4800);
	// destructor
	~GPS(void);

	// open a serial port for communication to a GPS unit
	bool open(char* device, tcflag_t baudrate=B4800);

	// close the serial port
	bool close(void);

	// check if data is available
	int available(void);

	// process a single sentence
	virtual int update(void);

	// send a command to a GPS unit
	bool command(char* cmd, int len);

	// utility functions to convert geodetic to ECEF positions
	static void ecef(float latitude, float longitude, float altitude, float geoid,
			 double& x, double& y, double& z);
	void ecef(double& x, double& y, double& z) {
	    ecef(latitude, longitude, altitude, geoid, x, y, z);
	}

	// utility functions to convert geodetic to UTM positions
	static void utm(float latitude, float longitude, double& x, double& y);
	void utm(double& x, double& y) { utm(latitude, longitude, x, y); }

	// height from the reference ellipsoid
	float height(void) { return altitude + geoid; }
};



// verify a sentence using a checksum
inline bool GPS::verify(char* msg, int len, char* checksum)
{
    // convert the checksum string to an integer
    int msb = checksum[0] - '0';
    if (msb > 9) msb -= 7;
    int lsb = checksum[1] - '0';
    if (lsb > 9) lsb -= 7;
    int orig =  msb * 16 + lsb;

    // compute the checksum of the sentence
    int test = 0;
    for (int i=0; i<len; i++) test ^= msg[i];

    // check the integrity of the sentence
    if (test == orig)
	return true;
    else
	return false;
}


// read a sentence from a GPS (blocking opr)
inline int GPS::sread(char* msg)
{
    memset(msg,0,150);
    pthread_mutex_lock(&mutex);

    // wait until the command arrives the other side
    tcdrain(serial);

    // wait for the sentence start delimiter
    while (::read(serial, msg, 1) < 0 || msg[0] != DCS)
	;

    // read a sentence
    int len = 0;
    do {
	len += ::read(serial, msg+len, 1);
    } while (msg[len-1] != DCE && msg[len-1] != CSF);
    len--;

	
    // check if there is a checksum byte
    if (msg[len] == CSF)
    {
	gettimeofday(&tv_, NULL);
  	sprintf(someData,"%ld.%06ld,%s\n",tv_.tv_sec,tv_.tv_usec,msg);
	fprintf(rawLogFile,"%s",someData);
	fflush(rawLogFile);


	::read(serial, msg+len, 1);		// read checksum
	::read(serial, msg+len+1, 1);
	::read(serial, msg+len+2, 1);		// read out CR
	::read(serial, msg+len+3, 1);		// read out LF
	if (! verify(msg, len, msg+len)) {
//	    msg[len+2] = '\0';
//	    cerr << "[Error] checksum error <" << msg << ">" << endl;
	    len = -1;
	}
    }
    else
	::read(serial, msg+len, 1);		// read out LF

    // done
    pthread_mutex_unlock(&mutex);
    return len;
}


// convert a latitude string to a floating point number
inline double GPS::str2lat(char* lat, char ns)
{
    double latitude = atof(lat+2) / 60.0;
    lat[2] = '\0';
    latitude += atol(lat);
    return (ns == 'N') ? latitude : -latitude;
}


// convert a longitude string to a floating point number
inline double GPS::str2lon(char* lon, char ew)
{
    double longitude = atof(lon+3) / 60.0;
    lon[3] = '\0';
    longitude += atol(lon);
    return (ew == 'E') ? longitude : -longitude;
}


// class definition : DGPS interface
class DGPS : public GPS
{
    private:
	int ss;					// server socket descriptor

	char rtcm_buffer[MAX_RTCM];		// buffer for RTCM data
	int rtcm_len;

    public:
	// constructor
	DGPS(char* dgps_server=(char*)"localhost", char* device=0, tcflag_t baudrate=B4800);
	// destructor
	~DGPS(void);

	// process a single sentence
	virtual int update(void);
};


#endif	// __GPS_H_
