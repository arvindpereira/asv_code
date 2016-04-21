//-----------------------------------------------------------------------------
// gps.cc : implements GPS class
//
//	- original by Boyoon Jung (boyoon@robotics.usc.edu)
//	- modified by Arvind Pereira (menezesp@usc.edu)
//-----------------------------------------------------------------------------
#include "gps.h"
#include <math.h>
#include <cstdlib>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <time.h>
#include <sys/time.h>
// header files for debugging info
#include <iostream>
using std::cerr;
using std::endl;

extern int quiet;


// constructor [GPS]
GPS::GPS(char* device, tcflag_t baudrate)
{
	cerr<<"Creating GPS object \n";
    if (device)
	this->open(device, baudrate);
    else
	serial = 0;

    // initialize a mutex
    pthread_mutex_init(&mutex, NULL);

    // set default values
    set_defaults();
}


// destructor [GPS]
GPS::~GPS(void)
{
    // close the serial port
    this->close();

    // release a mutex
    pthread_mutex_destroy(&mutex);
}


// open a serial port for communication to 3DM-G
bool GPS::open(char* device, tcflag_t baudrate)
{
	serial=::open(device,O_RDWR | O_NOCTTY | O_NONBLOCK);
	cerr<<"Canonical_tty opened \n"<<device;
	flush (cerr);
	tcgetattr(serial,&old_conf);
	termios newtio;
	bzero(&newtio,sizeof(termios));

	newtio.c_cflag=baudrate|CS8|CLOCAL|CREAD;
	
	newtio.c_iflag=IGNPAR;
	newtio.c_oflag=0;
   	newtio.c_lflag=ICANON;
   	newtio.c_cc[VINTR] 	=0;
	newtio.c_cc[VQUIT] 	=0;
	newtio.c_cc[VERASE]	=0;
	newtio.c_cc[VKILL] 	=0;
	newtio.c_cc[VEOF] 	=4;
	newtio.c_cc[VTIME] 	=0;
	newtio.c_cc[VMIN] 	=1;
	newtio.c_cc[VSWTC] 	=0;
	newtio.c_cc[VSTART] 	=0;
	newtio.c_cc[VSTOP] 	=0;
	newtio.c_cc[VSUSP] 	=0;
	newtio.c_cc[VEOL] 	=0;
	newtio.c_cc[VREPRINT]	=0;
	newtio.c_cc[VDISCARD]	=0;
	newtio.c_cc[VWERASE] 	=0;
	newtio.c_cc[VLNEXT]  	=0;
	newtio.c_cc[VEOL2] 	=0;

	tcflush(serial,TCIFLUSH);
	tcsetattr(serial,TCSANOW,&newtio);

    	cerr << "[Info] inializing GPS... ";
    	// while (update() != GPRMC)  usleep(10000);
    	cerr << "Done." << endl;
	flush(cerr);
	return true;
}


// close the serial port
bool GPS::close(void)
{
    if (serial)
    {
	// restore the old configuration
	::tcsetattr(serial, TCSAFLUSH, &old_conf);
	// close the device file
	::close(serial);
    }

    // done
    return true;
}


// set default values
void GPS::set_defaults(void)
{
    // global position system fix data
    strncpy(fix_time, "000000", 6);
    latitude = longitude = 0.0f;
    fix_quality = 0;
    nsat_used = 0;
    dop = 99.9;
    altitude = 0.0f;
    geoid = 0.0f;
    dgps_age = 0;
    drs_id = 0;

    // GPS DOP and active satellites
    mode = 'M';
    fix_type = 1;
    prns_used[0] = -1;
    dop_pos = dop_horiz = dop_vert = 99.9f;

    // satellites in view
    nsat_view = 0;
}


// check if data is available
int GPS::available(void)
{
    int nbytes;
    struct timeval Timeout;
    pthread_mutex_lock(&mutex);
/*
    if (ioctl(serial, TIOCINQ, &nbytes) < 0)
    {
	cerr << "[Error] ioctl for TIOCINQ." << endl;
	nbytes = 0;
    } 
*/
	Timeout.tv_sec = 0;
	Timeout.tv_usec = 1000;
	FD_SET(serial, &readfs);
	int result = select(FD_SETSIZE, &readfs, NULL, NULL, &Timeout);
    	if(result>0)
	{
		ioctl(serial, TIOCINQ, &nbytes);
	}else nbytes = 0;
	pthread_mutex_unlock(&mutex);
	return nbytes;
}



// process a single sentence
int GPS::update(void)
{
    char* token;

    // check if data is available
    if (available() <= 0) return GPNON;
	
    // read the next sentence
    int len;
    while ((len = sread(buffer)) < 0)
	;

	
if (!quiet)	printf("%s\n",buffer);
    // process the sentence
    int id = parser.init(buffer, len);
    if (id >= 0)
    {
	// update the information
	switch (id)
	{
	    // global positioning system fix data
	    case GPGGA:
	    {
//    		cerr<<"Parsing GGA";
		token = parser.next();			// UTC time of position fix
		strncpy(fix_time, token, 6);

		token = parser.next();			// latitude
		char ns = parser.next_char();
		latitude = str2lat(token, ns);

		token = parser.next();			// longitude
		char ew = parser.next_char();
		longitude = str2lon(token, ew);

		fix_quality = parser.next_int();	// GPS quality indication
		nsat_used = parser.next_int();		// number of satellites in use
		dop = parser.next_float();		// horizontal dilution of position

		altitude = parser.next_float();		// altitude
		char m = parser.next_char();
		geoid = parser.next_float();		// height of geoid
		m = parser.next_char();

		dgps_age = parser.next_int();		// # secs since last RTCM trans.
		drs_id = parser.next_int();		// differential reference ID
		break;
	    }

	    case GPRMC:					// Added Jun 9, '06 by Arvind Pereira.
	    {
		token=parser.next();
		strncpy(fix_time, token, 6);
		
		char status = parser.next_char(); 		// A = valid pos, V = NAV receiver Warning

		token = parser.next();			// latitude
		char ns = parser.next_char();
		latitude = str2lat(token, ns);

		token = parser.next();			// longitude
		char ew = parser.next_char();
		longitude = str2lon(token, ew);

		GndSpd = parser.next_float();	// Speed over Gnd in Knots
		
		Course = parser.next_float();	// Course over Gnd in degrees (0-359)

		fix_date[6];
		token = parser.next();
		strncpy(fix_date, token, 6);

		mag_var = parser.next_float(); 	// Magnetic Variation in degrees ( 0 - 180 )
		mag_ew  = parser.next_char();	// Magnetic Variation direction (E or W)
		
		// Check for NMEA mode to be 2.30 for this one...
		rmc_mode = parser.next_char();		// A=Autonomous, D=Differential, E=Estimated, N=Data not valid.
		break;
	    }

	    // GPS DOP and active satellites
	    case GPGSA:
	    {
		mode = parser.next_char();		// mode
		fix_type = parser.next_int();		// fix type
		int idx = 0;
		for (int i=0; i<12; i++) {		// PRNs
		    token = parser.next();
		    if (strlen(token)) prns_used[idx++] = atoi(token);
		}
		prns_used[idx] = -1;	// end-of-list
		dop_pos = parser.next_float();		// position dilution of precision
		dop_horiz = parser.next_float();	// horizontal dilution of precision
		dop_vert = parser.next_float();		// vertical dilution of precision
		break;
	    }

	    // GPS satellites in view
	    case GPGSV:
	    {
		int ns = parser.next_int();		// number of sentences
		int snum = parser.next_int();		// sentence index
		nsat_view = parser.next_int();
		for (int i=(snum-1)*4; i<(snum)*4 || i<nsat_view; i++)
		{
		    sat_view[i].prn = parser.next_int();	// PRN
		    sat_view[i].elevation = parser.next_float();// elevation
		    sat_view[i].azimuth = parser.next_float();	// azimuth
		    sat_view[i].snr = parser.next_int();	// signal strength
		}

		// do not process until full data are ready
		if (snum < ns) id = GPPND;
		break;
	    }

	    // Estimated error information
	    case PGRME:
	    {
		char m;
		horiz_error = parser.next_float();
		m = parser.next_char();
		vert_error = parser.next_float();
		m = parser.next_char();
		pos_error = parser.next_float();
		m = parser.next_char();
		break;
	    }

	    // 3D velocity information
	    case PGRMV:
	    {
		vel_east = parser.next_float();
		vel_north = parser.next_float();
		vel_up = parser.next_float();
		break;
	    }

	    default:
		break;

	    /*
	    // not supported yet
	    default:
		cerr << "[Debug] unsupported sentence ID (" << id << ")" << endl;
	    */
	}
    }

    /*
    buffer[len] = '\0';
    cerr << buffer << endl;
    */
    
    // done
    return id;
}


// send a command to a GPS
bool GPS::command(char* cmd, int len)
{
    bool result;
    pthread_mutex_lock(&mutex);

    int sent = 0;
    int remained = len;
    while (sent < len)
    {
	int s = ::write(serial, cmd+sent, remained);
	if (s >= 0) {
	    sent += s;
	    remained -= s;
	}
	else {
	    result = false;
	    break;
	}
    }
    result = true;

    pthread_mutex_unlock(&mutex);
    return result;
}


// utility functions to convert geodetic to ECEF positions
void GPS::ecef(float lat, float lon, float alt, float geo,
	       double& x, double& y, double& z)
{
    double slat = sin(lat* M_PI / 180);
    double clat = cos(lat* M_PI / 180);
    double slon = sin(lon* M_PI / 180);
    double clon = cos(lon* M_PI / 180);

    double height = alt + geo;
    double N = WGS84_A / sqrt(1 - WGS84_E*WGS84_E * slat*slat);

    x = (N + height) * clat * clon;
    y = (N + height) * clat * slon;
    z = ((WGS84_B*WGS84_B) / (WGS84_A*WGS84_A) * N + height) * slat;
}


// utility functions to convert geodetic to UTM positions
void GPS::utm(float lat, float lon, double& x, double& y)
{
    // constants
    const static double m0 = (1 - UTM_E2/4 - 3*UTM_E4/64 - 5*UTM_E6/256);
    const static double m1 = -(3*UTM_E2/8 + 3*UTM_E4/32 + 45*UTM_E6/1024);
    const static double m2 = (15*UTM_E4/256 + 45*UTM_E6/1024);
    const static double m3 = -(35*UTM_E6/3072);

    // compute the central meridian
    int cm = (lon >= 0.0) ? ((int)lon - ((int)lon)%6 + 3) : ((int)lon - ((int)lon)%6 - 3);

    // convert degrees into radians
    double rlat = lat * M_PI/180;
    double rlon = lon * M_PI/180;
    double rlon0 = cm * M_PI/180;

    // compute trigonometric functions
    double slat = sin(rlat);
    double clat = cos(rlat);
    double tlat = tan(rlat);

    // decide the flase northing at origin
    double fn = (lat > 0) ? UTM_FN_N : UTM_FN_S;

    double T = tlat * tlat;
    double C = UTM_EP2 * clat * clat;
    double A = (rlon - rlon0) * clat;
    double M = WGS84_A * (m0*rlat + m1*sin(2*rlat) + m2*sin(4*rlat) + m3*sin(6*rlat));
    double V = WGS84_A / sqrt(1 - UTM_E2*slat*slat);

    // compute the easting-northing coordinates
    x = UTM_FE + UTM_K0 * V *
	(A + (1-T+C)*pow(A,3)/6 + (5-18*T+T*T+72*C-58*UTM_EP2)*pow(A,5)/120);
    y = fn + UTM_K0 * (M + V * tlat *
	(A*A/2 + (5-T+9*C+4*C*C)*pow(A,4)/24 + (61-58*T+T*T+600*C-330*UTM_EP2)*pow(A,6)/720));
}


// convert a string to a sentence ID
int NMEAParser::token2id(char token[])
{
    // NMEA-0183 sentences [GPS receiver]
    if (! strncmp(token, "GP", 2))
    {
	if (! strcmp(token+2, "ALM")) return GPALM;
	else if (!strcmp(token+2, "BOD")) return GPBOD;
	else if (!strcmp(token+2, "BWC")) return GPBWC;
	else if (!strcmp(token+2, "BWR")) return GPBWR;
	else if (!strcmp(token+2, "DBT")) return GPDBT;
	else if (!strcmp(token+2, "GGA")) return GPGGA;
	else if (!strcmp(token+2, "GLL")) return GPGLL;
	else if (!strcmp(token+2, "GSA")) return GPGSA;
	else if (!strcmp(token+2, "GSV")) return GPGSV;
	else if (!strcmp(token+2, "HDM")) return GPHDM;
	else if (!strcmp(token+2, "HSC")) return GPHSC;
	else if (!strcmp(token+2, "MTW")) return GPMTW;
	else if (!strcmp(token+2, "R00")) return GPR00;
	else if (!strcmp(token+2, "RMB")) return GPRMB;
	else if (!strcmp(token+2, "RMC")) return GPRMC;
	else if (!strcmp(token+2, "RTE")) return GPRTE;
	else if (!strcmp(token+2, "VHW")) return GPVHW;
	else if (!strcmp(token+2, "VWR")) return GPVWR;
	else if (!strcmp(token+2, "VTG")) return GPVTG;
	else if (!strcmp(token+2, "WCV")) return GPWCV;
	else if (!strcmp(token+2, "WDC")) return GPWDC;
	else if (!strcmp(token+2, "WDR")) return GPWDR;
	else if (!strcmp(token+2, "WPL")) return GPWPL;
	else if (!strcmp(token+2, "XTE")) return GPXTE;
	else if (!strcmp(token+2, "XTR")) return GPXTR;
	else {
	    cerr << "[Debug] unknown or not supported sentence ID (" << token << ")" << endl;
	    return GPERR;
	}
    }

    // NMEA sentences [Loran-C receiver]
    else if (! strncmp(token, "LC", 2))
    {
	if (! strcmp(token+2, "GLL")) return LCGLL;
	else if (!strcmp(token+2, "VTG")) return LCVTG;
	else {
	    cerr << "[Debug] unknown or not supported sentence ID (" << token << ")" << endl;
	    return GPERR;
	}
    }

    // proprietary sentences [Garmin]
    else if (! strncmp(token, "PGRM", 4))
    {
	if (! strcmp(token+4, "B")) return PGRMB;
	else if (!strcmp(token+4, "C")) return PGRMC;
	else if (!strcmp(token+4, "CE")) return PGRMCE;
	else if (!strcmp(token+4, "C1")) return PGRMC1;
	else if (!strcmp(token+4, "C1E")) return PGRMC1E;
	else if (!strcmp(token+4, "E")) return PGRME;
	else if (!strcmp(token+4, "F")) return PGRMF;
	else if (!strcmp(token+4, "I")) return PGRMI;
	else if (!strcmp(token+4, "IE")) return PGRMIE;
	else if (!strcmp(token+4, "O")) return PGRMO;
	else if (!strcmp(token+4, "T")) return PGRMT;
	else if (!strcmp(token+4, "V")) return PGRMV;
	else if (!strcmp(token+4, "V")) return PGRMV;
	else if (!strcmp(token+4, "Z")) return PGRMZ;
	else {
	    cerr << "[Debug] unknown or not supported sentence ID (" << token << ")" << endl;
	    return GPERR;
	}
    }

    // proprietary sentences [Starlink]
    else if (!strncmp(token, "PSL", 3))
    {
	if (! strcmp(token+3, "IB")) return PSLIB;
	else {
	    cerr << "[Debug] unknown or not supported sentence ID (" << token << ")" << endl;
	    return GPERR;
	}
    }

    // proprietary sentences [Magellan]
    else if (! strncmp(token, "PMGN", 4))
    {
	if (! strcmp(token+4, "ST")) return PMGNST;
	else {
	    cerr << "[Debug] unknown or not supported sentence ID (" << token << ")" << endl;
	    return GPERR;
	}
    }

    // unknown or not supported
    cerr << "[Debug] unknown or not supported sentence ID (" << token << ")" << endl;
    return GPERR;
}


// initialize a NMEA parser
int NMEAParser::init(char sentence[], int len)
{
    // make a copy of the sentence
    if (len < 0) this->len = strlen(sentence);
    else this->len = len;
    strncpy(buffer, sentence, len);
    sentence[len] = '\0';

    // initialize the pointer
    ptr = 0;

    // return the sentence ID
    return token2id(next());
}


// retrive the next token
char* NMEAParser::next(void)
{
    // check if there is a token left
    if (ptr > len) return buffer+len;

    // search for the end of a token
    int end = ptr;
    while (end < len && buffer[end] != ',' && buffer[end] != '\0')
	end++;

    // return the found token
    buffer[end] = '\0';
    int start = ptr;
    ptr = end + 1;
    return buffer+start;
}


// retrive the next token as a character
char NMEAParser::next_char(void)
{
    // check if there is a token left
    if (ptr > len) return -1;

    // search for the end of a token
    int end = ptr;
    while (end < len && buffer[end] != ',' && buffer[end] != '\0')
	end++;

    // return the found token
    buffer[end] = '\0';
    int start = ptr;
    ptr = end + 1;
    return buffer[start];
}


// retrive the next token as an integer
int NMEAParser::next_int(void)
{
    // check if there is a token left
    if (ptr > len) return -1;

    // search for the end of a token
    int end = ptr;
    while (end < len && buffer[end] != ',' && buffer[end] != '\0')
	end++;

    // return the found token in integer format
    buffer[end] = '\0';
    int start = ptr;
    ptr = end + 1;
    return atoi(buffer+start);
}


// retrive the next token as a floating point
float NMEAParser::next_float(void)
{
    // check if there is a token left
    if (ptr > len) return -1.0f;

    // search for the end of a token
    int end = ptr;
    while (end < len && buffer[end] != ',' && buffer[end] != '\0')
	end++;

    // return the found token in floating-point format
    buffer[end] = '\0';
    int start = ptr;
    ptr = end + 1;
    return float(atof(buffer+start));
}


// constructor [DGPS]
DGPS::DGPS(char* dgps_server, char* device, tcflag_t baudrate)
    	: GPS(device, baudrate)
{
    // set the server address
    hostent* host = gethostbyname(dgps_server);
    if (host == NULL)
    {
	cerr << "[Error] unknown host (" << dgps_server << ")" << endl;
	ss = -1;
    }

    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(DGPS_PORT);
    memcpy(&addr.sin_addr, host->h_addr_list[0], host->h_length);
    bzero(&addr.sin_zero, 0);

    // make a connection to a server
    ss = ::socket(AF_INET, SOCK_STREAM, 0);
    if (ss < 0 || ::connect(ss, (sockaddr*)&addr, sizeof(addr)) < 0)
    {
	if (ss >= 0) ::close(ss);
	ss = -1;
	cerr << "[Error] cannot connect to a DGPS server." << endl;
    }
}


// destructor [DGPS]
DGPS::~DGPS(void)
{
    // disconnect from a server
    if (ss >= 0) ::close(ss);
}


// process a single sentence
int DGPS::update(void)
{
    // process RTCM data if exist
    if (ss >= 0)
    {
	int nbytes;
	if (ioctl(ss, FIONREAD, &nbytes) < 0)
	{
	    cerr << "[Error] ioctl for FIONREAD." << endl;
	    nbytes = 0;
	}

	if (nbytes > 0)
	{
	    rtcm_len = ::read(ss, rtcm_buffer, MAX_RTCM);
	    if (rtcm_len > 0) {
		GPS::command(rtcm_buffer, rtcm_len);
		rtcm_buffer[rtcm_len] = '\0';
		cerr << "RTCM: " << rtcm_buffer << endl;
	    }
	}
    }

    // process NMEA data
    return GPS::update();
}
