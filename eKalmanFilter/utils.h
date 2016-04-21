/* (c) Srikanth Saripalli
 * srik@robotics.usc.edu
 */

#ifndef _UTILS_H_
#define _UTILS_H_
#ifdef __cplusplus
extern "C" 
{
#endif
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

typedef int8_t		index_t;
typedef double SECS;
typedef double FEET, METERS, INCHES, CM;
typedef	double DEGREES, RADIANS;
typedef double NEWTONS, LBS, NM, INCHLB;

#define DEG2RAD			(M_PI/180.0)
#define DEG_TO_RAD(deg)		((deg)*DEG2RAD)
#define RAD_TO_DEG(rad) 	((rad)/DEG2RAD)
#define SQR(x) ((x)*(x))


#define GEO_TYNDALL_MERIDIAN       -87
#define GEO_GAINESVILLE_MERIDIAN   -81
#define GEO_JPL_MERIDIAN           -117


/*SMALL_A = Semi-major axis cnst (meters), E2 = eccentricity squared*/
#define GEO_SMALL_A_CLARK1866  6378206.4
#define GEO_E2_CLARK1866       0.0067686580
 
#define GEO_SMALL_A_WGS84      6378137.0
#define GEO_E2_WGS84           0.006694380
/*******************************************************************/
 
#define GEO_SMALL_A         GEO_SMALL_A_WGS84
#define GEO_E2              GEO_E2_WGS84


#define GEO_SMALL_K0        0.9996       /*central meridian scale factor*/
#define GEO_E0              500000.0     /*false easting (meters)       */
#define GEO_N0              10000000.0   /*false northing (meters)      */

/********************************************************************
 * Convert geodetic coordinates (latitude,longitude) to UTM coord-
 * inates (easting,northing).  latitute and longitude are passed
 * in as degrees.  easting and northing are passed out in meters.
 ********************************************************************/
void convertGEOtoUTM (DEGREES latitude, DEGREES longitude, 
		      METERS *easting, METERS *northing);

/********************************************************************
 * Convert UTM coordinates (easting,northing) to geodetic coord-     
 * inates (latitude,longitude).   easting and northing are passed in 
 * as meters.  latitude and longitude are passed out in degrees.     
 ********************************************************************/
void convertUTMtoGEO (METERS easting, METERS northing, 
		      DEGREES *latitude, DEGREES *longitude);


/*****************************************************************
 * Change the central meridian, used in the GEO to UTM conversions
 *****************************************************************/
void setCentralMeridian (DEGREES latitude, DEGREES longitude);


/*****************************************************************
 * Convert compass type heading to xy coord system heading
 *****************************************************************/
RADIANS compassToCartesianHeading (RADIANS heading);

/*****************************************************************
 * Convert cartesian type heading to compass type heading
 *****************************************************************/
RADIANS cartesianToCompassHeading (RADIANS heading);

void ecef2llh(
        double x, 
        double y, 
        double z,
        double *lat,
        double *lon,
        double *height
);

void llh2ecef(
        double lat,
        double lon,
        double height,
        double ecef[3]
);
void nedpts(
        double llh1[3], 
        double llh2[3], 
        double ned[3],
        double lat,
        double lon
);
void ecef2tangent(
        double latitude, 
        double longitude, 
        double rot[3][3]
);


double heading_tan(double y, double x);
/*
void
accel2euler(
	double *	    THETAm,
	const double *	    accel,
        const double *      field,
        const double *      quat
);
*/
void
accel2euler(
	double *	THETAm,
	const double *	accel,
    	const double *  quat
);

void mulNxM(
	void *OUT_ptr,
	const void *A_ptr,
	const void *B_ptr,
	index_t	   n,
	index_t	   m,
	index_t	   p,
	int8_t	   transpose_B,
	int8_t	   add,
        int        verbose
);

void inv33(
    double A[3][3],
    double B[3][3]
);


void print_matrix(
	const char *name,
	const void *M_p,
	index_t	   n,
	index_t	   m
    );


int dpsi_dq(double *q, double *derivative);
int dphi_dq(double *q, double *derivative);
int dtheta_dq(double *q, double *derivative);
static double limit(double value, double min, double max)
{
	        if( value < min )
			return min;
		if( value > max )
		        return max;
		return value;
}
static inline double turn_direction(
		double                  command,
		double                  current
)
{
	        if( current > M_PI_2 && command < -M_PI_2 )
		        return 2 * M_PI + command - current;
		if( current < -M_PI_2 && command > M_PI_2 )
		        return -2 * M_PI + command - current;
		return command - current;
}


#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif
#ifdef __cplusplus
}
#endif
#endif
