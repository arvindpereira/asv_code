/* Utility Routines
 * (c) Srikanth Saripalli
 * srik@robotics.usc.edu
 */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "utils.h"
#include "quat.h"


#ifndef EPSILON
#define EPSILON 1e-5
#endif


int centralMeridian = GEO_JPL_MERIDIAN;
static RADIANS lambda0 = DEG_TO_RAD(GEO_JPL_MERIDIAN);
static int initialLat = 0;
static int cm_set = FALSE;

int get_central_meridian(DEGREES longitude)
/********************************************************************/
/*This function returns the central meridian of the zone            */
/*that longitude (in deg) lies in.  If longitude is negative, i.e., */
/*in the western hemisphere, then central meridian is negative.     */
/********************************************************************/
{
  if(longitude >= 0.0)
    return((int)longitude - ((int)longitude)%6 + 3);
  else
    return((int)longitude - ((int)longitude)%6 - 3);
}    


/********************************************************************/
/* Convert geodetic coordinates (latitude,longitude) to UTM coord-  */
/* inates (easting,northing).  latitute and longitude are passed    */
/* in as degrees.  easting and northing are passed out in meters.   */
/********************************************************************/

void convertGEOtoUTM (DEGREES latitude, DEGREES longitude, 
		      METERS *easting, METERS *northing)
{
  //  static int cm_set = 0;
  double A, T, C, ep2, N, M1, M2, M3, M4, M, X1, X2, X, Y1, Y2, Y ;
  double phe, lambda, s_phe, c_phe, t_phe ;
  double A2, A3, A4, E4, E6;

  if(!cm_set)
    {
      /*determine central meridian when a valid longitude is specified*/
      if(fabs(longitude) > 0.0 && fabs(longitude) <= 180.0) {
	setCentralMeridian(latitude, longitude);
      }
    }

  phe = DEG_TO_RAD(latitude);
  lambda = DEG_TO_RAD(longitude);

  s_phe = sin(phe) ;
  c_phe = cos(phe) ;
  t_phe = tan(phe) ;

  ep2	= GEO_E2/(1.0 - GEO_E2) ;
  N	= GEO_SMALL_A/sqrt(1.0 - GEO_E2*SQR(s_phe)) ;
  T	= SQR(t_phe);
  C	= ep2*SQR(c_phe);
  A	= c_phe*(lambda - lambda0) ;
  A2    = SQR(A);
  A3    = A*A2;
  A4    = SQR(A2);
  E4     = SQR(GEO_E2);
  E6     = E4*GEO_E2;
  M1	= 1.0 - GEO_E2/4.0 - 3.0*E4/64.0 - 5.0*E6/256.0 ;
  M2	= 3.0*GEO_E2/8.0 + 3.0*E4/32.0 + 45.0*E6/1024.0 ;
  M3	= 15.0*E4/256.0 + 45.0*E6/1024.0 ;
  M4	= 35.0*E6/3072.0 ;
  M	= GEO_SMALL_A*( M1*phe - M2*sin(2.0*phe) + M3*sin(4.0*phe) -
	    M4*sin(6.0*phe) ) ;

  X1	= (1.0 - T + C)*(A3/6.0) ;
  X2	= (5.0 - 18.0*T + SQR(T) + 72.0*C - 58.0*ep2)*(A4*A/120.0) ;
  Y1	= (5.0 - T + 9.0*C + 4.0*SQR(C))*(A4/24.0) ;
  Y2	= (61.0 - 58.0*T + SQR(T) + 600.0*C - 330.0*ep2)*(A4*A2/720.0) ;

  X	= GEO_SMALL_K0*N*(A + X1 + X2) ;
  Y	= GEO_SMALL_K0*( M + N*t_phe*(A2/2.0 + Y1 + Y2) ) ;

  *easting  = (X + GEO_E0) ;
  *northing = Y ;
  if (phe < 0.0)
    *northing += GEO_N0 ;
}

/*******************************************************************
 * Convert UTM coordinates (easting,northing) to geodetic coord-     
 * inates (latitude,longitude).   easting and northing are passed in 
 * as meters.  latitude and longitude are passed out in degrees.     
 ********************************************************************/

void convertUTMtoGEO (METERS easting, METERS northing, 
		      DEGREES *latitude, DEGREES *longitude)
{
  double N1, R1, M, D, T1, C1, ep2, e1, mu, mu1, x, y;
  double E4, E6, tmp, e1sq, C1sq, D2, D4;
  double phe1, s_phe1, c_phe1, t_phe1, phe2, phe3, phe4, phe11, phe12, phe13 ;
  double lambda1, lambda2 ;

//  if(cm_set == FALSE) a_log(A_LOG_WARNING, "geoConvert: Central Meridian not "
//			    "set before UTMtoGEO conversion, using default\n");

  x = easting - GEO_E0 ;
  y = northing ;
  if (initialLat < 0)
    y -= GEO_N0 ;

  E4      = SQR(GEO_E2);
  E6      = E4 * GEO_E2;
  M	  = y/GEO_SMALL_K0 ;
  mu1	  = 1.0 - GEO_E2/4.0 - 3.0*E4/64.0 - 5.0*E6/256.0 ;
  mu	  = M/( GEO_SMALL_A*mu1 ) ;
  tmp     = sqrt(1.0 - GEO_E2);
  e1	  = ( 1.0 - tmp )/( 1.0 + tmp ) ;
  e1sq    = SQR(e1);
  phe11   = 3.0*e1/2.0 - 27.0*(e1*e1sq)/32 ;
  phe12   = 21.0*e1sq/16.0 - 55*SQR(e1sq)/32.0 ;
  phe13   = 151.0*(e1*e1sq)/96.0 ;
  phe1	  = mu + phe11*sin(2.0*mu) + phe12*sin(4.0*mu) + phe13*sin(6.0*mu) ;
  s_phe1  = sin(phe1) ;
  c_phe1  = cos(phe1) ;
  t_phe1  = tan(phe1) ;

  ep2	  = GEO_E2/(1.0 - GEO_E2) ;
  C1	  = ep2*SQR(c_phe1);
  C1sq    = SQR(C1);
  T1	  = SQR(t_phe1) ;
  N1	  = GEO_SMALL_A/sqrt( 1.0-GEO_E2*SQR(s_phe1)) ;
  R1	  = GEO_SMALL_A*(1.0 - GEO_E2)/pow( (1.0-GEO_E2*SQR(s_phe1)),1.5 ) ;
  D	  = x/(N1*GEO_SMALL_K0) ;
  D2      = SQR(D);
  D4      = SQR(D2);

  phe2    = ( N1*t_phe1 )/R1 ;
  phe3	  = ( (5.0 + 3.0*T1 + 10.0*C1 - 4.0*C1sq -
	       9.0*ep2)*D4 )/24.0 ;
  phe4	  = ( (61.0 + 90.0*T1 + 298.0*C1 + 45.0*SQR(T1) -
	       252.0*ep2 - 3.0*C1sq)*D4*D2)/720.0 ;

  lambda1 = ( (1.0 + 2.0*T1 + C1)*D2*D )/6.0 ;
  lambda2 = ( (5.0 - 2.0*C1 + 28.0*T1 - 3.0*C1sq + 8.0*ep2 +
	       24.0*SQR(T1))*D4*D )/120.0 ;

  *latitude  = RAD_TO_DEG( phe1 - phe2*(D*D/2 - phe3 + phe4) ) ;
  *longitude = RAD_TO_DEG(lambda0 + (D - lambda1 + lambda2)/c_phe1 ) ;
}


/*****************************************************************
 * Change the central meridian, used in the GEO to UTM conversions
 *****************************************************************/
void setCentralMeridian(DEGREES latitude, DEGREES longitude)
{
  centralMeridian = get_central_meridian(longitude);
  lambda0 = DEG_TO_RAD((double)centralMeridian);
  initialLat = (int)latitude;
  cm_set = TRUE;
//  a_log(A_LOG_NOTICE, "geoConvert: Central meridian set to %d, initial "
//	"latitude is %d\n", centralMeridian, initialLat);
}


/*****************************************************************
 * convert (0 to 2PI, north=0 rad) angles to (-PI to PI, east=0
 * rad) angles
 *****************************************************************/
RADIANS compassToCartesianHeading(RADIANS heading)
{
  if((heading >= 0.0) && (heading <= 3.0*M_PI/2.0))
    return((M_PI/2.0) - heading);
  else
    return((5.0*M_PI/2.0) - heading);
}



/*****************************************************************
 * convert (-PI to PI, east=0 rad) angles to (0 to 2PI, north=0
 * rad) angles
 *****************************************************************/
RADIANS cartesianToCompassHeading(RADIANS heading)
{
  /*bound heading between PI and PI   */
  while(heading > M_PI)
    heading = heading - 2.0 * M_PI;
  while(heading < -M_PI)
    heading = heading + 2.0 * M_PI;
 
  if((heading <= M_PI/2.0) && (heading >= -M_PI))
    return((M_PI/2.0) - heading);
  else
    return((5.0*M_PI/2.0) - heading);
}


void print_matrix(
	const char *		name,
	const void *		M_p,
	index_t			n,
	index_t			m
    )
{
	const double *		M = (double *)M_p;
	index_t			i;
	index_t			j;

	fprintf(stderr, "%s=\n", name );
	for( i=0 ; i<n ; i++ )
	{
		for( j=0 ; j<m ;j++ )
			fprintf(stderr, "% 3.8f ", M[i*m + j] );
		fprintf(stderr, "\n" );
	}
}



void ecef2llh(
        double x, 
        double y, 
        double z, 
        double *lat, 
        double *lon, 
        double *height
)
{
    double a = 6378137.0;
    double b = 6356752.3142;
    double f = (a-b)/a;
    double e = sqrt(2*f - f*f);
    double h = 0.0;
    int hold = 100;
    double N = a;
    double sinphi;
    double phi;
    double hN;
    
    (*lon) = atan2(y,x)*180/M_PI;
    do{
        hold = (int)h;
        sinphi = z/(N*(1-(e*e)) +h);
        phi = atan((z+(e*e)*N*sinphi)/(sqrt(x*x+y*y)));
        N = a/sqrt(1-(e*e)*sin(phi)*sin(phi));
        hN = sqrt(x*x + y*y)/cos(phi);
        h = (hN - N);
        (*height) = h;
    }while(hold !=(int)h);
    (*lat) = phi*180/M_PI;
}

void llh2ecef(
        double lat,
        double lon,
        double height,
        double ecef[3]
)
{
    double a = 6378137.0;
    double b = 6356752.3142;
    double f = (a-b)/a;
    double e = sqrt(2*f - f*f);
    double N = a/sqrt(1 - (e*e)*sin(lat*M_PI/180.0)*sin(lat*M_PI/180.0));
    ecef[2] = (N*(1-(e*e))+height)*sin(lat*M_PI/180.0);
    ecef[0] = (N+height)*cos(lat*M_PI/180.0)*cos(lon*M_PI/180.0);
    ecef[1] = (N+height)*cos(lat*M_PI/180.0)*sin(lon*M_PI/180.0);
}

void ecef2tangent(
        double latitude, 
        double longitude, 
        double rot[3][3]
)
{
    double slat = sin(latitude*M_PI/180.0);
    double slon = sin(longitude*M_PI/180.0);
    double clat = cos(latitude*M_PI/180.0);
    double clon = cos(longitude*M_PI/180.0);

    rot[0][0] = -slat*clon;
    rot[0][1] = -slat*slon;
    rot[0][2] =  clat;

    rot[1][0] = -slon;
    rot[1][1] =  clon;
    rot[1][2] =  0.0;

    rot[2][0] = -clat*clon;
    rot[2][1] = -clat*slon;
    rot[2][2] = -slat;
}

void nedpts(
        double llh1[3], 
        double llh2[3], 
        double ned[3],
        double lat,
        double lon
)
{
    double ecef1[3], ecef2[3],ecef[3];
    double C[3][3];
    llh2ecef(llh1[0],llh1[1],llh1[2], ecef1);
    llh2ecef(llh2[0],llh2[1],llh2[2], ecef2);
    ecef[0] = ecef1[0] - ecef2[0];
    ecef[1] = ecef1[1] - ecef2[1];
    ecef[2] = ecef1[2] - ecef2[2];
    ecef2tangent(lat,lon,C);

    ned[0] = C[0][0]*ecef[0] + C[0][1]*ecef[1] + C[0][2]*ecef[2];
    ned[1] = C[1][0]*ecef[0] + C[1][1]*ecef[1] + C[1][2]*ecef[2];
    ned[2] = C[2][0]*ecef[0] + C[2][1]*ecef[1] + C[2][2]*ecef[2];
}
    
double heading_tan(double y, double x)
{
	double	a;

	if (x < 0.0) {
		a = M_PI - atan(y/x);
	} else if ((x > 0.0) && (y < 0.0)) {
		a = -atan(y/x);
	} else if ((x > 0.0) && (y > 0.0)) {
		a = 2*M_PI - atan(y/x);
	} else if ((x == 0.0) && (y < 0.0)) {
		a = M_PI/2.0;
	} else if ((x == 0.0) && (y > 0.0)) {
		a = 1.5*M_PI;
	} else {
		a = 0.0;
	}

	if (a > M_PI)
		a -= 2*M_PI;

	return a;
}


static inline int8_t is_zero(const double *f, int verbose)
{
	const unsigned long long int *x = (const unsigned long long int *) f;
    
	if( *x == 0 ){
        if( verbose ==1 && (*f) > 0.0){
            fprintf(stderr, "f%f\n", *f);
        }
	    return 1;
    }
	return 0;
}

/*
void accel2euler(
	double       *      THETAm,
	const double *		accel,
	const double *      field,
    const double *      quat
)
{
    double x,y,yaw;	
    double euler[3];
    static double smoothed_yaw;
    int L =2 ;
    double S = 0.001, D, G;
    static int init = 1;
    double mag;
    mag = sqrt(field[0]*field[0] + field[1]*field[1] + field[2]*field[2]);
    quat2euler(quat, euler);
    
    //THETAm[1] = -atan2(accel[1], -accel[2]);
    //THETAm[0] = -asin(accel[0]/-(sqrt((accel[0]*accel[0]) 
    //                + (accel[1]*accel[1]) + (accel[2]*accel[2]))));
    
    THETAm[1] = -atan2(accel[1], sqrt(accel[1]*accel[1] + accel[2]*accel[2]));
    THETAm[0] = -asin(accel[0]/-sqrt(accel[0]*accel[0] + accel[2]*accel[2]));
    
    //x = (field[0]*cos(euler[1]) +field[1]*sin(euler[1])*sin(euler[0])-field[2]*cos(euler[0])*sin(euler[1]));
    //y = (field[1]*cos(euler[0]) + field[2]*sin(euler[0]));
    
    x = (field[0]*cos(THETAm[0]) +field[1]*sin(THETAm[1])*sin(THETAm[0])-field[2]*cos(THETAm[1])*sin(THETAm[0]));
    y = (field[1]*cos(THETAm[1]) + field[2]*sin(THETAm[1]));
    yaw = heading_tan(y, x) - ((11.17*M_PI)/180.0);
    //yaw = heading_tan(field[1], field[0]) - ((11.17*M_PI)/180.0);
    if (init){
        smoothed_yaw = yaw;
        init = 0;
    }
    
    D = yaw - smoothed_yaw;
    G = S + S*(D/L)*(D/L);
    smoothed_yaw += D*G;
    //THETAm[2] = yaw;
    THETAm[2] = smoothed_yaw;
    
    //fprintf(stderr, "roll %g pitch %g ", THETAm[0]*180/M_PI, THETAm[1]*180/M_PI);
    //fprintf(stderr, "a %g b %g c %g ",field[0]*cos(THETAm[0]),field[1]*sin(THETAm[1])*sin(THETAm[0]), field[2]*cos(THETAm[1])*sin(THETAm[0]));
    //fprintf(stderr, " fx %g fy %g fz %g x %g, y %g\r", field[0], field[1], field[2], x, y);
    
    //fprintf(stderr, " yaw %g smoothed %g\r\n", yaw*180.0/M_PI, smoothed_yaw*180/M_PI);
    //fflush(stdout);
}
*/

void accel2euler(
	double *			THETAm,
	const double *		accel,
    const double *      quat
)
{
    	double yaw;	
    	double euler[3];
    	quat2euler(quat, euler);
	//THETAm[0] = -atan2(accel[1], -accel[2]);
	//THETAm[1] = -asin(accel[0]/(-sqrt((accel[0]*accel[0]) 
        //            + (accel[1]*accel[1]) + (accel[2]*accel[2]))));
        // changed by sri
	THETAm[0] = -atan2(accel[1], -accel[2]);
	THETAm[1] = -asin(accel[0]/accel[2]);
    	yaw = euler[2];
	THETAm[2] = yaw;
}

/*
 * Perform the matrix multiplication A[n,m] * B[m,p], storing the
 * result in OUT[n,p].
 *
 * If transpose_B is set, B is assumed to be a [p,m] matrix that is
 * tranversed in column major order instead of row major.  This
 * has the effect of transposing B during the computation.
 *
 * If add == 0, OUT  = A * B.
 * If add >  0, OUT += A * B.
 * If add <  0, OUT -= A * B.
 */

void mulNxM(
	void *			OUT_ptr,
	const void *		A_ptr,
	const void *		B_ptr,
	index_t			n,
	index_t			m,
	index_t			p,
	int8_t			transpose_B,
	int8_t			add,
    int verbose
)
{
	index_t			i;
	index_t			j;
	index_t			k;

	double *OUT = (double *)OUT_ptr;
	const double *		A = (double *)A_ptr;
	const double *		B = (double *)B_ptr;
	for( i=0 ; i<n ; i++ )
	{
		const double *		A_i = A + i * m;
		double *			O_i = OUT + i * p;

		for( j=0 ; j<p ; j++ )
		{
			double			s = 0;
			double *			O_i_j = O_i + j;

			for( k=0 ; k<m ; k++ )
			{
				const double *		a = A_i + k;
				const double *		b;
				if( is_zero(a, verbose ) ){
					continue;
                }
				if( transpose_B )
					b = B + j * m + k;
				else
					b = B + k * p + j;

				if( is_zero(b, verbose ) ){
					continue;
                }
				s += *a * *b;
			}


			if( add == 0 ){
				*O_i_j = s;
            }
			else
			if( add > 0 )
				*O_i_j += s;
			else
				*O_i_j -= s;
		}
	}
}

void inv33(
    double A[3][3],
    double B[3][3]
)
{
     double det;

    /* Check for NULL inputs */
    if ((A == NULL) || (B == NULL)){
        return;
    }
    /* Check for non-distinct output */
    if (A == B){
	    return;
    }
    /* Compute the determinant */
    det = A[0][0] * (A[1][1] * A[2][2] - A[2][1] * A[1][2])
	- A[1][0] * (A[0][1] * A[2][2] - A[2][1] * A[0][2])
	+ A[2][0] * (A[0][1] * A[1][2] - A[1][1] * A[0][2]);
    if((det < 0.00001) && (det > -0.00001)){
        //fprintf(stderr, " small det\n");
        if(det > 0.0)
            det = 0.00001;
        if(det < 0.0)
            det = -0.00001;
        
    }
    if(is_zero(&det, 0))
        return;

    B[0][0] =  (A[1][1] * A[2][2] - A[2][1] * A[1][2]) / det;
    B[0][1] = -(A[0][1] * A[2][2] - A[2][1] * A[0][2]) / det;
    B[0][2] =  (A[0][1] * A[1][2] - A[1][1] * A[0][2]) / det;

    B[1][0] = -(A[1][0] * A[2][2] - A[2][0] * A[1][2]) / det;
    B[1][1] =  (A[0][0] * A[2][2] - A[2][0] * A[0][2]) / det;
    B[1][2] = -(A[0][0] * A[1][2] - A[1][0] * A[0][2]) / det;

    B[2][0] =  (A[1][0] * A[2][1] - A[2][0] * A[1][1]) / det;
    B[2][1] = -(A[0][0] * A[2][1] - A[2][0] * A[0][1]) / det;
    B[2][2] =  (A[0][0] * A[1][1] - A[1][0] * A[0][1]) / det;

}


int dpsi_dq(double *q, double *derivative)
{
    const double q0 = q[0];
    const double q1 = q[1];
    const double q2 = q[2];
    const double q3 = q[3];
    const double t1 = 1 - 2 * (q2*q2 + q3*q3);
    const double t2 = 2 * (q1*q2 + q0*q3);
    double err;
    if(fabs(t1*t1 + t2*t2) < 0.001)
        err = 10000.0;
    else
        err = 2 / (t1*t1 + t2*t2);

    derivative[0] = err * (q3 * t1);
    derivative[1] = err * (q2 * t1);
    derivative[2] = err * (q1 * t1 + 2 * q2 * t2);
    derivative[3] = err * (q0 * t1 + 2 * q3 * t2);

    return 0;
}

int dphi_dq(double *q, double *derivative)
{
    const double q0 = q[0];
    const double q1 = q[1];
    const double q2 = q[2];
    const double q3 = q[3];
    const double t1 = 1 - 2 * (q1*q1 + q2*q2);
    const double t2 = 2 * (q3*q2 + q0*q1);
    double err;
    if(fabs(t1*t1 + t2*t2) < 0.001)
        err = 10000.0;
        else
        err = 2 / (t1*t1 + t2*t2);

    derivative[0] = err * (q1 * t1);
    derivative[1] = err * (q0 * t1 + 2 * q1 * t2);
    derivative[2] = err * (q3 * t1 + 2 * q2 * t2);
    derivative[3] = err * (q2 * t1);

    return 0;
}


int dtheta_dq(double *q, double *derivative)
{
    const double q0 = q[0];
    const double q1 = q[1];
    const double q2 = q[2];
    const double q3 = q[3];
    const double t1 = 2 * (q1*q3 - q0*q2);
    double err;
    
    err =  -1/sqrt(1 - (t1*t1));

    derivative[0] = -2*q2*err;
    derivative[1] = 2*q3*err;
    derivative[2] = -2*q0*err;
    derivative[3] = 2*q1*err;

    return 0;
}


