// Navigation Code...
//------------------------------------------------------------------------------
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>


// #include "Constants.h"
#include "Utm.h"
#include "qboatVars.h"
#include "do_share.h"

#ifndef PI
	#define PI 3.1415926535
#endif
#define TO_RAD PI/180.0
#define k0  0.9996

extern SharedASVData	*shared;

NAV_params navparams;
extern NavSensorData navsensor;

extern double lat,lon;

typedef struct
{
	double m[4][4];
	int row,col;
}matrix;

//extern void utm(double,double,double,double);
extern void LLtoUTM(int, double, double, double*, double*, char *);
void disp_matrix(matrix m);
// void utm();

extern void WayPtGuidance();

matrix R, T, vel, dvl, pos,DT;
// matrix rm,rm1,m1,m2,vel,dvl,pos,DT,arate,rate;
extern FILE *mz_data; extern int log_data;
//------------------------------------------------------------------------------
void arot_matrix()
{
	float sy,cy,sp,cp,tp,sr,cr; // Assuming sin,cos,tan use radians.
	float local_epsi=0.00000001;  // a v.small num. to use in place of zero

	sy=sin(shared->SensorData.yaw * TO_RAD);
	cy=cos(shared->SensorData.yaw * TO_RAD);
   	sp=sin(shared->SensorData.pitch * TO_RAD); 	// navsensor.ahrs_data.Pitch * TO_RAD);
   	cp=cos(shared->SensorData.pitch * TO_RAD); 	// navsensor.ahrs_data.Pitch * TO_RAD);
	if(cp>local_epsi) tp=sp/cp; else tp=sp/local_epsi;
	sr=sin(shared->SensorData.roll * TO_RAD); 	// navsensor.ahrs_data.Roll * TO_RAD);
   	cr=cos(shared->SensorData.roll * TO_RAD); 	// navsensor.ahrs_data.Roll * TO_RAD);

   	R.m[0][0]=cy*cp;
   	R.m[0][1]=cy*sp*sr-sy*cr;
   	R.m[0][2]=cy*sp*cr+sy*sr;
   	R.m[1][0]=sy*cp;
   	R.m[1][1]=sy*sp*sr+cy*cr;
   	R.m[1][2]=sy*sp*cr-cy*sr;
   	R.m[2][0]=-sp;
   	R.m[2][1]=cp*sr;
   	R.m[2][2]=cp*cr;

   	T.m[0][0]=1;
   	T.m[0][1]=sr*tp;
   	T.m[0][2]=cr*tp;
   	T.m[1][0]=0;
   	T.m[1][1]=cr;
   	T.m[1][2]=-sr;
   	T.m[2][0]=0;
  	T.m[2][1]=sr/cp;
   	T.m[2][2]=cr/cp;

   	R.row=3;
   	R.col=3;

   	T.row=3;
   	T.col=3;
}
//------------------------------------------------------------------------------
matrix matrix_multiply(matrix m1,matrix m2)
{ 	int row,col,i;
   	matrix r;

	for(row=0;row<m1.row;row++)
	{
   		for(col=0;col<m2.col;col++)
		{
   	 		r.m[row][col]=0.0;
   			for(i=0;i<m1.col;i++)
			{
         	 		r.m[row][col]+= m1.m[row][i] * m2.m[i][col];
   			}
  		 }
	}
   	r.row=m1.row;
   	r.col=m2.col;
   return r;
}
//------------------------------------------------------------------------------
matrix matrix_add(matrix m1,matrix m2)
{ 	int row,col ;
   	matrix r;

   for(row=0;row<m1.row;row++)
   	for(col=0;col<m2.col;col++)
         r.m[row][col]= m1.m[row][col] + m2.m[row][col];
   r.row=m1.row;
   r.col=m2.col;
   return r;
}
 //------------------------------------------------------------------------------
matrix matrix_substract(matrix m1,matrix m2)
{ 	int row,col ;
   	matrix r;

   for(row=0;row<m1.row;row++){
   	for(col=0;col<m2.col;col++)
         r.m[row][col]= m1.m[row][col] - m2.m[row][col];
      }
   r.row=m1.row;
   r.col=m2.col;
   return r;
}
//------------------------------------------------------------------------------
matrix matrix_transpose(matrix m)
{ 	int row,col;
   matrix r;

   for(row=0;row<m.row;row++){
   	for(col=0;col<m.col;col++)
         r.m[col][row]= r.m[row][col];
   }
   r.row=m.col;
   r.col=m.row;
   return r;
}
void disp_matrix(matrix m)
{
	int i, j;
	for(i=0;i< m.row;i++)
	{
   		for( j=0;j<m.col;j++)
     			printf("%5.3f ",m.m[i][j]);
      		printf("\n");
   	}
   	
}
//------------------------------------------------------------------------------
void store_matrix(matrix m)
{
	int i, j;
	for(i=0;i< m.row;i++)
	{
   		for( j=0;j<m.col;j++)
     		{	if(log_data && shared->ASV_shared.log_data) fprintf(mz_data,"%5.3f",m.m[i][j]); 
			if(j<(m.col-1) && log_data) fprintf(mz_data,","); }
      		if(log_data) fprintf(mz_data," ; ");
   	}
}
//------------------------------------------------------------------------------
void init_matrices()
{	int i,j;
	dvl.row=3; dvl.col=1;
	vel.row=3; vel.col=1;
	DT.row=3; DT.col=3;
	pos.row=3; pos.col=1;
	for(i=0;i<3;i++)
   	   for(j=0;j<3;j++)
     		DT.m[i][j]=0;
}

void update_matrices(int dt)
{
  int i;
  for(i=0;i<3;i++)
  {
  	dvl.m[i][0]=navsensor.dvl_data.watervel[i];
  	DT.m[i][i]=dt;
  }
}

void dead_reckon()
{
   arot_matrix();
   vel= matrix_multiply(R ,dvl); // Convert velocity (body to world)
   if(log_data && shared->ASV_shared.log_data) fprintf(mz_data,"\n$NAVVEL:"); store_matrix(vel);
   //  printf("rate transform\n");
   // rate= matrix_multiply(rm1,arate);
   pos =matrix_add(pos, matrix_multiply(DT,vel));
   navparams.Dead_X=pos.m[0][0]; navparams.Dead_Y=pos.m[1][0]; navparams.Dead_Z=pos.m[2][0];
   navparams.X=pos.m[0][0]; navparams.Y=pos.m[1][0];
//  pos.m[2]=z // directly from depth sensor
}

void do_nav()
{
	static int counter;
	/*	To be done in the near-future:
		1) Guidance using only GPS (UTM-based)
		2) Guidance DVL-aided (Dead-reckoning)
		3) Kalman Filter for position estimation.
		4) Kalman Filter with bias estimation.	*/
	double Lat,Lon;	char Zone[20]; 
	
	dead_reckon();
	// On the ASV force all Z-coordinates to zero!!! We aren't planning to dive.
	navparams.Z=0; navparams.Dead_Z=0;
	
	Lat=navsensor.gps_data.lat_deg+navsensor.gps_data.lat_min/60.0;
	Lon=navsensor.gps_data.lon_deg+navsensor.gps_data.lon_min/60.0;
	Lat=(navsensor.gps_data.lat_dir=='N')?Lat:-Lat;
	Lon=(navsensor.gps_data.lon_dir=='E')?Lon:-Lon;
	counter++;
	
	if(navsensor.gps_data.GPS_status==GPS_FIX_YES) // && shared->FLAGS_shared.GPS_pause==USE_GPS) 
   	{
		// Remember to allow Datum configuration via GUI.	
		LLtoUTM(23,Lat,Lon,&UTMN,&UTME,Zone);
		shared->ASV_shared.NAV_status=USE_GPS;
		navparams.UTME=UTME; navparams.UTMN=UTMN;
		if(counter%10==0)
		   if(log_data && shared->ASV_shared.log_data) fprintf(mz_data,"\n$NAVUTM,%13.2f,%13.2f",UTME,UTMN);
		// Get actual X,Y by subtracting UTM converted from this.
		pos.m[0][0]=navparams.UTME-navparams.UTME_ref; navparams.X=pos.m[0][0];
		pos.m[1][0]=navparams.UTMN-navparams.UTMN_ref; navparams.Y=pos.m[1][0];
		if(counter%10==0)
		   if(log_data && shared->ASV_shared.log_data) 
		fprintf(mz_data,"\n$NAVGPS,%7.2f,%7.2f,%7.2f",navparams.X,navparams.Y,navparams.Z);
	}
   	if(counter%10==0) if(log_data && shared->ASV_shared.log_data) 
	fprintf(mz_data,"\n$NAVDREC,%7.2f,%7.2f,%7.2f",navparams.Dead_X,navparams.Dead_Y,navparams.Dead_Z);
}
//------------------------------------------------------------------------------
