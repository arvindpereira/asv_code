#include <stdio.h>
#include <stdio.h>
#include <math.h>
#include "do_share.h"
#include <sys/time.h>
#include <time.h>
#include "pid.h"

extern Q_SHARED *shared;
extern FILE * logFile;
extern PID *speedPID;

#define RAD_2_DEG	180.0/M_PI
#define DEG_2_RAD	M_PI/180.0

extern struct timeval tv;

void WayPtGuidance()
{
	// Calculate the desired heading using present position
	double x_err, y_err, z_err, rho_squared, rho, vel; 
	float eps=0.0000001,heading;
	static int counter;

//	printf("\nWaypoint guidance...");
	if(shared->navData.des_speed == 0 && shared->qboatData.WayPtAchv!=WAYPT_ACHV) 
		shared->navData.des_speed= 70;
	

	x_err= shared->navData.UTME_ref - shared->navData.UTME;	// auv_pid_data.PID_inputs.DesPos.X-navparams.X;
	y_err= shared->navData.UTMN_ref - shared->navData.UTMN;	//auv_pid_data.PID_inputs.DesPos.Y-navparams.Y;

	if(fabs(y_err)<eps) y_err=eps; // Divide-by-zero prevention	
	heading=atan2(x_err,y_err) * RAD_2_DEG;
//	if(heading<0) heading=360-fabs(heading);
	// Added 07/12/2007 to take des_heading (0,360)
	if(heading<0) heading=heading+360;	
	shared->navData.des_heading=heading;

	rho_squared=x_err*x_err+y_err*y_err;
	rho=sqrt(rho_squared);
	counter++;
	gettimeofday(&tv, NULL);
	fprintf(logFile,"\n$GUIDDATA,%ld.%06ld,%lf,%lf,%lf,%lf,%f,%f,%f,%d",tv.tv_sec,tv.tv_usec,
		shared->navData.UTME_ref,shared->navData.UTMN_ref, shared->navData.UTME, shared->navData.UTMN,			
		heading,rho,shared->navData.des_speed,shared->qboatData.WayPtAchv);
	fprintf(stderr,"\nGUID,%f,%f,%f",heading,rho,shared->navData.des_speed);	
	if(rho<shared->navData.AcceptRad)
	{  
		shared->qboatData.WayPtAchv= WAYPT_ACHV;
		shared->navData.des_speed = -5;
	//	if(log_data && shared->ASV_shared.log_data) 
		fprintf(logFile,"\n$GUIDWPT,%f,WPAchv",shared->navData.des_heading);
	}
	
	if(counter%10==0 && !shared->qboatData.WayPtAchv)
	{
	    fprintf(logFile,"\n$GUIDWPT,%f,WPNotAchv",shared->navData.des_heading);
	}
}



void DoCarrotStickGuidance()
{
	// A slightly different approach to the entire idea of guidance.
	// Here we attempt to stay on the path which we have planned.
	// We use a PID controller on the cross track error to dangle a 
	// way-point command to the boat which attempts to compensate for the
	// difference the present location of the boat has from the original planned path.

	double pX,pY;
	double aX,aY,bX,bY;
	double m, c;
	double errX,errY;
	double theta,thetaDegs;	
	// Calculate angle between desired path and our path
	double magPA,magAB,magError,dotProdAB_PA;
	// Gains...
	double kp= 0.5;		// correct the angle by this gain...
	double noGo = 0.5;	// do not bother with drifts smaller than this..	
	// Output...
	double output;

	// Update our local variables with global information
	pX = shared->navData.UTME;
	pY = shared->navData.UTMN;
	aX = shared->navData.lastUTME;
	aY = shared->navData.lastUTMN;
	bX = shared->navData.UTME_ref;
	bY = shared->navData.UTMN_ref;	

	magPA = sqrt((pX-aX)*(pX-aX) + (pY-aY)*(pY-aY));
	magAB = sqrt((bX-aX)*(bX-aX) + (bY-aY)*(bY-aY));

	dotProdAB_PA = (bX-aX)*(pX-aX) + (bY-aY)*(pY-aY);

	// This is what screwed up the controller!!! // theta = acos( ((pX - aX) * ( bX - aX ) + (pY - bY) * ( bY - aY))/(magPA*magAB));
	theta = acos(dotProdAB_PA/(magPA*magAB));

	magError = magPA * sin(theta);
	if(magError>noGo) 
	{ 

	// Now we want to use this error to produce either a steering or an error correction for the output of the  LOS controller.
	// The control law I am aiming for will attempt to change the angle based on how far away we are from the desired location.
	// A simple rule that I am going to use at present to not drive the system unstable due to GPS errors is to make changes 
	// where the magError is larger than a certain amount.
	
	// I am assuming that P control may be sufficient and so am not going to declare a PID controller class here.
	// If we see that it is not producing good results, I will change this behavior.
	// The output angle will NOW* incorporate the change in heading that is desired...
	thetaDegs = theta * 180.0/3.1415926535;
	if(thetaDegs > 90.0)
		thetaDegs = -thetaDegs + 90.0;

	output = -thetaDegs * kp ; // I think this is already proportional to magError...  * magError;
	gettimeofday(&tv, NULL);
	   if(output>30) output=30;
	   if(output<-30) output=-30;
	   if(fabs(thetaDegs)<=90) {
	//	shared->navData.des_heading = shared->navData.des_heading + output;
		fprintf(logFile,"\n$LINEFLW,%ld.%06ld,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
			tv.tv_sec,tv.tv_usec,noGo,pX,pY,aX,aY,bX,bY,magError,magPA,thetaDegs,output,shared->navData.des_heading);
	   }
	}
	
}



void stationKeep(int init)
{
	// This code should put the boat into a mode where it attempts to station-keep based upon Kalman-filter position data.
	//
	// double des_X,des_Y = shared->navData.StationN;
	double x_err, y_err, rho;
	double heading, eps = 0.0001;
	double SpeedKp = 15.0, SpeedKd = -5.0;
	double SpeedPIDout ;
	int direction = 1;
	const double MAX_SPEED = 40, MAX_REV_SPEED = -40;

/*	if(init==0) {
		des_X = shared->kalmanData.Xpos;
		des_Y = shared->kalmanData.Ypos;
	}
	else {
*/
		// We are going to adjust the speed of the vehicle such that it turns around in the current location...
	
		x_err = shared->navData.UTME_ref - shared->kalmanData.Xpos; // shared->navData.UTME; // auv_pid_data.PID_inputs.DesPos.X-navparams.X;
        	y_err = shared->navData.UTMN_ref - shared->kalmanData.Ypos; // shared->navData.UTMN; //auv_pid_data.PID_inputs.DesPos.Y-navparams.Y;

        	if(fabs(y_err)<eps) y_err=eps; // Divide-by-zero prevention
        	 heading=atan2(x_err,y_err) * RAD_2_DEG;

		direction = 1;
		
		// We need to check if our direction needs to be reversed...
		// We do this only if we are close to the goal... If we are far away, we make the boat turn around and return
	   rho = sqrt ( x_err * x_err + y_err * y_err );
	   if( rho < 2 ) {
			if(fabs(heading * RAD_2_DEG )>=90) {
			  heading = atan( x_err / y_err ) * RAD_2_DEG;
			  direction = -1;
			}
			if(rho<1) { heading = shared->SensorData.cYaw; }
	    }
/*	    if( rho < 0.1 ) {
		shared->navData.StationE = shared->navData.UTME; shared->navData.StationN = shared->navData.UTMN; 
	    } */  
	    // Now we need to set these as the heading values and the speed should be proportional to the distance...	
	    SpeedPIDout = speedPID->doPID(0, rho * direction, shared->kalmanData.Xvel, 0 );	// Speed PID...
	    //SpeedPIDout = SpeedKp * direction * rho ;
	    //if( SpeedPIDout > MAX_SPEED ) SpeedPIDout = MAX_SPEED;
	    //if( SpeedPIDout < MAX_REV_SPEED ) SpeedPIDout = MAX_REV_SPEED;
	  
	    if(heading < 0) heading = heading + 360;
	    shared->navData.des_heading = heading ;
	    shared->navData.des_speed = SpeedPIDout;
	    gettimeofday(&tv,NULL);	    
	    fprintf(logFile,"\n$STATIONKP,%ld.%06ld,%f,%f,%f,%f,%f,%f,%f,%f,%f",tv.tv_sec,tv.tv_usec,
				shared->navData.UTME_ref,shared->navData.UTMN_ref,
				shared->kalmanData.Xpos, shared->kalmanData.Ypos,
				x_err, y_err, rho, shared->navData.des_speed, shared->navData.des_heading);	
//	}
}
