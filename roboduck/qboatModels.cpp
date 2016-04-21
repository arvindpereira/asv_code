/*
 *	This file contains functions that contain Model-based or empirically collected 
 *	data from the boat which can be used to convert from non-linear to semi-linear etc.
 *	spaces.
 *
 * 	Model based on data gathered at Prof.Sukhatme's swimming pool.
 */
#include <stdio.h>
#include "qboatModels.h"

extern FILE *logFile;
int FwdThrusts = 1, RevThrusts = 0;

float getInterpValue(int arrayDesc,float myVal, int x, int f_o_x,int lim) {
	// Looks up expt data and returns a linear interpolated value corresponding to it.
	// arr 		-  array which needs to be searched in.
	// myVal 	-  value to be interpolated.
	// x		-  column in which myVal type values exist.
	// f_o_x	-  column in which we need the interpolated function value.
	// 
	// Note : The values are in increasing order toward zero along the y-axis.
	// 	Ensure that this is the case with all measurements.
	// 	
	float thisVal, prevVal;
	float interpVal, x1, x2, y1, y2;

	for (int i = 0; i< lim-1 ; i++) {
			x1 = (arrayDesc==FwdThrusts)? FwdThrMap[i][x] : RevThrMap[i][x]; 
			x2 = (arrayDesc==FwdThrusts)? FwdThrMap[i+1][x] : RevThrMap[i+1][x];
			y1 = (arrayDesc==FwdThrusts)? FwdThrMap[i][f_o_x] : RevThrMap[i][f_o_x];
			y2 = (arrayDesc==FwdThrusts)? FwdThrMap[i+1][f_o_x] : RevThrMap[i+1][f_o_x];

			if(i==0 && myVal > x1 ) return y1;	// reqd Thrust exceeds tabulated value.

		if (x1>= myVal && x2 <= myVal) {
			interpVal =(myVal - x2)*(y2-y1)/(x2-x1) + y2;
			return interpVal;
		} 			
	}
	return 0;
}

int applyThrustMapping(float reqdLeftThrust, float reqdRightThrust, int *ThrCmdLeft, int *ThrCmdRight) {
	// We want to be able to apply thrusts on both thrusters without saturating them...
	// (Possible Auto-gain tuning method???)

	float left, right;
	if(reqdLeftThrust >=0) {
		left = getInterpValue(FwdThrusts,reqdLeftThrust*2,TotThrust,ThrCmds, FwdLkupLen);
		}
	else {
		left = getInterpValue(RevThrusts, -reqdLeftThrust*2, TotThrust, ThrCmds, RevLkupLen);
		left = -left;	// This is a reverse thrust...
	}
	if (reqdRightThrust >=0) {
		right = getInterpValue(FwdThrusts, reqdRightThrust*2, TotThrust, ThrCmds, FwdLkupLen);
	}
	else {
		right = getInterpValue(RevThrusts, -reqdRightThrust*2, TotThrust, ThrCmds, RevLkupLen);
		right = -right;
	}
	*ThrCmdLeft  = (int)left;
	*ThrCmdRight = (int)right;
} 
