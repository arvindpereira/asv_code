/*
	Author 	: Arvind A de Menezes Pereira.
	Date	: Sun, Jan 28, 2007.
*/

#ifndef _QBOAT_PID_H_
 #define _QBOAT_PID_H_

// Loop State
#define OPENLOOP	1
#define CLOSEDLOOP	0

#define SINGLE_LOOP_FB	0
#define RATE_FB		1



class PID
{
	

	private:
		char   PIDName[20];			// Useful for logging...
		int    LoopState;		// Open/Closed Loop...
		int    RateType;		// Feedback loop types... (General/RateFb)
		double Kp,Kd,Ki;		// Gains
		double Max,Min;			// Term range..
		double Last_Err,Acc_Err;	// Last Error, and Accumulated Error
		double Ref,Meas;		// Reference Signal and Measured Signal.
		double Out;			// PID output...
		int    Integral,Derivative;	// P/PD/PI/PID
		long   Iteration;		// Number of Iterations... 		
		double lastTime;
		double delta;			// For integration and differentiation.
		double *Kp4mSharedMem, *Ki4mSharedMem, *Kd4mSharedMem; // Gains from the Shared memory...
		double *pTermInSharedMem, *iTermInSharedMem, *dTermInSharedMem; // PID outputs into Shared Memory...
		int	update4mSharedMem;	// Flag that tells the PID to use new gains from the Shared memory
	public:
		PID();
		void setGains(double Prop, double Diff, double Integ);
		void setLimits(double max, double min);
		double doPID(double Ref,double Meas,double Rate,int yaw);
		void setName(char *);
		void setSHMPointers(double *,double *,double *, double *, double *, double *);
		
};

#endif
