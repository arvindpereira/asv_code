/*
 *
 * 17 state extended direct feedforward kalman filter
 * (c) Srikanth Saripalli
 * srik@robotics.usc.edu
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "quat.h"
#include "utils.h"
#include "gpsins.h"
#include "matrix.h"


/*
 * global defines 
 *
 */

#define g 9.81

static double A[17][17];
static double P[17][17];
static double DCM[3][3];
static double Pdot[17][17];
static double state[17];
double gpsins_trace;
double delta;

double error_in_measurement(
        double measured, 
        double estimated
)
{
    if((estimated > M_PI_2) && (measured < -M_PI_2))
        return (2*M_PI + measured - estimated);
    else if( (estimated < -M_PI_2) && (measured > M_PI_2))
        return (-2*M_PI + measured - estimated);
    else
        return (measured - estimated);
}
void generate_A(
        const double pqr[3]
)
{
        double p,q,r,q0,q1,q2,q3,u,v,w;

        p = pqr[0];
        q = pqr[1];
        r = pqr[2];

        q0 = state[6];
        q1 = state[7];
        q2 = state[8];
        q3 = state[9];

        u = state[3];
        v = state[4];
        w = state[5];

        memset(A, 0, sizeof(A));

        /* XYZ relative to XYZ is zero */
        // A[0..3][0..3] = 0;

        /* XYZ relative to UVW = DCM transpose */
        A[0][3] = DCM[0][0];
        A[0][4] = DCM[1][0];
        A[0][5] = DCM[2][0];

        A[1][3] = DCM[0][1];
        A[1][4] = DCM[1][1];
        A[1][5] = DCM[2][1];

        A[2][3] = DCM[0][2];
        A[2][4] = DCM[1][2];
        A[2][5] = DCM[2][2];

        /*
         * XYZ relative to Q
         * 
         */
        A[0][6] =                - (2 * v * q3) + (2 * w * q2); // dx/dq0
        A[0][7] =                  (2 * v * q2) + (2 * w * q3); // dx/dq1
        A[0][8] = - (4 * u * q2) + (2 * v * q1) + (2 * w * q0); // dx/dq2
        A[0][9] = - (4 * u * q3) - (2 * v * q0) + (2 * w * q1); // dx/dq3

        A[1][6] =   (2 * u * q3)                - (2 * w * q1); // dy/dq0
        A[1][7] =   (2 * u * q2) - (4 * v * q1) - (2 * w * q0); // dy/dq1
        A[1][8] =   (2 * u * q1)                - (2 * w * q3); // dy/dq2
        A[1][9] =   (2 * u * q0) - (4 * v * q3) + (2 * w * q2); // dy/dq3

        A[2][6] = - (2 * u * q2) + (2 * v * q1);                // dz/dq0
        A[2][7] =   (2 * u * q3) + (2 * v * q0) - (4 * w * q1); // dz/dq1
        A[2][8] = - (2 * u * q0) + (2 * v * q3) - (4 * w * q2); // dz/dq2
        A[2][9] =   (2 * u * q1) + (2 * v * q2);                // dz/dq3

        /*
         * UVW relative to UVW 
         *
         */
        A[3][3] =  0;          // 
        A[3][4] =  r;          //
        A[3][5] = -q;          // du / d(psi bias)

        A[4][3] = -r;          // dv / d(phi bias)
        A[4][4] =  0;          // dv / d(theta bias)
        A[4][5] =  p;          // dv / d(psi bias)

        A[5][3] =  q;          // dw / d(phi bias)
        A[5][4] = -p;          // dw / d(theta bias)
        A[5][5] =  0;          // dw / d(psi bias)




        /*
         * Body frame velocity relative to Q
         * 
         */
        A[3][6] = -2 * g * q2;  // du/dq0
        A[3][7] =  2 * g * q3;  // du/dq1
        A[3][8] = -2 * g * q0;  // du/dq2
        A[3][9] =  2 * g * q1;  // du/dq3

        A[4][6] =  2 * g * q1;  // dv/dq0
        A[4][7] =  2 * g * q0;  // dv/dq1
        A[4][8] =  2 * g * q3;  // dv/dq2
        A[4][9] =  2 * g * q2;  // dv/dq3

        A[5][6] =  0;           // dw/dq0
        A[5][7] = -4 * g * q1;  // dw/dq1
        A[5][8] = -4 * g * q2;  // dw/dq2
        A[5][9] =  0;           // dw/dq3

        /*
         * Velocity in body frame relative to gravity
         */
        A[3][10] = DCM[0][2];   // du/dG
        A[4][10] = DCM[1][2];   // dv/dG
        A[5][10] = DCM[2][2];   // dw/dG

        /*
         * Gyro bias relative to the velocities
         * dV = A + DCM * G - Wx * V
         */
        A[3][11] =  0;          // du / d(phi bias)
        A[3][12] =  w;          // du / d(theta bias)
        A[3][13] = -v;          // du / d(psi bias)

        A[4][11] = -w;          // dv / d(phi bias)
        A[4][12] =  0;          // dv / d(theta bias)
        A[4][13] =  u;          // dv / d(psi bias)

        A[5][11] =  v;          // dw / d(phi bias)
        A[5][12] = -u;          // dw / d(theta bias)
        A[5][13] =  0;          // dw / d(psi bias)

        A[3][14] = -1.0;
        A[3][15] =  0;
        A[3][16] =  0;

        A[4][14] =  0;
        A[4][15] = -1.0;
        A[4][16] =  0;

        A[5][14] =  0;
        A[5][15] =  0;
        A[5][16] = -1.0;

        /* Q wrt to Q
         *
         */
        A[6][6]  =  0;
        A[6][7]  = -p/2.0;
        A[6][8]  = -q/2.0;
        A[6][9]  = -r/2.0;

        A[7][6]  =  p/2.0;
        A[7][7]  =  0;
        A[7][8]  =  r/2.0;
        A[7][9]  = -q/2.0;

        A[8][6]  =  q/2.0;
        A[8][7]  = -r/2.0;
        A[8][8]  =  0;
        A[8][9]  =  p/2.0;

        A[9][6]  =  r/2.0;
        A[9][7]  =  q/2.0;
        A[9][8]  = -p/2.0;
        A[9][9]  =  0;
        
        /*
         * Gyro bias relative to quaternion state
         */
        A[6][11] =  q1/2;         // dq0 / d(phi bias)
        A[6][12] =  q2/2;         // dq0 / d(theta bias)
        A[6][13] =  q3/2;         // dq0 / d(psi bias)

        A[7][11] = -q0/2;         // dq1 / d(phi bias)
        A[7][12] =  q3/2;         // dq1 / d(theta bias)
        A[7][13] = -q2/2;         // dq1 / d(psi bias)

        A[8][11] = -q3/2;         // dq2 / d(phi bias)
        A[8][12] = -q0/2;         // dq2 / d(theta bias)
        A[8][13] =  q1/2;         // dq2 / d(psi bias)

        A[9][11] =  q2/2;         // dq3 / d(phi bias)
        A[9][12] = -q1/2;         // dq3 / d(theta bias)
        A[9][13] = -q0/2;         // dq3 / d(psi bias)
}

void propogate_covariance(void)
{
        index_t         i;
        index_t         j;
        double          temp[17][17];
        double          temp1[17][17];
        memset(Pdot, 0, sizeof(Pdot));

        // Position estimate noise
        // Arvind is going to mess around with these noises tonight!!!! (8/20/2007)
        Pdot[0][0] = 0.1;//0.001
        Pdot[1][1] = 0.1;
        Pdot[2][2] = 0.4;

        // Velocity estimate noise
        Pdot[3][3] = 0.1;//0.001
        Pdot[4][4] = 0.1;
        Pdot[5][5] = 0.1;

        // Quaterion attitude estimate noise
        Pdot[6][6] = 0.000005;//0.000001
        Pdot[7][7] = 0.000005;//0.001
        Pdot[8][8] = 0.000005;//0.000005
        Pdot[9][9] = 0.000005;

        // Gravity
        Pdot[10][10]       = 0.01;

        // Gyro bias
	
        Pdot[11][11] = 0.002; //0.002;//0.002
        Pdot[12][12] = 0.002; //0.002;
        Pdot[13][13] = 0.002; //0.002;
        /*
        Pdot[11][11] = 0.1;
        Pdot[12][12] = 0.1;
        Pdot[13][13] = 0.1;
	*/
        Pdot[14][14] = 0.01;//0.01
        Pdot[15][15] = 0.01;//0.1
        Pdot[16][16] = 0.01;
        
        mulNxM( Pdot, A, P, 17, 17, 17, 0, 1, 0);
        mulNxM( Pdot, P, A, 17, 17, 17, 1, 1, 0);

        mulNxM(temp, A, P, 17, 17, 17, 0, 0, 0);
        mulNxM(temp1, temp, A, 17, 17, 17, 1, 0, 0);

        gpsins_trace = 0;

        for( i=0 ; i<17 ; i++ ){
            for( j=0 ; j<17 ; j++ ){
                P[i][j] += Pdot[i][j] * delta + temp1[i][j]*delta*delta;
                if( i == j )
                    gpsins_trace += P[i][i] * P[i][i];
            }
        }
}

void propogate_state(
        const double pqr[3],
        const double accel[3],
        int verbose
)
{
        index_t i;
        
        double quat[4];
        double Qdot[4];
        double qdot[4];
        double dU, dV, dW, du, dv, dw, vel[3];
        double dpos[3], dposnew[3];
        double s,cs,ss, lambda, n, q0, q1, q2, q3;
                        
        
        const double p = pqr[0]/2.0;
        const double q = pqr[1]/2.0;
        const double r = pqr[2]/2.0;
        
        //print_matrix("DCM", (void *) DCM, 3, 3); 
        quat[0] = state[6];
        quat[1] = state[7];
        quat[2] = state[8];
        quat[3] = state[9];
        
        Qdot[0] = (-p * quat[1] - q * quat[2] -r*quat[3]);
        Qdot[1] = ( p * quat[0] - q * quat[3] + r * quat[2]);
        Qdot[2] = ( p * quat[3] + q * quat[0] -r * quat[1]);
        Qdot[3] = (-p * quat[2] + q * quat[1] + r*quat[0]);
    
	//see gavriltes and van der merwe 
	//very experimental code
        /*	
	s = delta*sqrt((p*p) + (q*q) + (r*r));
	cs = cos(s);
	ss = sin(s)/s;
	lambda = 1 - (quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3]);
	n = 30.0;
	q0 = ((cs + n*delta*lambda)*quat[0]) + (Qdot[0]*delta*ss);
	q1 = ((cs + n*delta*lambda)*quat[1]) + (Qdot[1]*delta*ss);
	q2 = ((cs + n*delta*lambda)*quat[2]) + (Qdot[2]*delta*ss);
	q3 = ((cs + n*delta*lambda)*quat[3]) + (Qdot[3]*delta*ss);

	quat[0] = q0;
	quat[1] = q1;
	quat[2] = q2;
	quat[3] = q3;
	*/
        	
        for( i=0 ; i<4 ; i++ )
            quat[i] += (Qdot[i]*delta);
    
	
        normq( quat );

        for(i = 0; i < 4; i ++)
            state[i +6] = quat[i];
	
        //Position state: X += DCM.tranpose * uvw * dt 
        
        for( i=0 ; i<3 ; i++ )
            state[i] += delta * (
                  DCM[0][i] * state[3]
                + DCM[1][i] * state[4]
                + DCM[2][i] * state[5]
            );
        
        dU = ( accel[0] + DCM[0][2] * g
            + 0
            + pqr[2] * state[4]
            - pqr[1] * state[5]
        );

        dV = ( accel[1] + DCM[1][2] * g
            - pqr[2] * state[3]
            + 0
            + pqr[0] * state[5]
        );

        dW = ( accel[2] + DCM[2][2] * g
            + pqr[1] * state[3]
            - pqr[0] * state[4]
            + 0
        );
        
        state[3] += dU * delta;
        state[4] += dV * delta;
        state[5] += dW * delta;
        
        /* 
        dpos[0] = DCM[0][0] * state[3];
        dpos[1] = DCM[1][1] * state[4];
        dpos[2] = DCM[2][2] * state[5];

        vel[0] = state[3] + dU * delta;
        vel[1] = state[4] + dV * delta;
        vel[2] = state[5] + dW * delta;
        
        qdot[0] = (-p * quat[1] - q * quat[2] -r*quat[3]) * delta;
        qdot[1] = ( p * quat[0] - q * quat[3] + r * quat[2]) * delta;
        qdot[2] = ( p * quat[3] + q * quat[0] -r * quat[1]) * delta;
        qdot[3] = (-p * quat[2] + q * quat[1] + r*quat[0]) * delta;

        du = ( accel[0] * g + DCM[0][2] * g
                + 0
                + pqr[2] * vel[1]
                - pqr[1] * vel[2]
                );

        dv = ( accel[1] * g + DCM[1][2] * g
                - pqr[2] * vel[0]
                + 0
                + pqr[0] * vel[2]
                );

        dw = ( accel[2] * g + DCM[2][2] * g
                + pqr[1] * vel[0]
                - pqr[0] * vel[1]
                + 0
                );

        dposnew[0] = DCM[0][0] * vel[0];
        dposnew[1] = DCM[1][1] * vel[1];
        dposnew[2] = DCM[2][2] * vel[2];

        for(i = 0; i < 4; i++){
            quat[i] = state[i + 6] + ((delta/2.0)*(qdot[i]+Qdot[i]));
        }
        
        normq(quat);

        for(i = 0; i < 4; i ++)
            state[i +6] = quat[i];

        for(i = 0; i <3; i++)
            state[i] = state[i] + ((delta/2.0)*(dpos[i] + dposnew[i]));

        state[3] += (dU + du)*delta/2.0;
        state[4] += (dV + dv)*delta/2.0;
        state[5] += (dW + dw)*delta/2.0;
        */
        if(verbose ==7){
            fprintf(stderr, " state %f %f %f\n", state[3], state[4], state[5]);
        }
}

void gpsins_init(
        const double angle[3],
        const double position[3],
        const double velocity[3],
        const double imu[3],
        int verbose
        )
{
        double quaternion[4];
        int i;

        
        memset(P, 0, sizeof(P));

        for( i = 0; i < 3; i++)
            P[i][i] = 1.0;

        state[0] = position[0];
        state[1] = position[1];
        state[2] = position[2];

        state[3] = velocity[0];
        state[4] = velocity[1];
        state[5] = velocity[2];

        euler2quat(quaternion, angle[0], angle[1], angle[2]);

        state[6] = quaternion[0];
        state[7] = quaternion[1];
        state[8] = quaternion[2];
        state[9] = quaternion[3];

        state[10] = g;

        state[11] = imu[0];
        state[12] = imu[1];
        state[13] = imu[2];

        state[14] = 0.0;
        state[15] = 0.0;
        state[16] = 0.0;
        
        if(verbose == 7)
            print_matrix("state", state, 17, 1);

}
void gpsins_reset(
        const double angle[3],
        const double position[3],
        const double velocity[3],
        const double imu[3]
        )
{
        double quaternion[4];
        int i;

        
        memset(P, 0, sizeof(P));

        for( i = 0; i < 17; i++)
            P[i][i] = 1.0;

        state[0] = position[0];
        state[1] = position[1];
        state[2] = position[2];

        state[3] = velocity[0];
        state[4] = velocity[1];
        state[5] = velocity[2];

        euler2quat(quaternion, angle[0], angle[1], angle[2]);

        state[6] = quaternion[0];
        state[7] = quaternion[1];
        state[8] = quaternion[2];
        state[9] = quaternion[3];

        state[10] = g;

        state[11] = imu[0];
        state[12] = imu[1];
        state[13] = imu[2];

        state[14] = 0;
        state[15] = 0;
        state[16] = 0;


}

void gpsins_kalman_attitude_update(
        double C[3][17],
        const double err[3],
        double attitude_sd,
	double yaw_sd
)
{
        double Einv[3][3];
        double E[3][3];
        double K[17][3];
        double compass_sd = yaw_sd*yaw_sd;
        //double temp[34];
        double temp[51];
        //double det;
        int i;
        double quat[4];
        
        memset(E, 0, sizeof(E));
        E[0][0] = attitude_sd;
        E[1][1] = attitude_sd;
        E[2][2] = compass_sd;

        //C * P
        mulNxM(temp, C, P , 3, 17, 17, 0 , 0, 0);

        //E+=(C*P)*C.transpose
        mulNxM(E, temp, C, 3, 17, 3, 1, 1, 0);

        //E = invert(E)
        inv33(E, Einv);        
        //P*C.tranpose
        mulNxM(temp, P , C, 17, 17, 3, 1, 0, 0);

        //K = (P*C.tranpose)*invert(E)
        mulNxM(K, temp, Einv, 17, 3, 3, 0, 0, 0);

        //X += K*err
        mulNxM(state, K, err, 17, 3, 1, 0, 1, 0);

        for(i = 0; i < 4; i++)
            quat[i] = state[i + 6];
        
        normq(quat);
        
        for(i = 0; i < 4; i++)
            state[i + 6] = quat[i];

        //C*P
        mulNxM(temp, C, P, 3, 17, 17, 0, 0, 0);

        //P-=K*(C*P)
        mulNxM(P, K, temp, 17, 3, 17, 0, -1, 0);

}

        
void gpsins_ahrs_update(
        const double pqr[3], 
        const double accel[3],
	//const double field[3],
	double gyro_sd,
	double yaw_sd
)
{
        double quat[4];
        double euler[3];
        int i;
        double roll_der[4], pitch_der[4], yaw_der[4];
        double THETAm[3];
        double eTHETA[3];
        double C[3][17];
        //double attitude_sd = 0.1182;
        double attitude_sd = gyro_sd*gyro_sd;
        
        memset(C, 0, sizeof(C));
        
        for(i = 0; i < 4; i++){
            quat[i] = state[i + 6];
        }

        accel2euler(THETAm, accel, quat);
        //accel2euler(THETAm, accel, field, quat);
        //THETAm[0] = roll;
        //THETAm[1] = pitch;
        quat2euler(quat, euler);
        dphi_dq(quat, roll_der);
        dtheta_dq(quat, pitch_der);
        dpsi_dq(quat, yaw_der);
    
        for( i = 0; i < 4; i++){
            C[0][i + 6] = roll_der[i];
        }
        
        for( i = 0; i < 4; i++){
            C[1][i + 6] = pitch_der[i];
        }
        for( i = 0; i < 4; i++){
            C[2][i + 6] = yaw_der[i];
        }
        eTHETA[0] = error_in_measurement(THETAm[0], euler[0]);
        eTHETA[1] = error_in_measurement(THETAm[1], euler[1]);
        eTHETA[2] = error_in_measurement(THETAm[2], euler[2]);
        gpsins_kalman_attitude_update(C, eTHETA, attitude_sd, yaw_sd);
        
}

void gpsins_state_update(
        const double imu[3],
        double accel[3],
        //double field[3],
	double gyro_sd,
	double yaw_sd,
        //const double roll,
        //const double pitch,
        const double time,
        int verbose
        )
{
        double pqr[3];
        double quat[4];
        int i;
        
        delta = time;
        for(i = 0; i < 3; i++){
            pqr[i] = imu[i] - state[i+11];
            accel[i] = accel[i] - state[i + 14];
        }

        for(i = 0; i <4; i++){
            quat[i] = state[i + 6];
        }
        
        quatDC(DCM, quat);

        propogate_state(pqr, accel, verbose);
        generate_A(pqr);
        propogate_covariance();
        //gpsins_ahrs_update(pqr, accel, gyro_sd, yaw_sd);
        //gpsins_ahrs_update(pqr, accel, field, gyro_sd, yaw_sd);
        //gpsins_ahrs_update(pqr, accel, field, gyro_sd, yaw_sd);
}

void gpsins_kalman_compass_update(
        double C[3][17],
        const double err[3],
        double compass_sd
)
{
        double Einv[3][3];
        double E[3][3];
        double K[17][3];
        double temp[51];
        int i;
        double quat[4];
        
        memset(E, 0, sizeof(E));
        E[0][0] = compass_sd;
        E[1][1] = compass_sd;
        E[2][2] = compass_sd;
        //E[1][1] = 0.05;
        //E[2][2] = 0.05;

        //C * P
        mulNxM(temp, C, P , 3, 17, 17, 0 , 0, 0);

        //E+=(C*P)*C.transpose
        mulNxM(E, temp, C, 3, 17, 3, 1, 1, 0);

        //E = invert(E)
        inv33(E, Einv);        
        //P*C.tranpose
        mulNxM(temp, P , C, 17, 17, 3, 1, 0, 0);

        //K = (P*C.tranpose)*invert(E)
        mulNxM(K, temp, Einv, 17, 3, 3, 0, 0, 0);

        //X += K*err
        mulNxM(state, K, err, 17, 3, 1, 0, 1, 0);

        for(i = 0; i < 4; i++)
            quat[i] = state[i + 6];
        
        normq(quat);
        
        for(i = 0; i < 4; i++)
            state[i + 6] = quat[i];

        //C*P
        mulNxM(temp, C, P, 3, 17, 17, 0, 0, 0);

        //P-=K*(C*P)
        mulNxM(P, K, temp, 17, 3, 17, 0, -1, 0);
}
                
void gpsins_compass_update(
        const double roll,
        const double pitch,
        const double yaw
)
{
        double quat[4];
        double euler[3];
        int i;
        double yaw_der[4];
        double roll_der[4];
        double pitch_der[4];
        double C[3][17];
        double err[3];
        double compass_sd = 0.0000009;
        
        memset(C, 0, sizeof(C));
        for(i = 0; i <4; i++){
            quat[i] = state[i + 6];
        }

        quat2euler(quat, euler);
        dphi_dq(quat, roll_der);
        dtheta_dq(quat, pitch_der);
        dpsi_dq(quat, yaw_der);
        
        for(i = 0; i < 4; i++)
            C[0][i + 6] = roll_der[i];
        for(i = 0; i < 4; i++)
            C[1][i + 6] = pitch_der[i];
        for(i = 0; i < 4; i++)
            C[2][i + 6] = yaw_der[i];
        /*
        err[0] = roll - euler[0];
        err[1] = pitch - euler[1];
        if((euler[2] > M_PI_2) && (yaw < -M_PI_2))
            err[2] = 2*M_PI + yaw - euler[2];
        else if( ( euler[2] < -M_PI_2) && (yaw > M_PI_2))
            err[2] = -2*M_PI + yaw - euler[2];
        else
            err[2] = yaw - euler[2];
        */
        err[0] = error_in_measurement(roll, euler[0]);
        err[1] = error_in_measurement(pitch, euler[1]);
        err[2] = error_in_measurement(yaw, euler[2]);
        
        gpsins_kalman_compass_update(C, err, compass_sd);

}


void gpsins_kalman_gps_pos_update(
        double C[3][17],
        const double err[3],
        const double gps_x,
        const double gps_y,
        const double gps_z
)
{
        double Einv[3][3];
        double E[3][3];
        double K[17][3];
        double temp[51];
        //double CP[3][17];
        //double Pct[17][3];
        int i,j;
        double quat[4];
        
        memset(E, 0, sizeof(E));
        
        E[0][0] = gps_x * gps_x;
        E[1][1] = gps_y * gps_y;
        E[2][2] = gps_z * gps_z;

        //C * P
        //print_matrix("C", C, 3, 17);
        /*
    	for(i=0 ; i<17 ; i++ ){
		    for(j=0 ; j<3 ; j++ ){
		    	Pct[i][j] = P[i][j];
			    CP[j][i] = P[j][i];
		    }   
	    }
        */
        //print_matrix("Pct", Pct, 17, 3);


        mulNxM(temp, C, P , 3, 17, 17, 0 , 0, 0);
        //print_matrix("temp", temp, 3, 17);

        //E+=(C*P)*C.transpose
        mulNxM(E, temp, C, 3, 17, 3, 1, 1, 0);

        //E = invert(E)
        //inverse(E, Einv, 6);
        inv33(E, Einv);

        //P*C.tranpose
        mulNxM(temp, P , C, 17, 17, 3, 1, 0, 0);

        //K = (P*C.tranpose)*invert(E)
        mulNxM(K, temp, Einv, 17, 3, 3, 0, 0, 0);
        //print_matrix("gain", K, 17, 3);
        
        //changes by sri r194
        /* 
        for(i = 3; i < 17; i++)
            for(j = 0; j < 3; j++)
                K[i][j] = 0.f;
        */
        
        //X += K*err
        mulNxM(state, K, err, 17, 3, 1, 0, 1, 0);

        for(i = 0; i < 4; i++)
            quat[i] = state[i + 6];
        
        normq(quat);
        
        for(i = 0; i < 4; i++)
            state[i + 6] = quat[i];

        //C*P
        mulNxM(temp, C, P, 3, 17, 17, 0, 0, 0);

        //P-=K*(C*P)
        mulNxM(P, K, temp, 17, 3, 17, 0, -1, 0);
}

void gpsins_gps_pos_update(
        const double position[3],
        const double gps_x,
        const double gps_y,
        const double gps_z,
        int verbose
)
{
    double C[3][17];
    double err[3];
    int i;
    
    memset(C, 0, sizeof(C));
    //fprintf(stderr, " inside gps update\n");
    
    for(i = 0; i < 3; i++){
        C[i][i] = 1.0;
    }

    for(i = 0; i < 3; i++)
        err[i] = position[i] - state[i];

    if(verbose == 5){
    fprintf(stderr, " err %f %f %f\n",
                err[0], err[1], err[2]);
    }
    gpsins_kalman_gps_pos_update(C, err, gps_x, gps_y, gps_z);
}

void gpsins_kalman_gps_vel_update(
        double C[3][17],
        const double err[3],
        const double vel_x,
        const double vel_y,
        const double vel_z
)
{
        double Einv[3][3];
        double E[3][3];
        double K[17][3];
        double temp[51];
        int i,j;
        double quat[4];
        
        memset(E, 0, sizeof(E));
        
        E[0][0] = vel_x * vel_x;
        E[1][1] = vel_y * vel_y;
        E[2][2] = vel_z * vel_z;

        //C * P
        mulNxM(temp, C, P , 3, 17, 17, 0 , 0, 0);

        //E+=(C*P)*C.transpose
        mulNxM(E, temp, C, 3, 17, 3, 1, 1, 0);

        //E = invert(E)
        //inverse(E, Einv, 6);
        inv33(E, Einv);

        //P*C.tranpose
        mulNxM(temp, P , C, 17, 17, 3, 1, 0, 0);

        //K = (P*C.tranpose)*invert(E)
        mulNxM(K, temp, Einv, 17, 3, 3, 0, 0, 0);

        //changes by sri r194
        /* 
        for(i = 6; i < 17; i++)
            for(j = 0; j < 3; j++)
                K[i][j] = 0.f;
        */
        //X += K*err
        mulNxM(state, K, err, 17, 3, 1, 0, 1, 0);

        for(i = 0; i < 4; i++)
            quat[i] = state[i + 6];
        
        normq(quat);
        
        for(i = 0; i < 4; i++)
            state[i + 6] = quat[i];

        //C*P
        mulNxM(temp, C, P, 3, 17, 17, 0, 0, 0);

        //P-=K*(C*P)
        mulNxM(P, K, temp, 17, 3, 17, 0, -1, 0);
}

void gpsins_gps_vel_update(
        const double vel[3],
        const double vel_x,
        const double vel_y,
        const double vel_z,
        int verbose
)
{
    double C[3][17];
    double err[3];
    double quat[4];
    double rot[3][3];
    double velocity[3];
    int i;
    double angle[3];
    double b_vel[3];
    
    memset(C, 0, sizeof(C));
    //fprintf(stderr, " inside gps update\n");
    /* Please check this before using this */ 
    for(i = 0; i < 3; i++){
        C[i][i + 3] = 1.0;
    }

    for(i = 0; i < 4; i++){
	    quat[i] = state[i + 6];
    }
    
        
    quatDC(rot, quat);
    mulNxM(velocity, rot, vel, 3, 3, 1, 0, 0, 0);
    velocity[2] = vel[2];
    for(i = 0; i < 3; i++)
        err[i] = velocity[i] - state[i + 3];
    
    //use just heading to rotate from N-E velocity
    //to body velocity in the helicopter frame of
    //reference. Ideally you would want to use the
    //above code..
     /* 
    quat2euler(quat, angle);
    b_vel[2] = vel[2];
    b_vel[0] = vel[0] * cos(angle[2]) + vel[1] * sin(angle[2]);
    b_vel[1] = vel[0] * -sin(angle[2]) + vel[1] * cos(angle[2]);
    for(i = 0; i < 3; i++)
        err[i] = b_vel[i] - state[i + 3];
        */
    

    if(verbose == 5){
    fprintf(stderr, " err %f %f %f\n",
                err[0], err[1], err[2]);
    }
    gpsins_kalman_gps_vel_update(C, err, vel_x, vel_y, vel_z);
}

void gpsins_get_state(
        double position[3],
        double velocity[3],
        double attitude[3],
        double bias[3],
        double abias[3],
        double *trace
)
{
    double quat[4];
    int i;
    double euler[3];
    
    for(i = 0; i < 4; i++)
        quat[i] = state[i + 6];

    quat2euler(quat, euler);

    for(i = 0; i < 3; i++){
        position[i] = state[i];
        velocity[i] = state[i + 3];
        bias[i] = state[i + 11];
        abias[i] = state[i + 14];
        attitude[i] = euler[i] * (180.0/M_PI);
    }
    (*trace) = gpsins_trace;
}
    
