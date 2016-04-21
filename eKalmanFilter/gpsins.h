/* (c) Srikanth Saripalli
 * srik@robotics.usc.edu
 * GPS + IMU + COMPASS 17 state kalman filter
 * The 17 states are
 * Position (x,y,z)
 * Velocity (vx, vy, vz)
 * Attitude (roll, pitch,yaw)
 * Gyro Bias in roll, pitch and yaw)
 * Acceration biases in x, y, z
 * Add gravity as a term to take care of any changes in g component
 */


#ifndef _GPSINS_H_
#define _GPSINS_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
double error_in_measurement(
        double measured,
        double estimated
        );
void generate_A(
        const double pqr[3]
        );

void propogate_covariance(void);
    
void propogate_state(
        const double pqr[3],
        const double accel[3],
        int verbose
        );

void gpsins_init(
        const double angle[3], 
        const double position[3], 
        const double velocity[3],
        const double imu[3],
        int verbose
        );

void gpsins_reset(
        const double angle[3], 
        const double position[3], 
        const double velocity[3],
        const double imu[3]
        );

void gpsins_kalman_attitude_update(
        double C[3][17],
        const double err[3],
        const double attitude_sd,
	const double yaw_sd
        );
    
void gpsins_ahrs_update(
        const double pqr[3],
        const double accel[3],
        //const double field[3],
	const double gyro_sd,
	const double yaw_sd
        //const double roll,
        //const double pitch
        );

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
        );

void gpsins_kalman_compass_update(
        double C[3][17],
        const double err[3],
        double compass_sd
        );
void gpsins_compass_update (
        const double roll,
        const double pitch,
        const double yaw
        );

void gpsins_kalman_gps_pos_update(
        double C[3][17],
        const double err[3],
        const double gps_x,
        const double gps_y,
        const double gps_z
        );

void gpsins_gps_pos_update( 
        const double position[3], 
        const double gps_x,
        const double gps_y,
        const double gps_z,
        int verbose
        );

void gpsins_kalman_gps_vel_update(
        double C[3][17],
        const double err[3],
        const double vel_x,
        const double vel_y,
        const double vel_z
        );

void gpsins_gps_vel_update( 
        const double vel[3], 
        const double vel_x,
        const double vel_y,
        const double vel_z,
        int verbose
        );


void gpsins_get_state(
        double position[3],
        double velocity[3],
        double attitude[3],
        double bias[3],
        double abias[3],
        double *trace
        );
#endif
