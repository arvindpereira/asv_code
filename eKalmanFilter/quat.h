/*
 * (c) Srikanth Saripalli
 * srik@robotics.usc.edu
 */

#ifndef _QUAT_H_
#define _QUAT_H_


/*
 * This will construct a direction cosine matrix from 
 * euler angles in the standard rotation sequence 
 * [phi][theta][psi] from NED to body frame
 *
 * body = tBL(3,3)*NED
 */

static inline void eulerDC(double tBL[3][3],double phi,double theta,double psi)
{
        const double cpsi       = cos(psi);
        const double cphi       = cos(phi);
        const double ctheta     = cos(theta);
        const double spsi       = sin(psi);
        const double sphi       = sin(phi);
        const double stheta     = sin(theta);

        tBL[0][0] = cpsi*ctheta;
        tBL[0][1] = spsi*ctheta;
        tBL[0][2] = -stheta;

        tBL[1][0] = -spsi*cphi + cpsi*stheta*sphi;
        tBL[1][1] =  cpsi*cphi + spsi*stheta*sphi;
        tBL[1][2] = ctheta*sphi;

        tBL[2][0] =  spsi*sphi + cpsi*stheta*cphi;
        tBL[2][1] = -cpsi*sphi + spsi*stheta*cphi;
        tBL[2][2] = ctheta*cphi;

}

/*
 * This will construct a direction cosine matrix from 
 * quaternions in the standard rotation  sequence
 * [phi][theta][psi] from NED to body frame
 *
 * body = tBL(3,3)*NED
 * q(4,1)
 */

void quatDC(double tBL[3][3],const double q[4]);


/*
 * This will construct the euler omega-cross matrix
 * wx(3,3)
 * p, q, r (rad/sec)
 */

void eulerWx(double Wx[3][3],double p,double q,double r);

/*
 * This will construct the quaternion omega matrix
 * W(4,4)
 * p, q, r (rad/sec)
 */

void quatW(double W[4][4],double p,double q,double r);


/*
 * This will normalize a quaternion vector q
 * q/norm(q)
 * q(4,1)
 */

void normq(double q[4]);


/*
 * This will convert from quaternions to euler angles
 * q(4,1) -> euler[phi;theta;psi] (rad)
 */

void quat2euler(const double q[4],double euler[3]);


/*
 * This will convert from euler angles to quaternion vector
 * phi, theta, psi -> q(4,1)
 * euler angles in radians
 */

void euler2quat(double q[4],double phi,double theta,double	psi);

#endif
