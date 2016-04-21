@author: Arvind Pereira

Code used on USC's robotic boats (Roboducks a.k.a. Q-boats)
-----------------------------------------------------------

This is a portion of the code used on USC's robotic boats or Autonomous Surface Vehicles. Each directory contains code which is specific to a particular module which needs to be running on the vehicle.

Designed to compile on a standard Linux distribution such as Debian/Ubuntu/Red Hat etc. Requires gcc and g++ at a minimum. Will not compile on OS-X or Windows since they use different APIs for supporting shared memory and message queues.

roboduck - GNC code including PIDs etc. (written in C/C++ by Arvind Pereira) 
common   - common code such as timers, shared memory access routines and so on which are common to everything (mostly written by Arvind Pereira)
eKalmanFilter - Kalman Filtering code (mostly written by Srikanth Saripalli for the USC helicopter, but was modified for use on the USC boats by Arvind Pereira)
gpsReader - GPS reader for NMEA GPS devices. (originally written by Buyoon Jung and extended by Arvind Pereira)
RoboduckServer - Server used to communicate data back and forth between a C# GUI (entirely written by Arvind Pereira)
rudderControl  - code to interface with the rudder actuator
scripts  - Scripts that start everything up on the robot
tcmReader - TCM compass reader
windClient - Wind Sensor server (used to transmit data back and forth)
xbowReader - CrossBow IMU reader

The system is designed to use shared-memory for data to be shared between processes.

A structure is resident in shared memory which contains a pool of all the important data required to be used by other processes. Each of these processes is responsible for populating a particular structure associated with itself if it is the producer of some data. It also updates the time field for its data with the current time so other processes know that the value changed when they come by to utilize it. For example, the Kalman filter which runs at a high rate essentially polls the data from various sensors as it updates itself.

This allows us to decouple all our modules into standalone programs. If any one of these malfunctions and crashes, a monitoring shell script can attempt to restart that particular program. Other programs may utilize the fact that the data has not been updated for a while to know when the data might be too old and can thus stop sending commands to the thrusters and so on.

Code is lightly commented, but should be readable for the most part.

To compile just run 'make'
