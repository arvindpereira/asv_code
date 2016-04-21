#ifndef __XBOW_READER_H__
#define __XBOW_READER_H__

#include <termios.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/signal.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

class xbow
{
	// Serial Port Related Issues
	int 	fd;
	struct 	termios oldtio;
	struct 	termios newtio;
	fd_set 	readfs;
	struct 	timeval Timeout;
	char 	PortName[200];
	double 	lastDMUupdate;
	unsigned char circBuf[1000];	
	unsigned int  readPtr, totXbowBuf;

	// Data from the DMU
	float 	roll, pitch;
	float 	rollRate, pitchRate, yawRate;	
	float 	xAccel, yAccel, zAccel;
	float 	temp;
	float 	timeDMU;
	unsigned char checksum;

	// If the gyro is in Voltage Mode
	float GyroVoltsX,GyroVoltsY,GyroVoltsZ;
	float AccelVoltsX, AccelVoltsY, AccelVoltsZ;

   public :
	xbow() {	strcpy(PortName,"/dev/ttyS0");		}
	xbow(const char *serial) { 	strcpy(PortName,serial); 	}
	int open_serial_port();
	int close_serial_port();

	float tofloat(unsigned char *);	
	int init_xbow(); 
	void poll_xbow();
	int  check_xbow(unsigned char *temp, int res);
	void read_xbow_polled();
	unsigned char checksum_xbow(unsigned char *buf, int res);
	void tokenize_xbow_data(unsigned char *);
	void display_data_xbow();
	int peek_xbow();	
	int read_xbow_cont(unsigned char *temp, int res);
};


#endif
