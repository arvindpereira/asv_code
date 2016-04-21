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

class RudderControl
{
	// Serial Port Related Issues
	int 	fd;
	struct 	termios oldtio;
	struct 	termios newtio;
	struct 	timeval Timeout;
	char 	PortName[200];

   public :
	RudderControl() {	strcpy(PortName,"/dev/ttyS0");		}
	RudderControl(const char *serial) { 	strcpy(PortName,serial); 	}
	int open_serial_port();
	int close_serial_port();

	float tofloat(unsigned char *);	
	int InitRudder(); 
	int SendAngle(double);
};


#endif
