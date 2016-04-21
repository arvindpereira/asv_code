//---------------------------------------------------------------------------
// flight_serial.h : utility functions for serial port access
//---------------------------------------------------------------------------
#ifndef __FLIGHT_SERIAL_H
#define __FLIGHT_SERIAL_H

#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
//---------------------------------------------------------------------------
// open a serial port, and configure it with given parameters
//
// 	- parameters:
// 		const char device[]     : device file name
// 		speed_t speed           : communication speed
// 		struct termios* old_cfg : storage to save old conf. (or NULL)
//
// 	- return value : file descriptor, or -1 if an error occured
//
int serial_open(const char device[], speed_t speed, struct termios* old_cfg);


//---------------------------------------------------------------------------
// close a serial port
//
// 	- parameters:
// 		int serial              : file descriptor
// 		struct termios* old_cfg : old configuration saved (or NULL)
//
// 	- return value : 0 on success, or -1 if an error occured
//
int serial_close(int serial, struct termios* old_cfg);


//---------------------------------------------------------------------------
// read data using a single read() call
//
// 	- parameters:
// 		int serial : file descriptor
// 		int buffer : storage for read data ( >= max )
// 		int max    : maximum number of bytes to read
//
// 	- return value : the number of bytes read, or -1 if an error occured
//
int serial_read(int serial, char buffer[], int max);


//---------------------------------------------------------------------------
// read exactly N bytes in blocking mode
//
// 	- parameters:
// 		int serial    : file descriptor
// 		int buffer    : storage for read data
// 		int nbytes    : the number of bytes to read
// 		doube timeout : timeout in second
// 		                infinite blocking if timeout < 0.0
//
// 	- return value : 0 on success, or -1 if an error occured
//
int serial_readn(int serial, char buffer[], int nbytes, double timeout);


//---------------------------------------------------------------------------
// write data in blocking mode
//
// 	- parameters:
// 		int serial : file descriptor
// 		int buffer : data to write ( >= max )
// 		int max    : number of bytes to write
//
// 	- return value : the number of bytes written, or -1 if an error occured
//
int serial_write(int serial, unsigned char buffer[], int size);


//---------------------------------------------------------------------------
// check how many bytes are available
//
// 	- parameters:
// 		int serial : file descriptor
//
// 	- return value : the number of bytes available right now,
// 	                 or -1 if an error occured
//
int serial_available(int serial);


//---------------------------------------------------------------------------
// change the speed of a serial port already open
//
//	- parameters:
//		int serial    : file descriptor
//		speed_t speed : new communication speed
//
//	- return value : 0 on success, or -1 if an error occured
//
int serial_setspeed(int serial, speed_t speed);


#endif	// __FLIGHT_SERIAL_H
