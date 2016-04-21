//---------------------------------------------------------------------------
// flight_serial.c : implementation of serial port utility functions
//---------------------------------------------------------------------------
#include "flight_serial.h"
#include "flight_time.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>


//---------------------------------------------------------------------------
// open a serial port, and configure it with given parameters
//
int serial_open(const char device[], speed_t speed, struct termios* old_cfg)
{
    int serial;			// file descriptor
    struct termios cfg;		// terminal info
    int rv;			// return value

    // open the serial port
    if ((serial=open(device, O_RDWR|O_NOCTTY|O_NONBLOCK, S_IRUSR|S_IWUSR)) < 0)
    {
	perror("[Error:serial] cannot open the serial port");
	return -1;
    }

    // make it sure that the device is in non-blocking mode
    rv = fcntl(serial, F_GETFL, 0);
    if (rv >= 0 && ! (rv & O_NONBLOCK))
    {
	rv |= O_NONBLOCK;
	fcntl(serial, F_SETFL, 0);	// no error checking
    }

    // retrieve the current terminal info
    if (tcgetattr(serial, &cfg) < 0)
	perror("[Warning:serial] cannot retrieve terminal info");
    else if (old_cfg)
	*old_cfg = cfg;		// save the current conf. if requested

    // configure the port with given parameters
    cfmakeraw(&cfg);
    cfsetispeed(&cfg, speed);
    cfsetospeed(&cfg, speed);
    cfg.c_cc[VMIN] = 1;		// do not waste any time
    cfg.c_cc[VTIME] = 0;
    if (tcsetattr(serial, TCSAFLUSH, &cfg) < 0 || tcflush(serial, TCIOFLUSH) < 0)
    {
	perror("[Error:serial] cannot set terminal attributes");
	close(serial);
	return -1;
    }

    // success
    return serial;
}


//---------------------------------------------------------------------------
// close a serial port
//
int serial_close(int serial, struct termios* old_cfg)
{
    // discard any pending output
    tcflush(serial, TCIOFLUSH);

    // restore the old configuration if requested
    if (old_cfg)
    {
	if (tcsetattr(serial, TCSAFLUSH, old_cfg) < 0)
	    perror("[Warning:serial] cannot restore terminal info");
    }

    // close the serial port
    return close(serial);
}


//---------------------------------------------------------------------------
// read data using a single read() call
//
int serial_read(int serial, char buffer[], int max)
{
    int rv;		// returned value

    // wait until the command arrives the other side
    tcdrain(serial);

    // invoke read() once
    if ((rv=read(serial, buffer, max)) < 0)
	perror("[Error:serial] fail to read data");

    return rv;
}


//---------------------------------------------------------------------------
// read exactly N bytes in blocking mode
//
int serial_readn(int serial, char buffer[], int nbytes, double timeout)
{
    double deadline = time_current_fp() + timeout;
    int rv;		// returned value

    // wait until the command arrives the other side
    tcdrain(serial);

    // wait until the requested amount of data is ready
    while (serial_available(serial) < nbytes)
    {
	if (timeout > 0.0 && time_current_fp() > deadline)
	    break;
	else
	    usleep(100);	// yield
    }

    // read out the request amount of data
    rv = read(serial, buffer, nbytes);
    if (rv == nbytes)
	return rv;		// success
    else
    {
	perror("[Error:serial] fail to read data");
	return rv;		// error (weird case)
    }
}


//---------------------------------------------------------------------------
// write data in blocking mode
//
int serial_write(int serial, unsigned char buffer[], int size)
{
    int count = 0;
    int rv;		// returned value

    do
    {
	rv = write(serial, buffer+count, size-count);
	if (rv >= 0)
	    count += rv;
	else
	{
	    perror("[Error:serial] fail to write data");
	    return rv;
	}
    } while (count < size);

    return count;
}


//---------------------------------------------------------------------------
// check how many bytes are available
//
int serial_available(int serial)
{
    int nbytes;

#ifdef TIOCINQ
    if (ioctl(serial, TIOCINQ, &nbytes) < 0)
#else
    if (ioctl(serial, FIONREAD, &nbytes) < 0)
#endif
	return -1;

    return nbytes;
}


//---------------------------------------------------------------------------
// change the speed of a serial port already open
//
int serial_setspeed(int serial, speed_t speed)
{
    struct termios cfg;		// terminal info

    // retrieve the current terminal info
    if (tcgetattr(serial, &cfg) < 0)
	perror("[Warning:serial] cannot retrieve terminal info");

    // configure the port with given parameters
    cfmakeraw(&cfg);
    cfsetispeed(&cfg, speed);
    cfsetospeed(&cfg, speed);
    cfg.c_cc[VMIN] = 1;		// do not waste any time
    cfg.c_cc[VTIME] = 0;
    if (tcsetattr(serial, TCSAFLUSH, &cfg) < 0 || tcflush(serial, TCIOFLUSH) < 0)
    {
	perror("[Error:serial] cannot set terminal attributes");
	return -1;
    }

    // success
    return 0;
}

