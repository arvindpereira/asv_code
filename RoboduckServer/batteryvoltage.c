#include <termio.h>
#include <sys/fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdlib.h>

#include "boat.h"

void * monitor_battery(void * arg)
{
    int        m;        
    int        fd_dev;   
    char       c;        
    struct  termios    tbuf;  
    FILE *fptr;

   fptr = fopen("dmmdump.txt","w");

   if (( fd_dev = open(BATTERY_PORT, O_RDWR, 0)) < 0 ) {
        printf("Unable to open tty port specified\n");
        exit(1);
    }

    /* set up the line parameters : 7,2,N */
    tbuf.c_cflag = CS7|CREAD|CSTOPB|B1200|CLOCAL;
    tbuf.c_iflag = IGNBRK;
    tbuf.c_oflag = 0;
    tbuf.c_lflag = 0;
    tbuf.c_cc[VMIN] = 1; 
    tbuf.c_cc[VTIME]= 0; 
    if (tcsetattr(fd_dev, TCSANOW, &tbuf) < 0) {
        printf("Unable to set device '%s' parameters\n",BATTERY_PORT);
        exit(1);
    }

    /* Set DTR (to +12 volts) */
    m = TIOCM_DTR;
    if (ioctl(fd_dev, TIOCMSET, &m) < 0) {
        printf("Unable to set '%s' modem status\n",BATTERY_PORT);
        exit(1);
    }

    char *reading = malloc(sizeof(char)*20);
    int ctr;
    double voltage;
    char ** end_ptr;
    while (monitor.battery==1) {
      write(fd_dev,"D", 1);
      reading[0] = 0;
      do {
	read(fd_dev, &c, 1);
	sprintf(reading,"%s%c",reading,c);
      } while (c != '\r');
      voltage = atof(reading+3); 
      printf("Voltage: %f\n",voltage);
      test.boat_battery_voltage = voltage;
      fflush(stdout);
      sleep(1);
    }
    free(reading);
}

