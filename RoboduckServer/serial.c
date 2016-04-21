#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/poll.h>
#include <time.h>

extern int     motorposLR,motorposFB;
int     adc_buf_len;
int     stamp_baud;                                   /* Stamp baud rate */
int     stamp_fd;                                     /* File descriptor for the stamp unit */

char command[8];
int errno;

int winch_fd;

int setupWinch(){
  struct termios term2, oldterm2;
  int i;
  char * winch_port="/dev/ttyUSB3";

  //winch_port = WINCH_PORT; //PORT_USB1;

  // open it.  non-blocking at first, in case there's no RS485 unit.
  winch_fd = open(winch_port, O_RDWR| O_SYNC|O_NONBLOCK, S_IRUSR | S_IWUSR);
  if (winch_fd < 0) {
    printf("\nWINCH PORT OPEN ERROR: %s\n", strerror(errno));
    printf ("\nTRY RUNNING MODPROBE PL2303 AND THEN RETRY ... EXITING....");
    fflush(stdout);
    return(-1);
  }  
  
  printf("\nPORT %s OPEN SUCCESSFUL",winch_port);
  
  tcgetattr(winch_fd,&oldterm2);

#if HAVE_CFMAKERAW
  cfmakeraw(&term2);
#endif
  
  cfsetispeed(&term2, B19200);
  i= cfsetospeed(&term2, B19200);
  
  #define BAUDRATE B19200
  
  //  bzero(&term,sizeof(term));
  term2.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
  term2.c_iflag = IGNPAR | ICRNL;
  term2.c_oflag = 0;
  term2.c_lflag = ICANON;
  term2.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  
  term2.c_cc[VINTR]=0;
  term2.c_cc[VQUIT]=0;
  term2.c_cc[VERASE]=0;
  term2.c_cc[VKILL]=0;
  term2.c_cc[VEOF]=4;
  term2.c_cc[VTIME]=1;
  //need to sometimes reset this to 0 or else may not work
  term2.c_cc[VMIN]=0;
  term2.c_cc[VSWTC]=0;
  
  tcflush(winch_fd,TCIFLUSH);
  tcsetattr(winch_fd,TCSANOW,&term2);
  tcsetattr(winch_fd,TCSAFLUSH,&term2);
  
  fcntl(winch_fd, F_SETFL,0);

  write (winch_fd,"#oz;",4);

  sleep(2);

  write(winch_fd,"#ol,0.1;",9);

  sleep(2);

  write (winch_fd,"#oz;",4);

  char *status_winch;
  status_winch=(char *)malloc(100* sizeof(char));
  int in;
  in =0;
/* // Commenting this out...
  while (in==0){
    memset(status_winch,0,100);
    in=read(winch_fd,status_winch,100);
    printf ("\n Status : %d %s\n",in, status_winch);
  }
  while (in!=0){
    memset(status_winch,0,100);
    in=read(winch_fd,status_winch,100);
    printf ("\n Status : %d %s\n",in, status_winch);
  }
*/
  sleep(2);
  
}
