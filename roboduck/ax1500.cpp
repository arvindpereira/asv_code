/*
	Author : Arvind Antonio de Menezes Pereira
	Date   : Monday, Jan 29, 2007.
	Desc.  : This program should be able to talk to the AX-1500 board on the Q-boat
		 and allow the computer control over the thrusters.
*/
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/signal.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

/*
	TODO: Write a new class for serial communications, which can be re-used more effectively.
*/

struct termios oldtio,newtio;
int fd_ax1500;

int init_tty_non_canon(struct termios *oldtio,struct termios *newtio) 
{
	int fd_ax1500;
	fd_ax1500=open("/dev/ttyUSB0",O_RDWR  | O_NOCTTY  |O_NONBLOCK);
	
	tcgetattr(fd_ax1500,oldtio);
	bzero(newtio,sizeof(newtio));

	// cfmakeraw(newtio); 
	
	newtio->c_cflag=(B9600|CS7|CREAD|PARENB) & ~(PARODD |CRTSCTS );
	newtio->c_iflag=   IGNBRK & ~IGNPAR | ISTRIP | INPCK;
	newtio->c_oflag=0;
   	newtio->c_lflag= ~( ICANON | ECHOE | ECHO | ISIG) ;

   	newtio->c_cc[VINTR] 	=0;
	newtio->c_cc[VQUIT] 	=0;
	newtio->c_cc[VERASE]	=0;
	newtio->c_cc[VKILL] 	=0;
	newtio->c_cc[VEOF] 	=4;
	newtio->c_cc[VTIME] 	=0;
	newtio->c_cc[VMIN] 	=1;
	newtio->c_cc[VSWTC] 	=0;
	newtio->c_cc[VSTART] 	=0;
	newtio->c_cc[VSTOP] 	=0;
	newtio->c_cc[VSUSP] 	=0;
	newtio->c_cc[VEOL] 	=0;
	newtio->c_cc[VREPRINT]	=0;
	newtio->c_cc[VDISCARD]	=0;
	newtio->c_cc[VWERASE] 	=0;
	newtio->c_cc[VLNEXT]  	=0;
	newtio->c_cc[VEOL2] 	=0;
	
	tcsetattr(fd_ax1500,TCSANOW,newtio);
	usleep(10000);
	return fd_ax1500;
}


int OpenAX1500(int next)
{
	char buf[100]; int i;
	int n;
	if(!next)
	fd_ax1500=init_tty_non_canon(&oldtio,&newtio);
	if(fd_ax1500<0)
		fprintf(stderr,"Could not open a port to the AX-1500");

	tcflush(fd_ax1500,TCIFLUSH);
	tcflush(fd_ax1500,TCOFLUSH);
			
	memset( buf, 0, 100 );
	strcpy(buf,"\r");
	for(i = 0; i < 15; i++){
		n = write(fd_ax1500, buf,1 );
		usleep(100000);
	}
	memset( buf, 0, 100 );
	usleep(10000);
	n = read( fd_ax1500, buf, 100 );
	fprintf(stderr, "read %d bytes\n", n);
	if(strstr(buf,"OK"))
	{	fprintf(stderr,"\nAX-1500 in RS-232 mode."); return 1; }
	else {  fprintf(stderr,"\nAX-1500 did not respond."); return 0; }
}


int setAX1500(int leftMotor, int rightMotor)
{
	char lbuf[20],rbuf[20],tempbuf[20];
	int ok,i;
	static int bad4long;
	memset( lbuf, 0, 20 );
	memset( rbuf, 0, 20 );
	if(leftMotor<0)	strcpy(lbuf,"!a"); else strcpy(lbuf,"!A");
	if(rightMotor<0) strcpy(rbuf,"!b"); else strcpy(rbuf,"!B");
	leftMotor=(abs(leftMotor)>127)?127:abs(leftMotor);
	rightMotor=(abs(rightMotor)>127)?127:abs(rightMotor);	
	sprintf(tempbuf,"%02x\r",leftMotor); strcat(lbuf,tempbuf);
	sprintf(tempbuf,"%02x\r",rightMotor); strcat(rbuf,tempbuf);

	memset(tempbuf,0,20); // usleep(10000);
	for(i =0 ; i<strlen(rbuf); i++)
	{	write( fd_ax1500, &rbuf[i], 1); 


	while(read( fd_ax1500, tempbuf, 1)>=1)
		{ // printf("%c",tempbuf[0]);									      
		  usleep(100);
       		  if(rbuf[i]==tempbuf[0] || tempbuf[0]=='+') 
			  ok=1; 
		else { ok=0; break;} }	tempbuf[0]=0;
	}
	for(i =0 ; i<strlen(lbuf); i++)
	{	write( fd_ax1500, &lbuf[i], 1); while(read( fd_ax1500, tempbuf, 1)>=1){ 
		usleep(100); // printf("%c",tempbuf[0]); 
		if(lbuf[i]==tempbuf[0] || tempbuf[0]=='+') ok=1;  else { ok=0; break; } } tempbuf[0]=0;
	}
		
	tcflush(fd_ax1500,TCIFLUSH);
	if(!ok) { bad4long++; if(bad4long==10) {fprintf(stderr,"\nBad speed set cmd"); OpenAX1500(1); bad4long=0; } }	
	return ok;
}
