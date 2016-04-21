/// This file contains the functions that handle actual communications
// between the RoboDuck server and the GUI client.
// ( So named after Communication Protocol - Server End )

#define _COMMPROT_CPP_

#include "boat.h"
#include "CommProtS.h"
#include "server.h"
#include "databank.h"
#include "byteorder.h"
// #include "serial.c"
#include "do_share.h"
#include "Utm.h"
#include <fcntl.h>
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
#include "ms_timer.h"

extern Stat test;
extern Monitor monitor;
extern Q_SHARED *shared;
extern DIAG_SHARED *sharedDIAG;
extern Client_info ClientTable[20];	// Allows multiple clients to connect up to itself.
extern int last_entry;
extern FILE *logFile;
extern int next_command;
#define MAX_SIZE 500
extern int winch_fd;
double motorposLR, motorposFB;
unsigned char circular_buffer[MAX_SIZE];
extern unsigned int calc_crc(unsigned char *ptr, int count);

void ParseDouble(double *,unsigned char[],int);

FILE *fptr;

double amp1,freq1,amp2,freq2;
extern double begin_time;

extern int OpenMissionFile(char *MissFileName);
Stat current;
extern Stat test;
FILE *fstatus;
FILE *fd_mode;
FILE * data_dump;

void adjust_motorposLR(int change)
{
  motorposLR=motorposLR+change;
  if (motorposLR<10)  motorposLR=10;
  if (motorposLR>246) motorposLR=246;
}

int transmit_status(int fd,Stat *myStatus)
{
	unsigned char frame_2_send[250]; 	
    store_double(shared->SensorData.lat,&frame_2_send[0],0);
	store_double(shared->SensorData.lon,&frame_2_send[8],0);
	store_double(shared->SensorData.cYaw,&frame_2_send[16],0);
	store_double(shared->sondeData.winch_depth,&frame_2_send[24],0);
	store_double(shared->sondeData.sonde_temp,&frame_2_send[32],0);
	store_double(shared->sondeData.sonde_flo,&frame_2_send[40],0);
	store_double(myStatus->boat_battery_voltage,&frame_2_send[48],0);
	store_double(shared->SensorData.speed,&frame_2_send[56],0);
	store_double(shared->SensorData.yawRate,&frame_2_send[64],0);
	store_double(shared->navData.des_heading,&frame_2_send[72],0);
    store_double(shared->kalmanData.Xpos,&frame_2_send[80],0);
    store_double(shared->kalmanData.Ypos,&frame_2_send[88],0);
    store_double(shared->kalmanData.Yaw,&frame_2_send[96],0);
    store_double(shared->SensorData.xAccel,&frame_2_send[104],0);
    store_double(shared->SensorData.yAccel,&frame_2_send[112],0);
    store_double(sharedDIAG->pidParams.pTerm,&frame_2_send[120],0);
    store_double(sharedDIAG->pidParams.dTerm,&frame_2_send[128],0);
    store_double(sharedDIAG->pidParams.iTerm,&frame_2_send[136],0);
    store_double(shared->qboatData.T1,&frame_2_send[144],0);
    store_double(shared->qboatData.T2,&frame_2_send[152],0);
    store_double(shared->kalmanData.ImuBias[2],&frame_2_send[160],0);
	store_double(shared->kalmanData.Xvel,&frame_2_send[168],0);
	store_double(shared->kalmanData.Yvel,&frame_2_send[176],0);
	store_double(shared->kalmanData.Zvel,&frame_2_send[184],0);
	store_double(sharedDIAG->SpdPidParams.pTerm,&frame_2_send[192],0);
	store_double(sharedDIAG->SpdPidParams.dTerm,&frame_2_send[200],0);
	store_double(sharedDIAG->SpdPidParams.iTerm,&frame_2_send[208],0);

	send_frame(fd,NewStatus,0x28,frame_2_send,192);	
}

unsigned int CalcChksum(unsigned char frame[])
{
	unsigned int i,sum= 0xa5;		// An offset so that a frame with only zeroes
				// does not make it through!
	for(i=0;i<((frame[4]<<8)+frame[5]);i++) // LOF should be frame[3]&frame[4]
	{					   // it does not include chksum byte
		sum+=frame[i];
	}
	return sum&0xffff;
}
 
void zero_all(unsigned char frame[], int len)
{
	int i;

	for(i=0; i<len; i++)
		frame[i]=0;
}

void moveWinch(double required_depth)
{
	int in,ctr;
	char *status_winch;
	double current_depth;
      //(frame[6]+frame[7]*0.1+frame[8]*0.01);

      // Clear all previous messages

      status_winch=(char *) malloc(100*sizeof(char));
      in =0;
      ctr = 0;
      free(status_winch);

      current_depth = test.winch_depth;

      if (required_depth == test.winch_depth)
          return;

      if (required_depth==0.00) {
          write(winch_fd,"#oz;",4);
          return;
      }

      printf ("\n MOVE TO DEPTH %.2lf",required_depth);
      
      if ( current_depth > required_depth) { // Go Up
      	char * str;
	str = (char *)malloc(9*sizeof(char));

	sprintf (str,"#or,%.2lf;",required_depth);
	printf ("\nSetting depth to %s",str);
 	write(winch_fd,str,9);
	free (str);
      }
      if ( current_depth < required_depth) { // Go Up
     	char * str;
	str = (char *)malloc (9*sizeof(char));
	sprintf (str,"#ol,%.2lf;",required_depth);
	fflush(stdout); 
	printf ("\nSetting depth to %s",str);
 	write(winch_fd,str,9);
	free(str);
      }


      // Check for messages
      status_winch=(char *)malloc(100* sizeof(char));
      in =0;
      free(status_winch);
      
      test.winch_depth=required_depth;
      //      sleep(2);
}

void resetWinch()
{
      test.winch_depth=0.0;
      write(winch_fd,"#oz;",4);
}

int ProcessCommand(int fd,unsigned char frame[],int length) {
  int incrementLR,i,in,ctr,ijk ;
  double kp,kd,ki,yRateBiasComp,leftThComp,rightThComp,heading,velocity,*tempDblPtr;
  double targetLat, targetLong,kpSpd,kdSpd,kiSpd;
  double required_depth, current_depth;
  char * status_winch, *status_word;
  char * sonde_cmd;
  struct timeval  starttime, thistime,tv;
  double targetUTME,targetUTMN; char targetUTMzone[5];
  char command[4];
  int motorposRUD;
  
  incrementLR = 5;
  // Command is the second byte of the frame:
  printf("\n\nClient fd %d",fd);

  switch(frame[3])
    {
    case SetJoyParam:
      printf ("\nC=%d\nD=%d,R=%d", frame[6], frame[7], frame[8]);
      shared->qboatData.qboat_mode=QBOAT_DIRECT;
      motorposFB = frame[6]-127;
      motorposLR = frame[7]-127;
      sharedDIAG->rudderData.rudAng = frame[8]-127; 
      sharedDIAG->rudderData.lastRudTime = get_time();
	  gettimeofday(&tv, NULL);
      printf ("\n Forward Speed %d \n Diff Angle: %d, Rudder Angle: %f",tv.tv_sec,tv.tv_usec, motorposFB, motorposLR, sharedDIAG->rudderData.rudAng);
	  shared->qboatData.T1 = motorposLR;
	  shared->qboatData.T2 = motorposFB;
      command[0]=4;    command[1]=0;	command[2]=motorposFB;	command[3]=motorposLR;
      fprintf(logFile,"$JOYPARAMS,%ld.%06ld,%f,%f,%f\n",tv.tv_sec,tv.tv_usec,shared->qboatData.T1, shared->qboatData.T2, sharedDIAG->rudderData.rudAng);	
      //write_stamp();
      usleep(50000);
      break;

	  
	case SetPIDParams:
	  ParseDouble(&kp,frame,6);
	  ParseDouble(&kd,frame,14);
	  ParseDouble(&ki,frame,22);
	  ParseDouble(&yRateBiasComp,frame,30);
	  ParseDouble(&leftThComp,frame,38);
	  ParseDouble(&rightThComp,frame,46);
	  ParseDouble(&kpSpd,frame,54);
	  ParseDouble(&kdSpd,frame,62);
	  ParseDouble(&kiSpd,frame,70);
	  sharedDIAG->SpdPidParams.Kp = kpSpd;
	  sharedDIAG->SpdPidParams.Kd = kdSpd;
	  sharedDIAG->SpdPidParams.Ki = kiSpd;
	
	  sharedDIAG->pidParams.Kp = kp;	
	  sharedDIAG->pidParams.Kd = kd;	
	  sharedDIAG->pidParams.Ki = ki;
	  sharedDIAG->pidParams.yRateBiasComp=yRateBiasComp;
	  sharedDIAG->motorParams.leftThComp = leftThComp; 	
	  sharedDIAG->motorParams.rightThComp = rightThComp; 	
	 break;

	case MoveToLatLong:
	  ParseDouble(&targetLat,frame,6);
	  ParseDouble(&targetLong,frame,14);
	  printf ("\n TARGET-LAT %lf \n TARGET-LONG %lf", targetLat, targetLong);
	  LLtoUTM(23, targetLat, targetLong, 
			 &targetUTMN, &targetUTME, targetUTMzone);
	  shared->navData.UTMN_ref = targetUTMN;
	  shared->navData.UTME_ref = targetUTME;
	  shared->qboatData.qboat_mode= QBOAT_GUIDANCE;
	  shared->qboatData.WayPtAchv = WAYPT_NOTACHV;
	  fprintf(logFile,"\n$TARGET,%lf,%lf\n",shared->navData.UTMN_ref,shared->navData.UTME_ref);
	  // shared->navData.StationN = shared->navData.UTMN_ref; shared->navData.StationE = shared->navData.UTME_ref;	
	  break;

    case SetSpdHdg:
	  ParseDouble(&heading,frame,6);
	  ParseDouble(&velocity,frame,14);
	  fprintf(logFile,"\n Heading = %lf, Speed = %lf",heading,velocity);
	  shared->navData.des_heading = heading; shared->navData.des_speed = velocity;
	  shared->qboatData.qboat_mode = QBOAT_AUTOPILOT;
	  break;

    case CommentInsert:

      gettimeofday(&tv, NULL);
      		fprintf(logFile,"$COMMENT,%ld.%06ld,",tv.tv_sec,tv.tv_usec);	
		for(int ctr=6;ctr<length;ctr++){
      			fprintf(logFile,"%c",frame[ctr]);	
		}
		fprintf(logFile,"\n");	
		fflush(logFile);
	  break;
	
    case Start: /* Start the boat */
      printf("\nStart Rxed.");
      motorposLR=0; // 128;
      motorposFB=64; //128;
      command[0]=4;	command[1]=0;	command[2]=4;	command[3]=0;
      //write_stamp();
            
      // Make the boat actually move.
      break;
    
    case Stop:
      printf("\nStop Rxed.");
      
      motorposFB=0; //128;
      command[0]=4;    command[1]=0;	command[2]=1;	command[3]=motorposFB;
      //write_stamp();
      usleep(10000);
      motorposLR=0; //128;
      command[0]=4;    command[1]=0;	command[2]=2;	command[3]=motorposLR;
      //write_stamp();
      
      // Make Fwd Spd zero, Rud zero, Reset modes back to default
      break;
            
    case Pause:
      printf("\nPause 10 sec. Rxed.");
      motorposFB=128;
      command[0]=4;    command[1]=0;	command[2]=1;	command[3]=motorposFB;
      //write_stamp();
      
      usleep(10);
      
      motorposFB=192;
      command[0]=4;    command[1]=0;	command[2]=1;	command[3]=motorposFB;
      //write_stamp();
      
      // Make Fwd Spd zero, Rud zero, but do not reset modes to default state. i.e. Pause misison execution.
      break;

    case Resume:
      printf("\nResume Rxed.");
      // Reapply Fwd Spd, Rud angle to those last commanded by mission etc.
      break;
      
    case ExecMis:
      fprintf (stderr,"\n Executing Mission.");
      fprintf (logFile,"\nBegun executing mission script"); fflush(logFile);	

      // File-name to be executed needs to be sent here... Send \0 at the end of file-name. Also include file extension.
      if(OpenMissionFile("/home/arvind/mission.msf")!=-1)	//(char*)&frame[6])!=-1)
      {	
	    shared->navData.lastUTME = shared->navData.UTME;
	    shared->navData.lastUTMN = shared->navData.UTMN;
	    shared->missData.missionLoaded=MISS_LOADED;
	    shared->missData.ExecutionStatus = EXEC_MISS; 	
	    shared->missData.WaitOnStatus = NO_WAIT;
	    next_command = 1;
	    fprintf(logFile,"\nFound mission.msf. Executing it...."); fflush(logFile);
      }else 
      { 
	    fprintf(logFile,"\nDid not find mission.msf...."); fflush(logFile);
	    shared->missData.missionLoaded = NO_MISS;
	    shared->missData.ExecutionStatus = NO_MISS;
      }
      break;
    
    case SetCurrentLoc:
	  // Debug command. Not for regular communications. (Should be disabled after debugging cycle).     
	  ParseDouble(&targetLat,frame,6);
	  ParseDouble(&targetLong,frame,14);
	  fprintf (logFile,"\n CURRENT-LAT %lf \n CURRENT-LONG %lf", targetLat, targetLong); fflush(stdout);
	  LLtoUTM(23, targetLat, targetLong, 
			 &targetUTMN, &targetUTME, targetUTMzone);
	  shared->navData.UTMN = targetUTMN;
	  shared->navData.UTME = targetUTME;
	  shared->SensorData.lat = targetLat;
	  shared->SensorData.lon = targetLong;
	  // shared->qboatData.qboat_mode= QBOAT_GUIDANCE;
	  // shared->qboatData.WayPtAchv = WAYPT_NOTACHV;	
	  break;
    
    case SetFlSampRate:
      printf("\nSet Fluorometer Samp. Rate");
      break;
      
    case SetThrSampRate:
      printf("\nSet Thermistor Samp. Rate");
      break;
      
    case GetWtrSample:
      for (i=0;i<50;i++)
	     printf("\nCollect a Water Sample");
      usleep(100000);
      command[0]=5;    command[1]=0;	command[2]=2;	command[3]=0;
      //write_stamp();
      break;
      
    case TakeFile:
      printf("\nBegin creating File with given name.");
      frame[length]=0;
      // The File name will be given in the remaining bytes... Should be a null-terminated string...
      printf("\n%s\n",((char*)&frame[6]));
      //fptr=fopen(&frame[6],"w");
      fptr=fopen("task.txt","w");
      printf ("Exiting file name creation");
      fflush(stdout);
      break;
      
    case TakePacket:
      printf("\nReceive packet # (part of file)");
      fwrite(&frame[6],sizeof(char),length-6,fptr);
      fclose(fptr);
      fflush(stdout);
      break;
      

      // ============= WINCH CONTROL ==========================/

      /*---------------------LOGIN -------------------*/
    case LoginSonde:
      printf ("\n\n\nLogin into THE SONDE");
      fflush(stdout);
      write(winch_fd,"#oz;",4);

      status_winch=(char *) malloc(100*sizeof(char));
      in =0;
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
      free(status_winch);
      sleep(1);

      write(winch_fd,"#h1,Hydrolab;",13);
      test.winch_depth=0.0;
      break;

      /*---------------------LOGOUT ------------------*/
    case LogoutSonde:
      write(winch_fd,"#h2;",4);
      test.winch_depth=0.0;
      break;

      /*---------------------STOP -------------------*/
    case StopWinch:
      printf ("\n\n\nSTOP THE SONDE");
      fflush(stdout);
      write(winch_fd,"s",1);
      break;

      /*---------------------REBOOT -------------------*/
    case RebootWinch:
      write(winch_fd,"#ob;",4);
      break;

      /*---------------------RESET -------------------*/
    case ResetWinch:
      //test.winch_depth=0.0;
      //write(winch_fd,"#oz;",4);
      resetWinch();
      break;
      
      /*---------------------DOWNLOAD DATA -------------------*/
    case DownloadSondeData:

      // Check for messages.Clear previous messages

      status_winch=(char *)malloc(100* sizeof(char));

      status_word=(char *)malloc(50000* sizeof(char));
      memset(status_word,0,50000);

      usleep(1000);

      gettimeofday(&starttime, NULL);

      in =0;
      while (in==0){
	memset(status_winch,0,100);
	in=read(winch_fd,status_winch,1);
	printf ("\n StatusA : %d %s --> %s \n",in, status_winch, status_word);
      }
      while (in!=0){
	memset(status_winch,0,100);
	in=read(winch_fd,status_winch,1);
	status_word = strcat(status_word,status_winch);
	printf ("\n StatusB : %d %s --> %s\n",in, status_winch,status_word);
      }
      free(status_winch);


      /* Download Command */
      printf("\n\nDownload Sonde Data-===============>\n\n");
      write(winch_fd,"#hh,1;",6);

      // Check for messages
      status_winch=(char *)malloc(100* sizeof(char));

      status_word=(char *)malloc(50000* sizeof(char));
      memset(status_word,0,50000);

      usleep(1000);
      in =0;
      while (in==0){
	memset(status_winch,0,100);
	in=read(winch_fd,status_winch,1);
	printf ("\n StatusA : %d %s --> %s \n",in, status_winch, status_word);
      }
      while (in!=0){
	memset(status_winch,0,100);
	in=read(winch_fd,status_winch,1);
	status_word = strcat(status_word,status_winch);
	printf ("\n StatusB : %d %s --> %s\n",in, status_winch,status_word);
      }
      free(status_winch);
      
      ijk=0;
      while (ijk == 0) {
	write(winch_fd,"a",1);
	// Check for messages
	status_winch=(char *)malloc(100* sizeof(char));
	in =0;
	while (in==0){
	  memset(status_winch,0,100);
	  in=read(winch_fd,status_winch,1);
	  // printf ("\n StatusA : %d %s --> %s \n",in, status_winch, status_word);
	}
	while (in!=0){
	  memset(status_winch,0,100);
	  in=read(winch_fd,status_winch,1);
	  status_word = strcat(status_word,status_winch);
	  // printf ("\n StatusB : %d %s --> %s\n",in, status_winch,status_word);
	}	
	
	if (strstr(status_word,"file finished") !=NULL)
	  ijk=1;

      }

      printf ("\n StatusB : %d %s --> %s %d\n",in, status_winch,status_word, sizeof(status_word));
      
      printf ("\nFILE DOWNLOAD COMPLETE!\n");
      data_dump = fopen("data.txt","w+");
      fwrite(status_word,50000,sizeof(char),data_dump);
      fclose(data_dump);

      printf ("\nStart Time = %6ld %7ld",starttime.tv_sec, starttime.tv_usec);
      gettimeofday(&thistime, NULL);
      printf ("\nEnd Time = %6ld %7ld",thistime.tv_sec, thistime.tv_usec);

      sleep(2);

      free(status_winch);
    
      break;

      /*---------------------SET SONDE SAMPLING PARAMETERS -------------------*/
    case SondeSamplingSetting:
      printf ("\nStart Time : %02d/%02d/200%1d,%02d:%02d:%02d;",frame[6],frame[7],frame[8],frame[9],frame[10],frame[11]);
      printf ("\nStop Time : %02d/%02d/200%1d,%02d:%02d:%02d;",frame[12],frame[13],frame[14],frame[15],frame[16],frame[17]);
      printf ("\nSampling interval : %02d sec.", frame[18]);
      printf ("\nSensor Warmup time : %02d sec.", frame[19]);

      sonde_cmd = (char *)malloc(30 * sizeof(char));
      sprintf (sonde_cmd,"#hg,3,1,1,%02d/%02d/200%1d,%02d:%02d:%02d;",frame[6],frame[7],frame[8],frame[9],frame[10],frame[11]);
      write(winch_fd,sonde_cmd,30);
      printf ("\n Sonde CMD - %s",sonde_cmd);
      free (sonde_cmd);

      sonde_cmd = (char *)malloc(30 * sizeof(char));
      sprintf (sonde_cmd,"#hg,3,1,2,%02d/%02d/200%1d,%02d:%02d:%02d;",frame[12],frame[13],frame[14],frame[15],frame[16],frame[17]);
      write(winch_fd,sonde_cmd,30);
      printf ("\n Sonde CMD - %s",sonde_cmd);
      free (sonde_cmd);

      sonde_cmd = (char *)malloc(13 * sizeof(char));
      sprintf (sonde_cmd,"#hg,3,1,3,%02d;", frame[18]);
      write(winch_fd,sonde_cmd,13);
      printf ("\n Sonde CMD - %s",sonde_cmd);
      free (sonde_cmd);

      sonde_cmd = (char *)malloc(13 * sizeof(char));
      sprintf (sonde_cmd,"#hg,3,1,4,%02d;", frame[19]);
      write(winch_fd,sonde_cmd,13);
      printf ("\n Sonde CMD - %s",sonde_cmd);
      free (sonde_cmd);
      /*---------------------SET WINCH DEPTH -------------------*/
    case SetWinchDepth:

      printf ("\n Received %d %d %lf %d",frame[6],frame[7], frame[7]*0.01,frame[8]);
      required_depth= (double)(frame[6]+ (frame[7]*0.01));
      moveWinch(required_depth);
      break;
      
    case GetStatus:
      printf("\nSending the Status back");
      
      printf ("\nDepth %.2lfm",test.winch_depth);
      printf ("\nFrame Size = %d",sizeof(Stat));
      transmit_status(fd,&test);
      fflush(stdout);
      break;
    default:
      fprintf(logFile,"Unknown Frame type...\n");
      break;
    }
}


int Check_Command(int fd,int len,unsigned char frame[]) {
  int i,chksum_loc;
  
  if(len>=6) {	
    printf("\n\nThe length of the frame is:%d",len);
    chksum_loc=(int)frame[4]*256+frame[5];
    
    
    chksum_loc=(int)frame[4]*256+frame[5]; 
    printf("Chksum_loc=%d, %x, %x",chksum_loc,frame[chksum_loc], frame[chksum_loc+1]); 
    
    if(frame[0]==(unsigned char)0xa3)	{		// SOF character...
      printf("CalcCRC=%d , frame_chksum=%d",calc_crc(frame,chksum_loc),((unsigned int)frame[chksum_loc]*256+frame[chksum_loc+1]));
      if(calc_crc(frame,chksum_loc)==((unsigned int)frame[chksum_loc]*256+frame[chksum_loc+1])) {   
	printf ("In Check Command");
	
	// Frame is valid... proceed...
	ProcessCommand(fd,frame,chksum_loc);
	printf("Frame Checked out right!!!");
      }
      else printf("Frame did not check 0ut!!!");
    }
    else printf("SOF invalid"); 
  }
  else printf("Frame length invalid");
}


void ParseDouble(double *dByter, unsigned char frame[],int index){
	int i;
	double *tempDblptr = (double*)&frame[index];
	*dByter = *tempDblptr;
	/*for(i=0;i<8;i++){
	 *dByter++ = frame[index+i];
	}*/
}
