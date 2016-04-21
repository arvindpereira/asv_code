/*
	Author : Arvind A de Menezes Pereira
	Descr  : Mission interpreter and command transfer program. Look at mission.h for command details.
	Date   : Feb 8, 2007.
*/

#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <ms_timer.h>
#include "mission.h"
#include "do_share.h"
#include "Utm.h"
#include <unistd.h>
#include <fcntl.h>

#define LINE_LENGTH 100


int command_number;
int next_command;
extern int winch_fd;

struct timeval TimeOut;
double before, now, time_diff,wait_time, time_out;
extern Q_SHARED *shared;
FILE *mission_file;
extern FILE *logFile;
// Open a mission file and display it...



int CheckCommand(char *cmdLine)
{
	//	Mission_CmdList
	int i,cmd_id;
	for(i=0 ; i< NUM_OF_MISSION_CMDS ; i++)
	{
		if(strstr(cmdLine,Mission_CmdList[i].Command_Name))
		{
			printf("\nCommand %s found.",Mission_CmdList[i].Command_Name); 
			fprintf(logFile,"\nCommand %s found.",Mission_CmdList[i].Command_Name);
			fflush(logFile);
			cmd_id = Mission_CmdList[i].Cmd_Id; break;
		}	
	}
	if(i==NUM_OF_MISSION_CMDS)
	{	printf("\nCould not identify...");  return NOT_FOUND; }
	else return cmd_id;
}


int OpenMissionFile(char *MissFileName)
{
	int num_bytes,i=0,num_cmds=0;
	char BufLine[LINE_LENGTH+1],*ptr;

	mission_file = fopen (MissFileName, "r");
	rewind(mission_file);
	if(mission_file != NULL) fprintf(stderr, "Mission file %s opened... ",MissFileName);
	else {	fprintf(stderr, "Error!!! Unable to open Mission file %s.",MissFileName); return -1; }

	// Read a line of text...
	while((ptr=fgets(BufLine,LINE_LENGTH,mission_file))!=NULL)
	{
		++i;
		if(CheckCommand(ptr)!=NOT_FOUND) ++num_cmds;
		fprintf(stderr,"\nLine %d, : %s",i,ptr);
		fprintf(logFile,"\nLine %d, : %s",i,ptr);
	}
	
	// Set the total number of commands...
	shared->missData.numCmds = num_cmds;
	rewind(mission_file);
	next_command = 1;
	return num_cmds;
}


int ExecLine(char *ptr,int cmd_id)
{
	char *tok, UtmZone[5];
	double templat, templon, tempUtmN, tempUtmE, tempT1, tempT2, tempdist;
	// Presently, I am assuming that the GUI has already Parsed the mission file accurately and that it does
	// not have any changes!	

	tok = strtok(ptr,",\n");
	switch(cmd_id)
	{
		case GotoLL:
		  	fprintf(stderr,"\n\n\nGotoLL: %s\n\n",ptr);
			fprintf(logFile,"\nGotoLL: %s\n",ptr);	
			fflush(logFile);
			tok = strtok(NULL,",");
			templat = atof(tok);  tok = strtok(NULL,"\n");
			templon	= atof(tok);
			fprintf(stderr,"GotoLL : lat = %f, lon = %f",templat,templon); fflush(logFile);
			// LLtoUTM(int ReferenceEllipsoid, const double Lat, const double Long, 
			// double *UTMNorthing, double *UTMEasting, char* UTMZone)
			LLtoUTM(23, templat, templon, &tempUtmN, &tempUtmE, UtmZone);

			// Update previous UTM references...
			tempdist = (shared->navData.lastUTME - shared->navData.UTME)*(shared->navData.lastUTME - shared->navData.UTME) +
					(shared->navData.lastUTMN - shared->navData.UTMN) * (shared->navData.lastUTMN - shared->navData.UTMN);
			if(sqrt(tempdist)>1000)
			{
				shared->navData.lastUTME = shared->navData.UTME;
				shared->navData.lastUTMN = shared->navData.UTMN;
			}
			else
			{
				shared->navData.lastUTME = shared->navData.UTME_ref;
				shared->navData.lastUTMN = shared->navData.UTMN_ref;
			}
			shared->missData.WaitOnStatus = WAIT_WAYPT;
			shared->qboatData.qboat_mode = QBOAT_GUIDANCE;
			shared->qboatData.WayPtAchv = WAYPT_NOTACHV;
			shared->navData.UTME_ref = tempUtmE; shared->navData.UTMN_ref = tempUtmN;	
			fprintf(logFile,"\n$LLSET,%.6f, %f,%f, %f,%f", get_time(),shared->navData.UTME_ref, shared->navData.UTMN_ref,
			templat,templon); fflush(logFile);
			next_command = 0;
			break;
		case GotoXY:
			printf("\nGotoXY: %s",ptr);
			fprintf(logFile,"\nGotoXY : %s",ptr);
			tok = strtok(NULL,",");
			tempUtmN = atof(tok);  tok = strtok(NULL,"\n");
			tempUtmE = atof(tok);
			shared->navData.UTME_ref = tempUtmE; shared->navData.UTMN_ref = tempUtmN;
			shared->missData.WaitOnStatus = WAIT_WAYPT;
			shared->qboatData.qboat_mode = QBOAT_GUIDANCE;
			shared->qboatData.WayPtAchv = WAYPT_NOTACHV;
			next_command = 0;
			break;
		case SetTimeOut:
			printf("\nSetTimeOut: %s",ptr);
			tok = strtok(NULL,"\n");
			wait_time = atof(tok);
			shared->missData.WaitOnStatus = WAIT_TIMEOUT;
			time_out = get_time()+wait_time;
			fprintf(logFile,"Setting a time-out of %f seconds.\nExpires at Time = %f.\nCurrent Time = %f",wait_time,time_out,get_time());
			fflush(logFile);
			next_command = 0;
			break;
		case SetThrusters:
		//	shared->qboatData.qboat_mode = QBOAT_DIRECT;
			tok = strtok(NULL,","); tempT1 = atof(tok); // shared->qboatData.T1 = atof(tok);
			tok = strtok(NULL,"\n"); tempT2 = atof(tok); // shared->qboatData.T2 = atof(tok);
			shared->qboatData.qboat_mode = QBOAT_DIRECT;
			shared->qboatData.T1 = tempT1; shared->qboatData.T2 = tempT2;
			fprintf(logFile,"\nSetThrusters: %f, %f",shared->qboatData.T1, shared->qboatData.T2);
			fflush(logFile);
			break;
		case  SetSpeed:
			tok = strtok(NULL,"\n");	shared->navData.des_speed = atof(tok);
			fprintf(logFile,"\nSet Speed: %f",shared->navData.des_speed); fflush(logFile);
			break;
		case SetRudder:
			tok = strtok(NULL,"\n"); shared->navData.des_heading = atof(tok);
			fprintf(logFile,"\nSet Rudder: %f",shared->navData.des_heading);
			break;
		case SetHeading:
			tok = strtok(NULL,"\n"); shared->navData.des_heading = atof(tok);
			fprintf(logFile,"\nSet Heading: %f",shared->navData.des_heading);
			break;
		case WinchDepth:
			tok = strtok(NULL,"\n"); shared->sondeData.winch_depth = atof(tok); 
			printf("\nWinchDepthr: %s",ptr);
			fprintf(logFile,"\nSetting Winch to go to depth of %f",shared->sondeData.winch_depth); fflush(logFile);
			// TODO: Start moving the winch to this depth!
			// moveWinch(shared->sondeData.winch_depth);
			break;
		case WinchReset:
			shared->sondeData.winch_depth = 0;
			printf("\nWinchReset: %s",ptr);
			// resetWinch();
			// TODO: Send command to winch to reset the winch...
			break;
		case SetupPID:
			printf("\nSetupPID: %s",ptr);
			// Not Implemented for Q-boat test on Tuesday...
			break;
		case Comment:
			fprintf(logFile,"\nComment: %s",ptr); fflush(logFile);
			break;
		
		default:
			printf("\nUnknown command found ...");
	}	

}

// Implement the time-out and command structures right now!!!
int AnalyzeMissionCommand()
{

	int i,num_bytes,cmd_id;
	char BufLine[LINE_LENGTH+1],*ptr;
	
	// Start reading the file..
	if(next_command)
	while((ptr=fgets(BufLine, LINE_LENGTH, mission_file))!=NULL)
	{
		++i;
		if(ptr[0]!='\n' && strlen(ptr)>2)
		{	if((cmd_id=CheckCommand(ptr))!=NOT_FOUND)
			{
				// Check up if this command needs to be run...
				// For Winch time-outs this thread can sleep... 
				// For GotoXY or GotoLL it cud busyloop for now...
				fprintf(logFile,"\nExecuting %s as cmd %d",ptr,cmd_id); fflush(logFile);
				ExecLine(ptr,cmd_id);
				// After one command is done, we go back...
				return 1;
			}
		}
		else
		{	// New line... Goto the next line.. we don't wanna do anything with this!
			printf("\n");
		}					
	}
	return 0;
}


void do_mission()
{
	shared->missData.lineNum = command_number;

	if(shared->missData.ExecutionStatus == EXEC_MISS)
	{	
		if(shared->missData.WaitOnStatus == WAIT_TIMEOUT)
		{
			if(get_time()>=time_out)
			{
			    fprintf(logFile,"\n\nTimeout expired at %f.\n\n",get_time());
			    shared->missData.WaitOnStatus = NO_WAIT;	//Turn off mode?
			    next_command=1;
			}
		}
		else 
		if(shared->missData.WaitOnStatus == WAIT_WAYPT)
		{
		  if(shared->qboatData.WayPtAchv==WAYPT_ACHV)	  
		  {
		  	printf("\n\nWay-point achieved...\n\n");
			shared->qboatData.WayPtAchv=WAYPT_NOTACHV;
			shared->missData.WaitOnStatus = NO_WAIT;
			next_command=1;
		  }
		}
		if(command_number>shared->missData.numCmds && next_command)	
		{	
			shared->missData.ExecutionStatus = NO_MISS ;// MISS_COMPLETE;
			printf("\nMission Completed...\n\n");
			shared->missData.WaitOnStatus = NO_WAIT;
			next_command=1;
		}
		while(next_command && command_number<shared->missData.numCmds)
	   	{

	   	 if(AnalyzeMissionCommand()) {  ++command_number;
		  printf("\nCommand Number = %ld",command_number);
		  fprintf(logFile,"\nCmdNo = %ld",command_number);
		  fflush(logFile);
		 }
		}
		usleep(10);
	
	}
}
