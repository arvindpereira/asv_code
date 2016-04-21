#ifndef _mission_h_
	#define _mission_h_

#define NUM_OF_MISSION_CMDS	11

#define GotoXY			0xa1
#define GotoLL			0xa2
#define SetTimeOut		0xa3
#define SetThrusters		0xa4
#define SetRudder		0xa5
#define SetHeading		0xa6
#define SetSpeed		0xa7

#define WinchDepth		0xa8
#define WinchReset		0xa9

#define SetupPID		0xaa

#define Comment			0xab

#define NOT_FOUND		99999

typedef struct
{
	int Cmd_Id;
	int  NumParams;
	char Command_Name[50];
}Command;

Command Mission_CmdList[NUM_OF_MISSION_CMDS] =
{
	{ GotoXY, 2, "WaypointNE" },
	{ GotoLL, 2, "WaypointLL" },
	{ SetTimeOut, 1, "Timeout" },
	{ SetThrusters, 2, "Thrusters" },
	{ SetRudder, 1, "Rudder" },
	{ SetHeading, 1, "Heading" },
	{ SetSpeed, 1, "Speed" },
	{ WinchDepth, 1, "WinchDepth" },
	{ WinchReset, 0, "WinchReset" },
	{ SetupPID, 7, "PIDSET" },
	{ Comment, 1, "REM" }		// Comments are strings after COMMENT.
};



#endif
