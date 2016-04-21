#ifndef _COMMPROT_H__
	#define _COMMPROT_H__

// These are the command IDs for various commands to/from the boat

// Basic Mission Control
#define Start	0x25
#define Stop	0x26
#define Pause	0x27
#define Resume	0x28

#define LoadMis	0x30
#define ExecMis	0x31

// Boat control primitives
#define SetRotor	0x40
#define SetRudder	0x41
#define TurnRight	0x42	// These commands are supposed to be
#define TurnLeft	0x43	// in increments or decrements of 5%
#define MoveForward	0x44
#define MoveReverse	0x45
#define SetJoyParam     0x46

#define SetJoyParam     0x46
#define SetPIDParams    0x47
#define MoveToLatLong   0x48
#define SetSpdHdg	0x49

// Sampling primitives
#define SetFlSampRate	0x50
#define SetThrSampRate	0x51
#define GetWtrSample	0x52

// Boat status data message
#define NewStatus	0x60		// To conserve band-width I am going to use
#define GetStatus	0x61		// a compressed packet method of transfer.

// File Upload/Download information
#define TakePacket		0x70
#define ResendPacket	0x71
#define CancelTx		0x72
#define TakeFile		0x73

//Winch Control Primitives
#define LoginSonde              0xA0
#define LogoutSonde             0xA1
#define DownloadSondeData       0xA2
#define StopWinch               0xA3
#define SetWinchDepth           0xA4
#define RebootWinch             0xA5
#define ResetWinch              0xA6
#define SondeSamplingSetting    0xA7

#define CommentInsert		0xA8

// Debugging set commands
#define SetCurrentLoc		0xB1


/**********************************************************/
// For Subscription, the clients are going to ask for 
// certain data types 
#define NOT_FOUND	0x999

#define READ_SUBSCR	1
#define WRITE_SUBSCR	2
#define UNSUBSCRIBE	0

  #ifndef _COMMPROT_CPP_
//   extern char DataTypeNames[MAX_DATA_TYPES][20];
   extern unsigned int CalcChksum(unsigned char frame[]);
   extern int ProcessCommand(int fd,unsigned char frame[],int length);
   extern int Check_Command(int fd,int len,unsigned char frame[]);
   extern void display_matrix(const double mat[6][6]);
   extern void zero_all(unsigned char frame[], int len);
  #endif
#endif
