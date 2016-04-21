#ifndef _COMMPROTC_H__
	#define _COMMPROTC_H__

#define NewStatus	0x60
#define GetStatus	0x61


// Definitions of IDs for Server and Client used in Source, Dest...
#define DBEX_SERVER	0x27
// Arvind's module for data exchange

#define PURSUER1	0X12
#define PURSUER2	0x13
#define PURSUER3	0x14


#define EVADER1		0x15
#define EVADER2		0x16

#define GetConnected	0x89
#define Disconnect	0x90

// IMPORTANT: Make sure that you also update changes in CommProtS.h
/**********************************************************/
// For Subscription, the clients are going to ask for 
// certain data types 
#define NOT_FOUND	0x999

#define READ_SUBSCR	1
#define WRITE_SUBSCR	2
#define UNSUBSCRIBE	0

#define MAX_DATA_TYPES 6	// Keep Updating this everytime
enum datatypes
{	
	DtypePursuerBidTable, DtypeEvaderLoc, DtypePursuerLoc
};

extern unsigned int calc_crc(unsigned char frame[], int);
extern unsigned int CalcChksum(unsigned char frame[]);
extern void display_matrix(const double mat[6][6]);
extern void zero_all(unsigned char *, int len);
 
/*
  #ifndef _COMMPROTC_CPP_
   extern char DataTypeNames[MAX_DATA_TYPES][20];
   extern int ProcessCommand(int fd,unsigned char frame[],int length);
   extern int Check_Command(int fd,int len,unsigned char frame[]);
  #endif
*/
#endif
