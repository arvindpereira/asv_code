/* 

   This file contains the data structures that are required for the
   maintainance of the database.

   Important data structures are:

   1) Subscription details:
	Will be a linked list with the following information:
	int client_fd,
	int read_subscription_flags;
	int write_subscription_flags;

   2) Data-synchronization locks:
	Presently being implemented without inbuild semaphores...
	However, I shall add true linux semaphore support soon!
*/
#ifndef _DATABANK__H_
 #define _DATABANK__H_

typedef struct
{
	int client_fd;
	int client_id;
	long int read_subscription_flags;
	long int write_subscription_flags;
	char client_name[40];
}Client_info;

extern int last_entry;
#define JOY_CONTROL_VECTOR 64

// Here are some of the communication protocol primitives...
// Communication is to begin with a standard 4 byte header that contains Comm-protocol and
// Client specific information.

// [ 0xb2 ]      [ Client Id ]    [ LOF ]    [ data - bytes ] [ 2byte checksum +0xa5 ] 
// DbeX frame    Unique Hex Id    Length     double for f.p.    Calc. chksum add 0xa5


#endif
