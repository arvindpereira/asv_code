#ifndef _CLIENT_COMM_H_
#define _CLIENT_COMM_H_	

#include "ClientSocket.h"
#include "SocketException.h"
#include <iostream>
#include <pthread.h>
#include <string>
#include "Comm.h"


#define DBEX_IP     "128.125.163.226" // 206
 


/* This File contains the Client Communication Functions that 
   enable communication between DbeX Server and other clients */
// Author : Arvind.

/*
	In this new model there is going to be a table with a list of 
	Registered Communication Buffers...
	This table is part of this ClientComm Class.
	It will hold a pointer to each of the registered buffers
	and will redirect data to that particular object type.

	Procedure:
	1. Object constructs itself and registers for a buffer after providing CmdId and length
	2. Object keeps reading data from buffer using its own read function that
		it uses only on the ClientComm object that it initiated a communication with.
		If no data is available it will be returned -1 else it will have the length of data.
	3. To send data the object uses the send_frame method we decided before.
*/

#define MAX_BUFFERS	20
#define MAX_BUFSIZE 	1024
#define PACKETSIZE_COMP 1024
#define NUM_CLIENTS 20
#define MAX_SIZE 2500	// Max Size of the 
#define MIN_SIZE 8	// Min Packet Size is 8 bytes

/*
Client Comm will look at the list of registered CommBuffers to check which data is meant
for which Object that wants to receive it...

The entire circular buffer shall be implemented within the ClientComm class

*/

class ClientComm : public IComm
{
   private:
    //	unsigned char buf[MAX_BUFSIZE]; //??? What is this used for???
    char *ptr;
    ClientSocket *my_sock;
	int dest_id;
	int my_id;

	pthread_t get_data;
	void *thread_result;
	pthread_t listener;
	int res,bye;

	/* Client Circular buffer for receive */
	unsigned char CircBuffer[MAX_SIZE];
	unsigned int  writeptr,readptr;
	unsigned long totreadbuf;


// I am adding a thread to read data into its own Circular buffer...
// This class will have its own Circular buffer that will receive
// data and parse it!!!

   public:
	int numRegisteredBufs;	// Number of buffers registered until now...
	class CommBuffer *buf_table[MAX_BUFFERS];	// table of ptrs to registered buffers

	ClientComm();
	ClientComm (std::string host, int port,int);
	~ClientComm();

	void exit();
	int addToCircBuf(unsigned char *temp,int len);	// Circular buffer reader...
	int send_frame(unsigned char cmd,unsigned char dest_id,
			unsigned char *data,int len);
	friend void* listen_thread(void *arg);
	int register_buffer(unsigned char,
		                int write_len=STD_DATA_LEN,
				int read_len=STD_DATA_LEN);
/*	int register_buffer(unsigned char,
			    void (*)(),
		                int write_len=STD_DATA_LEN,
				int read_len=STD_DATA_LEN); */
  	int receive_frame(int tokenNo,unsigned char *data);	
	void clear_buffer(unsigned char *buf,long bufsize);
	int copyToBuffer(int len,unsigned char my_buffer[]);
	bool checkForCommand(int len, unsigned char my_buffer[]);
	int subscribe(unsigned char subscribe_frame[]);
};


// Class added to enable independent data access per client
/*
	Q.Why use it?
	A. This class will allow clients to create instances of a buffer
	   from which they can read. Assuming we have an object of type
	   "state". When it initializes itself, it will have to register
	   a "CommBuffer" for itself in the CommBuffer table and also
	   indicate the CmdId for the frame that needs to be extracted
	   into it.
	    The client function (eg.CovMatrix read() method) will require
	   to know how to extract its own data).
	4/18/2006:
		Struck upon an idea to do -> Receive Callback Function Registration. 
		Call-back registration will allow us to execute a function that
		the user wishes for each data-type based upon the data that he/she
		had processed... (Not Implemented!)
*/



class CommBuffer
{
	friend class ClientComm;
protected:
	char data[MAX_SIZE];
	unsigned char cmd_id;	// Which Frame to associate with this buffer.
	unsigned int  read_len;
	unsigned int  write_len;
	bool callBackRegistered; // should be made true if a callBack is registered.
public:
	unsigned int  received;
	unsigned int  overflows;
	void (*CallbackFn)(char *);	// a call-back function.
	CommBuffer() 
	{
		overflows=0;
		received=0;
		write_len=STD_DATA_LEN;
		read_len=STD_DATA_LEN;
	}
	CommBuffer(unsigned char cmd_id,int write_len=STD_DATA_LEN,int read_len=STD_DATA_LEN)
	{
		this->write_len=write_len;
		this->read_len=read_len;
		this->cmd_id=cmd_id;
		received=0;
		overflows=0;
		for(int i=0; i<MAX_SIZE;data[i++]=0); // Clear buffer...
	}
	/*
	int registerReceiveCallback(void (*myCallback)(char *)) 
	{
		CallbackFn=myCallback;
		callBackRegistered=true;	
	};	// Will be called after data is copied to the client buffer...
	 int callReceiveCallback() { if(callBackRegistered) { (*CallbackFn)(char *); return 0; } else return -1; } // use to invoke callback. 
	*/
};
#endif
