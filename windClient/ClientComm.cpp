#include <iostream>
#include <stdio.h>
#include <pthread.h>
#include "ClientSocket.h"
#include "ClientComm.h"
#include "CommProtC.h"
#include <do_share.h>
#include <ms_timer.h>

using namespace std;
void* listen_thread(void *arg);
extern Q_SHARED *shared;
extern DIAG_SHARED *sharedDIAG;

ClientComm::ClientComm()
{
	my_sock=new ClientSocket("localhost",4000);
	// ::myself=this;
	// Now I should Create a Thread and keep on checking for a client
	writeptr=0,readptr=0;
	totreadbuf=0;
	numRegisteredBufs=0;
	res=pthread_create(&listener,NULL,listen_thread,(void*)this);	
}
	
ClientComm::ClientComm (std::string host, int port, int myId)
{
	my_id=myId;
	writeptr=0,readptr=0;
	totreadbuf=0;
	// ::myself=this;
	my_sock=new ClientSocket(host,port); 
	numRegisteredBufs=0;
	res=pthread_create(&listener,NULL,listen_thread,(void*)this);
}

ClientComm::~ClientComm()
{
	delete my_sock;
}

void ClientComm::exit()
{
	int res=0;
	this->bye=1;
	res=pthread_join(listener, &thread_result);
}

/* This File contains C++ Code for Client Communications... */
int ClientComm::send_frame(unsigned char cmd,unsigned char dest_id,unsigned char *data,int len)
{
	int i;
	unsigned char frame_2_send[1024];
	clear_buffer(frame_2_send,1024);
	
	frame_2_send[0]=(unsigned char)0xa3; // Our Protocol's SOF
	frame_2_send[1]=my_id;
	frame_2_send[2]=dest_id;
	frame_2_send[3]=cmd;
	frame_2_send[4]=((len+6)/256)&0xff;
	frame_2_send[5]=(len+6)&0xff;
	for(i=6;i<(len+6);i++,data++)
	  frame_2_send[i]=*data;
	unsigned int chksum= calc_crc(frame_2_send,len+6);// CalcChksum(frame_2_send);
	cout<<"  chksum ="<<chksum;
	frame_2_send[i]=(chksum>>8)&0xff;
	frame_2_send[i+1]=chksum&0xff;
	my_sock->send_bytes(frame_2_send,i+2);
}

void ClientComm::clear_buffer(unsigned char *buf,long bufsize)
{
	long i;
	for(i=0;i<bufsize;i++,buf++)
		*buf=0;
}


// Adds a buffer to CommBuffer, returns token_No
int ClientComm::register_buffer(unsigned char msgFilter,int write_len,int read_len)	
{
	// Instantiate a new Buffer.... set its cmdId to msgFilter
	numRegisteredBufs++;
	if(numRegisteredBufs<=MAX_BUFFERS)
	{	
		buf_table[numRegisteredBufs-1]=new CommBuffer(msgFilter);
		return (numRegisteredBufs-1);	// Return the tokenNo to be used for data-reads
	}else { numRegisteredBufs--; return -1; } // Failure... 
}

// Adds a buffer to CommBuffer, returns token_No and also allows a callback function to be registered.
/* TODO: Try registering a callback function later!
int ClientComm::register_buffer(unsigned char, void (*)(), int write_len=STD_DATA_LEN,int read_len=STD_DATA_LEN)
{
	// 
}
*/

int ClientComm::receive_frame(int tokenNo,unsigned char *data)
// Checks which buffer sends the data to 
{
	// Get the data from the Buffer with this tokenNo
	if(buf_table[tokenNo]->received)
	{	memcpy(data,buf_table[tokenNo]->data,buf_table[tokenNo]->read_len);
		buf_table[tokenNo]->received=0; return buf_table[tokenNo]->read_len;
	}
	else return -1;
}

// The little thread whose sole job is to look out for incoming packets from the server
void* listen_thread(void *arg)
{
	ClientComm *myself=(ClientComm*)arg;
	struct timeval timeout; int debug;
	unsigned char temp_read[MAX_SIZE];
	int num_rxed;
	
	while(myself->bye==0)
	{
		num_rxed=myself->my_sock->receive_bytes(temp_read);
		if(num_rxed>0)
		{
			myself->addToCircBuf(temp_read,num_rxed);
			printf("\nA packet was received....");
		}else if(num_rxed==-1) printf("A read error took place!!!");
	}
	if(myself->bye)
	  printf("Thread is exiting...");
	pthread_exit(NULL);
}

// Sure hope this doesn't have any bugs in it!!! If so I'm scr**ed!!!
int ClientComm::addToCircBuf(unsigned char *temp,int len)
{
	int i,j,k,l,loc,search=0,sync_loc,chksum_loc,dist=0,lof,lof_loc1,lof_loc2;
	int success=0,done=0,chksum;
	unsigned char compare_buffer[MAX_SIZE],chksum_debug;

	for(i=0;i<len;i++,totreadbuf++)	// Copy data to the circular buffer
  	{    l=writeptr++;
	     CircBuffer[l]=*(temp+i);
	     //printf(" %x,",Client_CircBuffers[client][l]);
	     if(writeptr==(MAX_SIZE)) 
			writeptr=0;
	     if(totreadbuf>MAX_SIZE) totreadbuf=MAX_SIZE;
	}	
  	k=readptr; // After copying contents, check for sync
	
	printf("\nCame to Circ-buf");
 	while(!done)	// Keep looking for valid packets until you run out of bytes
  	{
		dist=writeptr-readptr;
		if(dist<0) dist=dist+MAX_SIZE;
		if(dist<MIN_SIZE) done=1; // Collect more bytes and return...
		
		if(CircBuffer[k]==0xa3) // Found Sync byte
		{
			printf("\nFound sync");  // Debug
			// See if there is a packet in the buffer
			lof_loc1=k+4; if(lof_loc1>=MAX_SIZE) lof_loc1-=MAX_SIZE;
			lof_loc2=k+5; if(lof_loc2>=MAX_SIZE) lof_loc2-=MAX_SIZE;
			lof=(int)(CircBuffer[lof_loc1])*256+CircBuffer[lof_loc2];
			printf("\nlof=%d, lof_loc1=%d, lof_loc2=%d",lof,lof_loc1,lof_loc2); 
			
			if((lof+2)<=PACKETSIZE_COMP && (lof+2)>6) // This is possibly a valid Sync...
			{
				if((lof+2)<=dist)
				{ // Copy data from sync onwards into compare-buffer...
				  for(i=0;i<(lof+2);i++)
				  {		 	
				  	compare_buffer[i]=CircBuffer[k++];
 					if(k==MAX_SIZE) k=0;
				  }
				  // Now with it copied... test if its checksum is correct!
				  chksum_loc=(unsigned int)compare_buffer[4]*256+compare_buffer[5];
				  printf("\nChecking the Checksum at loc =%d",chksum_loc);
		
				  if( calc_crc(compare_buffer, chksum_loc) ==// CalcChksum(compare_buffer)==
					((unsigned int)compare_buffer[chksum_loc]*256+
 						compare_buffer[chksum_loc+1]))
					{
						success=1;
						printf("\nValid packet...");
						checkForCommand(chksum_loc,compare_buffer);
						// copyToBuffer(chksum_loc,compare_buffer);
						readptr=k;
					}
					else cout<<"Error... CRC did not match!"<<endl;
				}else done=1; // We still need to collect more bytes...
			}
		}
	}
	if(success) 
	  return success; 
	else return -1; // Unsuccessful
}

bool ClientComm::checkForCommand(int len, unsigned char my_buffer[])
{
	// Check if we have received any of the other cmd-ids and do some stuff based on it...
	int count,found=0;
	double WindSpeed, WindDirn, SonarBat;
	printf("%d\n", len);
	switch(my_buffer[3])
	{
		case NewStatus:
		  WindSpeed=*((double *)&my_buffer[6]);
		  WindDirn  =*((double *)&my_buffer[14]);
		  SonarBat  =*((double *)&my_buffer[22]);

		  cout<<"WindSpeed="<<WindSpeed<<", WindDirn="<<WindDirn<<", SonarBat="<<SonarBat<<endl;
		  sharedDIAG->windData.rawWindSpd = WindSpeed;
		  sharedDIAG->windData.rawWindDirn = WindDirn;
		  sharedDIAG->windData.windDirnLocal = WindDirn/(1024 /* - deadzone */) * 360; // + offset
		  sharedDIAG->windData.windDirnGlobal = sharedDIAG->windData.windDirnLocal + shared->SensorData.cYaw; // + offset 
		  sharedDIAG->windData.windSpeed = WindSpeed; // * conv_factor;
		  sharedDIAG->windData.battVoltage = SonarBat; 
		  sharedDIAG->windData.lastWindUpdateTime = get_time();
	// 	  tempRoundInfo=(RoundInfoPacket*)&my_buffer[6];
		  fflush(stdout);
		  return true;
		default:
		  return false;
	}

}


int ClientComm::copyToBuffer(int len,unsigned char my_buffer[])
{
	int found=0;
	// Run a check from the begining to the end of the registered
	// buffer table to figure out which client this is for!!!
	for(int i=0,j; i<numRegisteredBufs;i++)
	{
//		printf("\nReceived something...");
 		for(j=0;j<len;j++)
 			printf("%x,",my_buffer[j]);
			
		if(checkForCommand(len,my_buffer)==false)
		if(buf_table[i]->cmd_id==my_buffer[3])	// Checking MsgFilter
		{
			printf("\nCopying to buffer[%d]...",i);
 			memcpy(buf_table[i]->data,&my_buffer[6],len-8);
			buf_table[i]->read_len=len-6;
			if(buf_table[i]->received)
			{ buf_table[i]->overflows++; }	// Just overwrote something...
			buf_table[i]->received=1; 
			found=1;
		} 
	}
	if(found) printf("\nData matched with buffer..."); 
//	else printf("\nData did not match any buffer...");
}


int ClientComm::subscribe(unsigned char subscribe_frame[]) // A max of 20 different datatypes at this point of time...
{
	// send_frame(Subscribe,DBEX_SERVER,subscribe_frame,MAX_DATA_TYPES);
}
