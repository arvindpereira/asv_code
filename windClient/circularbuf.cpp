/*
Author: Arvind Antonio de Menezes Pereira

 Circular Buffer Implementation for Client Server Data buffering
 There will be one circular buffer per client connected to the server.
 For now, I am going to statically allocate buffers for each client.
 The index to these buffers will have to be maintained by the ClientTable entry
 for that particular client.
 When Clients are removed, they will need to be removed from the ClientTable...
 However Clients will continue to retain their circular buffer...
*/
#include <stdio.h>
#include "CommProtS.h"
#include "circularbuf.h"

unsigned char Client_CircBuffers[NUM_CLIENTS][MAX_SIZE];
unsigned int Client_readptr[NUM_CLIENTS]={0,0,0,0,0,0,0,0,0,0,0,0,0,0};	
		// Remember to initialize these to zero at initialization time
unsigned int Client_writeptr[NUM_CLIENTS]={0,0,0,0,0,0,0,0,0,0,0,0,0,0};
		// Remember to initialize these to zero at initialization time
unsigned long Client_totreadbuf[NUM_CLIENTS]={0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int get_client_packet(int fd,int client,unsigned char *temp,int len)
{
	int i,j,k,l,search=0,sync_loc,chksum_loc,dist=0,lof,lof_loc1,lof_loc2;
	int success=0,done=0;
	unsigned char compare_buffer[PACKETSIZE_COMP+1],chksum_debug;

	printf("\nCirc-buffer client=%d, len=%d, Client_writeptr=%d",
		client,len,Client_writeptr[client]);
	for(i=0;i<len;i++,Client_totreadbuf[client]++)	// Copy data to the circular buffer
  	{    l=Client_writeptr[client]++;
	     Client_CircBuffers[client][l]=*(temp+i);
//	     printf(" %x,",Client_CircBuffers[client][l]);
	     if(Client_writeptr[client]==(MAX_SIZE)) 
		Client_writeptr[client]=0;
	     if(Client_totreadbuf[client]>MAX_SIZE) Client_totreadbuf[client]=MAX_SIZE;
	}	
  	k=Client_readptr[client]; // After copying contents, check for sync
	
//	printf("\nCame to Circ-buf");
 	while(!done)	// Keep looking for valid packets until you run out of bytes
  	{
		dist=Client_writeptr[client]-Client_readptr[client];
		if(dist<0) dist=dist+MAX_SIZE;
		if(dist<MIN_SIZE) done=1; // Collect more bytes and return...
		
		if(Client_CircBuffers[client][k]==0xa3) // Found Sync byte
		{
//			printf("\nFound sync");  // Debug
			// See if there is a packet in the buffer
			lof_loc1=k+4; if(lof_loc1>=MAX_SIZE) lof_loc1-=MAX_SIZE;
			lof_loc2=k+5; if(lof_loc2>=MAX_SIZE) lof_loc2-=MAX_SIZE;
			lof=(int)(Client_CircBuffers[client][lof_loc1])*256
				+Client_CircBuffers[client][lof_loc2];
//			printf("\nlof=%d, lof_loc1=%d, lof_loc2=%d",lof,lof_loc1,lof_loc2); 
			if((lof+2)<=PACKETSIZE_COMP) // This is possibly a valid Sync...
			{
				if((lof+2)<=dist)
				{ // Copy data from sync onwards into compare-buffer...
				  for(i=0;i<(lof+2);i++)
				  {		 	
				  	compare_buffer[i]=Client_CircBuffers[client][k++];
 					if(k==MAX_SIZE) k=0;
				  }
				  // Now with it copied... test if its checksum is correct!
				  chksum_loc=(int)compare_buffer[4]*256+compare_buffer[5];
		
				  if(CalcChksum(compare_buffer)==
					((unsigned int)compare_buffer[chksum_loc]*256+
 						compare_buffer[chksum_loc+1]))
					{
						success=1;
//						printf("\nValid packet...");
						// Extract packet and place it in the buffer// 
						Check_Command(fd,len,compare_buffer);
						// Call handler...
						Client_readptr[client]=k;
					}
				}else done=1; // We still need to collect more bytes...
			}
		}
	}
	if(success) 
	  return success; 
	else return -1; // Unsuccessful
}
