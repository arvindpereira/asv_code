#ifndef _CIRCULARBUF_H_
	#define _CIRCULARBUF_H_

#define SYNC_DVL 0xB3
#define PACKETSIZE_COMP 1024
#define NUM_CLIENTS 20
#define MAX_SIZE 2500	// Max Size of the 
#define MIN_SIZE 8	// Min Packet Size is 8 bytes

extern unsigned char Client_CircBuffers[NUM_CLIENTS][MAX_SIZE];
extern unsigned int Client_readptr[NUM_CLIENTS];
// Remember to initialize these to zero at initialization time
extern unsigned int Client_writeptr[NUM_CLIENTS];
// Remember to initialize these to zero at initialization time
extern unsigned long Client_totreadbuf[NUM_CLIENTS];
extern int get_client_packet(int fd,int client,unsigned char *temp,int len);
#endif
