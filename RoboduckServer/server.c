// Author: Arvind Antonio de Menezes Pereira
// Simple Server creation, maintenance code... 
/*
	This server was designed to run on PC-platforms running a version of the Linux
	operating system. The author hopes that the same can be ported with minimal
	changes to work satisfactorily as part of the communicaition blocks used on 
	the NAMOS-Roboduck and GUI.

	Issues that need to be addressed are:
	1) If the Linux-EMSTAR laptop is eliminated... how will communication take place? Can
		we have a forwarding node using say a Stargate board through which a platform-independent
		gateway can	be established?
	2) At present the GUI opens a client socket and connects to the server on the boat.
		Buoy data does not enter the GUI directly.... It has to come through the 
		Linux-EMSTAR laptop.
*/
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <pthread.h>
#include <string.h>

#include "boat.h"
#include "server.h"
#include "CommProtS.h"
#include "databank.h"
#include "byteorder.h"
#include "circularbuf.h"

extern int server_sockfd, client_sockfd;
extern socklen_t server_len, client_len;
struct sockaddr_in server_address, client_address;
extern int result,bye;
extern fd_set readfds,testfds;	
extern unsigned char buf[MAX_BUFSIZE];
extern int choice,fd,nread,client_present,write_client;
extern int client_fds[10]; char *ptr;
extern int display_once;
extern int last_entry;

extern Client_info ClientTable[20];
extern double CovMatrix_Updt[6][6]; 

extern int find_clientfd(int client_fd);


unsigned int calc_crc(unsigned char *ptr, int count) {
  unsigned int crc,i; 

  crc=0;
  while(--count>=0) {
    crc^=((int)*ptr)<<8; ptr++;
    for(i=0;i<8;i++)
      {
	if(crc&0x8000)
	  crc^=0x1021;
	else crc<<=1;
      }
  }
  
  return crc;
}



void clear_buffer(unsigned char *buf,long bufsize)
{
	long i;
	for(i=0;i<bufsize;i++,buf++)
		*buf=0;
}

void create_socket()
{
	unlink("server_socket");
	server_sockfd=socket(AF_INET, SOCK_STREAM,0);
	
	server_address.sin_family=AF_INET;
	server_address.sin_addr.s_addr=htonl(INADDR_ANY);
	server_address.sin_port=htons(10000);
	server_len=sizeof(server_address);
	
	bind(server_sockfd, (struct sockaddr*)&server_address, server_len);

	// Create a connection queue and initialize readfds to handle input
	// from server_sockfd
	listen(server_sockfd,5);

	printf("Listen queue created... Awaiting clients...\n");	
	FD_ZERO(&readfds);	// Initialize readfds with no fds...
	FD_SET(server_sockfd,&readfds);	// readfds has server_sockfd...
}

int send_frame(int fd,unsigned char cmd,unsigned char dest_id,unsigned char *data,int len)
{
	int i;
	unsigned char frame_2_send[1024];
	Stat *newStat;
	clear_buffer(frame_2_send,1024);
	
	frame_2_send[0]=0xa3; // Our Protocol's SOF
	frame_2_send[1]=0x27; //DBEX_SERVER;
	frame_2_send[2]=dest_id;
	frame_2_send[3]=cmd;
	frame_2_send[4]=((len+6)/256)&0xff;
	frame_2_send[5]=(len+6)&0xff;
	for(i=6;i<(len+6);i++,data++)
	  frame_2_send[i]=*data;

	printf("Size = %d",i);
	int chksum=calc_crc(frame_2_send,len+6);
	frame_2_send[i]=(chksum>>8)&0xff;
	frame_2_send[i+1]=chksum&0xff;

	printf("\nSending... ");
	write(fd,frame_2_send,i+2);
	for(i=0;i<len+8;i++)
		printf("%d,",frame_2_send[i]);
}

int add_client(int fd)
{
	int j;
    client_len=sizeof(client_address);
    client_sockfd=accept(server_sockfd,(struct sockaddr*)&client_address, &client_len);
    FD_SET(client_sockfd, &readfds);
    printf("adding client on fd %d\n",client_sockfd);
    ClientTable[last_entry++].client_fd=client_sockfd;
	for(j=0;j<last_entry;j++)
		printf("\nClient[%d] is at fd=%d ",j,ClientTable[j].client_fd);
       
    return client_sockfd;
}

void remove_client(int fd)
{
	int j,i,debug;
	
  	close(fd);
  	FD_CLR(fd,&readfds);
  	printf("removing client on fd %d\n",fd);
	//DebugStart
	printf("Before removal:");
	for(debug=0;debug<client_present;debug++)
		printf("\nclient_fds[%d]=%d",debug,client_fds[debug]);
	//DebugEnd

	int k;
	for(j=client_present-1,i=0;j>=0;j--,i++)
	{	if(client_fds[j]==fd)	// This is to be removed...
		{
			for(k=last_entry-1;k>j;k--)
			{	client_fds[k-1]=client_fds[k];
				ClientTable[k-1].client_fd=ClientTable[k].client_fd;
				ClientTable[k-1].client_id=ClientTable[k].client_id;
				strcpy(ClientTable[k-1].client_name,ClientTable[k].client_name);
				ClientTable[k-1].read_subscription_flags=ClientTable[k].read_subscription_flags;
				ClientTable[k-1].write_subscription_flags=ClientTable[k].write_subscription_flags;
			}
			last_entry--;
			ClientTable[k].client_fd=ClientTable[k].client_id=0;
			ClientTable[k].client_name[0]=0;
			ClientTable[k].read_subscription_flags=ClientTable[k].write_subscription_flags=0;
			break;
		}
	}
	for(;j<client_present;j++)
	  client_fds[j]=client_fds[j+1];  
	client_present--;
	//DebugStart
	printf("\nAfter removal:");
	for(debug=0;debug<client_present;debug++)	
		printf("\nclient_fds[%d]=%d",debug,client_fds[debug]);
	//DebugEnd
}


void read_client(int fd)
{
	int len,client,i;
	clear_buffer(buf,MAX_BUFSIZE);
	len=read(fd,&buf,MAX_BUFSIZE);
	
	printf("\nrecieved... %s (%d bytes) from fd %d\n",buf,len,fd);
	for(i=0;i<len;i++)
	{
		printf("%x,",buf[i]);
	}

	 Check_Command(fd,len,(unsigned char*)buf);
	// Send a frame for the heck of it! ;)

	//send_frame(fd,NewStatus,0x26,(unsigned char*)"",0);
/*
	client=find_clientfd(fd);
	if(client==NOT_FOUND) { printf("Client was not found..."); }
	else printf("Client found with fd %d at loc %d",ClientTable[client].client_fd,client);
	get_client_packet(fd,client,buf,len); */
	printf("\nOut of there...");
}

void *listen_thread(void *arg)
{
	struct timeval timeout; int debug;
	
	while(bye==0)
	{
		usleep(10);
		testfds=readfds;
		timeout.tv_sec=1;
		timeout.tv_usec=500000;
		result=select(FD_SETSIZE,&testfds,(fd_set*)0,(fd_set*)0,
			&timeout);
		for(fd=0; fd<FD_SETSIZE; fd++)
		{
		  if(FD_ISSET(fd,&testfds))
		  {
			if(fd==server_sockfd)
			{
			   client_fds[client_present++]=add_client(fd);
   			  // write_client=client_present-1; // New connection gets data...
			   //StartDebug
			   for(debug=0;debug<client_present;debug++)
				printf("\nclient_fds[%d]=%d",debug,client_fds[debug]);
			   //EndDebug                                          
			}
		  	else
			{
			  ioctl(fd,FIONREAD,&nread);
			  if(nread==0)
			  {
				remove_client(fd);
//				write_client=client_present-1;
			  }
			  else { read_client(fd); display_once=0; }
			}
		   }
		}
	}
	if(bye)
	  printf("Thread is exiting...");
	pthread_exit(NULL);
}
