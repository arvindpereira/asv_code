#ifndef __SERVER_H_
	#define __SERVER_H_
extern void *listen_thread(void *arg);
extern void read_client(int fd);
extern void clear_buffer(unsigned char *buf,long bufsize);
extern void remove_client(int fd);
extern int add_client(int fd);
extern void create_socket();
extern int send_double(int fd,int num);
extern int send_frame(int fd,unsigned char cmd,unsigned char dest_id,unsigned char *data,int len);

#define MAX_BUFSIZE	4096

#endif
