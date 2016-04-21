#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/shm.h>
#include <sys/ipc.h>
#include "tcm.h"
#include "flight_time.h"
#include "flight_serial.h"
#include "do_share.h"

char *usage = 
"Usage: tcm -s <serial port> -v <verbosity>\n";


static int serial;
static struct termios old_serial_cfg;
static int found_header = 0;
tcm_compass_t tcm_data;
static int verbose = 0;
static int kill_tcm = 0;
static char device[PATH_MAX] = "/dev/ttyUSB0";

char dir_name[200],full_name[300],log_directory[100]="/home/arvind/";
time_t t;
struct tm *tm;
int len;
struct timeval tv;
struct sigaction sa;
FILE *logFile;
void *shared_memory = (void *) 0;
Q_SHARED *shared;
int shmid;

void createDirectory()
{
 	sprintf(full_name, "%s%%Y%%m%%d/", log_directory);
  	time(&t);
  	tm = localtime(&t);
  	len = strftime(dir_name, sizeof(dir_name) - 1, full_name, tm);
  	if(-1 == mkdir(dir_name, 0700))
	{
		if(errno != EEXIST)
		{
			perror("imu reader unable to create directory");
		}
  	}
}

int openTCMlogfile()
{
	char file_name[100];
	gettimeofday(&tv,NULL);
	strncpy(full_name, dir_name, sizeof(full_name)-1);
	sprintf(file_name,"TCM%ld",tv.tv_sec);
	strcat(full_name,file_name);
	if ((logFile = fopen(full_name, "wb")) == NULL)
	{
		fprintf(stderr, "cannot open %s for binary file", full_name);
		perror("and the reason is");
		// Close port and quit...	
                return 0;
	}
	else return 1;
}



void attach_memory(int key)
{
	shmid = shmget ((key_t)key, sizeof(Q_SHARED), 0666| IPC_CREAT);

	if(shmid==-1) {
		printf("shmget failure\n");
		exit(EXIT_FAILURE);
	}

	shared_memory = shmat(shmid, (void*)0,0);
	if(shared_memory == (void*)-1) {
		printf("shmat failure");
		exit(EXIT_FAILURE); }
	
		printf("Memory attached at %X\n",(int)shared_memory);
	shared = (Q_SHARED*)shared_memory;
	sleep(1);
}



void process_arguments(int argc, char *argv[])
{
	int c, errflag = 0;
	while((c = getopt(argc, argv, "v:s:")) != -1){
		switch(c){
			case 'v':
				verbose = atoi(optarg);
				fprintf(stderr, "verbose = %d\n",verbose);
			break;
			case 's':
				strcpy(device, optarg);
				fprintf(stderr, "device = %s\n",device);
			break;
			case '?':
			default:
				++errflag;
				printf(usage);
				exit(-1);
			break;
		}
	}
}

void kill_tcm_compass(int p)
{
	fprintf(stderr, "CTRL+C killing tcm compass server\n");
	kill_tcm = 1;
	close_tcm_compass();
	exit(-1);
}



int init_tcm_compass(tcm_config_t *cfg){
	serial = serial_open(cfg->device, cfg->speed, &old_serial_cfg);
	if(serial < 0){
		fprintf(stderr, "[Error] failed to open \"%s\" !\n", cfg->device);
		return -1;
	}
	return serial;
}

int close_tcm_compass(void)
{
	return serial_close(serial, &old_serial_cfg);
}


unsigned char chekcsum (unsigned char *buf)
{
	int i;
	unsigned char cksum = 0;
	for(i = 0; i < 15; i++){
		cksum ^= buf[i];
	}
	return cksum;
}

int read_tcm_compass(void)
{
	unsigned char data[1024];
	int rv = -1;

	rv = read_tcm_packet(data);
	return rv;
}

int read_tcm_packet(unsigned char packet[])
{
   	unsigned char cksum, c_cksum;
	char *ep;
	int i = 0;
	char *p = (char *) packet;
	static long old_sec, time_diff;
	char lineOfData[1024];

	memset((void *)&tcm_data, '\0', sizeof(tcm_compass_t));
	while(packet[0] != '$'){
		serial_readn(serial, (char *)packet, 1, 0.01);
	}
	do{
		i++;
   		serial_readn(serial, (char*)&packet[i],1, 0.1);
		cksum ^= packet[i];
	}while(packet[i] != '*');sprintf(lineOfData,"%ld%06ld,%.3f,%.3f,%.3f\n",tv.tv_sec, tv.tv_usec,
				shared->SensorData.TCMroll, shared->SensorData.TCMpitch, shared->SensorData.TCMyaw);
	serial_readn(serial, (char *)packet+1, 1, 0.01);

	gettimeofday(&tv, NULL);

	
   	//cksum = packet[15];
	for(p = packet; *p; p = ep){
		switch(*p) {
		case 'C':
	               tcm_data.heading = strtod(p+1, &ep); shared->SensorData.TCMyaw = tcm_data.heading;
	        break;
	        case 'P':
	               tcm_data.pitch = strtod(p+1, &ep); shared->SensorData.TCMpitch = tcm_data.pitch;
	        break;
	        case 'R':
	               tcm_data.roll = strtod(p+1, &ep); shared->SensorData.TCMroll = tcm_data.roll;
	        break;
	        case 'X':
	               tcm_data.x = strtod(p+1, &ep);
	        break;
	        case 'Y':
	               tcm_data.y = strtod(p+1, &ep);
	        break;
	        case 'Z':
	               tcm_data.z = strtod(p+1, &ep);
	        break;
	        case 'T':
	               if (p[5] == 'E')
		               p[5] = 'Q';
	       
	               tcm_data.t = strtod(p+1, &ep);
	       
	               if (p[5] == 'Q')
		               p[5] = 'E';
	        break;
	        case 'E':
	               tcm_data.e = (unsigned short) strtol(p+1, &ep, 16);
	        break;
	        case '*':
	               c_cksum= strtol(p+1, &ep, 16);
	        break;
	        default:
	               ep = p+1;
	        break;
	        }
	}

	sprintf(lineOfData,"%ld%06ld,%.3f,%.3f,%.3f\n",tv.tv_sec, tv.tv_usec,
				shared->SensorData.TCMroll, shared->SensorData.TCMpitch, shared->SensorData.TCMyaw);

	time_diff=tv.tv_sec-old_sec;
	if(time_diff>1)
	{
	#ifndef _QUIET_
		printf("Writing : %s\n",lineOfData);
	#endif
		old_sec=tv.tv_sec;
	}
        fwrite((const char *)lineOfData,1,strlen(lineOfData), logFile);


	if(c_cksum != cksum){
		tcm_data.new_data = -1;	
		return -1;
	}
	else{
		tcm_data.new_data = 1;	
		tcm_data.timestamp = time_current_fp();
		return 0;
	}

}

int main(int argc, char *argv[])
{
	const char *program = argv[0];
	struct sigaction sa;
	tcm_config_t cfg;
	int tcm_compass_fd = -1;

	createDirectory();
	if(!openTCMlogfile())
		exit(-1);

	attach_memory(QBOATShareKey);
	sleep(2);
	
	memset(&sa, 0, sizeof(sa));
	sa. sa_handler = &kill_tcm_compass;
	sigaction(SIGINT, &sa, NULL);

	process_arguments(argc, argv);

	strcpy(cfg.device, device);
	cfg.speed = DEFAULT_COMPASS_SPEED;

	tcm_compass_fd = init_tcm_compass(&cfg);
	if(tcm_compass_fd < 0){
		fprintf(stderr, "failed to initialize TCM COMPASS\n");
		return -1;
	}else
		fprintf(stderr, "TCM COMPASS initialized\n");

	while(!kill_tcm){
		read_tcm_compass();
		
		if(verbose == 1){
			fprintf(stderr, "%8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n",
					tcm_data.heading, tcm_data.roll, tcm_data.pitch, tcm_data.x, tcm_data.y, tcm_data.z);
		}
	}
	close_tcm_compass();
	return 0;
}
