#include <stdio.h>
#include <fstream.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <Comm.h>
#include <ClientComm.h>
#include <do_share.h>

extern Q_SHARED *shared;
extern DIAG_SHARED *sharedDIAG; 
using namespace std;
main() {
	int i=0;
	cout<<"Hello World!!!!";

	ClientComm windServerConn("localhost",10000,1);
	attach_mem();

	while(1) {
		sleep(1);
		cout<<++i;
	}

}
