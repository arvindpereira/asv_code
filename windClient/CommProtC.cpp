// This file contains the send and receive functions that shall handle the communication
// in the DbeX framework.
// 
#include <iostream>
#define _COMMPROTC_CPP_
#include "CommProtC.h"
#include "byteorder.h"
#include "Comm.h"

/*
extern double CovMatrix_Meas[6][6]; 
extern double CovMatrix_Pred[6][6]; 
extern double CovMatrix_Updt[6][6]; 
extern double CovMatrix_Truth[6][6];
// Due to the subscription service, semaphores will not be required on reads!!!

extern double State_Meas[6];
extern double State_Pred[6];
extern double State_Updt[6];
extern double State_Truth[6];
*/

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

unsigned int CalcChksum(unsigned char frame[])
{
	unsigned int i,sum= 0xa5;		// An offset so that a frame with only zeroes
	if(((unsigned)frame[4]*256+frame[5])>1023 ||((unsigned)frame[4]*256+frame[5])<6 ) 
	{ printf("\n\n\nInvalid size. Error"); return 0; }
	else
	for(i=0;i<((frame[4]<<8)+frame[5]);i++) // LOF should be frame[3]&frame[4]
	{					   // it does not include chksum byte
		sum+=frame[i];
	}
	return sum&0xffff;
}
 


void display_matrix(double mat[6][6])
{
	for(int i=0,j;i<6;i++)
	{
		printf("\n");
		for(j=0;j<6;j++)
			printf(" %f ",mat[i][j]);
	}
	printf("\n");	

}


void zero_all(unsigned char frame[], int len)
{
	for(int i=0; i<len; i++)
		frame[i]=0;
}
