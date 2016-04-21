// Author: Arvind Antonio de Menezes Pereira
// This file is written to reorder data types depending upon the machine endian-ness
#include <string.h>

unsigned char *store_double(double num,unsigned char *ptr,int reverse)
{
	// If reverse is set then store the byte in reverse order...
	unsigned char *num_ptr,buf[10];
	int i;
	num_ptr=(unsigned char*)&num;
	if(reverse==1)
	{  for(i=sizeof(double)-1;i>=0;i--)
		buf[i]=*(num_ptr++); }
	else if(reverse==0)
	{ for(i=0;i<sizeof(double);i++)
		buf[i]=*(num_ptr++);
	}
	else if(reverse==2) // If we are using int-endian storage...
	{
		for(i=4;i<sizeof(double);i++)
			buf[i]=*(num_ptr++);
		for(i=0;i<4;i++)
			buf[i]=*(num_ptr++);
	}
	memcpy(ptr,buf,sizeof(double));
}

double read_reverse_double(unsigned char *ptr)
{
	unsigned char buf[10]; int i;
	for(i=sizeof(double)-1;i>=0;i--)
		buf[i]=*(ptr++);
	return (*(double*)buf);
}


unsigned char *store_float(float num,unsigned char *ptr,int reverse)
{
	unsigned char *num_ptr,buf[10];
	int i;
	num_ptr=(unsigned char*)&num;
	if(reverse)
	  for(i=sizeof(float)-1;i>=0;i--)
		buf[i]=*(num_ptr++);
	else for(i=0;i<sizeof(float);i++)
		buf[i]=*(num_ptr++);
	
	memcpy(ptr,buf,sizeof(float));
}

float read_reverse_float(unsigned char *ptr)
{
	unsigned char buf[10]; int i;
	for( i=sizeof(float)-1;i>=0;i--)
		buf[i]=*(ptr++);
	return (*(float*)buf);
}

unsigned char *store_int(int num, unsigned char *ptr, int reverse)
{
	unsigned char *num_ptr,buf[10]; int i;
	num_ptr=(unsigned char*)&num;
	if(reverse)
		for(i=sizeof(int)-1;i>=0;i--)
			buf[i]=*(num_ptr++);
	else for(i=0;i<sizeof(int);i++)
		buf[i]=*(num_ptr++);
	memcpy(ptr,buf,sizeof(float));
}

int read_reverse_int(unsigned char *ptr)
{
	unsigned char buf[10]; int i;
	for( i=sizeof(int)-1;i>=0;i--)
		buf[i]=*(ptr++);
	return (*(int*)buf);
}


unsigned char *store_long(long int num, unsigned char *ptr, int reverse)
{
	unsigned char *num_ptr,buf[10]; int i;
	num_ptr=(unsigned char*)&num;
	if(reverse)
		for(i=sizeof(long int)-1; i>=0; i--)
			buf[i]=*(num_ptr++);
	else for(i=0;i<sizeof(int);i++)
		buf[i]=*(num_ptr++);
	memcpy(ptr,buf,sizeof(float));
}

long int read_reverse_long(unsigned char *ptr)
{
	unsigned char buf[10]; int i;
	for( i=sizeof(long int)-1;i>=0;i--)
		buf[i]=*(ptr++);
	return (*(long int*)buf);
}
