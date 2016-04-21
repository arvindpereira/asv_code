#ifndef _BYTEORDER_H_
	#define _BYTEORDER_H_

extern char *store_double(double num,unsigned char *ptr,int reverse);
extern unsigned char *store_float(float num,unsigned char *ptr,int reverse);
extern unsigned char *store_int(int num, unsigned char *ptr, int reverse);
extern unsigned char *store_long(long int num, unsigned char *ptr, int reverse);
extern float read_reverse_float(unsigned char *ptr);
extern double read_reverse_double(unsigned char *ptr);
extern long int read_reverse_long(unsigned char *ptr);
extern int read_reverse_int(unsigned char *ptr);

#endif
