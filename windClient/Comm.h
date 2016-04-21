/********************************************************************
	filename: 	Comm.h
	file path:	HEAD\cpp
	author:		Ilya Eckstein, Arvind Pereira.
*********************************************************************/

#ifndef __COMM_H__
#define __COMM_H__

#include <pthread.h>

#define STD_DATA_LEN	296


class IComm
{
public: 
	virtual int send_frame(unsigned char cmd,unsigned char dest_id,
						   unsigned char *data,int len) = 0;
	virtual int receive_frame(int tokenNo, unsigned char *data) = 0;
	virtual int register_buffer(unsigned char, int write_len=STD_DATA_LEN, int read_len=STD_DATA_LEN) = 0;
	virtual void clear_buffer(unsigned char *buf,long bufsize) = 0;
	virtual void exit() = 0;
};



class Streamable
{
public: 
	virtual void read(IComm& stream, int desc);
	virtual void write(IComm& stream, int datatype) const;
};


class Runnable
{
public:

	virtual bool start(); 
	void stop() { m_stopFlag = 0; } 

	
protected:  // data
	bool m_stopFlag;
	pthread_t m_threadObj; 


	
protected:  // methods
	virtual void run() = 0; 
	static void* entryPoint(void* pThis); 
};


class Service 
{
public: 
	virtual void turnOn() = 0; 
	virtual void turnOff() {}
	virtual void reset() {}
	
	virtual int subscribe(IComm &client) = 0; 

protected: // data
	int m_commToken; 
	IComm* m_pDbex; 
};


#endif // __COMM_H__

