CC = gcc 
LD = $(CC)

CFLAGS = -c -O2 

SRCS = tcm.c
HDRS = tcm.h
OBJS = isisimu.o tcm.o rt2.o kalman.o query-state.o
INCLUDES = -I../common -I.. -I. 
COMMONDIR = ../common

LDFLAGS = -L$(COMMONDIR)
INSTALLDIR = ../../bin
all: tcm

tcm: tcm.c tcm.h
	$(CC) $(CFLAGS) $(INCLUDES) tcm.c
	$(CC) $(LDFLAGS) tcm.o -lcommon -o tcm

install:
	make; cp tcm $(INSTALLDIR)
clean:
	rm -f $(OBJS) tcm *~
