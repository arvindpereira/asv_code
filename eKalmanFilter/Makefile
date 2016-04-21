# $Author: srik $
# $Name:  $
# $Id: Makefile 4 2005-07-29 01:52:46Z srik $
# $Source: /home/sagan-01/srik/CVSROOT/heli-sim/state/Makefile,v $
# $Log: Makefile,v $
# Revision 1.1.1.1  2004/03/03 11:03:06  srik
# Initial Release 0.1
#
#

CC = g++ 
LD = $(CC)

CFLAGS = -c -O2 
LDFLAGS =  

SRCS = gpsins.c matrix.c quat.c utils.c
SRCKALMAN = kalman.cpp
INCLUDES = gpsins.h matrix.h quat.h utils.h
OBJS = gpsins.o matrix.o quat.o utils.o
LIBINS = libins.a
KALMAN = kalman
INCLUDEDIR = -I../flight -I../common
INSTALLDIR = ../../bin
all: $(LIBINS) $(KALMAN)
$(LIBINS): $(SRCS) $(HDRS)
	$(CC) $(CFLAGS) $(INCLUDEDIR) gpsins.c
	$(CC) $(CFLAGS) $(INCLUDEDIR) matrix.c
	$(CC) $(CFLAGS) $(INCLUDEDIR) quat.c
	$(CC) $(CFLAGS) $(INCLUDEDIR) utils.c
	ar -cr $(LIBINS) $(OBJS)
	ranlib $(LIBINS)

$(KALMAN): $(SRCKALMAN) $(HDRS)
	$(CC) $(CFLAGS) $(INCLUDEDIR) kalman.cpp
	$(CC) -o $(KALMAN) $(KALMAN).o -L . -L ../common -lins -lm -lmstimer
	

install:
	make; cp $(LIBINS) $(INSTALLDIR)
clean:
	rm -f $(OBJS) $(LIBINS) *~
