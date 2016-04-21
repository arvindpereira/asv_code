# $Author: srik $
# $Name:  $
# $Id: Makefile 4 2005-07-29 01:52:46Z srik $
# $Source: /home/sagan-01/srik/CVSROOT/heli-sim/controller/Makefile,v $
# $Log: Makefile,v $
# Revision 1.1.1.1  2004/03/03 11:03:06  srik
# Initial Release 0.1
#
#

CC = gcc 
LD = $(CC)

CFLAGS = -c -O2 

SRCS = flight_serial.c flight_time.c 
HDRS = flight_serial.h flight_time.h
OBJS = flight_serial.o flight_time.o
INCLUDES = -I. -I..
LIBSDIR =
LIBCOMMON = libcommon.a 
LIBTIMER = libmstimer.a
LIBSHM = libshm.a
LIBLOGGER = liblog.a
LDFLAGS = -L$(LIBSDIR) 
INSTALLDIR = ../../bin
all: $(LIBCOMMON)
$(LIBCOMMON): $(SRCS) $(HDRS)
	$(CC) $(CFLAGS) $(INCLUDES) flight_serial.c
	$(CC) $(CFLAGS) $(INCLUDES) flight_time.c
	ar -cr $(LIBCOMMON) $(OBJS)
	ranlib $(LIBCOMMON)
	make -f Makefile.MsTimer
	make -f Makefile.SharedMem
	make -f Makefile.Logger

install:
	make; cp $(LIBCOMMON) $(INSTALLDIR)
clean:
	rm -f $(OBJS) $(LIBCOMMON) $(LIBTIMER) $(LIBSHM) $(LIBLOGGER) *~
