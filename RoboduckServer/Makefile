CC = g++

CFLAGS = \
	-g \
	-D_THREAD_SAFE \
	-D_REENTRANT

COMMONDIR = ../common/

DBEXINC = \
	-I./ \
	-I${COMMONDIR} \

DBEX_OBJS = \
	circularbuf.o \
	byteorder.o \
	CommProtS.o \
	mission.o \
	databank.o \
	server.o \
	do_share.o \
	utm.o \
	sysIdent.o \
	serial.o

LIBS =	-lpthread -lm -lmstimer

LIBPATH = \
	-L${COMMONDIR}

all : dbex

dbex : ${DBEX_OBJS}
	${CC} ${CFLAGS} -o $@ ${DBEX_OBJS} ${LIBPATH} ${LIBS} 

clean:
	rm -f ${DBEX_OBJS}
	find . -type d -name SunWS_cache | xargs rm -rf
	rm -f dbex


.SUFFIXES : .o .c
.c.o :
	$(CC) $(CFLAGS) -o $@  ${DBEXINC} -c $<
