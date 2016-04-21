# Common Makefile that will make the code in all the directories...
# Author: Arvind Pereira.
#
#
#

INCLUDEDIR = include
COMMONDIR  = common
#IMUDIR	   = 3dmgReader
GPSDIR	   = gpsReader
#AX1500DIR  = ax1500Reader
SERVERDIR  = RoboduckServer
# CAMCDIR    = camClient
KALMANDIR  = eKalmanFilter
RDUCKDIR   = roboduck
XBOWDIR	   = xbowReader
# STEREODIR  = StereoCode
RUDDERDIR  = rudderControl

SUBDIRS = $(COMMONDIR) $(KALMANDIR) $(IMUDIR) $(GPSDIR) $(AX1500DIR) $(SERVERDIR) $(CAMCDIR) $(RDUCKDIR) $(STEREODIR) $(XBOWDIR) $(RUDDERDIR)


# Make everything
#

all :
	for i in $(SUBDIRS) ; do \
		(cd $$i ; make ) ; \
	done

common:
	@cd $(COMMONDIR) ; make

3dmgReader:
	@cd $(IMUDIR)	; make

kalman:
	@cd $(KALMANDIR) ; make

gpsReader:
	@cd $(GPSDIR) ; make

dbex:
	@cd $(SERVERDIR) ; make

roboduck:
	@cd $(RDUCKDIR) ; make

stereo:
	@cd $(STEREODIR) ; make

xbowReader:
	@cd $(XBOWDIR) ; make

rudder:
	@cd $(RUDDERDIR) ; make

clean:
	rm -f *~;
	for i in $(SUBDIRS) ; do \
		( cd $$i ; make clean ); \
	done
