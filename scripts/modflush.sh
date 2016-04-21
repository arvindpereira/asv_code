#!/bin/sh

rmmod sbp2
rmmod video1394
rmmod raw1394
rmmod ohci1394
rmmod ieee1394


sleep 0.2
modprobe video1394
modprobe raw1394

sleep 0.2

if [ -c "/dev/video1394-0" ]; then
   mkdir /dev/video1394
   ln -s /dev/video1394-0 /dev/video1394/0
fi

if [ -c "/dev/video1394-1" ]; then
   mkdir /dev/video1394
   ln -s /dev/video1394-1 /dev/video1394/1
fi

if [ -c "/dev/video1394/0" ]; then
   chmod a+rw /dev/video1394/0
fi
if [ -c "/dev/video1394/1" ]; then
   chmod a+rw /dev/video1394/1
fi
if [ -c "/dev/video1394/2" ]; then
   chmod a+rw /dev/video1394/2
fi
if [ -c "/dev/video1394/3" ]; then
   chmod a+rw /dev/video1394/3
fi

if [ -c "/dev/video1394-0" ]; then
   chmod a+rw /dev/video1394-0
fi
if [ -c "/dev/video1394-1" ]; then
   chmod a+rw /dev/video1394-1
fi
if [ -c "/dev/video1394-2" ]; then
   chmod a+rw /dev/video1394-2
fi
if [ -c "/dev/video1394-3" ]; then
   chmod a+rw /dev/video1394-3
fi

if [ -c "/dev/raw1394" ]; then
   chmod a+rw /dev/raw1394
fi

