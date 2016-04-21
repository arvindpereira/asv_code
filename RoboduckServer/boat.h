#ifndef _BOAT__H_
 #define _BOAT__H_

#include <stdio.h>

#define PORT_S0 	"/dev/ttyS0" 
#define PORT_S1 	"/dev/ttyS1" 
#define PORT_S2 	"/dev/ttyS2" 
#define PORT_S3 	"/dev/ttyS3"

#define PORT_USB0 	"/dev/ttyUSB0" 
#define PORT_USB1 	"/dev/ttyUSB1" 
#define PORT_USB2 	"/dev/ttyUSB2" 
#define PORT_USB3 	"/dev/ttyUSB3"

#define PORT_USB_S0 	"/dev/usb/tts/0" 
#define PORT_USB_S1 	"/dev/usb/tts/1" 
#define PORT_USB_S2 	"/dev/usb/tts/2" 
#define PORT_USB_S3 	"/dev/usb/tts/3"


#define WINCH_PORT   PORT_USB3
#define GPS_PORT     PORT_S2
#define BATTERY_PORT PORT_USB3

struct timelog {
  int hh;
  int mm;
  int ss;
  int cc;
};

typedef struct{
  double chl;
  double temp;
  double lat;
  double lon;
  double heading;
  double speed;
  double direction;
  double targetLat;
  double targetLon;
  double time;
  double sats;
  double dgps;
  double winch_depth;
  double sonde_time;
  double sonde_flo;
  double sonde_temp;
  double sonde_do;
  double sonde_ph;
  double sonde_battery_voltage;
  double boat_battery_voltage;
}Stat;

// Stat test;

typedef struct {
  int battery;
  int gps;
  int imu;
} Monitor;

// Monitor monitor;

#endif
