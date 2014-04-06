//#include <progmem.h>
//#include <EEPROM.h>
#include <FastSerial.h>
#include <AP_Common.h>
//#include <AP_Math.h>
#include <GCS_MAVLink.h>

#include <ArduIllusion.h>

#define _VERSION "V0.1"

#define gaugePort Serial3
#define mavlPort Serial2

FIGaugeSet gaugeSet;

FastSerialPort0(Serial);
FastSerialPort1(Serial1);
FastSerialPort2(mavlPort);
FastSerialPort3(gaugePort);


#define toRad(x) (x*PI)/180.0
#define toDeg(x) (x*180.0)/PI

unsigned long hb_timer;
unsigned long timer;

// data streams active and rates
#define MAV_DATA_STREAM_POSITION_ACTIVE 1
#define MAV_DATA_STREAM_RAW_SENSORS_ACTIVE 1
#define MAV_DATA_STREAM_EXTENDED_STATUS_ACTIVE 1
#define MAV_DATA_STREAM_RAW_CONTROLLER_ACTIVE 0
#define MAV_DATA_STREAM_EXTRA1_ACTIVE 1
#define MAV_DATA_STREAM_EXTRA2_ACTIVE 1 //VFR_HUD

// update rate is times per second (hz)
#define MAV_DATA_STREAM_POSITION_RATE 2
#define MAV_DATA_STREAM_RAW_SENSORS_RATE 1
#define MAV_DATA_STREAM_EXTENDED_STATUS_RATE 2
#define MAV_DATA_STREAM_RAW_CONTROLLER_RATE 1
#define MAV_DATA_STREAM_EXTRA1_RATE 1
#define MAV_DATA_STREAM_EXTRA2_RATE 4

// APM custom modes in heartbeat package
#define CUST_MODE_PLANE_MANUAL 0
#define CUST_MODE_PLANE_CIRCLE 1
#define CUST_MODE_PLANE_STABILIZE 2
#define CUST_MODE_PLANE_TRAINING 3
#define CUST_MODE_PLANE_FLY_BY_WIRE_A 5
#define CUST_MODE_PLANE_FLY_BY_WIRE_B 6
#define CUST_MODE_PLANE_AUTO 10
#define CUST_MODE_PLANE_RTL 11
#define CUST_MODE_PLANE_LOITER 12
#define CUST_MODE_PLANE_GUIDED 15
#define CUST_MODE_PLANE_INITIALISING 16

unsigned long Distance_Home=0;
unsigned long Distance3D_Home=0;
int Angle_Home=0;
int Constrain_Angle_Home = 0;
unsigned long Bearing_Home=0;
unsigned long SvBearingHome = 0;
float offset = 0;
int beat=0;

// Flight data
unsigned char GCS_UNITS=2;      // Display units, 0=metric, 1=imperial land, 2=imperial nautic

float pitch=0;       // Pitch angle
float roll=0;        // Roll angle
float yaw=0;         // Yaw angle
long altitude=0;     // Altitude ( --> Find out if GPS or baro!)
unsigned int ias=0;  // Indicated air speed
unsigned int grs=0;  // Ground speed
int heading=0;       // Heading (not course over ground!)
unsigned int cog=0;  // Course over ground (from GPS)
int vsi=0;           // Vertical speed

// Technical data
uint8_t received_sysid=0;   // ID of heartbeat sender
uint8_t received_compid=0;  // component id of heartbeat sender
int uavtype=0;              // type of the UAV
int bmode=0;                // MAVLink basic mode
int cmode=0;                // MAVLink custom mode
int gpsfix=0;               // GPS fix status, 0 = no fix, 1 = dead reckoning, 2 = 2D-fix, 3 = 3D-fix
int numSats=0;              // Number of satellites used in position fix
unsigned int vbat=0;        // battery voltage
unsigned long mav_utime=0;  // ??

// Ground station stuff
int status_mavlink=0; // Changes to 1 when a valid MAVLink package was received, 0 when no package or an invalid package was received

void setup() {
  Serial.begin(57600);
  mavlPort.begin(57600);
  gaugePort.begin(38400);
  gaugeSet.Init();
  Serial.println("Started");
}

void loop() {
  if (gcs_update()) {                  // Only update screen when a valid MAVLink package was received
    gaugeSet.setRoll(roll);
    gaugeSet.setPitch(pitch);
  }
}

boolean gcs_update()
{
    boolean result = false;
    status_mavlink=0;
    mavlink_message_t msg;
    mavlink_status_t status;

    while (mavlPort.available())
    {
      char c = mavlPort.read();
      if(mavlink_parse_char(0, c, &msg, &status)) {
        gcs_handleMessage(&msg);
        status_mavlink = 1;
        result=true;
      }
    }
    return result;
}


int freeRam() {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}



