//#include <progmem.h>
//#include <EEPROM.h>
#include <FastSerial.h>
//#include <AP_Common.h>
//#include <AP_Math.h>
#include <GCS_MAVLink.h>

#include <Wire.h>
#include <RTClib.h>

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
unsigned int gpshdop=0;
int tempcelsius;     // Temperature from Baro
byte utch,utcm,utcs; // Time
byte timezone = +3;
byte localhour = 0;

// Technical data
uint8_t received_sysid=0;   // ID of heartbeat sender
uint8_t received_compid=0;  // component id of heartbeat sender
int uavtype=0;              // type of the UAV
int bmode=0;                // MAVLink basic mode
int cmode=0;                // MAVLink custom mode
int gpsfix=0;               // GPS fix status, 0 = no fix, 1 = dead reckoning, 2 = 2D-fix, 3 = 3D-fix
int numSats=0;              // Number of satellites used in position fix
unsigned int vbat=0;        // battery voltage
long mav_uptime=0;          // ??
long systemtime=0;          // ??
long oldtime=0;


// Ground station stuff
int status_mavlink=0; // Changes to 1 when a valid MAVLink package was received, 0 when no package or an invalid package was received

void setup() {
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  Serial.begin(115200,256,256);
  mavlPort.begin(57600,256,256);
  gaugePort.begin(38400,256,256);
  delay(3000);
  gaugeSet.setLight(101,0); // Alti
  gaugeSet.setLight(103,0); // Horizon
  gaugeSet.setLight(105,0); // Gyro
  gaugeSet.setLight(109,0); // Clock
  delay(1000);
  gaugeSet.gsa16_setPressureMode(0,1);
  gaugeSet.gsa16_setAltitude(0);
  gaugeSet.gsa16_setIntensity(1,1,true);
  gaugeSet.setLight(101,192); // Alti
  gaugeSet.setLight(103,192); // Horizon
  gaugeSet.setLight(105,192); // Gyro
  gaugeSet.setLight(109,192); // Clock
  gaugeSet.gsa34_setSpeed(96,96);
  gaugeSet.gsa40_setSpeed(96);
  digitalWrite(13,LOW);
}

void loop() {
  if (gcs_update()) {                  // Only update screen when a valid MAVLink package was received
    gaugeSet.gsa34_setRoll(roll);
    gaugeSet.gsa34_setPitch(pitch);
    gaugeSet.gsa16_setAltitude(altitude);
    gaugeSet.gsa40_setDisc(heading);
    gaugeSet.gsa40_setBug(Angle_Home);
    gaugeSet.gsa72_setVolt(vbat);
    gaugeSet.gsa72_setTempC(tempcelsius);
    if ( (systemtime > (oldtime+300)) ) {
      gaugeSet.gsa72_setUTC(utch,utcm,utcs);
      localhour = utch + timezone;
      if (localhour > 23) localhour -= 24;
      gaugeSet.gsa72_setLocal(localhour);
      oldtime = systemtime;
    }
    gaugeSet.gsa72_setFLT(mav_uptime);
    if (gpshdop <= 9999) gaugeSet.gsa16_setPressure(gpshdop);
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
        digitalWrite(13,HIGH);
        gcs_handleMessage(&msg);
        status_mavlink = 1;
        result=true;
        digitalWrite(13,LOW);
      }
    }
    return result;
}


int freeRam() {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}



