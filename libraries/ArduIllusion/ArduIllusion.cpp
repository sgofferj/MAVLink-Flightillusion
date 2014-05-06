//
// Library for controlling FLight Illusion sim gauges
//
// Copyright (c) 2013 Stefan Gofferje. All rights reserved.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later
// version.
//
// This library is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
// PURPOSE. See the GNU Lesser General Public License for more
// details.
//
// You should have received a copy of the GNU Lesser General
// Public License along with this library; if not, write to the
// Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
// Boston, MA 02110-1301 USA
//

#include <FastSerial.h>
#include <ArduIllusion.h>

// ----- Constructor ---------------------------------------------------------
FIGaugeSet::FIGaugeSet() {
  
}
// ----- Private -------------------------------------------------------------
void FIGaugeSet::sendCommand(byte id, byte cmd, long value) {
  byte buffer[]={0x00,0x00,0x00,0x00,0x00,0x00};
  byte bytethree, int2, cnt;
  long long1;
  buffer[1]=id;      // Gauge ID
  buffer[5]=0xff;    // Terminator
  
  long1 = abs(value);
  int2 = long1 >> 8;
  
  buffer[3]=(long1 & 0xff) | 0x01;
  buffer[4]=(int2 & 0xff) | 0x02;

  bytethree = cmd << 4;
  bytethree |= (long1 & 0x01);
  bytethree |= (int2 & 0x02);
  if (value < 0) bytethree |= 0x0C;
  else bytethree |= 0x08; 

  buffer[2]=bytethree & 0xff;
  
  for (cnt=0;cnt<=5;cnt++) {
    gaugePort.write(buffer[cnt]);
  }
}

// ----- Public --------------------------------------------------------------

// ----- General functions ---------------------------------------------------

void FIGaugeSet::Init(byte gauge) {
  sendCommand(gauge,CMD_RESET,0);
}

void FIGaugeSet::Query(byte gauge) {
  sendCommand(gauge,CMD_QUERY,0);
}

void FIGaugeSet::setLight(byte gauge, byte light) {
  sendCommand(gauge,CMD_SETLIGHT,light);
}

// ----- GSA-34, GSA-35 Attitude indicators ----------------------------------

void FIGaugeSet::gsa34_setSpeed(byte roll, byte pitch) {
  sendCommand(GSA34_ID,CMD_SETSPEED,roll);
  sendCommand(GSA34_ID,CMD_SETSPEED,-1*pitch);
}
  
void FIGaugeSet::gsa34_setRoll(long angle) {
  sendCommand(GSA34_ID,GSA34_CMD_SETROLL,(360-(angle+rollOffset))*rollSteps);
}

void FIGaugeSet::gsa34_setPitch(long angle) {
  if ( (angle >= -20) && (angle <= 20) ) {
    sendCommand(GSA34_ID,GSA34_CMD_SETPITCH,((-1*angle*pitchSteps)+pitchOffset));
  }
}
  
// ----- GSA-16 Digital Altimeter ---------------------------------------------

void FIGaugeSet::gsa16_setSpeed(byte speed) {
  sendCommand(GSA16_ID,CMD_SETSPEED,speed);
}

void FIGaugeSet::gsa16_setAltitude(long altitude) {
  if ( (altitude >= -9999) && (altitude <= 50000) ) {
    sendCommand(GSA16_ID,GSA16_CMD_SETALT,altitude);
  }
}

void FIGaugeSet::gsa16_setPressure(long pressure) {
  sendCommand(GSA16_ID,GSA16_CMD_SETPRES,pressure);
}

void FIGaugeSet::gsa16_setPressureMode(byte unit, byte control) {
  long value = (control << 8) + unit;
  sendCommand(GSA16_ID,GSA16_CMD_SETPRMOD,value);
}

void FIGaugeSet::gsa16_setIntensity(byte altitude, byte pressure, bool night) {
  long value = 0;
  if (night) value = 16384;
  altitude &= 31;
  pressure &= 31;
  value += (altitude << 8) + pressure;
  sendCommand(GSA16_ID,GSA16_CMD_SETINTENS,value);
  Serial.println(value);
}

  
  