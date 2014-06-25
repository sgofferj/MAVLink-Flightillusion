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

#ifndef ArduIllusion_h
#define ArduIllusion_h

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#ifndef gaugePort
#define gaugePort Serial3
#endif

#define rollSteps 10.667
#define rollOffset 180
#define pitchSteps 16
#define pitchOffset 320
#define gyroSteps 12
#define gyroOffset 0

#define GSA16_ID 101
#define GSA34_ID 103
#define GSA40_ID 105
#define GSA72_ID 109
#define GSA20_ID 110

#define CMD_RESET 1
#define CMD_SETADR 2
#define CMD_SETSPEED 3
#define CMD_QUERY 7
#define CMD_SETLIGHT 8
#define GSA34_CMD_SETROLL 4
#define GSA34_CMD_SETPITCH 5
#define GSA16_CMD_SETALT 4
#define GSA16_CMD_SETPRMOD 5
#define GSA16_CMD_SETPRES 12
#define GSA16_CMD_SETINTENS 10
#define GSA40_CMD_SETDISC 5
#define GSA40_CMD_SETBUG 6
#define GSA40_CMD_SETENCODERMASK 9
#define GSA20_CMD_SETPOS 5
#define GSA72_CMD_SETLOCAL 4
#define GSA72_CMD_SETUTC 5
#define GSA72_CMD_SETFLT 6
#define GSA72_CMD_SETVOLT 9
#define GSA72_CMD_SETTEMPC 11
#define GSA72_CMD_SETTEMPF 12

class FIGaugeSet {
  public:
    FIGaugeSet();

    // ----- General functions --------------------------------------------------------------------------------
    void Init(byte gauge);
    void Query(byte gauge);
    void setLight(byte gauge, byte light);  // Low byte: DL000000, D=display, L=lights
    void setAddress(byte gauge,byte address);
    
    // ----- GSA-34, GSA-35 Attitude indicators ---------------------------------------------------------------
    void gsa34_setSpeed(byte roll, byte pitch);
    void gsa34_setRoll(long angle);
    void gsa34_setPitch(long angle);

    // ----- GSA-16 Digital Altimeter -------------------------------------------------------------------------
    void gsa16_setSpeed(byte speed);
    void gsa16_setAltitude(long altitude);
    void gsa16_setPressure(long pressure);
    void gsa16_setPressureMode(byte unit, byte control);  // unit: 0=inHg, 1=mmHg; control: 0: user, 1: MAV
    void gsa16_setIntensity(byte altitude, byte pressure, bool night);

    // ----- GSA-40 Gyrocompass ---------------------------------------------------
    void gsa40_setSpeed(byte speed);
    void gsa40_setDisc(int degree);
    void gsa40_setBug(int degree);
    void gsa40_setEncoderMask(byte left, byte right);

    // ----- GSA-20 IAS -----------------------------------------------------------
    void gsa20_setSpeed(byte speed);
    void gsa20_setIAS(int mps);
 
    // ----- GSA-72 General Aviation Clock -------------------------------------
    void gsa72_setUTC(byte hours, byte minutes, byte seconds);
    void gsa72_setFLT(unsigned long seconds);
    void gsa72_setFLT(byte hours, byte minutes, byte seconds);
    void gsa72_setLocal(byte hours);
    void gsa72_setTempC(int decicelsius);
    void gsa72_setVolt(long millivolts);
    
  private:
    void sendCommand(byte id, byte cmd, long value);
};
#endif