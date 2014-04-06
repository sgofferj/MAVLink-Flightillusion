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

#define GAUGE_ATTI 103

#define CMD_RESET 1
#define CMD_SETSPEED 3
#define CMD_SETROLL 4
#define CMD_SETPITCH 5
#define CMD_SETLIGHT 8


class FIGaugeSet {
  public:
    FIGaugeSet();
    void Init();
    void setLight(char gauge, char light);
    void setRoll(float angle);
    void setPitch(float angle);
  private:
    void sendCommand(char id, char cmd, long value);
};
#endif