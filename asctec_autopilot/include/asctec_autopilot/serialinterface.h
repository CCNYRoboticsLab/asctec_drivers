/*
 *  AscTec Autopilot Serial Interface
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *  William Morris <morris@ee.ccny.cuny.edu>
 *  http://robotics.ccny.cuny.edu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ASCTEC_AUTOPILOT_SERIALINTERFACE_H
#define ASCTEC_AUTOPILOT_SERIALINTERFACE_H

#include <stdio.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <time.h>
#include <errno.h>
#include <bitset>

#include <ros/ros.h>

#include "asctec_autopilot/crc16.h"
#include "asctec_autopilot/telemetry.h"

namespace asctec
{
  class SerialInterface
  {
  public:
    SerialInterface (std::string port, uint32_t speed);
    ~SerialInterface ();

    void output (char *output, int len);
    void output (unsigned char *output, int len);
    bool getPackets (Telemetry *telemetry);
    void sendControl (Telemetry *telemetry);
    void sendEstop (Telemetry *telemetry);
    void dumpDebug (void);
    bool getPacket (char *spacket, unsigned char &packet_type, unsigned short &packet_crc, unsigned short &packet_size);

    uint32_t serialport_bytes_rx_;
    uint32_t serialport_bytes_tx_;
    int *scan;
    bool status;
    int pt[800];
    int counter;
  private:
      speed_t bitrate (int);
    void flush ();
    void drain ();
    void stall (bool);
    int wait (int);

    int dev_;
    std::string serialport_name_;
    uint32_t serialport_speed_;
    speed_t serialport_baud_;
  };
}
#endif
