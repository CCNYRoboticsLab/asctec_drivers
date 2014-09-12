/*
 *  AscTec Autopilot Serial Interface
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  William Morris <morris@ee.ccny.cuny.edu>
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *  Steven Bellens <steven.bellens@mech.kuleuven.be>
 *  Patrick Bouffard <bouffard@eecs.berkeley.edu>
 *
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

#include <stdio.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <cstdlib>
#include <time.h>
#include <errno.h>
#include <bitset>

#include <ros/ros.h>

#include "asctec_autopilot/crc16.h"
#include "asctec_autopilot/autopilot.h"
#include "asctec_autopilot/telemetry.h"
#include "asctec_autopilot/serialinterface.h"

// C++ is a horrible version of C
extern "C" {
  #include <unistd.h>
  #include <fcntl.h>
}

namespace asctec
{
  SerialInterface::SerialInterface (std::string port, uint32_t speed):serialport_name_ (port), serialport_speed_ (speed)
  {
    struct termios tio;
      status = false;
      serialport_baud_ = bitrate (serialport_speed_);
      ROS_INFO ("Initializing serial port...");

      dev_ = open(serialport_name_.c_str (),O_RDWR | O_NOCTTY | O_NDELAY);
      ROS_DEBUG ("dev: %d", dev_);
      ROS_ASSERT_MSG (dev_ != -1, "Failed to open serial port: %s %s", serialport_name_.c_str (), strerror (errno));

      ROS_ASSERT_MSG (tcgetattr (dev_, &tio) == 0, "Unknown Error: %s", strerror (errno));

      cfsetispeed (&tio, serialport_baud_);
      cfsetospeed (&tio, serialport_baud_);

      tio.c_iflag = 0;
      tio.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL);
      tio.c_iflag |= IGNBRK;

      tio.c_oflag = 0;
      tio.c_oflag &= ~(OPOST | ONLCR);

      tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;
      tio.c_cflag &= ~(PARENB | CRTSCTS | CSTOPB);

      tio.c_lflag = 0;
      tio.c_lflag |= NOFLSH;
      tio.c_lflag &= ~(ISIG | IEXTEN | ICANON | ECHO | ECHOE);

      ROS_ASSERT_MSG (tcsetattr (dev_, TCSADRAIN, &tio) == 0, "Unknown Error: %s", strerror (errno));

      tio.c_cc[VMIN] = 0;
      tio.c_cc[VTIME] = 0;

      tcflush (dev_, TCIOFLUSH);

      ROS_ASSERT_MSG (dev_ != NULL, "Could not open serial port %s", serialport_name_.c_str ());
      ROS_INFO ("Successfully connected to %s, Baudrate %d\n", serialport_name_.c_str (), serialport_speed_);
  }

  SerialInterface::~SerialInterface ()
  {
    ROS_DEBUG ("Destroying Serial Interface");
    flush ();
    close (dev_);
  }

  void SerialInterface::flush ()
  {
    tcflush (dev_, TCIOFLUSH);
  }

  void SerialInterface::drain ()
  {
    ROS_ASSERT_MSG (tcdrain (dev_) == 0, "Drain Error: %s", strerror (errno));
  }

  int SerialInterface::wait (int bytes_requested)
  {
    int bytes_available=0;
    unsigned int i=0;

    while (bytes_available < bytes_requested)
    {
      ioctl(dev_,FIONREAD,&bytes_available);
      usleep(1);
      if (i>650 && bytes_available < bytes_requested)
      {
        ROS_ERROR("Timeout: %d bytes available %d bytes requested",bytes_available,bytes_requested);
        return bytes_available;
      }
      i++;
    }
    return bytes_available;
  }

  speed_t SerialInterface::bitrate (int Bitrate)
  {
    switch (Bitrate)
    {
      case 9600:
        return B9600;
      case 19200:
        return B19200;
      case 38400:
        return B38400;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
      case 230400:
        return B230400;
      default:                 // invalid bitrate
        return B0;
    }
  }

  bool SerialInterface::getPacket (char *spacket, unsigned char &packet_type, unsigned short &packet_crc,
                                   unsigned short &packet_size)
  {
    char stoken[4];
    char ssize[2];
    char stype[1];
    char scrc[2];
    int bytes;

    int i;

    ROS_DEBUG ("  SerialInterface::getPacket()");
    // get beginning (">*>")
    stoken[0] = '\0';
    stoken[1] = '\0';
    stoken[2] = '\0';
    stoken[3] = '\0';

    wait(3);
    i = read (dev_,stoken, 3);
    if (i == 0 || strncmp (stoken, ">*>", 3) != 0)
    {
      ROS_DEBUG ("    dev: %zd", (size_t) dev_);
      ROS_ERROR ("    Error Reading Packet Header: %s", strerror (errno));
      ROS_ERROR ("    Read (%d): %s", i, stoken);
      //ROS_BREAK();
      //while (read (dev_,stoken, 1) != 0) {}
      flush ();
      return false;
    }
    serialport_bytes_rx_ += 3;
    ROS_DEBUG ("    Packet Header OK");

    // get packet size
    wait(2);
    i = read (dev_,ssize, 2);
    if (i == 0)
    {
      ROS_ERROR ("Error Reading Packet Size: %s", strerror (errno));
      flush ();
      return false;
    }
    serialport_bytes_rx_ += 2;
    memcpy (&packet_size, ssize, sizeof (packet_size));
    //ROS_DEBUG ("Packet size: %d", packet_size);

    // get packet type
    wait(1);
    i = read (dev_, stype, 1);
    if (i == 0)
      return false;
    serialport_bytes_rx_ += 1;
    memcpy (&packet_type, stype, sizeof (packet_type));
    //ROS_DEBUG ("Packet type: %d", packet_type);

    // get packet
    wait(packet_size);
    i = read (dev_, spacket, packet_size);
    if (i == 0)
      return false;
    serialport_bytes_rx_ += packet_size;
    //ROS_DEBUG ("Packet string: ok");

    // get packet crc
    wait(2);
    i = read (dev_, scrc, 2);
    if (i == 0)
      return false;
    serialport_bytes_rx_ += sizeof (scrc);
    memcpy (&packet_crc, scrc, sizeof (packet_crc));
    //ROS_DEBUG ("Packet crc: %d", packet_crc);

    // get closing ("<#<")
    wait(3);
    i = read (dev_, stoken, 3);
    if (i == 0 || strncmp (stoken, "<#<", 3) != 0)
    {
      ROS_ERROR ("Error Reading Packet Footer: %s", strerror (errno));
      ROS_DEBUG ("Read (%d): %s", i, stoken);
      while (read (dev_, stoken, 1) != 0)
      {
        stoken[1] = '\0';
        ROS_DEBUG ("%s", stoken);
      }
      flush ();
      drain ();
      ROS_DEBUG ("Packet Footer Corrupt!!");
      return false;
    }
    serialport_bytes_rx_ += 3;
    //ROS_DEBUG ("Packet Footer OK");

    return true;
  }

  void SerialInterface::output (char *output, int len)
  {
    int i;
    ROS_DEBUG ("SerialInterface::output()");
    serialport_bytes_tx_ += len;
    //ROS_DEBUG ("Writing %d element(s): %s", len, output);
    //ROS_DEBUG ("dev: %zd", (size_t) dev_);
    //flush ();
    i = write (dev_, output, len);
    if (i != len)
    {
      ROS_ERROR ("Error wrote %d out of %d element(s): %s", i, len, strerror (errno));
      ROS_BREAK ();
    }
    ROS_DEBUG ("Write completed");
  }

  void SerialInterface::output (unsigned char *output, int len)
  {
    int i;
    ROS_DEBUG ("SerialInterface::output()");
    serialport_bytes_tx_ += len;
    //ROS_INFO ("Writing %d element(s): %s", len, output);
    //ROS_DEBUG ("dev: %zd", (size_t) dev_);
    //ROS_DEBUG ("FOO");
    i = write (dev_, output, len);
    if (i != len)
    {
      ROS_ERROR ("Error wrote %d out of %d element(s): %s", i, len, strerror (errno));
      ROS_BREAK ();
    }
  }
  void SerialInterface::sendWaypoint (Telemetry * telemetry)
  {
    int i;
    char data[5];
    struct Telemetry::WAYPOINT wp = telemetry->WAYPOINT_INPUT_;

    if(!telemetry->waypointEnabled_) return;

    //ROS_DEBUG ("sendWaypoint started");
    flush();
    unsigned char cmd[] = ">*>ws";
    if(wp.chksum != (short)(wp.yaw + wp.height + wp.time + wp.X + wp.Y + wp.max_speed + wp.pos_acc + wp.properties + wp.wp_number + (short) 0xAAAA)){
        ROS_INFO("invalid WP checksum: %d != %d", wp.chksum, (short)(wp.yaw + wp.height + wp.time + wp.X + wp.Y + wp.max_speed + wp.pos_acc + wp.properties + wp.wp_number + (short) 0xAAAA));
      return;
    }
    output(cmd,5);
    output((unsigned char*) &wp, sizeof(struct Telemetry::WAYPOINT));
    ROS_INFO("sending waypoint to pelican: size of WAYPOINT %zd", sizeof(struct Telemetry::WAYPOINT));
    wait(5);
    //ROS_INFO("Data Available");
    i = read (dev_,data,5);
    if (i != 5) {
      ROS_ERROR("Waypoint Response : Insufficient Data");
      flush();
      return;
    }
    if (strncmp(data,">a",2) != 0) {
      ROS_ERROR("Corrupt Response Header %c%c (%0x%0x)",data[0],data[1],data[0],data[1]);
      flush();
      return;
    }
    if (strncmp(data+3,"a<",2) != 0) {
      ROS_ERROR("Corrupt Response Footer %c%c (%0x%0x)",data[3],data[4],data[3],data[4]);
      flush();
      return;
    }
    telemetry->wp_received_ = false;
    ROS_INFO("Waypoint Response Code %0x",data[2]);
    //ROS_INFO ("sendWaypoint completed" );
  }

  void SerialInterface::sendControl (Telemetry * telemetry)
  {
    int i;
    char data[5];

    if(!telemetry->controlEnabled_) return;
    //ROS_DEBUG ("sendControl started");
    flush();
    unsigned char cmd[] = ">*>di";
    //telemetry->dumpCTRL_INPUT();
    if (telemetry->controlInterval_ != 0 && ((telemetry->controlCount_ - telemetry->controlOffset_) % telemetry->controlInterval_ == 0)) {
      if(telemetry->CTRL_INPUT_.chksum != telemetry->CTRL_INPUT_.pitch + telemetry->CTRL_INPUT_.roll + telemetry->CTRL_INPUT_.yaw + telemetry->CTRL_INPUT_.thrust + telemetry->CTRL_INPUT_.ctrl + (short) 0xAAAA){
        //ROS_INFO("invalid CtrlInput checksum: %d !=  %d", telemetry->CTRL_INPUT_.chksum, telemetry->CTRL_INPUT_.pitch + telemetry->CTRL_INPUT_.roll + telemetry->CTRL_INPUT_.yaw + telemetry->CTRL_INPUT_.thrust + telemetry->CTRL_INPUT_.ctrl + (short) 0xAAAA);
        return;
      }
      output(cmd,5);
      output((unsigned char*) &telemetry->CTRL_INPUT_, 12);
      //ROS_INFO("writing control to pelican: size of CTRL_INPUT_ %zd", sizeof(telemetry->CTRL_INPUT_));
      wait(5);
      //ROS_INFO("Data Available");
      i = read (dev_,data,5);
      if (i != 5) {
        ROS_ERROR("Control Response : Insufficient Data");
        flush();
        return;
      }
      if (strncmp(data,">a",2) != 0) {
        ROS_ERROR("Corrupt Response Header %c%c (%0x%0x)",data[0],data[1],data[0],data[1]);
        flush();
        return;
      }
      if (strncmp(data+3,"a<",2) != 0) {
        ROS_ERROR("Corrupt Response Footer %c%c (%0x%0x)",data[3],data[4],data[3],data[4]);
        flush();
        return;
      }
      ROS_DEBUG("Control Response Code %0x",data[2]);
    }
    //ROS_INFO ("sendControl completed" );
  }

  void SerialInterface::sendEstop(Telemetry * telemetry)
  {
    static bool sent_estop_reported = false;
    if (!telemetry->controlEnabled_)
      return;
    //ROS_DEBUG ("sendControl started");
    flush();
    unsigned char cmd[] = ">*>me";
    output(cmd, 5);
    if (!sent_estop_reported)
    {
      ROS_WARN("Sent E-Stop command!");
      sent_estop_reported = true;
    }
  }

  bool SerialInterface::getPackets (Telemetry * telemetry)
  {
    flush ();
    ROS_DEBUG ("SerialInterface::getPackets");
    char cmd[16];
    char spacket[1024];
    unsigned char packet_type;
    unsigned short packet_crc;
    unsigned short packet_size;
    unsigned int i;
    bool result = false;
    ros::Time packetTime;

    ROS_DEBUG ("  Requesting %04x %zd packets", (short) telemetry->requestPackets_.to_ulong (),
              telemetry->requestPackets_.count ());
    //sprintf (cmd, ">*>p%c", (short) telemetry->requestPackets_.to_ulong ());
    sprintf (cmd, ">*>p");
    cmd[4] = 0xFF & telemetry->requestPackets_.to_ulong ();
    cmd[5] = 0xFF & telemetry->requestPackets_.to_ulong () >> 8;
    output (cmd, 6);
    ROS_DEBUG ("  Command: %s", cmd);
    for (i = 0; i < telemetry->requestPackets_.count (); i++)
    {
      packetTime = ros::Time::now();  // Presumes that the AutoPilot is grabbing the data for each packet
                                      // immediately prior to each packet being sent, as opposed to gathering
                                      // all the data at once and then sending it all. Either way is a guess
                                      // unless we get some info from AscTec one way or the other..
      bool read_result = getPacket (spacket, packet_type, packet_crc, packet_size);

      if (read_result)
      {
        ROS_DEBUG ("  Read successful: type = %d, crc = %d", packet_type, packet_crc);

        if (packet_type == Telemetry::PD_LLSTATUS)
        {
          ROS_DEBUG ("  Packet type is LL_STATUS");
          memcpy (&telemetry->LL_STATUS_, spacket, packet_size);
          telemetry->timestamps_[RequestTypes::LL_STATUS] = packetTime;
          if (crc_valid (packet_crc, &telemetry->LL_STATUS_, sizeof (packet_size)))
          {
            result = true;
          }
          //telemetry->dumpLL_STATUS();
        }
        else if (packet_type == Telemetry::PD_IMURAWDATA)
        {
          ROS_DEBUG ("  Packet type is IMU_RAWDATA");
          memcpy (&telemetry->IMU_RAWDATA_, spacket, packet_size);
          telemetry->timestamps_[RequestTypes::IMU_RAWDATA] = packetTime;
          if (crc_valid (packet_crc, &telemetry->IMU_RAWDATA_, packet_size))
          {
            result = true;
          }
          //telemetry->dumpIMU_RAWDATA();
        }
        else if (packet_type == Telemetry::PD_IMUCALCDATA)
        {
          ROS_DEBUG ("  Packet type is IMU_CALCDATA");
          memcpy (&telemetry->IMU_CALCDATA_, spacket, packet_size);
          telemetry->timestamps_[RequestTypes::IMU_CALCDATA] = packetTime;
          if (crc_valid (packet_crc, &telemetry->IMU_CALCDATA_, packet_size))
          {
            result = true;
          }
          //telemetry->dumpIMU_CALCDATA();
        }
        else if (packet_type == Telemetry::PD_RCDATA)
        {
          ROS_DEBUG ("  Packet type is RC_DATA");
          memcpy (&telemetry->RC_DATA_, spacket, packet_size);
          telemetry->timestamps_[RequestTypes::RC_DATA] = packetTime;
          if (crc_valid (packet_crc, &telemetry->RC_DATA_, packet_size))
          {
            result = true;
          }
          //telemetry->dumpRC_DATA();
        }
        else if (packet_type == Telemetry::PD_CTRLOUT)
        {
          ROS_DEBUG ("  Packet type is CONTROLLER_OUTPUT");
          memcpy (&telemetry->CONTROLLER_OUTPUT_, spacket, packet_size);
          telemetry->timestamps_[RequestTypes::CONTROLLER_OUTPUT] = packetTime;
          if (crc_valid (packet_crc, &telemetry->CONTROLLER_OUTPUT_, packet_size))
          {
            result = true;
          }
          //telemetry->dumpCONTROLLER_OUTPUT();
        }
        else if (packet_type == Telemetry::PD_GPSDATA)
        {
          ROS_DEBUG ("  Packet type is GPS_DATA");
          memcpy (&telemetry->GPS_DATA_, spacket, packet_size);
          telemetry->timestamps_[RequestTypes::GPS_DATA] = packetTime;
          if (crc_valid (packet_crc, &telemetry->GPS_DATA_, packet_size))
          {
            result = true;
          }
          //telemetry->dumpGPS_DATA();
        }
        else if (packet_type == Telemetry::PD_GPSDATAADVANCED)
        {
          ROS_DEBUG ("  Packet type is GPS_DATA_ADVANCED");
          memcpy (&telemetry->GPS_DATA_ADVANCED_, spacket, packet_size);
          telemetry->timestamps_[RequestTypes::GPS_DATA_ADVANCED] = packetTime;
          if (crc_valid (packet_crc, &telemetry->GPS_DATA_ADVANCED_, packet_size))
          {
            result = true;
          }
          //telemetry->dumpGPS_DATA_ADVANCED();
        }
        else if (packet_type == Telemetry::PD_CURRENTWAY)
        {
          ROS_DEBUG ("  Packet type is CURRENTWAY");
          memcpy (&telemetry->WAYPOINT_, spacket, packet_size);
          telemetry->timestamps_[RequestTypes::WAYPOINT] = packetTime;
          if (crc_valid (packet_crc, &telemetry->WAYPOINT_, packet_size))
          {
            result = true;
          }
          //telemetry->dumpGPS_DATA_ADVANCED();
        }
        else
        {
          ROS_ERROR ("  Packet type (%#2x) is UNKNOWN", packet_type);
        }
      }
      else
      {
        // failed read
        ROS_ERROR ("  Read failed");
        break;
      }
    }
    ioctl(dev_,FIONREAD,&i);
    if (i != 0)
    {
      ROS_ERROR ("  Unexpected Data: Flushing receive buffer");
      flush();
    }
    return result;
  }
}

