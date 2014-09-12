/*
 *  AscTec Autopilot Telemetry
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
  Telemetry::Telemetry (ros::NodeHandle nh): nh_(nh)
  {
    requestCount_ = 0;
    pollingEnabled_ = false;
    requestPackets_ = 0;
    wp_received_ = false;
    memset (requestInterval_, 0, REQUEST_TYPES * sizeof (uint8_t));
    memset (requestOffset_, 0, REQUEST_TYPES * sizeof (uint8_t));
    REQUEST_BITMASK[RequestTypes::LL_STATUS] = 0x0001;
    REQUEST_BITMASK[RequestTypes::IMU_RAWDATA] = 0x0002;
    REQUEST_BITMASK[RequestTypes::IMU_CALCDATA] = 0x0004;
    REQUEST_BITMASK[RequestTypes::RC_DATA] = 0x0008;
    REQUEST_BITMASK[RequestTypes::CONTROLLER_OUTPUT] = 0x0010;
    REQUEST_BITMASK[RequestTypes::GPS_DATA] = 0x0080;
    REQUEST_BITMASK[RequestTypes::WAYPOINT] = 0x0100;
    REQUEST_BITMASK[RequestTypes::GPS_DATA_ADVANCED] = 0x0200;
    REQUEST_BITMASK[RequestTypes::CAM_DATA] = 0x0800;
    estop_ = false;

    // initialize pointers

    LLStatus_         = boost::make_shared<asctec_msgs::LLStatus>        ();
    IMURawData_       = boost::make_shared<asctec_msgs::IMURawData>      ();
    IMUCalcData_      = boost::make_shared<asctec_msgs::IMUCalcData>     ();
    RCData_           = boost::make_shared<asctec_msgs::RCData>          ();
    ControllerOutput_ = boost::make_shared<asctec_msgs::ControllerOutput>();
    GPSData_          = boost::make_shared<asctec_msgs::GPSData>         ();
    GPSDataAdvanced_  = boost::make_shared<asctec_msgs::GPSDataAdvanced> ();
    CurrentWay_  = boost::make_shared<asctec_msgs::CurrentWay> ();
    WAYPOINT_.navigation_status=-1;
    WAYPOINT_.distance_to_wp=-1;
  }
  Telemetry::~Telemetry ()
  {
  }

  void Telemetry::buildRequest ()
  {
    //ROS_DEBUG ("Telemetry::buildRequest()");
    // Clear previous packet request
    requestPackets_ ^= requestPackets_;
    for (int i = 0; i < REQUEST_TYPES; i++)
    {
      if (requestInterval_[i] != 0 && ((requestCount_ - requestOffset_[i]) % requestInterval_[i] == 0) &&
          (requestPublisher_[i].getNumSubscribers () > 0))
        requestPackets_ |= REQUEST_BITMASK[i];
    }
    // hack for now...
    requestPackets_ |= REQUEST_BITMASK[RequestTypes::WAYPOINT];
  }
  void Telemetry::publishPackets ()
  {
    for (int i = 0; i < REQUEST_TYPES; i++)
    {
      if (requestInterval_[i] != 0 && ((requestCount_ - requestOffset_[i]) % requestInterval_[i] == 0))
      {
        switch (i)
        {
          case RequestTypes::LL_STATUS:
            copyLL_STATUS ();
            LLStatus_->header.stamp = timestamps_[RequestTypes::LL_STATUS];
            //dumpLL_STATUS ();
            requestPublisher_[i].publish (LLStatus_);
            break;
          case RequestTypes::IMU_RAWDATA:
            copyIMU_RAWDATA ();
            IMURawData_->header.stamp = timestamps_[RequestTypes::IMU_RAWDATA];
            //dumpIMU_RAWDATA();
            requestPublisher_[i].publish (IMURawData_);
            break;
          case RequestTypes::IMU_CALCDATA:
            copyIMU_CALCDATA ();
            IMUCalcData_->header.stamp = timestamps_[RequestTypes::IMU_CALCDATA];
            //dumpIMU_CALCDATA();
            requestPublisher_[i].publish (IMUCalcData_);
            break;
          case RequestTypes::GPS_DATA:
            copyGPS_DATA ();
            GPSData_->header.stamp = timestamps_[RequestTypes::GPS_DATA];
            requestPublisher_[i].publish (GPSData_);
            break;
          case RequestTypes::RC_DATA:
            copyRC_DATA ();
            RCData_->header.stamp = timestamps_[RequestTypes::RC_DATA];
            requestPublisher_[i].publish (RCData_);
            break;
          case RequestTypes::CONTROLLER_OUTPUT:
            copyCONTROLLER_OUTPUT ();
            ControllerOutput_->header.stamp = timestamps_[RequestTypes::CONTROLLER_OUTPUT];
            requestPublisher_[i].publish (ControllerOutput_);
            break;
          case RequestTypes::GPS_DATA_ADVANCED:
            copyGPS_DATA_ADVANCED ();
            GPSDataAdvanced_->header.stamp = timestamps_[RequestTypes::GPS_DATA_ADVANCED];
           //dumpGPS_DATA_ADVANCED();
            requestPublisher_[i].publish (GPSDataAdvanced_);
            break;
          case RequestTypes::WAYPOINT:
            copyWAYPOINT ();
            CurrentWay_->header.stamp = timestamps_[RequestTypes::WAYPOINT];
            requestPublisher_[i].publish (CurrentWay_);
            break;
          default:
            ROS_DEBUG ("Unable to publish unknown type");
        }
      }
    }
  }

  void Telemetry::enablePolling (RequestType msg, uint8_t interval, uint8_t offset)
  {
    switch (msg)
    {
      case RequestTypes::LL_STATUS:
        requestPublisher_[msg] = nh_.advertise < asctec_msgs::LLStatus > (requestToString (msg).c_str (), 10);
        break;
      case RequestTypes::IMU_RAWDATA:
        requestPublisher_[msg] = nh_.advertise < asctec_msgs::IMURawData > (requestToString (msg).c_str (), 10);
        break;
      case RequestTypes::IMU_CALCDATA:
        requestPublisher_[msg] = nh_.advertise < asctec_msgs::IMUCalcData > (requestToString (msg).c_str (), 10);
        break;
      case RequestTypes::RC_DATA:
        requestPublisher_[msg] = nh_.advertise < asctec_msgs::RCData > (requestToString (msg).c_str (), 10);
        break;
      case RequestTypes::GPS_DATA:
        requestPublisher_[msg] = nh_.advertise < asctec_msgs::GPSData > (requestToString (msg).c_str (), 10);
        break;
      case RequestTypes::GPS_DATA_ADVANCED:
        requestPublisher_[msg] = nh_.advertise < asctec_msgs::GPSDataAdvanced > (requestToString (msg).c_str (), 10);
        break;
      case RequestTypes::WAYPOINT:
        requestPublisher_[msg] = nh_.advertise < asctec_msgs::CurrentWay > (requestToString (msg).c_str (), 10);
        break;
      case RequestTypes::CAM_DATA:
        // to be filled in 
        break;
      case RequestTypes::CONTROLLER_OUTPUT:
        requestPublisher_[msg] = nh_.advertise < asctec_msgs::ControllerOutput > (requestToString (msg).c_str (), 10);
        break;
    }

    ROS_INFO ("Publishing %s data", requestToString (msg).c_str());
    ROS_DEBUG ("Telemetry::enablePolling()");
    requestInterval_[msg] = interval;
    requestOffset_[msg] = offset;
    pollingEnabled_ = true;
  }

  void Telemetry::enableControl (Telemetry * telemetry_, uint8_t interval, uint8_t offset)
  {
    controlSubscriber_ = nh_.subscribe("CTRL_INPUT", 1, &Telemetry::copyCTRL_INPUT, telemetry_, ros::TransportHints().tcpNoDelay());
    ROS_INFO("Listening to %s data on topic: %s", "CTRL_INPUT","CTRL_INPUT");
    ROS_DEBUG ("Telemetry::enableControl()");
    estopSubscriber_ = nh_.subscribe("ESTOP", 1, &Telemetry::estopCallback, telemetry_, ros::TransportHints().tcpNoDelay());
    controlInterval_ = interval;
    controlOffset_ = offset;
    controlEnabled_ = true;
  }
  void Telemetry::enableWaypoints (Telemetry * telemetry_)
  {
    waypointSubscriber_ = nh_.subscribe("/asctec/WAYPOINT", 1, &Telemetry::copyWAYPOINT_INPUT, telemetry_, ros::TransportHints().tcpNoDelay());
    ROS_INFO("Listening to %s data on topic: %s", "WAYPOINT","/asctec/WAYPOINT");
    ROS_DEBUG ("Telemetry::enableWaypoints()");
    waypointEnabled_ = true;
  }
  void Telemetry::estopCallback(const std_msgs::Bool msg)
  {
    static bool info_printed = false;
    if (msg.data) {
      estop_ = true;
      if (!info_printed) {
        ROS_WARN("Heard e-stop command!");
        info_printed = true;
      }
    } else {
      ROS_WARN("Got e-stop message, but value was false");
    }
  }

  std::string Telemetry::requestToString (RequestTypes::RequestType t)
  {
    switch (t)
    {
      case RequestTypes::LL_STATUS:
        {
          return "LL_STATUS";
        }
      case RequestTypes::IMU_RAWDATA:
        {
          return "IMU_RAWDATA";
        }
      case RequestTypes::IMU_CALCDATA:
        {
          return "IMU_CALCDATA";
        }
      case RequestTypes::RC_DATA:
        {
          return "RC_DATA";
        }
      case RequestTypes::CONTROLLER_OUTPUT:
        {
          return "CONTROLLER_OUTPUT";
        }
      case RequestTypes::GPS_DATA:
        {
          return "GPS_DATA";
        }
      case RequestTypes::GPS_DATA_ADVANCED:
        {
          return "GPS_DATA_ADVANCED";
        }
      case RequestTypes::WAYPOINT:
        {
          return "CURRENT_WAY";
        }
      case RequestTypes::CAM_DATA:
        {
          return "CAM_DATA";
        }
    }
    return "Unknown";
  }

  void Telemetry::dumpLL_STATUS ()
  {
    ROS_INFO("LL_STATUS");
    ROS_INFO("--------------------------------");
    ROS_INFO("battery_voltage_1:%d",LL_STATUS_.battery_voltage_1);
    ROS_INFO("battery_voltage_2:%d",LL_STATUS_.battery_voltage_2);
    ROS_INFO("status:%d",LL_STATUS_.status);
    ROS_INFO("cpu_load:%d",LL_STATUS_.cpu_load);
    ROS_INFO("compass_enabled:%d",LL_STATUS_.compass_enabled);
    ROS_INFO("chksum_error:%d",LL_STATUS_.chksum_error);
    ROS_INFO("flying:%d",LL_STATUS_.flying);
    ROS_INFO("motors_on:%d",LL_STATUS_.motors_on);
    ROS_INFO("flightMode:%d",LL_STATUS_.flightMode);
    ROS_INFO("up_time:%d",LL_STATUS_.up_time);
    if (LL_STATUS_.flightMode == 97)
      ROS_INFO ("---------- SERIAL LINK ACTIVE !!! --------");
  }
  void Telemetry::dumpIMU_RAWDATA ()
  {
    ROS_INFO ("IMU_RAWDATA");
    ROS_INFO ("--------------------------------");
    ROS_INFO ("pressure:%d", IMU_RAWDATA_.pressure);
    ROS_INFO ("gyro_x:%d", IMU_RAWDATA_.gyro_x);
    ROS_INFO ("gyro_y:%d", IMU_RAWDATA_.gyro_y);
    ROS_INFO ("gyro_z:%d", IMU_RAWDATA_.gyro_z);
    ROS_INFO ("mag_x:%d", IMU_RAWDATA_.mag_x);
    ROS_INFO ("mag_y:%d", IMU_RAWDATA_.mag_y);
    ROS_INFO ("mag_z:%d", IMU_RAWDATA_.mag_z);
    ROS_INFO ("acc_x:%d", IMU_RAWDATA_.acc_x);
    ROS_INFO ("acc_y:%d", IMU_RAWDATA_.acc_y);
    ROS_INFO ("acc_z:%d", IMU_RAWDATA_.acc_z);
    ROS_INFO ("temp_gyro:%d", IMU_RAWDATA_.temp_gyro);
    ROS_INFO ("temp_ADC:%d", IMU_RAWDATA_.temp_ADC);
  }
  void Telemetry::dumpIMU_CALCDATA ()
  {
    ROS_INFO ("IMU_CALCDATA");
    ROS_INFO ("--------------------------------");
    ROS_INFO ("angle_nick:%d", IMU_CALCDATA_.angle_nick);
    ROS_INFO ("angle_roll:%d", IMU_CALCDATA_.angle_roll);
    ROS_INFO ("angle_yaw:%d", IMU_CALCDATA_.angle_yaw);
    ROS_INFO ("angvel_nick:%d", IMU_CALCDATA_.angvel_nick);
    ROS_INFO ("angvel_roll:%d", IMU_CALCDATA_.angvel_roll);
    ROS_INFO ("angvel_yaw:%d", IMU_CALCDATA_.angvel_yaw);
    ROS_INFO ("acc_x_calib:%d", IMU_CALCDATA_.acc_x_calib);
    ROS_INFO ("acc_y_calib:%d", IMU_CALCDATA_.acc_y_calib);
    ROS_INFO ("acc_z_calib:%d", IMU_CALCDATA_.acc_z_calib);
    ROS_INFO ("acc_x:%d", IMU_CALCDATA_.acc_x);
    ROS_INFO ("acc_y:%d", IMU_CALCDATA_.acc_y);
    ROS_INFO ("acc_z:%d", IMU_CALCDATA_.acc_z);
    ROS_INFO ("acc_angle_nick:%d", IMU_CALCDATA_.acc_angle_nick);
    ROS_INFO ("acc_angle_roll:%d", IMU_CALCDATA_.acc_angle_roll);
    ROS_INFO ("acc_absolute_value:%d", IMU_CALCDATA_.acc_absolute_value);
    ROS_INFO ("Hx:%d", IMU_CALCDATA_.Hx);
    ROS_INFO ("Hy:%d", IMU_CALCDATA_.Hy);
    ROS_INFO ("Hz:%d", IMU_CALCDATA_.Hz);
    ROS_INFO ("mag_heading:%d", IMU_CALCDATA_.mag_heading);
    ROS_INFO ("speed_x:%d", IMU_CALCDATA_.speed_x);
    ROS_INFO ("speed_y:%d", IMU_CALCDATA_.speed_y);
    ROS_INFO ("speed_z:%d", IMU_CALCDATA_.speed_z);
    ROS_INFO ("height:%d", IMU_CALCDATA_.height);
    ROS_INFO ("dheight:%d", IMU_CALCDATA_.dheight);
    ROS_INFO ("dheight_reference:%d", IMU_CALCDATA_.dheight_reference);
    ROS_INFO ("height_reference:%d", IMU_CALCDATA_.height_reference);
  }
  void Telemetry::dumpRC_DATA ()
  {
    ROS_INFO ("RC_DATA");
    ROS_INFO ("--------------------------------");
    ROS_INFO ("channels_in: %d %d %d %d %d %d %d %d", RC_DATA_.channels_in[0], RC_DATA_.channels_in[1],
              RC_DATA_.channels_in[2], RC_DATA_.channels_in[3], RC_DATA_.channels_in[4], RC_DATA_.channels_in[5],
              RC_DATA_.channels_in[6], RC_DATA_.channels_in[7]);
    ROS_INFO ("channels_out: %d %d %d %d %d %d %d %d", RC_DATA_.channels_out[0], RC_DATA_.channels_out[1],
              RC_DATA_.channels_out[2], RC_DATA_.channels_out[3], RC_DATA_.channels_out[4], RC_DATA_.channels_out[5],
              RC_DATA_.channels_out[6], RC_DATA_.channels_out[7]);
    ROS_INFO ("lock:%d", RC_DATA_.lock);
  }
  void Telemetry::dumpCONTROLLER_OUTPUT ()
  {
    ROS_INFO ("CONTROLLER_OUTPUT");
    ROS_INFO ("--------------------------------");
    ROS_INFO ("nick:%d", CONTROLLER_OUTPUT_.nick);
    ROS_INFO ("roll:%d", CONTROLLER_OUTPUT_.roll);
    ROS_INFO ("yaw:%d", CONTROLLER_OUTPUT_.yaw);
    ROS_INFO ("thrust:%d", CONTROLLER_OUTPUT_.thrust);
  }
  void Telemetry::dumpGPS_DATA ()
  {
    ROS_INFO ("GPS_DATA");
    ROS_INFO ("--------------------------------");
    ROS_INFO ("latitude:%d", GPS_DATA_.latitude);
    ROS_INFO ("longitude:%d", GPS_DATA_.longitude);
    ROS_INFO ("height:%d", GPS_DATA_.height);
    ROS_INFO ("speed_x:%d", GPS_DATA_.speed_x);
    ROS_INFO ("speed_y:%d", GPS_DATA_.speed_y);
    ROS_INFO ("heading:%d", GPS_DATA_.heading);
    ROS_INFO ("horizontal_accuracy:%d", GPS_DATA_.horizontal_accuracy);
    ROS_INFO ("vertical_accuracy:%d", GPS_DATA_.vertical_accuracy);
    ROS_INFO ("speed_accuracy:%d", GPS_DATA_.speed_accuracy);
    ROS_INFO ("numSV:%d", GPS_DATA_.numSV);
    ROS_INFO ("status:%d", GPS_DATA_.status);
  }
  void Telemetry::dumpGPS_DATA_ADVANCED ()
  {
    ROS_INFO ("GPS_DATA_ADVANCED");
    ROS_INFO ("--------------------------------");
    ROS_INFO ("latitude:%d", GPS_DATA_ADVANCED_.latitude);
    ROS_INFO ("longitude:%d", GPS_DATA_ADVANCED_.longitude);
    ROS_INFO ("height:%d", GPS_DATA_ADVANCED_.height);
    ROS_INFO ("speed_x:%d", GPS_DATA_ADVANCED_.speed_x);
    ROS_INFO ("speed_y:%d", GPS_DATA_ADVANCED_.speed_y);
    ROS_INFO ("heading:%d", GPS_DATA_ADVANCED_.heading);
    ROS_INFO ("horizontal_accuracy:%d", GPS_DATA_ADVANCED_.horizontal_accuracy);
    ROS_INFO ("vertical_accuracy:%d", GPS_DATA_ADVANCED_.vertical_accuracy);
    ROS_INFO ("speed_accuracy:%d", GPS_DATA_ADVANCED_.speed_accuracy);
    ROS_INFO ("numSV:%d", GPS_DATA_ADVANCED_.numSV);
    ROS_INFO ("status:%d", GPS_DATA_ADVANCED_.status);
    ROS_INFO ("latitude_best_estimate:%d", GPS_DATA_ADVANCED_.latitude_best_estimate);
    ROS_INFO ("longitude_best_estimate:%d", GPS_DATA_ADVANCED_.longitude_best_estimate);
    ROS_INFO ("speed_x_best_estimate:%d", GPS_DATA_ADVANCED_.speed_x_best_estimate);
    ROS_INFO ("speed_y_best_estimate:%d", GPS_DATA_ADVANCED_.speed_y_best_estimate);
  }
  void Telemetry::dumpCTRL_INPUT ()
  {
    ROS_INFO ("CTRL_INPUT");
    ROS_INFO ("--------------------------------");
    ROS_INFO ("pitch:%d", CTRL_INPUT_.pitch);
    ROS_INFO ("roll:%d", CTRL_INPUT_.roll);
    ROS_INFO ("yaw:%d", CTRL_INPUT_.yaw);
    ROS_INFO ("thrust:%d", CTRL_INPUT_.thrust);
    ROS_INFO ("ctrl:%d", CTRL_INPUT_.ctrl);
    ROS_INFO ("chksum:%d", CTRL_INPUT_.chksum);
  }
  void Telemetry::copyLL_STATUS ()
  {
    LLStatus_->battery_voltage_1 = LL_STATUS_.battery_voltage_1;
    LLStatus_->battery_voltage_2 = LL_STATUS_.battery_voltage_2;
    LLStatus_->status = LL_STATUS_.status;
    LLStatus_->cpu_load = LL_STATUS_.cpu_load;
    LLStatus_->compass_enabled = LL_STATUS_.compass_enabled;
    LLStatus_->chksum_error = LL_STATUS_.chksum_error;
    LLStatus_->flying = LL_STATUS_.flying;
    LLStatus_->motors_on = LL_STATUS_.motors_on;
    LLStatus_->flightMode = LL_STATUS_.flightMode;
    LLStatus_->up_time = LL_STATUS_.up_time;
  }
  void Telemetry::copyIMU_RAWDATA ()
  {
    IMURawData_->pressure = IMU_RAWDATA_.pressure;
    IMURawData_->gyro_x = IMU_RAWDATA_.gyro_x;
    IMURawData_->gyro_y = IMU_RAWDATA_.gyro_y;
    IMURawData_->gyro_z = IMU_RAWDATA_.gyro_z;
    IMURawData_->mag_x = IMU_RAWDATA_.mag_x;
    IMURawData_->mag_y = IMU_RAWDATA_.mag_y;
    IMURawData_->mag_z = IMU_RAWDATA_.mag_z;
    IMURawData_->acc_x = IMU_RAWDATA_.acc_x;
    IMURawData_->acc_y = IMU_RAWDATA_.acc_y;
    IMURawData_->acc_z = IMU_RAWDATA_.acc_z;
    IMURawData_->temp_gyro = IMU_RAWDATA_.temp_gyro;
    IMURawData_->temp_ADC = IMU_RAWDATA_.temp_ADC;
  }
  void Telemetry::copyIMU_CALCDATA ()
  {
    IMUCalcData_->angle_nick = IMU_CALCDATA_.angle_nick;
    IMUCalcData_->angle_roll = IMU_CALCDATA_.angle_roll;
    IMUCalcData_->angle_yaw = IMU_CALCDATA_.angle_yaw;
    IMUCalcData_->angvel_nick = IMU_CALCDATA_.angvel_nick;
    IMUCalcData_->angvel_roll = IMU_CALCDATA_.angvel_roll;
    IMUCalcData_->angvel_yaw = IMU_CALCDATA_.angvel_yaw;
    IMUCalcData_->acc_x_calib = IMU_CALCDATA_.acc_x_calib;
    IMUCalcData_->acc_y_calib = IMU_CALCDATA_.acc_y_calib;
    IMUCalcData_->acc_z_calib = IMU_CALCDATA_.acc_z_calib;
    IMUCalcData_->acc_x = IMU_CALCDATA_.acc_x;
    IMUCalcData_->acc_y = IMU_CALCDATA_.acc_y;
    IMUCalcData_->acc_z = IMU_CALCDATA_.acc_z;
    IMUCalcData_->acc_angle_nick = IMU_CALCDATA_.acc_angle_nick;
    IMUCalcData_->acc_angle_roll = IMU_CALCDATA_.acc_angle_roll;
    IMUCalcData_->acc_absolute_value = IMU_CALCDATA_.acc_absolute_value;
    IMUCalcData_->Hx = IMU_CALCDATA_.Hx;
    IMUCalcData_->Hy = IMU_CALCDATA_.Hy;
    IMUCalcData_->Hz = IMU_CALCDATA_.Hz;
    IMUCalcData_->mag_heading = IMU_CALCDATA_.mag_heading;
    IMUCalcData_->speed_x = IMU_CALCDATA_.speed_x;
    IMUCalcData_->speed_y = IMU_CALCDATA_.speed_y;
    IMUCalcData_->speed_z = IMU_CALCDATA_.speed_z;
    IMUCalcData_->height = IMU_CALCDATA_.height;
    IMUCalcData_->dheight = IMU_CALCDATA_.dheight;
    IMUCalcData_->dheight_reference = IMU_CALCDATA_.dheight_reference;
    IMUCalcData_->height_reference = IMU_CALCDATA_.height_reference;
  }
  void Telemetry::copyRC_DATA ()
  {
    for (int i = 0; i < 8; i++)
    {
      RCData_->channels_in[i] = RC_DATA_.channels_in[i];
      RCData_->channels_out[i] = RC_DATA_.channels_out[i];
    }
    RCData_->lock = RC_DATA_.lock;
  }

  void Telemetry::copyCONTROLLER_OUTPUT ()
  {
    ControllerOutput_->nick = CONTROLLER_OUTPUT_.nick;
    ControllerOutput_->roll = CONTROLLER_OUTPUT_.roll;
    ControllerOutput_->yaw = CONTROLLER_OUTPUT_.yaw;
    ControllerOutput_->thrust = CONTROLLER_OUTPUT_.thrust;
  }

  void Telemetry::copyGPS_DATA ()
  {
    GPSData_->latitude = GPS_DATA_.latitude;
    GPSData_->longitude = GPS_DATA_.longitude;
    GPSData_->height = GPS_DATA_.height;
    GPSData_->speed_x = GPS_DATA_.speed_x;
    GPSData_->speed_y = GPS_DATA_.speed_y;
    GPSData_->heading = GPS_DATA_.heading;
    GPSData_->horizontal_accuracy = GPS_DATA_.horizontal_accuracy;
    GPSData_->vertical_accuracy = GPS_DATA_.vertical_accuracy;
    GPSData_->speed_accuracy = GPS_DATA_.speed_accuracy;
    GPSData_->numSV = GPS_DATA_.numSV;
    GPSData_->status = GPS_DATA_.status;
  }

  void Telemetry::copyGPS_DATA_ADVANCED ()
  {
    GPSDataAdvanced_->latitude = GPS_DATA_ADVANCED_.latitude;
    GPSDataAdvanced_->longitude = GPS_DATA_ADVANCED_.longitude;
    GPSDataAdvanced_->height = GPS_DATA_ADVANCED_.height;
    GPSDataAdvanced_->speed_x = GPS_DATA_ADVANCED_.speed_x;
    GPSDataAdvanced_->speed_y = GPS_DATA_ADVANCED_.speed_y;
    GPSDataAdvanced_->heading = GPS_DATA_ADVANCED_.heading;
    GPSDataAdvanced_->horizontal_accuracy = GPS_DATA_ADVANCED_.horizontal_accuracy;
    GPSDataAdvanced_->vertical_accuracy = GPS_DATA_ADVANCED_.vertical_accuracy;
    GPSDataAdvanced_->speed_accuracy = GPS_DATA_ADVANCED_.speed_accuracy;
    GPSDataAdvanced_->numSV = GPS_DATA_ADVANCED_.numSV;
    GPSDataAdvanced_->status = GPS_DATA_ADVANCED_.status;
    GPSDataAdvanced_->latitude_best_estimate = GPS_DATA_ADVANCED_.latitude_best_estimate;
    GPSDataAdvanced_->longitude_best_estimate = GPS_DATA_ADVANCED_.longitude_best_estimate;
    GPSDataAdvanced_->speed_x_best_estimate = GPS_DATA_ADVANCED_.speed_x_best_estimate;
    GPSDataAdvanced_->speed_y_best_estimate = GPS_DATA_ADVANCED_.speed_y_best_estimate;
  }

  void Telemetry::copyWAYPOINT ()
  {
    CurrentWay_->navigation_status = WAYPOINT_.navigation_status;
    CurrentWay_->distance_to_wp = WAYPOINT_.distance_to_wp;
  }
  
  void Telemetry::copyWAYPOINT_INPUT(asctec_msgs::Waypoint msg){
    ROS_INFO("Received waypoint");
    WAYPOINT_INPUT_.wp_number = msg.wp_number;
    WAYPOINT_INPUT_.properties = msg.properties;
    WAYPOINT_INPUT_.max_speed = msg.max_speed;
    WAYPOINT_INPUT_.time = msg.time;
    WAYPOINT_INPUT_.pos_acc = msg.pos_acc;
    WAYPOINT_INPUT_.chksum = msg.chksum;
    WAYPOINT_INPUT_.X = msg.X;
    WAYPOINT_INPUT_.Y = msg.Y;
    WAYPOINT_INPUT_.yaw = msg.yaw;
    WAYPOINT_INPUT_.height = msg.height;
    wp_received_ = true;
  }
  
  void Telemetry::copyCTRL_INPUT(asctec_msgs::CtrlInput msg){
    CTRL_INPUT_.pitch = msg.pitch;
    CTRL_INPUT_.roll = msg.roll;
    CTRL_INPUT_.yaw = msg.yaw;
    CTRL_INPUT_.thrust = msg.thrust;
    CTRL_INPUT_.ctrl = msg.ctrl;
    CTRL_INPUT_.chksum = msg.chksum;
    //dumpCTRL_INPUT();
  }
}
