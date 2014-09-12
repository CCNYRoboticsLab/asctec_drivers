/*
 *  AscTec Autopilot Interface
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

#include "asctec_autopilot/autopilot.h"

namespace asctec
{
  AutoPilot::AutoPilot(ros::NodeHandle nh, ros::NodeHandle nh_private): 
    nh_(nh), 
    nh_private_(nh_private),
    diag_updater_(),
    first_wp_(true)
  {
    ROS_INFO ("Creating AutoPilot Interface");
    
    // **** get parameters

    if (!nh_private_.getParam ("freq", freq_))
      freq_ = 50.0;
    if (!nh_private_.getParam ("port", port_))
      port_ = "/dev/ttyUSB0";
    if (!nh_private_.getParam ("speed", speed_))
      speed_ = 57600;

    if (!nh_private_.getParam ("enable_LL_STATUS", enable_LL_STATUS_))
      enable_LL_STATUS_ = false;
    if (!nh_private_.getParam ("enable_IMU_RAWDATA", enable_IMU_RAWDATA_))
      enable_IMU_RAWDATA_ = false;
    if (!nh_private_.getParam ("enable_IMU_CALCDATA", enable_IMU_CALCDATA_))
      enable_IMU_CALCDATA_ = false;
    if (!nh_private_.getParam ("enable_RC_DATA", enable_RC_DATA_))
      enable_RC_DATA_ = false;
    if (!nh_private_.getParam ("enable_CONTROLLER_OUTPUT", enable_CONTROLLER_OUTPUT_))
      enable_CONTROLLER_OUTPUT_ = false;
    if (!nh_private_.getParam ("enable_GPS_DATA", enable_GPS_DATA_))
      enable_GPS_DATA_ = false;
    if (!nh_private_.getParam ("enable_GPS_DATA_ADVANCED", enable_GPS_DATA_ADVANCED_))
      enable_GPS_DATA_ADVANCED_ = false;
    if (!nh_private_.getParam ("enable_CONTROL", enable_CONTROL_))
      enable_CONTROL_ = false;
    if (!nh_private_.getParam ("enable_WAYPOINT", enable_WAYPOINT_))
      enable_WAYPOINT_ = false;
    if (!nh_private_.getParam ("enable_CURRENTWAY", enable_CURRENTWAY_))
      enable_CURRENTWAY_ = enable_WAYPOINT_;

    if (!nh_private_.getParam ("interval_LL_STATUS", interval_LL_STATUS_))
      interval_LL_STATUS_ = 1;
    if (!nh_private_.getParam ("interval_IMU_RAWDATA", interval_IMU_RAWDATA_))
      interval_IMU_RAWDATA_ = 1;
    if (!nh_private_.getParam ("interval_IMU_CALCDATA", interval_IMU_CALCDATA_))
      interval_IMU_CALCDATA_ = 1;
    if (!nh_private_.getParam ("interval_RC_DATA", interval_RC_DATA_))
      interval_RC_DATA_ = 1;
    if (!nh_private_.getParam ("interval_CONTROLLER_OUTPUT", interval_CONTROLLER_OUTPUT_))
      interval_CONTROLLER_OUTPUT_ = 1;
    if (!nh_private_.getParam ("interval_GPS_DATA", interval_GPS_DATA_))
      interval_GPS_DATA_ = 1;
    if (!nh_private_.getParam ("interval_GPS_DATA_ADVANCED", interval_GPS_DATA_ADVANCED_))
      interval_GPS_DATA_ADVANCED_ = 1;
    if (!nh_private_.getParam ("interval_CONTROL", interval_CONTROL_))
      interval_CONTROL_ = 1;
    if (!nh_private_.getParam ("interval_CURRENTWAY", interval_CURRENTWAY_))
      interval_CURRENTWAY_ = 1;

    if (!nh_private_.getParam ("offset_LL_STATUS", offset_LL_STATUS_))
      offset_LL_STATUS_ = 0;
    if (!nh_private_.getParam ("offset_IMU_RAWDATA", offset_IMU_RAWDATA_))
      offset_IMU_RAWDATA_ = 0;
    if (!nh_private_.getParam ("offset_IMU_CALCDATA", offset_IMU_CALCDATA_))
      offset_IMU_CALCDATA_ = 0;
    if (!nh_private_.getParam ("offset_RC_DATA", offset_RC_DATA_))
      offset_RC_DATA_ = 0;
    if (!nh_private_.getParam ("offset_CONTROLLER_OUTPUT", offset_CONTROLLER_OUTPUT_))
      offset_CONTROLLER_OUTPUT_ = 0;
    if (!nh_private_.getParam ("offset_GPS_DATA", offset_GPS_DATA_))
      offset_GPS_DATA_ = 0;
    if (!nh_private_.getParam ("offset_GPS_DATA_ADVANCED", offset_GPS_DATA_ADVANCED_))
      offset_GPS_DATA_ADVANCED_ = 0;
    if (!nh_private_.getParam ("offset_CONTROL", offset_CONTROL_))
      offset_CONTROL_ = 0;
    if (!nh_private_.getParam ("offset_CURRENTWAY", offset_CURRENTWAY_))
      offset_CURRENTWAY_ = 0;

    if (freq_ <= 0.0)
      ROS_FATAL ("Invalid frequency param");

    ros::Duration d (1.0 / freq_);

    // **** set up intefaces

    serialInterface_ = new asctec::SerialInterface(port_, speed_);
    serialInterface_->serialport_bytes_rx_ = 0;
    serialInterface_->serialport_bytes_tx_ = 0;

    ros::NodeHandle nh_rawdata(nh_, asctec::ROS_NAMESPACE);    // publish to "asctec" namespace
    telemetry_ = new asctec::Telemetry(nh_rawdata);
    
    // Diagnostics
    diag_updater_.add("AscTec Autopilot Status", this, &AutoPilot::diagnostics);
    diag_updater_.setHardwareID("none");
    diag_updater_.force_update();

    // **** enable polling
    if(enable_LL_STATUS_ == true)
      telemetry_->enablePolling (asctec::RequestTypes::LL_STATUS, interval_LL_STATUS_, offset_LL_STATUS_);
    if(enable_RC_DATA_)
      telemetry_->enablePolling (asctec::RequestTypes::RC_DATA, interval_RC_DATA_, offset_RC_DATA_);
    if(enable_CONTROLLER_OUTPUT_)
      telemetry_->enablePolling (asctec::RequestTypes::CONTROLLER_OUTPUT, interval_CONTROLLER_OUTPUT_, offset_CONTROLLER_OUTPUT_);
    if(enable_IMU_RAWDATA_)
      telemetry_->enablePolling(asctec::RequestTypes::IMU_RAWDATA, interval_IMU_RAWDATA_, offset_IMU_RAWDATA_);
    if(enable_IMU_CALCDATA_)
      telemetry_->enablePolling (asctec::RequestTypes::IMU_CALCDATA, interval_IMU_CALCDATA_, offset_IMU_CALCDATA_);
    if(enable_GPS_DATA_)
      telemetry_->enablePolling (asctec::RequestTypes::GPS_DATA, interval_GPS_DATA_, offset_GPS_DATA_);
    if(enable_GPS_DATA_ADVANCED_)
      telemetry_->enablePolling (asctec::RequestTypes::GPS_DATA_ADVANCED, interval_GPS_DATA_ADVANCED_,  offset_GPS_DATA_ADVANCED_);

    if(enable_CURRENTWAY_)
      telemetry_->enablePolling (asctec::RequestTypes::WAYPOINT, interval_CURRENTWAY_, offset_CURRENTWAY_);
    // **** enable control
    if(enable_CONTROL_ == true)
    {
      ROS_INFO("Control Enabled");
      telemetry_->enableControl(telemetry_, interval_CONTROL_, offset_CONTROL_);
    }
    else
    {
      ROS_INFO("Control Disabled");
    }

    // enable waypoints
    if(enable_WAYPOINT_)
    {
      ROS_INFO("Waypoints Enabled");
      telemetry_->enableWaypoints(telemetry_);
    }
    else
    {
      ROS_INFO("Waypoints Disabled");
    }

    timer_ = nh_private_.createTimer (d, &AutoPilot::spin, this);
    //serialInterface_->sendWaypoint(wpcontroller_->getNextWaypoint());
  }

  AutoPilot::~AutoPilot ()
  {
    ROS_INFO ("Destroying AutoPilot Interface");

    delete telemetry_;
    delete serialInterface_;
  }

  void AutoPilot::spin(const ros::TimerEvent & e)
  {
    //ROS_INFO ("spin()");
    //ROS_INFO ("RX: %03.3f Bps",float(serialInterface_->serialport_bytes_rx_)/1000*freq_);
    //ROS_INFO ("TX: %03.3f Bps",float(serialInterface_->serialport_bytes_tx_)/1000*freq_);
    //serialInterface_->serialport_bytes_rx_ = 0;
    //serialInterface_->serialport_bytes_tx_ = 0;
    telemetry_->publishPackets();
    telemetry_->controlCount_++;
    if(telemetry_->wp_received_)
    {
      serialInterface_->sendWaypoint(telemetry_);
    }
    if (telemetry_->estop_)
    {
      serialInterface_->sendEstop(telemetry_);
    }
    else
    {
      serialInterface_->sendControl(telemetry_);
    }
    telemetry_->buildRequest();
    telemetry_->requestCount_++;
    if (telemetry_->requestPackets_.count() > 0)
    {
      serialInterface_->getPackets(telemetry_);
    }
   
    last_spin_time_ = e.profile.last_duration.toSec();
    diag_updater_.update();
  }

  void AutoPilot::diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    if (telemetry_->estop_)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "E-STOP");
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    }
    stat.add("Serial Bytes TX", serialInterface_->serialport_bytes_tx_);
    stat.add("Serial Bytes RX", serialInterface_->serialport_bytes_rx_);
    stat.add("Last spin() duration [s]", last_spin_time_);
  }

}
