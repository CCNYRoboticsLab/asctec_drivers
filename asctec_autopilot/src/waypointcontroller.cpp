/*
 *  AscTec Waypoint Controller
 *  Copyright (C) 2014
 *  Matt Sheckells <mshecke1@jhu.edu>
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




#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include "asctec_msgs/Waypoint.h"
#include "asctec_autopilot/waypointcontroller.h"

namespace asctec
{

  WaypointController::WaypointController(ros::NodeHandle nh, ros::NodeHandle nh_private):
    nh_(nh),
    nh_private_(nh_private),
    cur_wp_(0)
  {
    ROS_INFO("Creating WaypointController");
    if (!nh_private_.getParam ("WAYPOINT_file", waypointFile_))
        waypointFile_ = "";
    
    if(!openWayFile(waypointFile_))
    {
      ROS_ERROR("Could not open waypoint file");
    }
    waypointPublisher_ = nh_.advertise < asctec_msgs::Waypoint > ("/asctec/WAYPOINT", 10);
    currentwaySubscriber_ = nh_.subscribe("CURRENT_WAY", 1, &WaypointController::handleCurrentway, this, ros::TransportHints().tcpNoDelay());
    
    
    timer_ = nh_private_.createTimer(1.0, &WaypointController::spin, this);
  
  }

  void WaypointController::handleCurrentway(asctec_msgs::CurrentWay msg)
  {
    ROS_INFO("CurrentWay navigation_status: 0x%02x", msg.navigation_status);
    if(msg.navigation_status == 0x07 && hasNextWaypoint())
    {
      ROS_INFO("Publishing next waypoint");
      waypointPublisher_.publish (getNextMessage());
    }
  }

  void WaypointController::spin(const ros::TimerEvent & e)
  {
    if(hasNextWaypoint() && cur_wp_ == 0 && waypointPublisher_.getNumSubscribers() > 0)
    {
      ROS_INFO("Publishing first waypoint");
      waypointPublisher_.publish(getNextMessage());
    }   
  }

  asctec_msgs::Waypoint WaypointController::getNextMessage()
  {
    asctec_msgs::Waypoint wpm;
    Telemetry::WAYPOINT wp = getNextWaypoint();

    wpm.wp_number = wp.wp_number;
    wpm.properties = wp.properties;
    wpm.max_speed = wp.max_speed;
    wpm.time = wp.time;
    wpm.pos_acc = wp.pos_acc;
    wpm.chksum = wp.chksum;
    wpm.X = wp.X;
    wpm.Y = wp.Y;
    wpm.yaw = wp.yaw;
    wpm.height = wp.height;

    return wpm;
  }

  bool WaypointController::openWayFile(std::string filename)
  {
    std::ifstream f;
    std::string line;

    f.open(filename.c_str());
    if(!f.is_open())
    {
      ROS_ERROR("Could not open waypoint file");
      return false;
    }
   
    std::getline(f, line); // skip header

    while(std::getline(f, line))
    {
      std::vector<std::string> fields;
      size_t cur_pos=0;
      size_t found_pos=0;
      while((found_pos = line.find(';', cur_pos)) != std::string::npos)
      {
        fields.push_back(line.substr(cur_pos, found_pos-cur_pos));
        cur_pos = found_pos+1;
      }
      if(fields.size() < 7)
      {
        f.close();
        ROS_ERROR("Waypoint file is wrong format");
        return false;
      }
      struct Telemetry::WAYPOINT wp;
      wp.wp_number = 1;
      wp.properties = WPROP_ABSCOORDS | WPROP_HEIGHTENABLED | WPROP_AUTOMATICGOTO;
      wp.max_speed = 100;
      wp.time = atof(fields[5].c_str())*100;
      wp.pos_acc = atof(fields[6].c_str())*1000;
      wp.X = atof(fields[2].c_str())*10000000;
      wp.Y = atof(fields[1].c_str())*10000000;
      wp.yaw = atof(fields[4].c_str());
      wp.height = atof(fields[3].c_str())*1000;
      wp.chksum = wp.yaw + wp.height + wp.time + wp.X + wp.Y + wp.max_speed + wp.pos_acc + wp.properties + wp.wp_number + (short) 0xAAAA;

      wps_.push_back(wp);
    }
    f.close();
    cur_wp_ = 0;
    return true;
  }

  bool WaypointController::hasNextWaypoint()
  {
    return cur_wp_ < wps_.size();
  }

  struct Telemetry::WAYPOINT WaypointController::getNextWaypoint()
  {
    cur_wp_++;
    return wps_.at(cur_wp_-1);
  }
}
