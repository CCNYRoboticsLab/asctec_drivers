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


#ifndef WAYPOINT_CONTROLLER_H_
#define WAYPOINT_CONTROLLER_H_

#include <ros/ros.h>
#include <string>
#include <vector>
#include "asctec_autopilot/telemetry.h"


namespace asctec
{
  class WaypointController
  {
  public:
    WaypointController(ros::NodeHandle nh, ros::NodeHandle nh_private);
    void spin(const ros::TimerEvent & e);
    void handleCurrentway(asctec_msgs::CurrentWay msg);

  private:
    bool openWayFile(std::string filename);
    bool hasNextWaypoint();
    struct Telemetry::WAYPOINT getNextWaypoint(); 
    asctec_msgs::Waypoint getNextMessage();

    ros::Timer timer_;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber currentwaySubscriber_;
    ros::Publisher waypointPublisher_;

    std::vector< struct Telemetry::WAYPOINT > wps_;
    unsigned int cur_wp_;
    std::string waypointFile_;
  };
}
#endif
