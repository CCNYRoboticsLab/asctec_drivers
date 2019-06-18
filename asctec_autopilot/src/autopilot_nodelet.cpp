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

#include "asctec_autopilot/autopilot_nodelet.h"
#include <pluginlib/class_list_macros.h>

typedef asctec::AutoPilotNodelet AutoPilotNodelet;

PLUGINLIB_EXPORT_CLASS (asctec::AutoPilotNodelet, nodelet::Nodelet);

void asctec::AutoPilotNodelet::onInit ()
{
  NODELET_INFO("Initializing Autopilot Nodelet");

  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh         = getMTNodeHandle();
  ros::NodeHandle nh_private = getMTPrivateNodeHandle();

  autopilot = new asctec::AutoPilot(nh, nh_private);  
}
