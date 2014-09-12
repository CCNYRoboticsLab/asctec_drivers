/*
 *  AscTec Waypoint Controller Node
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

#include "asctec_autopilot/waypointcontroller.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "autopilot");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  asctec::WaypointController autopilot(nh, nh_private);
  ros::spin ();
  return 0;
}
