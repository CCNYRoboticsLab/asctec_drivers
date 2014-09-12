/*
 *  AscTec Autopilot Interface
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

#ifndef ASCTEC_AUTOPILOT_AUTOPILOT_H
#define ASCTEC_AUTOPILOT_AUTOPILOT_H

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
#include <diagnostic_updater/diagnostic_updater.h>
#include <asctec_msgs/common.h> // for namespace and topic names

#include "asctec_autopilot/crc16.h"
#include "asctec_autopilot/telemetry.h"
#include "asctec_autopilot/serialinterface.h"

//const std::string rawdata_namespace_ = "asctec";

namespace asctec
{
  class AutoPilot
  {
    private:

      ros::Timer timer_;
      ros::NodeHandle nh_;
      ros::NodeHandle nh_private_;
    
      double freq_;
      std::string port_;
      int speed_;
      bool enable_LL_STATUS_;
      int interval_LL_STATUS_;
      int offset_LL_STATUS_;
      bool enable_IMU_RAWDATA_;
      int interval_IMU_RAWDATA_;
      int offset_IMU_RAWDATA_;
      bool enable_IMU_CALCDATA_;
      int interval_IMU_CALCDATA_;
      int offset_IMU_CALCDATA_;
      bool enable_RC_DATA_;
      int interval_RC_DATA_;
      int offset_RC_DATA_;
      bool enable_CONTROLLER_OUTPUT_;
      int interval_CONTROLLER_OUTPUT_;
      int offset_CONTROLLER_OUTPUT_;
      bool enable_GPS_DATA_;
      int interval_GPS_DATA_;
      int offset_GPS_DATA_;
      bool enable_GPS_DATA_ADVANCED_;
      int interval_GPS_DATA_ADVANCED_;
      int offset_GPS_DATA_ADVANCED_;
      bool enable_CONTROL_;
      int interval_CONTROL_;
      int offset_CONTROL_;
      bool enable_CURRENTWAY_;
      int interval_CURRENTWAY_;
      int offset_CURRENTWAY_;
      bool enable_WAYPOINT_;
      int interval_WAYPOINT_;
      int offset_WAYPOINT_;
 
      SerialInterface* serialInterface_;
      Telemetry* telemetry_;


      // Diagnostics
      diagnostic_updater::Updater diag_updater_;
      double last_spin_time_;

      bool first_wp_;

      void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

    public:

      AutoPilot (ros::NodeHandle nh, ros::NodeHandle nh_private);
      virtual ~AutoPilot();

      void enablePolling (uint16_t request, uint16_t interval);
      void spin (const ros::TimerEvent & e);
  }; // end class AutoPilot
} //end namespace asctec_autopilot

#endif
