/*
 *  AscTec Autopilot Telemetry
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

#ifndef ASCTEC_AUTOPILOT_TELEMETRY_H
#define ASCTEC_AUTOPILOT_TELEMETRY_H

#include "asctec_msgs/LLStatus.h"
#include "asctec_msgs/IMURawData.h"
#include "asctec_msgs/IMUCalcData.h"
#include "asctec_msgs/RCData.h"
#include "asctec_msgs/ControllerOutput.h"
#include "asctec_msgs/GPSData.h"
#include "asctec_msgs/GPSDataAdvanced.h"
#include "asctec_msgs/CtrlInput.h"
#include "asctec_msgs/CurrentWay.h"
#include "asctec_msgs/Waypoint.h"
#include <std_msgs/Bool.h>
#include <bitset>

namespace asctec
{
  namespace RequestTypes
  {
    enum RequestType
    {
      LL_STATUS,
      IMU_RAWDATA,
      IMU_CALCDATA,
      RC_DATA,
      CONTROLLER_OUTPUT,
      GPS_DATA,
      WAYPOINT,
      GPS_DATA_ADVANCED,
      CAM_DATA
    };
  }
  typedef RequestTypes::RequestType RequestType;

/**
 * \brief Telemetry interface for the AscTec AutoPilot.
 *
 * This class provides functions to help build request messages
 * and it also provides a place for both the SerialInterface and
 * the AutoPilot classes to pass telemetry information.
 *
 * The most widely used methods are:
 *   - Setup:
 *    - enablePolling()
 *    - buildRequest()
 *    - ros::init()
 */

  class Telemetry
  {
  public:
  /**
   * \brief Constructor
   *
   * This handles telemetry packet storage and processing.
   *
   */
      Telemetry(ros::NodeHandle nh);
  /**
   * \brief Destructor
   *
   * Please Recycle your electrons.
   */
    ~Telemetry();
    
  /** \brief Enables Polling of a Request Message
   *
   * Due to the limited bandwidth available over the wireless link this function
   * provides a means of selectivly enabling polling of various request messages.
   * The interval argument allows for some messages to be polled more frequently
   * than others, while the offset provides a way to space out the message requests.
   *
   * \param msg Message type to poll
   * \param interval Message Polling Interval (Message Hz = Polling HZ / interval)
   * \param offset (optional) Polling offset (interval = 2 & offset = 1 -> odd polling)
   *
   * \return Void.
   */
    void buildRequest ();
     
  /** \brief Enables Polling of a Request Message
   *
   * Due to the limited bandwidth available over the wireless link this function
   * provides a means of selectivly enabling polling of various request messages.
   * The interval argument allows for some messages to be polled more frequently
   * than others, while the offset provides a way to space out the message requests.
   *
   * \param msg Message type to poll
   * \param interval Message Polling Interval (Message Hz = Polling HZ / interval)
   * \param offset (optional) Polling offset (interval = 2 & offset = 1 -> odd polling)
   *
   * \return Void.
   */
    void enablePolling (RequestType msg, uint8_t interval = 1, uint8_t offset = 0);
    std::string requestToString(RequestTypes::RequestType t);
    void publishPackets();

    void enableControl (Telemetry * telemetry_, uint8_t interval = 1, uint8_t offset = 0);
    void enableWaypoints (Telemetry * telemetry_);

        
    void dumpLL_STATUS();
    void dumpIMU_RAWDATA();
    void dumpIMU_CALCDATA();
    void dumpRC_DATA();
    void dumpCONTROLLER_OUTPUT();
    void dumpGPS_DATA();
    void dumpGPS_DATA_ADVANCED();
    void dumpCTRL_INPUT();

    void copyLL_STATUS();
    void copyIMU_RAWDATA();
    void copyIMU_CALCDATA();
    void copyRC_DATA();
    void copyCONTROLLER_OUTPUT();
    void copyGPS_DATA();
    void copyGPS_DATA_ADVANCED();
    void copyWAYPOINT();
    void copyCTRL_INPUT(asctec_msgs::CtrlInput msg);
    void copyWAYPOINT_INPUT(asctec_msgs::Waypoint msg);
    void estopCallback(const std_msgs::Bool msg);
    
    bool pollingEnabled_;
    bool controlEnabled_;
    bool waypointEnabled_;
    uint16_t requestCount_;
    uint16_t controlCount_;
    std::bitset < 16 > requestPackets_;
    
    static const uint8_t REQUEST_TYPES = 9;
/*
    static const uint16_t REQUEST_BITMASK[REQUEST_TYPES] = {
      0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0080, 0x0100, 0x0200, 0x0800 };
*/
    uint16_t REQUEST_BITMASK[REQUEST_TYPES];
    uint8_t requestInterval_[REQUEST_TYPES];
    uint8_t requestOffset_[REQUEST_TYPES];
    ros::Publisher requestPublisher_[REQUEST_TYPES];
    ros::Time timestamps_[REQUEST_TYPES];

    uint8_t controlInterval_;
    uint8_t controlOffset_;
    ros::Subscriber controlSubscriber_;
    ros::Subscriber estopSubscriber_;

    ros::Subscriber waypointSubscriber_;
    bool wp_received_;
    
    //packet descriptors
    static const uint8_t PD_IMURAWDATA = 0x01;
    static const uint8_t PD_LLSTATUS = 0x02;
    static const uint8_t PD_IMUCALCDATA = 0x03;
    static const uint8_t PD_HLSTATUS = 0x04;
    static const uint8_t PD_DEBUGDATA = 0x05;

    static const uint8_t PD_CTRLOUT = 0x11;
    static const uint8_t PD_FLIGHTPARAMS = 0x12;
    static const uint8_t PD_CTRLCOMMANDS = 0x13;
    static const uint8_t PD_CTRLINTERNAL = 0x14;
    static const uint8_t PD_RCDATA = 0x15;
    static const uint8_t PD_CTRLSTATUS = 0x16;
    static const uint8_t PD_CTRLINPUT = 0x17;
    static const uint8_t PD_CTRLFALCON = 0x18;

    static const uint8_t PD_WAYPOINT = 0x20;
    static const uint8_t PD_CURRENTWAY = 0x21;
    static const uint8_t PD_NMEADATA = 0x22;
    static const uint8_t PD_GPSDATA = 0x23;
    static const uint8_t PD_SINGLEWAYPOINT = 0x24;
    static const uint8_t PD_GOTOCOMMAND = 0x25;
    static const uint8_t PD_LAUNCHCOMMAND = 0x26;
    static const uint8_t PD_LANDCOMMAND = 0x27;
    static const uint8_t PD_HOMECOMMAND = 0x28;
    static const uint8_t PD_GPSDATAADVANCED = 0x29;

    static const uint8_t PD_CAMERACOMMANDS = 0x30;

    struct LL_STATUS
    {
      //battery voltages in mV
      short battery_voltage_1;
      short battery_voltage_2;
      //don’t care
      short status;
      //Controller cycles per second (should be ˜1000)
      short cpu_load;
      //don’t care
      char compass_enabled;
      char chksum_error;
      char flying;
      char motors_on;
      short flightMode;
      //Time motors are turning
      short up_time;
    };

    struct IMU_RAWDATA
    {
      //pressure sensor 24-bit value, not scaled but bias free
      int pressure;
      //16-bit gyro readings; 32768 = 2.5V
      short gyro_x;
      short gyro_y;
      short gyro_z;
      //10-bit magnetic field sensor readings
      short mag_x;
      short mag_y;
      short mag_z;
      //16-bit accelerometer readings
      short acc_x;
      short acc_y;
      short acc_z;
      //16-bit temperature measurement using yaw-gyro internal sensor
      unsigned short temp_gyro;
      //16-bit temperature measurement using ADC internal sensor
      unsigned int temp_ADC;
    };

    struct IMU_CALCDATA
    {
      // angles derived by integration of gyro_outputs, drift compensated by data fusion;
      // -90000..+90000 pitch(nick) and roll, 0..360000 yaw; 1000 = 1 degree
      int angle_nick;
      int angle_roll;
      int angle_yaw;
      // angular velocities, raw values [16 bit] but bias free
      int angvel_nick;
      int angvel_roll;
      int angvel_yaw;
      // acc-sensor outputs, calibrated: -10000..+10000 = -1g..+1g
      short acc_x_calib;
      short acc_y_calib;
      short acc_z_calib;
      // horizontal / vertical accelerations: -10000..+10000 = -1g..+1g
      short acc_x;
      short acc_y;
      short acc_z;
      // reference angles derived by accelerations only: -90000..+90000; 1000 = 1 degree
      int acc_angle_nick;
      int acc_angle_roll;
      // total acceleration measured (10000 = 1g)
      int acc_absolute_value;
      // magnetic field sensors output, offset free and scaled;
      // units not determined, as only the direction of the field vector is taken into account
      int Hx;
      int Hy;
      int Hz;

      //compass reading: angle reference for angle_yaw: 0..360000; 1000 = 1 degree
      int mag_heading;
      //pseudo speed measurements: integrated accelerations, pulled towards zero; units unknown; used for short-term position stabilization
      int speed_x;
      int speed_y;
      int speed_z;
      //height in mm (after data fusion)
      int height;
      //diff. height in mm/s (after data fusion)
      int dheight;
      //diff. height measured by the pressure sensor [mm/s]
      int dheight_reference;
      //height measured by the pressure sensor [mm]
      int height_reference;
    };

    struct RC_DATA
    {
      //channels as read from R/C receiver
      unsigned short channels_in[8];
      //channels bias free, remapped and scaled to 0..4095
      unsigned short channels_out[8];
      //Indicator for valid R/C receiption
      unsigned char lock;
    };

    struct CONTROLLER_OUTPUT
    {
      //attitude controller outputs; 0..200 = -100 ..+100%
      int nick;
      int roll;
      int yaw;
      //current thrust (height controller output); 0..200 = 0..100%
      int thrust;
    };

    struct GPS_DATA
    {
      //latitude/longitude in deg * 10ˆ7
      int latitude;
      int longitude;
      //GPS height in mm
      int height;
      //speed in x (E/W) and y(N/S) in mm/s
      int speed_x;
      int speed_y;
      //GPS heading in deg * 1000
      int heading;
      //accuracy estimates in mm and mm/s
      unsigned int horizontal_accuracy;
      unsigned int vertical_accuracy;
      unsigned int speed_accuracy;
      //number of satellite vehicles used in NAV solution
      unsigned int numSV;
      // GPS status information; 0x03 = valid GPS fix
      int status;
    };

    struct GPS_DATA_ADVANCED
    {
      //latitude/longitude in deg * 10ˆ7
      int latitude;
      int longitude;
      //GPS height in mm
      int height;
      //speed in x (E/W) and y(N/S) in mm/s
      int speed_x;
      int speed_y;
      //GPS heading in deg * 1000
      int heading;
      //accuracy estimates in mm and mm/s
      unsigned int horizontal_accuracy;
      unsigned int vertical_accuracy;
      unsigned int speed_accuracy;
      //number of satellite vehicles used in NAV solution
      unsigned int numSV;

      //GPS status information; 0x03 = valid GPS fix
      int status;
      //coordinates of current origin in deg * 10ˆ7RCData_
      int latitude_best_estimate;
      int longitude_best_estimate;
      //velocities in X (E/W) and Y (N/S) after data fusion
      int speed_x_best_estimate;
      int speed_y_best_estimate;
    };

    struct CURRENT_WAY {
      unsigned char dummy1;
      unsigned char properties;
      unsigned short nr_of_wp; //don't care

      unsigned char current_wp; //don't care
      unsigned char current_wp_memlocation; //don't care

      unsigned char status;  //don't care
      unsigned char dummy2;

      unsigned short navigation_status;          //see WP_NAVSTAT_... defines
      unsigned short distance_to_wp;             //distance to WP in dm
    }; 

    #define WP_NAVSTAT_REACHED_POS        0x01
    #define WP_NAVSTAT_REACHED_POS_TIME   0x02 //vehicle is within a radius of WAYPOINT.pos_acc and time to stay is over
    #define WP_NAVSTAT_20M                0x04 //vehicle within a 20m radius of the waypoint
    #define WP_NAVSTAT_PILOT_ABORT        0x08  //waypoint navigation aborted by safety pilot
    
    struct WAYPOINT
    {
      //always set to 1
      unsigned char wp_number;
      //don't care
      unsigned char dummy_1;
      unsigned short dummy_2;
      //see WPPROP defines below
      unsigned char properties;
      //max. speed to travel to waypoint in % (default 100)
      unsigned char max_speed;
      //time to stay at a waypoint (XYZ) in 1/100th s
      unsigned short time;
      //position accuracy to consider a waypoint reached in mm (default: 2500 (= 2.5 m))
      unsigned short pos_acc;
      //chksum = 0xAAAA + wp.yaw + wp.height + wp.time + wp.X + wp.Y + wp.max_speed + wp.pos_acc + wp.properties + wp.wp_number;
      short chksum;
      //waypoint coordinates in mm // longitude in abs coords
      int X;
      //waypoint coordinates in mm // latitude in abs coords
      int Y;
      //Desired heading at waypoint
      int yaw;
      //height over 0 reference in mm
      int height;
    };

    #define WPROP_ABSCOORDS     0x01
    #define WPROP_HEIGHTENABLED 0x02
    #define WPROP_YAWENABLED    0x04
    #define WPROP_AUTOMATICGOTO 0x10
    #define WPROP_CAM_TRIGGER   0x20


    struct CTRL_INPUT
    {
        //serial commands (= Scientific Interface)
        //pitch input: -2047..2047 (0=neutral)
        short pitch;
        //roll input: -2047..2047 (0=neutral)
        short roll;
        //R/C stick input: -2047..2047 (0=neutral)
        short yaw;
        //collective: 0..4095 = 0..100%
        short thrust;
        //control byte:
        //  bit 0: pitch control enabled
        //  bit 1: roll control enabled
        //  bit 2: yaw control enabled
        //  bit 3: thrust control enabled
        //  These bits can be used to only enable one axis at a time and thus to
        //  control the other axes manually. This usually helps a lot to set up
        //  and finetune controllers for each axis seperately.
        short ctrl;
        short chksum;
    };

/*

#define WPPROP_ABSCOORDS 0x01 //if set waypoint is interpreted as
absolute coordinates, else relative coords
#define WPPROP_HEIGHTENABLED 0x02 //set new height at waypoint
#define WPPROP_YAWENABLED 0x04 //set new yaw-angle at waypoint
(not yet implemented)
#define WPPROP_AUTOMATICGOTO 0x10 //if set, vehicle will not wait for
a goto command, but goto this waypoint directly
#define WPPROP_CAM_TRIGGER 0x20 //if set, photo camera is triggered
when waypoint is reached and time to stay is 80% up
Sending the waypoint structure to the vehicle:
The following string must be sent to the vehicle, directly followed by the actual waypoint
structure:
unsigned char string[]=">*>ws";
Commands for the waypoint navigation:
>*>wg "Goto waypoint"
>*>wl "Launch / Set Home
>*>we "End flight => land at current position"
>*>wh "Come home"
Sending the launch command when the vehicle is hovering with the switch on the R/C in
"GPS + Height control" sets the home position.
You will receive an acknowledge if a command or a waypoint was received correctly:
>a[1 byte packet descriptor]a<

*/

    struct LL_STATUS LL_STATUS_;
    struct IMU_RAWDATA IMU_RAWDATA_;
    struct IMU_CALCDATA IMU_CALCDATA_;
    struct RC_DATA RC_DATA_;
    struct CONTROLLER_OUTPUT CONTROLLER_OUTPUT_;
    struct GPS_DATA GPS_DATA_;
    struct GPS_DATA_ADVANCED GPS_DATA_ADVANCED_;
    struct CURRENT_WAY WAYPOINT_;
    struct CTRL_INPUT CTRL_INPUT_;
    struct WAYPOINT WAYPOINT_INPUT_;
    asctec_msgs::LLStatusPtr LLStatus_;
    asctec_msgs::IMURawDataPtr IMURawData_;
    asctec_msgs::IMUCalcDataPtr IMUCalcData_;
    asctec_msgs::RCDataPtr RCData_;
    asctec_msgs::ControllerOutputPtr ControllerOutput_;
    asctec_msgs::GPSDataPtr GPSData_;
    asctec_msgs::GPSDataAdvancedPtr GPSDataAdvanced_;
    asctec_msgs::CurrentWayPtr CurrentWay_;

    ros::NodeHandle nh_;
    //asctec_msgs::CtrlInput CtrlInput_;
    bool estop_;
    
  };                            // end class Telemetry
}                               //end namespace asctec
#endif
