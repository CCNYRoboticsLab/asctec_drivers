/*
 *  AscTec Autopilot Processor
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
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

#include "asctec_proc/asctec_proc.h"

namespace asctec
{

AsctecProc::AsctecProc(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private)
{
  ROS_INFO("Starting AsctecProc");

  ros::NodeHandle nh_rawdata  (nh_, asctec::ROS_NAMESPACE);
  ros::NodeHandle nh_procdata (nh_, "mav");

  // **** get parameters

  initializeParams();

  // **** initialize vaiables

  motors_on_ = false;
  engaging_ = false;

  ctrl_roll_ = 0;
  ctrl_pitch_ = 0;
  ctrl_yaw_ = 0;
  ctrl_thrust_ = 0;

  assembleCtrlCommands();

  // *** register publishers

  imu_publisher_  = nh_procdata.advertise<sensor_msgs::Imu>(
    "imu", 10);
  height_publisher_ = nh_procdata.advertise<mav_msgs::Height>(
    "pressure_height", 10);
  height_filtered_publisher_ = nh_procdata.advertise<mav_msgs::Height>(
    "pressure_height_filtered", 10);
  ctrl_input_publisher_ = nh_rawdata.advertise<asctec_msgs::CtrlInput>(
    asctec::CTRL_INPUT_TOPIC, 10);

  // **** register subscribers

  imu_calcdata_subscriber_ = nh_rawdata.subscribe(
    asctec::IMU_CALCDATA_TOPIC, 10, &AsctecProc::imuCalcDataCallback, this);
  ll_status_subscriber_ = nh_rawdata.subscribe(
    asctec::LL_STATUS_TOPIC, 5, &AsctecProc::llStatusCallback, this);

  if (enable_ctrl_thrust_)
  {
    cmd_thrust_subscriber_ = nh_procdata.subscribe(
      "cmd_thrust", 1, &AsctecProc::cmdThrustCallback, this);
  }
  if (enable_ctrl_roll_)
  {
    cmd_roll_subscriber_ = nh_procdata.subscribe(
      "cmd_roll", 1, &AsctecProc::cmdRollCallback, this);
  }
  if (enable_ctrl_pitch_)
  {
    cmd_pitch_subscriber_ = nh_procdata.subscribe(
      "cmd_pitch", 1, &AsctecProc::cmdPitchCallback, this);
  }
  if (enable_ctrl_yaw_)
  {
    cmd_yaw_subscriber_ = nh_procdata.subscribe(
      "cmd_yaw", 5, &AsctecProc::cmdYawCallback, this);
  }

  // **** services

  if(enable_state_changes_)
  {
    set_motors_on_off_srv_ = nh_procdata.advertiseService(
      "setMotorsOnOff", &AsctecProc::setMotorsOnOff, this);
  }

  get_motors_on_off_srv_ = nh_procdata.advertiseService(
    "getMotorsOnOff", &AsctecProc::getMotorsOnOff, this);
}

AsctecProc::~AsctecProc()
{
  ROS_INFO("Destroying AsctecProc");

}

void AsctecProc::initializeParams()
{
  if (!nh_private_.getParam ("enable_state_changes", enable_state_changes_))
    enable_state_changes_ = false;

  if (!nh_private_.getParam ("enable_ctrl_thrust", enable_ctrl_thrust_))
    enable_ctrl_thrust_ = false;
  if (!nh_private_.getParam ("enable_ctrl_pitch", enable_ctrl_pitch_))
    enable_ctrl_pitch_ = false;
  if (!nh_private_.getParam ("enable_ctrl_roll", enable_ctrl_roll_))
    enable_ctrl_roll_ = false;
  if (!nh_private_.getParam ("enable_ctrl_yaw", enable_ctrl_yaw_))
    enable_ctrl_yaw_ = false;

  if (!nh_private_.getParam ("max_ctrl_thrust", max_ctrl_thrust_))
    max_ctrl_thrust_ = 2200;
  if (!nh_private_.getParam ("max_ctrl_roll", max_ctrl_roll_))
    max_ctrl_roll_ = 300;
  if (!nh_private_.getParam ("max_ctrl_pitch", max_ctrl_pitch_))
    max_ctrl_pitch_ = 300;
  if (!nh_private_.getParam ("max_ctrl_yaw", max_ctrl_yaw_))
    max_ctrl_yaw_ = 600;
}

bool AsctecProc::setMotorsOnOff(mav_msgs::SetMotorsOnOff::Request  &req,
                                mav_msgs::SetMotorsOnOff::Response &res)
{
  state_mutex_.lock();
  engaging_ = true;

  if (req.on && !motors_on_)
  {
    ctrl_roll_ = 0;
    ctrl_pitch_ = 0;
    ctrl_yaw_ = 0;
    ctrl_thrust_ = 0;
    startMotors();
  }
  else
  {
    stopMotors();
  }

  engaging_ = false;
  state_mutex_.unlock();

  return (req.on == motors_on_);
}

bool AsctecProc::getMotorsOnOff(mav_msgs::GetMotorsOnOff::Request  &req,
                                mav_msgs::GetMotorsOnOff::Response &res)
{
  state_mutex_.lock();
  res.on = motors_on_;
  state_mutex_.unlock();

  return true;
}

void AsctecProc::llStatusCallback (const asctec_msgs::LLStatusPtr& ll_status_msg)
{
  // save the state of the motors
  motors_on_ = ll_status_msg->flying;
}

void AsctecProc::cmdRollCallback(const std_msgs::Float64ConstPtr& cmd_roll_msg)
{
  if (!motors_on_ || engaging_) return;

  state_mutex_.lock();

  // translate from cmd_roll [-1.0 to 1.0] to ctrl_roll [-2047 .. 2047],
  ctrl_roll_ = (int)(cmd_roll_msg->data * asctec::ROS_TO_ASC_ROLL);

  ROS_INFO ("cmd_roll received: %f (%d)", cmd_roll_msg->data, ctrl_roll_);

  // limit min/max output
  if (ctrl_roll_ > max_ctrl_roll_)
  {
    ROS_WARN("ctrl_roll of %d too big, clamping to %d!", ctrl_roll_, max_ctrl_roll_);
    ctrl_roll_ = max_ctrl_roll_;
  }
  else if (ctrl_roll_ < -max_ctrl_roll_)
  {
    ROS_WARN("ctrl_roll of %d too small, clamping to -%d!", ctrl_roll_, -max_ctrl_roll_);
    ctrl_roll_ = -max_ctrl_roll_;
  }

  publishCtrlInputMsg();

  state_mutex_.unlock();
}

void AsctecProc::cmdPitchCallback(const std_msgs::Float64ConstPtr& cmd_pitch_msg)
{
  if (!motors_on_ || engaging_) return;

  state_mutex_.lock();

  // translate from cmd_pitch [-1.0 to 1.0] to ctrl_pitch [-2047 .. 2047],
  ctrl_pitch_ = (int)(cmd_pitch_msg->data * asctec::ROS_TO_ASC_PITCH);

  ROS_DEBUG ("cmd_pitch received: %f (%d)", cmd_pitch_msg->data, ctrl_pitch_);

  // limit min/max output
  if (ctrl_pitch_ > max_ctrl_pitch_)
  {
    ROS_WARN("ctrl_pitch of %d too big, clamping to %d!", ctrl_pitch_, max_ctrl_pitch_);
    ctrl_pitch_ = max_ctrl_pitch_;
  }
  else if (ctrl_pitch_ < -max_ctrl_pitch_)
  {
    ROS_WARN("ctrl_pitch of %d too small, clamping to -%d!", ctrl_pitch_, -max_ctrl_pitch_);
    ctrl_pitch_ = -max_ctrl_pitch_;
  }

  publishCtrlInputMsg();

  state_mutex_.unlock();
}

void AsctecProc::cmdYawCallback(const std_msgs::Float64ConstPtr& cmd_yaw_rate_msg)
{
  if (!motors_on_ || engaging_) return;

  state_mutex_.lock();

  // translate from cmd_yaw [rad/s] to ctrl_yaw [-2047 .. 2047],
  ctrl_yaw_ = (int)(cmd_yaw_rate_msg->data * asctec::ROS_TO_ASC_YAW_RATE);

  ROS_DEBUG ("cmd_yaw received: %f (%d)", cmd_yaw_rate_msg->data, ctrl_yaw_);

  // limit min/max output
  if (ctrl_yaw_ > max_ctrl_yaw_)
  {
    ROS_WARN("ctrl_yaw of %d too big, clamping to %d!", ctrl_yaw_, max_ctrl_yaw_);
    ctrl_yaw_ = max_ctrl_yaw_;
  }
  else if (ctrl_yaw_ < -max_ctrl_yaw_)
  {
    ROS_WARN("ctrl_yaw of %d too small, clamping to -%d!", ctrl_yaw_, -max_ctrl_yaw_);
    ctrl_yaw_ = -max_ctrl_yaw_;
  }

  publishCtrlInputMsg();

  state_mutex_.unlock();
}

void AsctecProc::cmdThrustCallback(const std_msgs::Float64ConstPtr& cmd_thrust_msg)
{
  if (!motors_on_ || engaging_) return;

  state_mutex_.lock();

  // translate from cmd_thrust [0.0 to 1.0] to ctrl_thrust [0 to 4095],
  ctrl_thrust_ = (int)(cmd_thrust_msg->data * asctec::ROS_TO_ASC_THRUST);

  ROS_DEBUG ("cmd_thrust received: %f (%d)", cmd_thrust_msg->data, ctrl_thrust_);

  // limit min-max output
  if (ctrl_thrust_ > max_ctrl_thrust_)
  {
    ROS_WARN("ctrl_thrust of %d too big, clamping to %d!", ctrl_thrust_, max_ctrl_thrust_);
    ctrl_thrust_ = max_ctrl_thrust_;
  }
  else if (ctrl_thrust_ < 0)
  {
    ROS_WARN("ctrl_thrust of %d too small, clamping to 0!", ctrl_thrust_);
    ctrl_thrust_ = 0;
  }

  publishCtrlInputMsg();

  state_mutex_.unlock();
}

void AsctecProc::imuCalcDataCallback(const asctec_msgs::IMUCalcDataConstPtr& imu_calcdata_msg)
{
  // publish imu message
  sensor_msgs::ImuPtr imu_msg;
  imu_msg = boost::make_shared<sensor_msgs::Imu>();
  createImuMsg (imu_calcdata_msg, imu_msg);
  imu_publisher_.publish(imu_msg);

  // publish unfiltered height message
  mav_msgs::HeightPtr height_msg;
  height_msg = boost::make_shared<mav_msgs::Height>();
  createHeightMsg (imu_calcdata_msg, height_msg);
  height_publisher_.publish(height_msg);

  // publish filtered height message
  mav_msgs::HeightPtr height_filtered_msg;
  height_filtered_msg = boost::make_shared<mav_msgs::Height>();
  createHeightFilteredMsg (imu_calcdata_msg, height_filtered_msg);
  height_filtered_publisher_.publish(height_filtered_msg);
}

void AsctecProc::createHeightMsg(const asctec_msgs::IMUCalcDataConstPtr& imu_calcdata_msg,
                                       mav_msgs::HeightPtr& height_msg)
{
  // set header info
  height_msg->header.stamp    = imu_calcdata_msg->header.stamp;
  height_msg->header.frame_id = "imu"; // the frame seems arbitrary here

  height_msg->height = imu_calcdata_msg->height_reference  * asctec::ASC_TO_ROS_HEIGHT;
  height_msg->climb  = imu_calcdata_msg->dheight_reference * asctec::ASC_TO_ROS_HEIGHT;
}

void AsctecProc::createHeightFilteredMsg(const asctec_msgs::IMUCalcDataConstPtr& imu_calcdata_msg,
                                               mav_msgs::HeightPtr& height_filtered_msg)
{
  // set header info
  height_filtered_msg->header.stamp    = imu_calcdata_msg->header.stamp;
  height_filtered_msg->header.frame_id = "imu"; // the frame seems arbitrary here

  height_filtered_msg->height = imu_calcdata_msg->height  * asctec::ASC_TO_ROS_HEIGHT;
  height_filtered_msg->climb  = imu_calcdata_msg->dheight * asctec::ASC_TO_ROS_HEIGHT;
}

void AsctecProc::createImuMsg(const asctec_msgs::IMUCalcDataConstPtr& imu_calcdata_msg,
                                    sensor_msgs::ImuPtr& imu_msg)
{
  // set header info
  imu_msg->header.stamp    = imu_calcdata_msg->header.stamp;
  imu_msg->header.frame_id = "imu";

  // copy over linear acceleration
  imu_msg->linear_acceleration.x = imu_calcdata_msg->acc_x_calib * asctec::ASC_TO_ROS_ACC * -1.0;
  imu_msg->linear_acceleration.y = imu_calcdata_msg->acc_y_calib * asctec::ASC_TO_ROS_ACC * -1.0;
  imu_msg->linear_acceleration.z = imu_calcdata_msg->acc_z_calib * asctec::ASC_TO_ROS_ACC * -1.0;

/* // Uncomment these if you use covariances
  // define linear acceleration variance
  imuMsg->linear_acceleration_covariance[0] = 1.0;
  imuMsg->linear_acceleration_covariance[1] = 0.0;
  imuMsg->linear_acceleration_covariance[2] = 0.0;
  imuMsg->linear_acceleration_covariance[3] = 0.0;
  imuMsg->linear_acceleration_covariance[4] = 1.0;
  imuMsg->linear_acceleration_covariance[5] = 0.0;
  imuMsg->linear_acceleration_covariance[6] = 0.0;
  imuMsg->linear_acceleration_covariance[7] = 0.0;
  imuMsg->linear_acceleration_covariance[8] = 1.0;
*/
  // copy over angular_velocity - minus signs convert to ENU frame
  imu_msg->angular_velocity.x = imu_calcdata_msg->angvel_roll * asctec::ASC_TO_ROS_ANGVEL * -1.0;
  imu_msg->angular_velocity.y = imu_calcdata_msg->angvel_nick * asctec::ASC_TO_ROS_ANGVEL;
  imu_msg->angular_velocity.z = imu_calcdata_msg->angvel_yaw  * asctec::ASC_TO_ROS_ANGVEL * -1.0;

/* // Uncomment these if you use covariances
  // define angular_velocity variance
  imuMsg->angular_velocity_covariance[0] = 1.0;
  imuMsg->angular_velocity_covariance[1] = 0.0;
  imuMsg->angular_velocity_covariance[2] = 0.0;
  imuMsg->angular_velocity_covariance[3] = 0.0;
  imuMsg->angular_velocity_covariance[4] = 1.0;
  imuMsg->angular_velocity_covariance[5] = 0.0;
  imuMsg->angular_velocity_covariance[6] = 0.0;
  imuMsg->angular_velocity_covariance[7] = 0.0;
  imuMsg->angular_velocity_covariance[8] = 1.0;
*/

  // calculate quaternion orientation - minus signs convert to ENU frame
  tf::Quaternion orientation;
  orientation.setRPY(imu_calcdata_msg->angle_roll * asctec::ASC_TO_ROS_ANGLE * -1.0,
                     imu_calcdata_msg->angle_nick * asctec::ASC_TO_ROS_ANGLE,
                     imu_calcdata_msg->angle_yaw  * asctec::ASC_TO_ROS_ANGLE * -1.0);

  imu_msg->orientation.x = orientation.getX();
  imu_msg->orientation.y = orientation.getY();
  imu_msg->orientation.z = orientation.getZ();
  imu_msg->orientation.w = orientation.getW();
}

void AsctecProc::startMotors()
{
  // set the stick to lower left, wait for motors to engage,
  // and reset stick

  ROS_INFO ("Starting motors...");

  ctrl_input_publisher_.publish(ctrl_input_toggle_msg_);

  for (int i = 0; i < 15; ++i)
  {
    if (motors_on_) break;
    //printf("\tt\n");
    ros::Duration(0.20).sleep();
    ctrl_input_publisher_.publish(ctrl_input_toggle_msg_);
  }

  ctrl_input_publisher_.publish(ctrl_input_zero_msg_);

  ROS_INFO("Motors are ON");
}

void AsctecProc::stopMotors()
{
  // set the stick to lower left, wait for motors to disengage,
  // and reset stick

  ROS_INFO ("Stopping motors...");

  ctrl_input_publisher_.publish(ctrl_input_toggle_msg_);

  for (int i = 0; i < 15; ++i)
  {
    if (!motors_on_) break;
    //printf("\tt\n");
    ros::Duration(0.20).sleep();
    ctrl_input_publisher_.publish(ctrl_input_toggle_msg_);
  }

  ctrl_input_publisher_.publish(ctrl_input_zero_msg_);

  ROS_INFO("Motors are OFF");
}

void AsctecProc::assembleCtrlCommands()
{
  // **** Assemble toggle-motors message

  ctrl_input_toggle_msg_ = boost::make_shared<asctec_msgs::CtrlInput>();

  ctrl_input_toggle_msg_->thrust = 0;
  ctrl_input_toggle_msg_->roll   = 0;
  ctrl_input_toggle_msg_->pitch  = 0;
  ctrl_input_toggle_msg_->yaw    = -2047;
  ctrl_input_toggle_msg_->ctrl   = int(0b1100);

  ctrl_input_toggle_msg_->chksum = ctrl_input_toggle_msg_->roll + ctrl_input_toggle_msg_->pitch  +
                                   ctrl_input_toggle_msg_->yaw  + ctrl_input_toggle_msg_->thrust +
                                   ctrl_input_toggle_msg_->ctrl - 21846;

  // **** Assemble zero message

  ctrl_input_zero_msg_ = boost::make_shared<asctec_msgs::CtrlInput>();

  ctrl_input_zero_msg_->thrust = 0;
  ctrl_input_zero_msg_->roll   = 0;
  ctrl_input_zero_msg_->pitch  = 0;
  ctrl_input_zero_msg_->yaw    = 0;
  ctrl_input_zero_msg_->ctrl   = int(0b1100);

  ctrl_input_zero_msg_->chksum = ctrl_input_zero_msg_->roll + ctrl_input_zero_msg_->pitch  +
                                 ctrl_input_zero_msg_->yaw  + ctrl_input_zero_msg_->thrust +
                                 ctrl_input_zero_msg_->ctrl - 21846;
}

void AsctecProc::publishCtrlInputMsg()
{
  ROS_DEBUG("Publishing ctrl_input_msg");

  // **** Assemble the generic control input message

  asctec_msgs::CtrlInputPtr ctrl_input_msg;
  ctrl_input_msg = boost::make_shared<asctec_msgs::CtrlInput>();

  ctrl_input_msg->thrust = ctrl_thrust_;
  ctrl_input_msg->roll   = ctrl_roll_;
  ctrl_input_msg->pitch  = ctrl_pitch_;
  ctrl_input_msg->yaw    = ctrl_yaw_;
  ctrl_input_msg->ctrl  = int(0b0000);

  if (enable_ctrl_thrust_) ctrl_input_msg->ctrl |= 0b1000; // These are from CtrlInput.msg
  if (enable_ctrl_yaw_)    ctrl_input_msg->ctrl |= 0b0100;
  if (enable_ctrl_roll_)   ctrl_input_msg->ctrl |= 0b0010;
  if (enable_ctrl_pitch_)  ctrl_input_msg->ctrl |= 0b0001;

  // update checksum and timestamp, and publish
  ctrl_input_msg->chksum = ctrl_input_msg->roll + ctrl_input_msg->pitch  +
                            ctrl_input_msg->yaw  + ctrl_input_msg->thrust +
                            ctrl_input_msg->ctrl - 21846;
  ctrl_input_msg->header.stamp = ros::Time::now();
  ctrl_input_publisher_.publish(ctrl_input_msg);
}

} // end namespace asctec
