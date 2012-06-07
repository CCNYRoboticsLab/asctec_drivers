/*

Copyright (c) 2011, Markus Achtelik, ASL, ETH Zurich, Switzerland
You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <mav_common/comm_packets.h>
#include <mav_common/comm.h>
#include <mav_common/comm_types.h>

#include "sdk.h"
#include "main.h"

#include "system.h"
#include "LL_HL_comm.h"
#include "uart.h"
#include "time.h"
#include "irq.h"
#include "LPC214x.h"
#include "gpsmath.h"

struct WO_SDK_STRUCT WO_SDK;
struct WO_CTRL_INPUT WO_CTRL_Input;
struct RO_RC_DATA RO_RC_Data;
struct WO_DIRECT_MOTOR_CONTROL WO_Direct_Motor_Control;

volatile int64_t g_timestamp  = 0;

int64_t timeOffset = 0;
unsigned short time_step = 2000;
int64_t time_correction = 0;
extern float g_vz_p_f;
#define MAX_TOGGLE_CMD_TIME 2000000  // maximum toggle time allowed - 2s

unsigned int g_sdk_loops;             // SDK loops counter
float g_cpu_load_sum = 0.0;           // for filtered CPU load

MAV_STATUS_PKT        g_status_pkt;         // Pose + vel of MAV, from output of KF or directly from computer updates
MAV_CTRL_CMD          g_ctrl_cmd;
MAV_FLIGHT_STATE_PKT  g_flight_state_pkt;
MAV_RCDATA_PKT        g_rcdata_pkt;
MAV_CTRL_DEBUG_PKT    g_ctrl_debug_pkt;
MAV_POSE_PKT          g_pose_pkt;           // 9D state (pose + vel) of MAV in Comm unit
MAV_IMU_PKT           g_imu_pkt;            // imu (angles and linear accelerations)

MAV_DUMMY_PKT         g_dummy_pkt;
PacketInfo *          g_dummy_pkt_info;

MAV_FLIGHT_ACTION_PKT g_flight_action_pkt;
PacketInfo *          g_flight_action_pkt_info;

MAV_TIMESYNC_PKT      g_timesync_pkt;
PacketInfo *          g_timesync_pkt_info;

MAV_TX_FREQ_CFG_PKT   g_tx_freq_cfg_pkt;
PacketInfo *          g_tx_freq_cfg_pkt_info;

MAV_PID_CFG_PKT       g_pid_cfg_pkt;
PacketInfo *          g_pid_cfg_pkt_info;

MAV_CTRL_CFG_PKT      g_ctrl_cfg_pkt;
PacketInfo *          g_ctrl_cfg_pkt_info;

MAV_CTRL_INPUT_PKT    g_ctrl_input_pkt;
PacketInfo *          g_ctrl_input_pkt_info;

MAV_DES_POSE_PKT      g_des_pose_pkt;
PacketInfo *          g_des_pose_pkt_info;

MAV_DES_VEL_PKT       g_des_vel_pkt;
PacketInfo *          g_des_vel_pkt_info;

MAV_POSE2D_PKT        g_mav_pose2D_pkt;
PacketInfo *          g_mav_pose2D_pkt_info;

MAV_HEIGHT_PKT        g_mav_height_pkt;
PacketInfo *          g_mav_height_pkt_info;

MAV_KF_CFG_PKT        g_mav_kf_cfg_pkt;
PacketInfo *          g_mav_kf_cfg_pkt_info;

// *** for MAV state machine

short   g_motors_running;             // are the motors on?
int64_t g_toggle_motors_start_time;   // when we started toggling the motors
//int16_t g_land_thrust;                // while landing, this is the current thrust  // TODO: type

// *** for KF state estimation

uint8_t g_kf_x_enabled;
uint8_t g_kf_y_enabled;
uint8_t g_kf_z_enabled;
uint8_t g_kf_yaw_enabled;

void sdkInit(void)
{
  g_sdk_loops = 0;
  g_motors_running  = 0;

  // **** these should be sent by the CPU upon successful connection

  g_tx_freq_cfg_pkt.imu_period          = 0;
  g_tx_freq_cfg_pkt.rcdata_period       = 0;
  g_tx_freq_cfg_pkt.flight_state_period = 0;
  g_tx_freq_cfg_pkt.pose_period         = 0;
  g_tx_freq_cfg_pkt.status_period       = 0;
  g_tx_freq_cfg_pkt.ctrl_debug_period   = 0;

  g_tx_freq_cfg_pkt.imu_phase           = 0;
  g_tx_freq_cfg_pkt.rcdata_phase        = 0;
  g_tx_freq_cfg_pkt.flight_state_phase  = 0;
  g_tx_freq_cfg_pkt.pose_phase          = 0;
  g_tx_freq_cfg_pkt.status_phase        = 0;
  g_tx_freq_cfg_pkt.ctrl_debug_phase    = 0;

  // **** register packets to receive

  g_dummy_pkt_info         = registerPacket(MAV_DUMMY_PKT_ID,         &g_dummy_pkt);
  g_mav_pose2D_pkt_info    = registerPacket(MAV_POSE2D_PKT_ID,        &g_mav_pose2D_pkt);
  g_mav_height_pkt_info    = registerPacket(MAV_HEIGHT_PKT_ID,        &g_mav_height_pkt);
  g_mav_kf_cfg_pkt_info    = registerPacket(MAV_KF_CFG_PKT_ID,        &g_mav_kf_cfg_pkt);
  g_timesync_pkt_info      = registerPacket(MAV_TIMESYNC_PKT_ID,      &g_timesync_pkt);
  g_ctrl_cfg_pkt_info      = registerPacket(MAV_CTRL_CFG_PKT_ID,      &g_ctrl_cfg_pkt);
  g_pid_cfg_pkt_info       = registerPacket(MAV_PID_CFG_PKT_ID,       &g_pid_cfg_pkt);
  g_flight_action_pkt_info = registerPacket(MAV_FLIGHT_ACTION_PKT_ID, &g_flight_action_pkt);
  g_des_pose_pkt_info      = registerPacket(MAV_DES_POSE_PKT_ID,      &g_des_pose_pkt);
  g_ctrl_input_pkt_info    = registerPacket(MAV_CTRL_INPUT_PKT_ID,    &g_ctrl_input_pkt);
  g_tx_freq_cfg_pkt_info   = registerPacket(MAV_TX_FREQ_CFG_PKT_ID,   &g_tx_freq_cfg_pkt);
  g_des_vel_pkt_info       = registerPacket(MAV_DES_VEL_PKT_ID,      &g_des_vel_pkt);

  UART0_rxFlush();
  UART0_txFlush();

  startAutoBaud();
}

/** SDK_mainloop(void) is triggered @ 1kHz.
 *
 * WO_(Write Only) data is written to the LL processor after
 * execution of this function.
 *
 * RO_(Read Only) data is updated before entering this function
 * and can be read to obtain information for supervision or control
 *
 * WO_ and RO_ structs are defined in sdk.h
 *
 * The struct LL_1khz_attitude_data (defined in LL_HL_comm.h) can
 * be used to read all sensor data, results of the data fusion
 * and R/C inputs transmitted from the LL-processor. This struct is
 * automatically updated at 1 kHz.
 * */

void SDK_mainloop(void)
{
  unsigned int sdk_cycle_start_time = T1TC;
  WO_SDK.ctrl_mode = 0x00; //0x00: absolute angle and throttle control

  ++g_sdk_loops;

  // add beeping to mark stay-alive

  feedbackBeep();

  // parse serial port for data

  parseRxFifo();

  // process pose updates and fusion using KalmanFilter

  processKF();

  // process control commands - from PID or direct motor control

  processCtrl();

  // process changes of motor state - if motors change from ON to OFF, or
  // from OFF to ON (from LL read only structs) then the flight state of the
  // vehicle is updated accordingly

  processMotorStateChanges();

  // process Flight action requests
  // only when serial is enabled (rcdata[4])

  processFlightActionRequests();

  // process engage/disengage timeouts
  // only allow toggle motors commands to be sent for a certain period of time
  // after a timeout, go to error state

  processEngageDisengageTimeouts();

  // process landing speed
  // gradual landing, thrust decreases over time

  processLandingThrust();

  // determine motor commands based on the flight state

  processMotorCommands();

  // check to send packet data over serial port

  processSendData();

  // *************************************************************************

  UART_send_ringbuffer();

  //synchronizeTime();

  // ------------------------------------------------------------------------

  unsigned int dt;
  if (T1TC < sdk_cycle_start_time)
    dt = (processorClockFrequency() - sdk_cycle_start_time) + T1TC;
  else
    dt = T1TC - sdk_cycle_start_time;

  // calculate average cpu load in %
  float cpu_load = ControllerCyclesPerSecond * ((dt * 1e2) / processorClockFrequency());
  g_cpu_load_sum += cpu_load;
  
  if (g_sdk_loops % 100 == 0)
  {
    g_status_pkt.cpu_load = g_cpu_load_sum / 100.0;
    g_cpu_load_sum = 0.0;
  }

  g_status_pkt.battery_voltage = HL_Status.battery_voltage_1 / 1000.0; //mv to volts
  g_status_pkt.timestamp = g_timestamp;

  //watchdog();
}

inline void writeCommand(short pitch, short roll, short yaw, short thrust, short ctrl, short enable)
{
  WO_CTRL_Input.pitch  = pitch;
  WO_CTRL_Input.roll   = roll;
  WO_CTRL_Input.thrust = thrust;
  WO_CTRL_Input.yaw    = yaw;
  WO_CTRL_Input.ctrl   = ctrl;
  WO_SDK.ctrl_enabled  = enable;
}

inline void sendMavPoseData(void)
{
  writePacket2Ringbuffer(MAV_POSE_PKT_ID, (unsigned char*)&g_pose_pkt, sizeof(g_pose_pkt));
}

inline void sendImuData(void)
{
  g_imu_pkt.roll  = LLToSIAngleRoll (LL_1khz_attitude_data.angle_roll);
  g_imu_pkt.pitch = LLToSIAnglePitch(LL_1khz_attitude_data.angle_pitch);
  g_imu_pkt.yaw   = LLToSIAngleYaw  (LL_1khz_attitude_data.angle_yaw);
  g_imu_pkt.roll_rate  = LLToSIAngleRateRoll (LL_1khz_attitude_data.angvel_roll);
  g_imu_pkt.pitch_rate = LLToSIAngleRatePitch(LL_1khz_attitude_data.angvel_pitch);
  g_imu_pkt.yaw_rate   = LLToSIAngleRateYaw  (LL_1khz_attitude_data.angvel_yaw);
  writePacket2Ringbuffer(MAV_IMU_PKT_ID, (unsigned char*)&g_imu_pkt, sizeof(g_imu_pkt));
}

inline void sendFlightStateData(void)
{
  writePacket2Ringbuffer(MAV_FLIGHT_STATE_PKT_ID, (unsigned char*)&g_flight_state_pkt, sizeof(g_flight_state_pkt));
}

inline void sendRcData(void)
{
  for (int i = 0; i < 8; ++i)
    g_rcdata_pkt.channel[i] = RO_RC_Data.channel[i];

  writePacket2Ringbuffer(MAV_RCDATA_PKT_ID, (unsigned char*)&g_rcdata_pkt, sizeof(g_rcdata_pkt));
}

inline void sendStatusData(void)
{
  writePacket2Ringbuffer(MAV_STATUS_PKT_ID, (unsigned char*)&g_status_pkt, sizeof(g_status_pkt));
}

inline void sendCtrlDebugData(void)
{
  // debug packet - takes commands that were written to control
  // and sends back up to CPU

  g_ctrl_debug_pkt.cmd_roll_LL           = WO_CTRL_Input.roll ;
  g_ctrl_debug_pkt.cmd_pitch_LL          = WO_CTRL_Input.pitch ;
  g_ctrl_debug_pkt.cmd_yaw_rate_LL       = WO_CTRL_Input.yaw ;
  g_ctrl_debug_pkt.cmd_thrust_LL         = WO_CTRL_Input.thrust;

  g_ctrl_debug_pkt.roll_limit     = - SIToLLCmdRoll(g_ctrl_cfg_pkt.cmd_roll_limit); // -1 for coordinate system
  g_ctrl_debug_pkt.pitch_limit    =   SIToLLCmdPitch(g_ctrl_cfg_pkt.cmd_pitch_limit);
  g_ctrl_debug_pkt.yaw_rate_limit =   SIToLLCmdYawRate(g_ctrl_cfg_pkt.cmd_yaw_rate_limit);
  g_ctrl_debug_pkt.thrust_limit   =   SIToLLCmdThrust(g_ctrl_cfg_pkt.cmd_thrust_limit);

  g_ctrl_debug_pkt.ctrl_mode_roll = g_ctrl_cfg_pkt.ctrl_mode_roll;
  g_ctrl_debug_pkt.ctrl_mode_pitch = g_ctrl_cfg_pkt.ctrl_mode_pitch;
  g_ctrl_debug_pkt.ctrl_mode_yaw_rate  = g_ctrl_cfg_pkt.ctrl_mode_yaw_rate;
  g_ctrl_debug_pkt.ctrl_mode_thrust =  g_ctrl_cfg_pkt.ctrl_mode_thrust;

  writePacket2Ringbuffer(MAV_CTRL_DEBUG_PKT_ID, (unsigned char*)&g_ctrl_debug_pkt, sizeof(g_ctrl_debug_pkt));
}

inline unsigned short isSerialEnabled(void)
{
  return RO_RC_Data.channel[4];
}

inline void synchronizeTime(void)
{
  // check for timesync packet
  if (g_timesync_pkt_info->updated)
  {
    timeOffset = (900*timeOffset + 100 * (g_timesync_pkt.ts1 * 2 - g_timesync_pkt.tc1 - g_timestamp) / 2) / 1000;
    g_status_pkt.timesync_offset = timeOffset;

    if (timeOffset > 1e7 || timeOffset < -1e7)
    {
      g_timestamp = g_timesync_pkt.ts1;
      timeOffset = 0;
    }
    else if (timeOffset > 2000)
      timeOffset = 2000;
    else if (timeOffset < -2000)
      timeOffset = -2000;

    if (timeOffset > 0)
    {
      time_step = 4000 / timeOffset;
      time_correction = 1;
    }
    else if (timeOffset < 0)
    {
      time_step = -4000 / timeOffset;
      time_correction = -1;
    }
    else
    {
      time_step = 4000;
      time_correction = 0;
    }

    g_timesync_pkt_info->updated = 0;
  }

  // correct timestamp every step sdkloops by one us
  if (g_sdk_loops % time_step == 0)
  {
    g_timestamp += time_correction;
  }

  if (g_sdk_loops % 2000 == 0)
  {
    g_timesync_pkt.tc1 = g_timestamp;
    g_timesync_pkt.ts1 = 0;
    writePacket2Ringbuffer(MAV_TIMESYNC_PKT_ID, (unsigned char*)&g_timesync_pkt, sizeof(g_timesync_pkt));
    UART_send_ringbuffer();
  }
}
/*
inline void watchdog(void)
{
  static uint32_t lastTxPackets = 0;

  // check if a valid packet arrived in the HLI_COMMUNICATION_TIMEOUT s
  if ((g_sdk_loops % (ControllerCyclesPerSecond  * HLI_COMMUNICATION_TIMEOUT )) == 0)
  {
    if (UART_rxGoodPacketCount == lastTxPackets)
    {
      startAutoBaud();
    }
    lastTxPackets = UART_rxGoodPacketCount;
  }
}
*/

inline int checkTxPeriod(uint16_t period, uint16_t phase)
{
  if (period == 0)
    return 0;
  else
    return g_sdk_loops % period == phase;
}

inline void processKF()
{
  if (g_mav_kf_cfg_pkt_info->updated)
  {
    g_mav_kf_cfg_pkt_info->updated = 0;

    uint8_t kf_reset = 0;

    kf_reset         = g_mav_kf_cfg_pkt.enable_mask & (1<<MAV_KF_RESET_BIT);
    g_kf_x_enabled   = g_mav_kf_cfg_pkt.enable_mask & (1<<MAV_KF_X_BIT);
    g_kf_y_enabled   = g_mav_kf_cfg_pkt.enable_mask & (1<<MAV_KF_Y_BIT);
    g_kf_z_enabled   = g_mav_kf_cfg_pkt.enable_mask & (1<<MAV_KF_Z_BIT);
    g_kf_yaw_enabled = g_mav_kf_cfg_pkt.enable_mask & (1<<MAV_KF_YAW_BIT);

    kal_x.Sigma2Q1 = g_mav_kf_cfg_pkt.Q_x;
    kal_x.Sigma2Q2 = g_mav_kf_cfg_pkt.Q_x;
    kal_x.Sigma2R1 = g_mav_kf_cfg_pkt.R_x;
    kal_x.Sigma2R2 = g_mav_kf_cfg_pkt.R_vx;

    kal_y.Sigma2Q1 = g_mav_kf_cfg_pkt.Q_y;
    kal_y.Sigma2Q2 = g_mav_kf_cfg_pkt.Q_y;
    kal_y.Sigma2R1 = g_mav_kf_cfg_pkt.R_y;
    kal_y.Sigma2R2 = g_mav_kf_cfg_pkt.R_vy;

    kal_z.Sigma2Q1 = g_mav_kf_cfg_pkt.Q_z;
    kal_z.Sigma2Q2 = g_mav_kf_cfg_pkt.Q_z;
    kal_z.Sigma2R1 = g_mav_kf_cfg_pkt.R_z;
    kal_z.Sigma2R2 = g_mav_kf_cfg_pkt.R_vz_p;

    kal_yaw.Sigma2Q = g_mav_kf_cfg_pkt.Q_yaw;
    kal_yaw.Sigma2R = g_mav_kf_cfg_pkt.R_yaw;

    if (kf_reset != 0) resetKalmanFilter();
  }

  KFilter();

  // **** Update pose based on Kalman Filter (or direct CPU updates)

  if (g_kf_yaw_enabled != 0)
  {
    g_pose_pkt.yaw = kal_out.yaw_filtered;
  }
  else
  {
    g_pose_pkt.yaw = g_mav_pose2D_pkt.yaw;
  }

  if (g_kf_x_enabled != 0)
  {
    g_pose_pkt.x  = kal_out.pos_filtered[0];
    g_pose_pkt.vx = kal_out.vel_filtered[0];
  }
  else
  {
    g_pose_pkt.x  = g_mav_pose2D_pkt.x;
    g_pose_pkt.vx = g_mav_pose2D_pkt.vx;
  }

  if (g_kf_y_enabled != 0)
  {
    g_pose_pkt.y  = kal_out.pos_filtered[1];
    g_pose_pkt.vy = kal_out.vel_filtered[1];
  }
  else
  {
    g_pose_pkt.y  = g_mav_pose2D_pkt.y;
    g_pose_pkt.vy = g_mav_pose2D_pkt.vy;
  }

  if (g_kf_z_enabled != 0)
  {
    g_pose_pkt.z  = kal_out.pos_filtered[2];
    g_pose_pkt.vz = kal_out.vel_filtered[2];
  }
  else
  {
    g_pose_pkt.z  = g_mav_height_pkt.z;
    g_pose_pkt.vz = g_mav_height_pkt.vz;
  }

  g_pose_pkt.roll  = LLToSIAngleRoll (LL_1khz_attitude_data.angle_roll);
  g_pose_pkt.pitch = LLToSIAnglePitch(LL_1khz_attitude_data.angle_pitch);
}


inline void feedbackBeep()
{
  if (g_flight_state_pkt.state == MAV_STATE_ERROR)
  {
    // High frequency ERROR beep
    if (g_sdk_loops % 100 == 0)
      beeper(ON);
    if (g_sdk_loops % 100 == 50)
      beeper(OFF);
  }
  else if (isSerialEnabled() != 0)
  {
    // Double beep, serial is enabled
    if (g_sdk_loops % 1000 ==  0 || g_sdk_loops % 1000 == 100)
      beeper(ON);
    if (g_sdk_loops % 1000 == 50 || g_sdk_loops % 1000 == 150)
      beeper(OFF);
  }
  else
  {
    // Single beep, serial disable
    if (g_sdk_loops % 1000 == 0)
      beeper(ON);
    if (g_sdk_loops % 1000 == 50)
      beeper(OFF);
  }
}

inline void processMotorStateChanges()
{
  short motors_running = LL_1khz_attitude_data.status2 & 0x1;

  if (g_motors_running == 0 && motors_running != 0)
  {
    // motors just changed from to ON from Remote command
    g_flight_state_pkt.state = MAV_STATE_IDLE;
  }
  else if (g_motors_running != 0 && motors_running == 0)
  {
    if (g_flight_state_pkt.state != MAV_STATE_ERROR)
    {
      // motors just changed from ON to OFF
      g_flight_state_pkt.state = MAV_STATE_OFF;
    }
  }

  g_motors_running = motors_running;
}

inline void processFlightActionRequests()
{
  if (g_flight_action_pkt_info->updated)
  {
    if(isSerialEnabled() != 0)
    {
      if (g_flight_action_pkt.action == MAV_ACTION_TOGGLE_ENGAGE)
      {
        if (g_flight_state_pkt.state == MAV_STATE_OFF)
        {
          g_flight_state_pkt.state = MAV_STATE_ENGAGING;
          g_toggle_motors_start_time = g_timestamp;
        }
        else if (g_flight_state_pkt.state == MAV_STATE_IDLE)
        {
          g_flight_state_pkt.state = MAV_STATE_DISENGAGING;
          g_toggle_motors_start_time = g_timestamp;
        }
      }
      else if (g_flight_action_pkt.action == MAV_ACTION_ESTOP)
      {
        // estop
        g_flight_state_pkt.state = MAV_STATE_ERROR;
      }
      else if (g_flight_action_pkt.action == MAV_ACTION_TAKEOFF)
      {
        if (g_flight_state_pkt.state == MAV_STATE_IDLE)
        {
          // takeoff
          g_flight_state_pkt.state = MAV_STATE_FLYING;

          // reset the PID controls
          pidReset();
        }
      }
      else if (g_flight_action_pkt.action == MAV_ACTION_LAND)
      {
        if (g_flight_state_pkt.state == MAV_STATE_FLYING)
        {
          // land
          g_flight_state_pkt.state = MAV_STATE_LANDING;

          //g_land_thrust = g_ctrl_cmd.cmd_thrust;
        }
      }
    }

    g_flight_action_pkt_info->updated = 0;
  }
}

inline void processEngageDisengageTimeouts()
{
  if (g_flight_state_pkt.state == MAV_STATE_ENGAGING || g_flight_state_pkt.state == MAV_STATE_DISENGAGING)
  {
    if (g_timestamp - g_toggle_motors_start_time >= MAX_TOGGLE_CMD_TIME)
    {
      // go to error state
      g_flight_state_pkt.state = MAV_STATE_ERROR;
    }
  }
}

inline void processLandingThrust()
{
  if (g_flight_state_pkt.state == MAV_STATE_LANDING)
  {
   /* while (g_pose_pkt.z > 0.30) //TODO include desired landing height in the packet
    {
      g_ctrl_cfg_pkt.ctrl_mode_pitch = MAV_CTRL_MODE_POSITION;
      g_ctrl_cfg_pkt.ctrl_mode_roll = MAV_CTRL_MODE_POSITION;
      g_des_vel_pkt.vz = 0.3; // TODO include desired landing velocity in the packet
      processCtrl();
    }*/

    if (g_ctrl_cmd.cmd_thrust > 0)
    {
      // still landing - decrease thrust

      if (g_sdk_loops % LAND_THRUST_DECREASE_PERIOD == 0)
        g_ctrl_cmd.cmd_thrust -= LAND_THRUST_DECREASE_STEP;

      if (g_ctrl_cmd.cmd_thrust < 0) g_ctrl_cmd.cmd_thrust = 0; // prevent from going under 0
    }
    else
    {
      g_flight_state_pkt.state = MAV_STATE_IDLE;
    }
  }
}

inline void processMotorCommands()
{
  if (g_flight_state_pkt.state == MAV_STATE_ERROR)
  {
    // TODO: real estop here!
    writeCommand(0, 0, 0, 0, 0, 0);
  }
  else if (g_flight_state_pkt.state == MAV_STATE_ENGAGING || g_flight_state_pkt.state == MAV_STATE_DISENGAGING )
  {
    writeCommand(0, 0, 2047, 0, MAV_LL_CMD_YAW_RATE_MASK | MAV_LL_CMD_THRUST_MASK, 1);
  }
  else if (g_flight_state_pkt.state == MAV_STATE_IDLE || g_flight_state_pkt.state == MAV_STATE_OFF)
  {
    writeCommand(0, 0, 0, 0, MAV_LL_CMD_RPYT_MASK, 1);
  }
  else if (g_flight_state_pkt.state == MAV_STATE_LANDING)
  {
    writeCommand(0, 0, 0, SIToLLCmdThrust(g_ctrl_cmd.cmd_thrust), MAV_LL_CMD_THRUST_MASK, 1);
  }
  else if (g_flight_state_pkt.state == MAV_STATE_FLYING)
  {
    // fill out debug packet
    g_ctrl_debug_pkt.cmd_roll     = g_ctrl_cmd.cmd_roll;
    g_ctrl_debug_pkt.cmd_pitch    = g_ctrl_cmd.cmd_pitch;
    g_ctrl_debug_pkt.cmd_yaw_rate = g_ctrl_cmd.cmd_yaw_rate;
    g_ctrl_debug_pkt.cmd_thrust   = g_ctrl_cmd.cmd_thrust;

    WO_CTRL_Input.roll   = SIToLLCmdRoll   (g_ctrl_cmd.cmd_roll);
    WO_CTRL_Input.pitch  = SIToLLCmdPitch  (g_ctrl_cmd.cmd_pitch);
    WO_CTRL_Input.yaw    = SIToLLCmdYawRate(g_ctrl_cmd.cmd_yaw_rate);
    WO_CTRL_Input.thrust = SIToLLCmdThrust (g_ctrl_cmd.cmd_thrust);

    short ctrl_mask = 0x00;

    if (g_ctrl_cfg_pkt.ctrl_mode_roll     != MAV_CTRL_MODE_DISABLED) ctrl_mask |= MAV_LL_CMD_ROLL_MASK;
    if (g_ctrl_cfg_pkt.ctrl_mode_pitch    != MAV_CTRL_MODE_DISABLED) ctrl_mask |= MAV_LL_CMD_PITCH_MASK;
    if (g_ctrl_cfg_pkt.ctrl_mode_yaw_rate != MAV_CTRL_MODE_DISABLED) ctrl_mask |= MAV_LL_CMD_YAW_RATE_MASK;
    if (g_ctrl_cfg_pkt.ctrl_mode_thrust   != MAV_CTRL_MODE_DISABLED) ctrl_mask |= MAV_LL_CMD_THRUST_MASK;

    WO_CTRL_Input.ctrl  = ctrl_mask;
    WO_SDK.ctrl_enabled = 0x01;
  }
}

inline void processSendData()
{
  if (checkTxPeriod(g_tx_freq_cfg_pkt.rcdata_period, g_tx_freq_cfg_pkt.rcdata_phase))
  {
    sendRcData();
  }

  if (checkTxPeriod(g_tx_freq_cfg_pkt.imu_period, g_tx_freq_cfg_pkt.imu_phase))
  {
    sendImuData();
  }

  if (checkTxPeriod(g_tx_freq_cfg_pkt.flight_state_period, g_tx_freq_cfg_pkt.flight_state_phase))
  {
    sendFlightStateData();
  }

  if (checkTxPeriod(g_tx_freq_cfg_pkt.pose_period, g_tx_freq_cfg_pkt.pose_phase))
  {
    sendMavPoseData();
  }

  if (checkTxPeriod(g_tx_freq_cfg_pkt.status_period, g_tx_freq_cfg_pkt.status_phase ))
  {
    sendStatusData();
  }

  if (checkTxPeriod(g_tx_freq_cfg_pkt.ctrl_debug_period, g_tx_freq_cfg_pkt.ctrl_debug_phase))
  {
    sendCtrlDebugData();
  }
}

