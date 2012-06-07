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

#ifndef SDK_H
#define SDK_H

#include <inttypes.h>

#include "pid.h"
#include "uart.h"
#include "hardware.h"
#include "kalman.h"
#include "comm_util_LL.h"

#define CMD_MAX_PERIOD 100 // in [ms] ==> 10Hz

#define LAND_THRUST_DECREASE_STEP    0.01      // while landing, decrease thrust by this much
#define LAND_THRUST_DECREASE_PERIOD  1      // while landing, decrease thrust every # of cycles

typedef struct
{
  Position x;     // x position, in m
  Position y;     // y position, in m
  Position z;     // z position, in m

  Velocity vx;    // x velocity, in m/sec
  Velocity vy;    // y velocity, in m/sec
  Velocity vz;    // y velocity, in m/sec

  Angle roll;  // roll  orientation, in rad
  Angle pitch; // pitch orientation, in rad
  Angle yaw;   // yaw   orientation, in rad
} MAV_POSE_SI;

typedef struct
{
  Angle cmd_roll;      // roll  ,    in rad/....
  Angle cmd_pitch;     // pitch ,    in rad
  AngVelocity cmd_yaw_rate;  // yaw_rate , in rad/s
  Thrust cmd_thrust;    // thrust,    in %      [0, 100]

} MAV_CTRL_CMD;

typedef struct
{
  Position x;     // desired x position, in mm
  Position y;     // desired y position, in mm
  Position z;     // desired z position, in mm

  Velocity vx;    // desired x velocity, in mm/sec
  Velocity vy;    // desired y velocity, in mm/sec

  Angle yaw;   // desired yaw   orientation, in deg/100, [0, 36000)
} MAV_DES_POSE;

extern void SDK_mainloop(void);

void sdkInit(void);

inline void writeCommand(short pitch, short roll, short yaw, short thrust, short ctrl, short enable);
inline void sendImuData(void);
inline void sendRcData(void);
inline void sendFlightStateData(void);
inline void sendMavPoseData(void);
inline void sendStatusData(void);
inline void sendCtrlDebugData(void);

inline unsigned short isSerialEnabled(void);
inline void feedbackBeep(void);
inline void estop(void);

inline void processKF(void);
inline void processMotorStateChanges(void);
inline void processFlightActionRequests(void);
inline void processEngageDisengageTimeouts(void);
inline void processLandingThrust(void);
inline void processMotorCommands(void);
inline void processSendData(void);

/// adjusts HLP time to host PC time
/***
 * Timestamped packets get send around every 2 s to average the transmission delay.
 * Corrects at max 500 us per second. If the time offset is large, the server (host PC)
 * time is taken directly and synchronization starts from that time.
 */
inline void synchronizeTime(void);

/// gets called every sdk loops. Currently, only checks for packets from the PC and starts autobaud in case there wwas no communication in the last 10 s
//inline void watchdog(void);

/// checks if a packet has to be sent
inline int checkTxPeriod(uint16_t period, uint16_t phase);

struct WO_SDK_STRUCT
{
  unsigned char ctrl_mode;
  //0x00: "standard scientific interface" => send R/C stick commands to LL
  //0x01:	direct motor control
  //0x02: waypoint control (not yet implemented)

  unsigned char ctrl_enabled; 
  //0x00: Control commands are ignored by LL processor
  //0x01: Control commands are accepted by LL processor
};
extern struct WO_SDK_STRUCT WO_SDK;

struct RO_RC_DATA
{
  unsigned short channel[8];
  /*
   * channel[0]: Pitch
   * channel[1]: Roll
   * channel[2]: Thrust
   * channel[3]: Yaw
   * channel[4]: Serial interface enable/disable
   * channel[5]: manual / height control / GPS + height control
   *
   * range of each channel: 0..4095
   */
};
extern struct RO_RC_DATA RO_RC_Data;

struct WO_DIRECT_MOTOR_CONTROL
{
  unsigned char pitch;
  unsigned char roll;
  unsigned char yaw;
  unsigned char thrust;

  /*
   * commands will be directly interpreted by the mixer
   * running on each of the motor controllers
   *
   * range (pitch, roll, yaw commands): 0..200 = - 100..+100 %
   * range of thrust command: 0..200 = 0..100 %
   */

};
extern struct WO_DIRECT_MOTOR_CONTROL WO_Direct_Motor_Control;

struct WO_CTRL_INPUT
{ //serial commands (= Scientific Interface)
  short pitch; //Pitch input: -2047..+2047 (0=neutral)
  short roll; //Roll input: -2047..+2047	(0=neutral)
  short yaw; //(=R/C Stick input) -2047..+2047 (0=neutral)
  short thrust; //Collective: 0..4095 = 0..100%
  short ctrl; /*control byte:
   bit 0: pitch control enabled
   bit 1: roll control enabled
   bit 2: yaw control enabled
   bit 3: thrust control enabled
   bit 4: Height control enabled
   bit 5: GPS position control enabled
   */
};
extern struct WO_CTRL_INPUT WO_CTRL_Input;

#endif // SDK_H
