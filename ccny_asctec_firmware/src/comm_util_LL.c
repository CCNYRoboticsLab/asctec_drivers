#include "comm_util_LL.h"

inline float degreesToRadians(float angle)
{
  return angle * M_PI / 180.0;
}

inline float radiansToDegrees(float angle)
{
  return angle * 180.0 / M_PI;
}

inline float LLToSIClimb(int16_t climb)
{
  return ((float)(climb)) / 1000.0;
}

// ****************** accel ***********************

float LLToSIAccX(int16_t acc, float g)
{
  return ( - (float)(acc)  ) / g * GRAVITY_SI ;
}

float LLToSIAccY(int16_t acc, float g)
{
  return ( - (float)(acc)  ) / g * GRAVITY_SI ;
}

float LLToSIAccZ(int16_t acc, float g)
{
  return ( - (float)(acc)  ) / g * GRAVITY_SI ;
}

/*
int32_t LLtoCommAccX(int16_t acc, float g)
{
  return - (int32_t)((float)(acc) / g * GRAVITY_COMM);
}

int32_t LLtoCommAccY(int16_t acc, float g)
{
  return - (int32_t)((float)(acc) / g * GRAVITY_COMM);
}

int32_t LLtoCommAccZ(int16_t acc, float g)
{
  return - (int32_t)((float)(acc) / g * GRAVITY_COMM);
}
*/
// ****************** angle ***********************

float LLToSIAngleRoll(int16_t angle)
{
  float ta =  degreesToRadians(((float)(-angle)) / 100.0);
  //normalizeSIAngle2Pi(&ta);
  return ta;
}

float LLToSIAnglePitch(int16_t angle)
{
  float ta = degreesToRadians(((float)(angle)) / 100.0);
  //normalizeSIAngle2Pi(&ta);
  return ta;
}

float LLToSIAngleYaw(uint16_t angle)
{
  float ta = degreesToRadians(((float)(36000-angle)) / 100.0);
  //normalizeSIAngle2Pi(&ta);
  return ta;
}
/*
uint16_t LLtoCommAngleRoll(int16_t angle)
{
  return (int)((-(float)(angle)) / 36000.0  * 65535.0);
}

uint16_t LLtoCommAnglePitch(int16_t angle)
{
  return (int)(((float)(angle)) / 36000.0  * 65535.0);
}

uint16_t LLtoCommAngleYaw(uint16_t angle)
{
  return (int)((36000.0 - (float)(angle)) / 36000.0  * 65535.0);
}
*/
// ****************** angle rate ***********************

float LLToSIAngleRateYaw(int16_t angle_rate)
{
  return degreesToRadians( (float)(-angle_rate) * 0.015 );
}

float LLToSIAngleRateRoll(int16_t angle_rate)
{
  return degreesToRadians( (float)(-angle_rate) * 0.015 );
}

float LLToSIAngleRatePitch(int16_t angle_rate)
{
  return degreesToRadians( (float)(angle_rate) * 0.015 );
}
/*
int32_t LLtoCommAngleRateYaw(int16_t angle_rate)
{
  return (int32_t)((float)(-angle_rate) * 0.015 / 360.0 * 65535.0);
}
*/
// ******* motor commands ***

short SIToLLCmdRoll (float angle_cmd)
{
  return -((short)(angle_cmd * 2293.578 )); // minus to invert roll because of the frame conversion (ASCTEC coordinate frame(??) to ENU)
}

short SIToLLCmdPitch (float angle_cmd)
{
  return ((short)(angle_cmd * 2293.578 ));
}

short SIToLLCmdYawRate (float yaw_rate_cmd)
{
  return ((short)(yaw_rate_cmd * 460.37235)); // minus to invert yaw rate because of the frame conversion (ASCTEC coordinate frame(??) to ENU)
}

short SIToLLCmdThrust (float thrust_cmd)
{
  return (short)(thrust_cmd * 40.95 );
}
/*
short commToLLCmdRoll    (int16_t cmd_roll)
{
  return -(short) ( ((float)(cmd_roll)) / 65535 * (2.0 * M_PI) * 2293.578);
  //return -((short)((float) cmd_roll / 1000.0 * 2293.578 ));
}
short commToLLCmdPitch   (int16_t cmd_pitch)
{
  return (short) ( ((float)(cmd_pitch)) / 65535 * (2.0 * M_PI) * 2293.578);
  //return ((short)((float) cmd_pitch/ 1000.0 * 2293.578 ));
}
short commToLLCmdYawRate (int32_t cmd_yaw_rate)
{
  return -((short)((float)(cmd_yaw_rate)) / 65535 * (2.0 * M_PI) * 460.37235);
  //return -((short)((float) cmd_yaw_rate / 1000.0 * 460.37235));
}

short commToLLCmdThrust  (int16_t cmd_thrust)
{
  return (short)((float)(cmd_thrust) / 10000.0 * 4095 );
}
*/
