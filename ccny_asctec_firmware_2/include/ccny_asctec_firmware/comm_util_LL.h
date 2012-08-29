#ifndef COMM_UTIL_LL_H
#define COMM_UTIL_LL_H

#include <stdint.h>
#include <math.h>

#define GRAVITY_SI      9.810665            // in SI m/s^2
//#define GRAVITY_COMM    GRAVITY_SI * 1000.0 // in Comm  mm/s^2

inline float degreesToRadians(float angle);
inline float radiansToDegrees(float angle);

inline float LLToSIClimb(int16_t climb);

// ****************** accel ***********************

inline float LLToSIAccX(int16_t acc);
inline float LLToSIAccY(int16_t acc);
inline float LLToSIAccZ(int16_t acc);

/*
inline int32_t LLtoCommAccX(int16_t acc, float g);
inline int32_t LLtoCommAccY(int16_t acc, float g);
inline int32_t LLtoCommAccZ(int16_t acc, float g);
*/
// ****************** angle rate ***********************

inline float LLToSIAngleRateYaw  (int16_t angle_rate);
inline float LLToSIAngleRateRoll (int16_t angle_rate);
inline float LLToSIAngleRatePitch(int16_t angle_rate);
//inline int32_t LLtoCommAngleRateYaw(int16_t angle_rate);

// ****************** angle ***********************

inline float LLToSIAngleRoll(int16_t angle);
inline float LLToSIAnglePitch(int16_t angle);
inline float LLToSIAngleYaw(uint16_t angle);

/*
inline uint16_t LLtoCommAngleRoll (int16_t angle);
inline uint16_t LLtoCommAnglePitch(int16_t angle);
inline uint16_t LLtoCommAngleYaw  (uint16_t angle);
*/
// **** for direct motor control

inline short SIToLLCmdRoll   (float angle_cmd);
inline short SIToLLCmdPitch  (float angle_cmd);
inline short SIToLLCmdYawRate(float yaw_rate_cmd);
inline short SIToLLCmdThrust (float thrust_cmd);
/*
inline short commToLLCmdRoll    (int16_t cmd_roll);
inline short commToLLCmdPitch   (int16_t cmd_pitch);
inline short commToLLCmdYawRate (int32_t cmd_yaw_rate);
inline short commToLLCmdThrust  (int16_t cmd_thrust);
*/
#endif // COMM_UTIL_H
