#ifndef PID_H
#define PID_H

#include <inttypes.h>
#include <mav_common/comm.h>
#include <mav_common/comm_packets.h>
#include <mav_common/comm_types.h>

#include "sdk.h"
#include "LL_HL_comm.h"

typedef struct
{
  float kp;         //  Proportional Const
  float ki;         //  Integral Const
  float kd;         //  Derivative Const
  float kd2;        //  Derivative Const for angular velocity feedback
  float d_base;
  float bias;       //  constant term in the PID sum
  float sum_error;  //  Sums of Errors
  float max_sum_error; //  integrative saturation to avoid increasing error from the integration
  float max_error;    // maximum instantaneous error

} PID;

void pidReset(void);

float pidCalc(PID * pid, float error, float d_term, float d_base, float dt);

void processCtrl(void);

void pidParamUpdate(void);

#endif // PID_H

