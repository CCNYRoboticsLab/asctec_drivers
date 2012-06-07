#ifndef KALMAN_H
#define KALMAN_H

#include <inttypes.h>
#include <mav_common/comm_packets.h>
#include <mav_common/comm_util.h>
#include <mav_common/comm_types.h>

#include "matrices.h"
#include "LL_HL_comm.h"
#include "sdk.h"
#include "ssp.h"
#include "uart.h"
#include "comm_util_LL.h"

#define HEIGHT_PKT_TIMEOUT 200000 // 200 ms

typedef struct
{
  float A[2][2];
  float At[2][2];
  float K[2][2];
  float P[2][2];
  float T[2][2];
  float C[2][2];
  float I[2][2];
  float Q[2][2];
  float R[2][2];
  float Sigma2Q1;
  float Sigma2Q2;
  float Sigma2R1;
  float Sigma2R2;
  float B[2];
  float Input;
  float State[2];
  float Correction[2];
  float Res[2];
} KalPos;

typedef struct
{
  float K;
  float P;
  float T;
  float Q;
  float R;
  float Sigma2Q;
  float Sigma2R;
  float Input;
  float State;
  float Correction;
} KalYaw;

typedef struct
{
  float pos_filtered[3]; // Position
  float vel_filtered[3]; // linear velocity
  float yaw_filtered;    //yaw angle
} KalOut;

typedef struct
{
  float sigma2Q1x;
  float sigma2Q2x;
  float sigma2R1x;
  float sigma2R2vx;

  float sigma2Q1y;
  float sigma2Q2y;
  float sigma2R1y;
  float sigma2R2vy;

  float sigma2Q1z;
  float sigma2Q2z;
  float sigma2R1z;
  float sigma2R2vz;

  float sigma2Qyaw;
  float sigma2Ryaw;
} Covariance;

KalPos kal_x, kal_y, kal_z;
KalYaw kal_yaw;
KalOut kal_out;
Covariance covariance;

void KFilter(void);

void InitKalPos(KalPos *k);
void PredictPos(KalPos *k, float dt);
void CorrectPos(KalPos *k);

void InitKalYaw(KalYaw *k);
void PredictYaw(KalYaw *k, float dt);
void CorrectYaw(KalYaw *k);

void resetKalmanFilter(void);

#endif // KALMAN_H

