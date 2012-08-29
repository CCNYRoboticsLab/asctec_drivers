#include "kalman.h"

//volatile int64_t g_latest_kf_time = 0;
volatile int64_t g_last_height_pkt_time = 0;

float g_cos_psi;
float g_sin_psi;
float g_vz_p_f = 0;

float g_accel_x;
float g_accel_y;
float g_accel_z;

extern uint8_t g_kf_x_enabled;
extern uint8_t g_kf_y_enabled;
extern uint8_t g_kf_z_enabled;
extern uint8_t g_kf_yaw_enabled;
extern float g_imu_gravity;
extern MAV_IMU_PKT g_imu_pkt;

extern MAV_POSE2D_PKT        g_mav_pose2D_pkt;
extern PacketInfo *          g_mav_pose2D_pkt_info;

extern MAV_HEIGHT_PKT        g_mav_height_pkt;
extern PacketInfo *          g_mav_height_pkt_info;

extern MAV_KF_CFG_PKT        g_mav_kf_cfg_pkt;

extern MAV_CTRL_DEBUG_PKT    g_ctrl_debug_pkt;

extern unsigned int g_sdk_loops;

void InitKalPos(KalPos *k)
{
  k->Input = 0;

  for (int i=0; i<2; i++)
  {
    for (int j=0; j<2; j++)
    {
      if (i==j)
        k->A[i][j] = k->At[i][j] = k->P[i][j] = k->T[i][j] = k->I[i][j] = 1;
      else
        k->A[i][j] = k->At[i][j] = k->P[i][j] = k->T[i][j] = k->I[i][j] = 0;
    }
  }

  for (int i=0; i<2; i++)
  {
    for (int j=0; j<2; j++)
    {
        k->K[i][j] = k->Q[i][j] = k->R[i][j] = 0;
    }
  }

  for (int i=0; i<2; i++)
  {
    k->Correction[i] = k->Res[i] = 0;//KalPos->Est_out[i] =
  }

  k->Q[0][0] = k->Sigma2Q1;
  k->Q[1][1] = k->Sigma2Q2;

  k->R[0][0] = k->Sigma2R1;
  k->R[1][1] = k->Sigma2R2;
}

void InitKalYaw(KalYaw *k)
{
  k->Q = k->Sigma2Q;
  k->R = k->Sigma2R;
  k->P = k->T = 1;
  k->Input = k->State = k->Correction  = 0;//KalYaw->Est_out = KalYaw->Res KalYaw->B =
}

void PredictPos(KalPos *k, float dt)
{
  float dt2;
  float Est[2];
  float tmpA[2][2];
  float tmpB[2];
  float tmpA1[2][2];

  dt2 = (dt*dt)/2;

  k->A[0][1] = dt;
  k->At[1][0] = dt;
  k->B[0] = dt2;
  k->B[1] = dt;

  VectmultSc2(k->B, k->Input,tmpB);
  multMatVec2(k->A, k->State, Est);
  addVector2(Est,tmpB,k->State);

  // propagate covariances
  multMatrix2(k->A, k->T, tmpA); //mtmpA=A*T  2x2
  multMatrix2(tmpA, k->At, tmpA1);//mtmpC=A*T*A'  2x2
  addMatrix2(k->Q, tmpA1, k->P);// P = A*T*A'+ Q  2x2 matrix
}

void CorrectPos(KalPos *k)
{
  // Compute Kalman gains: K = P*C'*inv(C*P*C'+R)
  // C matrix = Identity matrix so: K = P*inv(P + R)
  float tmpP[2][2];
  float tmpP1[2][2];
  float tmpVect[2];

  addMatrix2(k->P, k->R, tmpP); // P + R
  invert2(tmpP, tmpP1); // (P + R)^-1
  multMatrix2(k->P, tmpP1, k->K); // K = P*(P + R)^-1

  // compute residual as difference between sensor output and estimated output (state)
  subVector2(k->Correction, k->State, k->Res);
  multMatVec2(k->K, k->Res, tmpVect); //K*residual

  //apply correction
  k->State[0] += tmpVect[0];
  k->State[1] += tmpVect[1];

  //Compute "a posteriori " covariance matrix T = (I-K*C)*P
  subMatrix2(k->I, k->K,tmpP); //I-K*C
  multMatrix2(tmpP, k->P, k->T); //(I-K*C)*P
}

void PredictYaw(KalYaw *k, float dt)
{
  float Est_yaw;
  Est_yaw = k->State + dt * k->Input;
  k->State = Est_yaw;
  normalizeSIAngle2Pi(&k->State);
  k->P = k->T + k->Q; // P = T + Q
}

void CorrectYaw(KalYaw *k)
{
  k->K = (k->P) / (k->P + k->R);

  float Res = k->Correction - k->State;
  normalizeSIAnglePi(&Res);

  k->State += k->K * Res;
  normalizeSIAngle2Pi(&k->State);

  //Compute "a posteriori " covariance T = (I-K*C)*P
  k->T = (1 - k->K) * k->P;
}

void KFilter (void)
{
  static unsigned short first_time = 1;
  //float accel_x, accel_y, accel_z;
  float roll, pitch, yaw, yaw_rate;
  float accel_x_wf, accel_y_wf, accel_z_wf;
  float dt = 0.001;
  float vz_p;

  vz_p = LLToSIClimb(LL_1khz_attitude_data.dheight);  // z velocity from pressure, SI

  g_vz_p_f = 0.995 * g_vz_p_f + 0.005 * vz_p;   // simple smoothing filter

  roll     = LLToSIAngleRoll (LL_1khz_attitude_data.angle_roll);
  pitch    = LLToSIAnglePitch(LL_1khz_attitude_data.angle_pitch);

  yaw_rate = LLToSIAngleRateYaw(LL_1khz_attitude_data.angvel_yaw);

  if (first_time == 1)
  {
    first_time = 0;
    resetKalmanFilter();
  }

  float cos_phi   = cos(roll);    float sin_phi   = sin(roll);
  float cos_theta = cos(pitch);   float sin_theta = sin(pitch);

  if (g_kf_yaw_enabled != 0)
  {
    yaw = kal_yaw.State;
  }
  else
  {
    yaw = LLToSIAngleYaw (LL_1khz_attitude_data.angle_yaw);
  }

  g_cos_psi = cos(yaw);
  g_sin_psi = sin(yaw);

  g_accel_x = LLToSIAccX(LL_1khz_attitude_data.acc_x);//, g_imu_gravity);
  g_accel_y = LLToSIAccY(LL_1khz_attitude_data.acc_y);//, g_imu_gravity);
  g_accel_z = LLToSIAccZ(LL_1khz_attitude_data.acc_z);//, g_imu_gravity);

  // body frame to world frame transform

  float accel_x_cos_theta = g_accel_x*cos_theta;
  float sin_phi_sin_theta = sin_phi*sin_theta;
  float cos_phi_sin_theta = cos_phi*sin_theta;

  accel_x_wf =  accel_x_cos_theta*g_cos_psi + g_accel_y*(sin_phi_sin_theta*g_cos_psi - cos_phi*g_sin_psi) + g_accel_z*(cos_phi_sin_theta*g_cos_psi + sin_phi*g_sin_psi);
  accel_y_wf =  accel_x_cos_theta*g_sin_psi + g_accel_y*(sin_phi_sin_theta*g_sin_psi + cos_phi*g_cos_psi) + g_accel_z*(cos_phi_sin_theta*g_sin_psi - sin_phi*g_cos_psi);
  accel_z_wf = -g_accel_x*sin_theta         + g_accel_y*sin_phi*cos_theta                                 + g_accel_z*cos_phi*cos_theta - GRAVITY_SI;

  g_imu_pkt.acc_x = g_accel_x;
  g_imu_pkt.acc_y = g_accel_y;
  g_imu_pkt.acc_z = g_accel_z;

  // debug purposes
  g_ctrl_debug_pkt.acc_x_wf = accel_x_wf;
  g_ctrl_debug_pkt.acc_y_wf = accel_y_wf;
  g_ctrl_debug_pkt.acc_z_wf = accel_z_wf;

  //dt  = (g_timestamp - g_latest_kf_time) * 0.000001;
  //g_latest_kf_time = g_timestamp;

  if (g_kf_yaw_enabled != 0)
  {
    kal_yaw.Input = yaw_rate;
    PredictYaw(&kal_yaw, dt);
    if (g_mav_pose2D_pkt_info->updated == 1)
    {
      kal_yaw.Correction = g_mav_pose2D_pkt.yaw;
      CorrectYaw(&kal_yaw);
    }
    kal_out.yaw_filtered = kal_yaw.State;
  }

  if (g_kf_x_enabled != 0)
  {
    kal_x.Input = accel_x_wf;
    PredictPos(&kal_x, dt);
    if (g_mav_pose2D_pkt_info->updated == 1)
    {
      kal_x.Correction[0] = g_mav_pose2D_pkt.x;
      kal_x.Correction[1] = g_mav_pose2D_pkt.vx;
      CorrectPos(&kal_x);
    }
    kal_out.pos_filtered[0] = kal_x.State[0];
    kal_out.vel_filtered[0] = kal_x.State[1];
  }

  if (g_kf_y_enabled != 0)
  {
    kal_y.Input = accel_y_wf;
    PredictPos(&kal_y, dt);
    if (g_mav_pose2D_pkt_info->updated==1)
    {
      kal_y.Correction[0] = g_mav_pose2D_pkt.y;
      kal_y.Correction[1] = g_mav_pose2D_pkt.vy;
      CorrectPos(&kal_y);
    }
    kal_out.pos_filtered[1] = kal_y.State[0];
    kal_out.vel_filtered[1] = kal_y.State[1];
  }

  if (g_kf_z_enabled != 0  )
  {
    kal_z.Input = accel_z_wf;
    PredictPos(&kal_z, dt);

    // correct z, vz from laser
    if (g_mav_height_pkt_info->updated == 1)
    {
      g_last_height_pkt_time = g_timestamp;
      kal_z.Sigma2R1 = g_mav_kf_cfg_pkt.R_z;
      kal_z.Sigma2R2 = g_mav_kf_cfg_pkt.R_vz;
      kal_z.Correction[0] = g_mav_height_pkt.z;
      kal_z.Correction[1] = g_mav_height_pkt.vz;
      CorrectPos(&kal_z);
    }
    //if (g_sdk_loops % 200 == 0 ) // old
    if (g_last_height_pkt_time - g_timestamp > HEIGHT_PKT_TIMEOUT)
    {
      // correct vz from pressure
      kal_z.Sigma2R1 = 100.00e6;//(float) g_mav_kf_cfg_pkt.R_z;
      kal_z.Sigma2R2 = g_mav_kf_cfg_pkt.R_vz_p;
      kal_z.Correction[0] = kal_z.State[0]; //force residual to zero to have the predicted state as output
      kal_z.Correction[1] = g_vz_p_f;
      CorrectPos(&kal_z);
    }

    kal_out.pos_filtered[2] = kal_z.State[0];
    kal_out.vel_filtered[2] = kal_z.State[1];
  }

  g_mav_pose2D_pkt_info->updated = 0;
  g_mav_height_pkt_info->updated = 0;
}

void resetKalmanFilter()
{
  InitKalPos(&kal_x);
  InitKalPos(&kal_y);
  InitKalPos(&kal_z);
  InitKalYaw(&kal_yaw);
}
