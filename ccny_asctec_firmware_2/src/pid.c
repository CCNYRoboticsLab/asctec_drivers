#include "pid.h"

//volatile int64_t g_latest_ctrl_time = 0;

PID pid_x, pid_y, pid_z, pid_yaw, pid_vx, pid_vy, pid_vz;

extern float g_cos_psi;
extern float g_sin_psi;

extern float g_accel_x;
extern float g_accel_y;
extern float g_accel_z;

extern MAV_POSE_PKT          g_pose_pkt;
extern MAV_CTRL_CMD          g_ctrl_cmd;
extern MAV_FLIGHT_STATE_PKT  g_flight_state_pkt;
extern MAV_CTRL_DEBUG_PKT    g_ctrl_debug_pkt;
extern MAV_CTRL_INPUT_PKT    g_ctrl_input_pkt;
extern MAV_CTRL_CFG_PKT      g_ctrl_cfg_pkt;
extern MAV_DES_VEL_PKT       g_des_vel_pkt;

extern MAV_DES_POSE_PKT      g_des_pose_pkt;
extern PacketInfo *          g_des_pose_pkt_info;

extern MAV_PID_CFG_PKT       g_pid_cfg_pkt;
extern PacketInfo *          g_pid_cfg_pkt_info;

//float g_vel_x_bf_last = 0;
//float g_vel_y_bf_last = 0;
//float g_vel_z_last = 0;
void pidReset(void)
{
  // Check if there's a new packet with desired pose, if not set des_pose to zero
  if (g_des_pose_pkt_info->updated == 0)
  {
    g_des_pose_pkt.x         = 0.0;
    g_des_pose_pkt.y         = 0.0;
    g_des_pose_pkt.z         = 0.0;
    g_des_vel_pkt.vx         = 0.0;
    g_des_vel_pkt.vy         = 0.0;
    g_des_vel_pkt.vz         = 0.0;
    g_des_vel_pkt.yaw_rate   = 0.0;
    g_des_pose_pkt.yaw       = 0.0;
  }

  //initialize PID variables
  pid_x.sum_error   = 0.0;
  pid_y.sum_error   = 0.0;
  pid_z.sum_error   = 0.0;
  pid_vx.sum_error  = 0.0;
  pid_vy.sum_error  = 0.0;
  pid_vz.sum_error  = 0.0;
  pid_yaw.sum_error = 0.0;
}

float pidCalc(PID * pid, float error, float d_term, float d_base, float dt)
{
  // clamp error
  if      (error >  pid->max_error) error =  pid->max_error;
  else if (error < -pid->max_error) error = -pid->max_error;

  pid->sum_error += error * dt;

  // prevent integral windup through clamping
  if      (pid->sum_error >  pid->max_sum_error) pid->sum_error =  pid->max_sum_error;
  else if (pid->sum_error < -pid->max_sum_error) pid->sum_error = -pid->max_sum_error;

  float error_pow = pow((1.0/d_base),abs(error));
  return  (pid->bias + pid->kp * error + (pid->kd *error_pow)* d_term + pid->ki * pid->sum_error);
}

void pidParamUpdate()
{
  // *************** X axis par *******************

  pid_x.kp            = g_pid_cfg_pkt.k_p_x;
  pid_x.ki            = g_pid_cfg_pkt.k_i_x;
  pid_x.kd            = g_pid_cfg_pkt.k_d_x;
  pid_x.d_base        = g_pid_cfg_pkt.d_base_x;
  pid_x.kd2           = g_pid_cfg_pkt.k_d2_x;
  pid_x.bias          = g_pid_cfg_pkt.bias_x;
  pid_x.max_error     = g_pid_cfg_pkt.max_err_x;
  pid_x.max_sum_error = g_pid_cfg_pkt.max_i_x;

  pid_vx.kp            = g_pid_cfg_pkt.k_p_vx;
  pid_vx.ki            = g_pid_cfg_pkt.k_i_vx;
  pid_vx.kd            = g_pid_cfg_pkt.k_d_vx;
  pid_vx.bias          = g_pid_cfg_pkt.bias_vx;
  pid_vx.max_error     = g_pid_cfg_pkt.max_err_vx;
  pid_vx.max_sum_error = g_pid_cfg_pkt.max_i_vx;

  // *************** Y axis par *******************

  pid_y.kp          = g_pid_cfg_pkt.k_p_y;
  pid_y.ki          = g_pid_cfg_pkt.k_i_y;
  pid_y.kd          = g_pid_cfg_pkt.k_d_y;
  pid_y.d_base      = g_pid_cfg_pkt.d_base_y;
  pid_y.kd2         = g_pid_cfg_pkt.k_d2_y;
  pid_y.bias        = g_pid_cfg_pkt.bias_y;
  pid_y.max_error     = g_pid_cfg_pkt.max_err_y;
  pid_y.max_sum_error = g_pid_cfg_pkt.max_i_y;

  pid_vy.kp         = g_pid_cfg_pkt.k_p_vy;
  pid_vy.ki         = g_pid_cfg_pkt.k_i_vy;
  pid_vy.kd         = g_pid_cfg_pkt.k_d_vy;
  pid_vy.bias       = g_pid_cfg_pkt.bias_vy;
  pid_vy.max_error     = g_pid_cfg_pkt.max_err_vy;
  pid_vy.max_sum_error = g_pid_cfg_pkt.max_i_vy;

  // *************** Z axis par *******************

  pid_z.kp             = g_pid_cfg_pkt.k_p_z;
  pid_z.ki             = g_pid_cfg_pkt.k_i_z;
  pid_z.kd             = g_pid_cfg_pkt.k_d_z;
  pid_z.kd2            = g_pid_cfg_pkt.k_d2_z;
  pid_z.bias           = g_pid_cfg_pkt.bias_z;
  pid_z.max_error      = g_pid_cfg_pkt.max_err_z;
  pid_z.max_sum_error  = g_pid_cfg_pkt.max_i_z;


  pid_vz.kp         = g_pid_cfg_pkt.k_p_vz;
  pid_vz.ki         = g_pid_cfg_pkt.k_i_vz;
  pid_vz.kd         = g_pid_cfg_pkt.k_d_vz;
  pid_vz.bias       = g_pid_cfg_pkt.bias_vz;
  pid_vz.max_error     = g_pid_cfg_pkt.max_err_vz;
  pid_vz.max_sum_error = g_pid_cfg_pkt.max_i_vz;

  // ****************** Yaw par *******************

  pid_yaw.kp             = g_pid_cfg_pkt.k_p_yaw;
  pid_yaw.ki             = g_pid_cfg_pkt.k_i_yaw;
  pid_yaw.kd             = g_pid_cfg_pkt.k_d_yaw;
  pid_yaw.bias           = g_pid_cfg_pkt.bias_yaw;
  pid_yaw.max_error      = g_pid_cfg_pkt.max_err_yaw;
  pid_yaw.max_sum_error  = g_pid_cfg_pkt.max_i_yaw;
}

void processCtrl(void)
{
  float dt = 0.001;//(g_timestamp - g_latest_ctrl_time) * 0.000001; //dt in sec
  //g_latest_ctrl_time = g_timestamp;
  // **** Check if there's a new packet with PID parameters ***************

  if (g_pid_cfg_pkt_info->updated != 0)
  {
    g_pid_cfg_pkt_info->updated = 0;
    pidParamUpdate();
  }

  if (g_flight_state_pkt.state == MAV_STATE_FLYING)
  {
    float roll     = LLToSIAngleRoll (LL_1khz_attitude_data.angle_roll);
    float pitch    = LLToSIAnglePitch(LL_1khz_attitude_data.angle_pitch);

    float a_x = g_accel_x * cos(pitch) + g_accel_y * sin(pitch)*sin(roll) + g_accel_z * sin(pitch)*cos(roll);
    float a_y = g_accel_y * cos(roll)  - g_accel_z * sin(roll);

    float vel_x_bf  = g_pose_pkt.vx * g_cos_psi + g_pose_pkt.vy * g_sin_psi;
    float vel_y_bf  = g_pose_pkt.vy * g_cos_psi - g_pose_pkt.vx * g_sin_psi;
    float vel_z = g_pose_pkt.vz;

    //float dv_x = (vel_x_bf - g_vel_x_bf_last)/dt;
    //float dv_y = (vel_y_bf - g_vel_y_bf_last)/dt;
    //float dv_z = (g_pose_pkt.vz - g_vel_z_last )/dt;

    //g_vel_x_bf_last = vel_x_bf;
    //g_vel_y_bf_last = vel_y_bf;
    //g_vel_z_last = vel_z;

    g_ctrl_debug_pkt.vel_x_bf = vel_x_bf;
    g_ctrl_debug_pkt.vel_y_bf = vel_y_bf;
    g_ctrl_debug_pkt.ax_bf = a_x;
    g_ctrl_debug_pkt.ay_bf = a_y;
    //g_ctrl_debug_pkt.az    = dv_z;

    // *************************** X axis ctrl*********************************

    if (g_ctrl_cfg_pkt.ctrl_mode_pitch == MAV_CTRL_MODE_POSITION)
    {
      float des_x_bf   = (g_des_pose_pkt.x - g_pose_pkt.x) * g_cos_psi + (g_des_pose_pkt.y - g_pose_pkt.y) * g_sin_psi;
      //float vel_x_bf   = g_pose_pkt.vx * g_cos_psi + g_pose_pkt.vy * g_sin_psi;
      //float pitch_rate = LLToSIAngleRatePitch (LL_1khz_attitude_data.angvel_pitch);

      g_ctrl_cmd.cmd_pitch = pidCalc(&pid_x, des_x_bf, -vel_x_bf, pid_x.d_base, dt);
      g_ctrl_debug_pkt.pid_error_x_bf = des_x_bf;
    }

    else if (g_ctrl_cfg_pkt.ctrl_mode_pitch == MAV_CTRL_MODE_VELOCITY)
    {
      //float des_vx_bf   = (g_des_pose_pkt.vx - g_pose_pkt.vx) * g_cos_psi + (g_des_pose_pkt.vy - g_pose_pkt.vy) * g_sin_psi;
      //float des_vx     = (float) g_des_pose_pkt.vx;
      //float current_vx = (float) g_pose_pkt.vx;
      float vx_error = g_des_vel_pkt.vx - vel_x_bf;
      g_ctrl_debug_pkt.pid_error_vx_bf = vx_error;

      g_ctrl_cmd.cmd_pitch = pidCalc(&pid_vx, vx_error, -a_x, 1.0, dt);
    }

    else if (g_ctrl_cfg_pkt.ctrl_mode_pitch == MAV_CTRL_MODE_DIRECT)
      g_ctrl_cmd.cmd_pitch = g_ctrl_input_pkt.cmd_pitch;

    else if (g_ctrl_cfg_pkt.ctrl_mode_pitch == MAV_CTRL_MODE_DISABLED)
      g_ctrl_cmd.cmd_pitch = 0;

    // set debug info
    g_ctrl_debug_pkt.pid_x_i_term = pid_x.sum_error;

    // *************************** Y axis ctrl *********************************

    if (g_ctrl_cfg_pkt.ctrl_mode_roll == MAV_CTRL_MODE_POSITION)
    {
      float des_y_bf  = (g_des_pose_pkt.y - g_pose_pkt.y)* g_cos_psi - (g_des_pose_pkt.x - g_pose_pkt.x) * g_sin_psi;
      //float vel_y_bf  = g_pose_pkt.vy * g_cos_psi - g_pose_pkt.vx * g_sin_psi;
      //float roll_rate = LLToSIAngleRateRoll (LL_1khz_attitude_data.angvel_roll);

      g_ctrl_cmd.cmd_roll = - pidCalc(&pid_y, des_y_bf, -vel_y_bf, pid_y.d_base, dt); // positive roll gives you negative y
      g_ctrl_debug_pkt.pid_error_y_bf = des_y_bf;
    }
    else if (g_ctrl_cfg_pkt.ctrl_mode_roll == MAV_CTRL_MODE_VELOCITY)
    {
      //float des_vy     = (float) g_des_pose_pkt.vy;
      //float current_vy = (float) g_pose_pkt.vy;

      float vy_error = g_des_vel_pkt.vy - vel_y_bf;
      g_ctrl_debug_pkt.pid_error_vy_bf = vy_error;
      g_ctrl_cmd.cmd_roll = -pidCalc(&pid_vy, vy_error, -a_y, 1.0, dt);
    }

    else if (g_ctrl_cfg_pkt.ctrl_mode_roll == MAV_CTRL_MODE_DIRECT)
      g_ctrl_cmd.cmd_roll = g_ctrl_input_pkt.cmd_roll;

    else if (g_ctrl_cfg_pkt.ctrl_mode_roll == MAV_CTRL_MODE_DISABLED)
      g_ctrl_cmd.cmd_roll = 0;

    // set debug info
    g_ctrl_debug_pkt.pid_y_i_term = pid_y.sum_error;

    // **************************** Z axis ctrl *********************************

    if (g_ctrl_cfg_pkt.ctrl_mode_thrust == MAV_CTRL_MODE_DISABLED)
    {
      g_ctrl_cmd.cmd_thrust = 0;
    }
    else
    {
      Thrust new_cmd_thrust = g_ctrl_cmd.cmd_thrust;

      if (g_ctrl_cfg_pkt.ctrl_mode_thrust == MAV_CTRL_MODE_POSITION)
      {
        float des_z     = g_des_pose_pkt.z;
        float current_z = g_pose_pkt.z;

        new_cmd_thrust = pidCalc(&pid_z, des_z - current_z, -vel_z, 1.0, dt);// - pid_z.kd2 * g_accel_z;
      }

      else if (g_ctrl_cfg_pkt.ctrl_mode_thrust == MAV_CTRL_MODE_VELOCITY)
            {
              float des_vz     = g_des_vel_pkt.vz;

              new_cmd_thrust = pidCalc(&pid_vz, des_vz - vel_z, -g_accel_z, 1.0, dt);
            }

      else if (g_ctrl_cfg_pkt.ctrl_mode_thrust == MAV_CTRL_MODE_DIRECT)
      {
        new_cmd_thrust = g_ctrl_input_pkt.cmd_thrust;
      }

      // spike guard
      double delta_cmd_thrust = new_cmd_thrust - g_ctrl_cmd.cmd_thrust;

      if      (delta_cmd_thrust > g_ctrl_cfg_pkt.cmd_thrust_delta_limit)
        g_ctrl_cmd.cmd_thrust += g_ctrl_cfg_pkt.cmd_thrust_delta_limit;
      else if (delta_cmd_thrust < -g_ctrl_cfg_pkt.cmd_thrust_delta_limit)
        g_ctrl_cmd.cmd_thrust -= g_ctrl_cfg_pkt.cmd_thrust_delta_limit;
      else
        g_ctrl_cmd.cmd_thrust = new_cmd_thrust;

      // set debug info
      g_ctrl_debug_pkt.pid_z_i_term = pid_z.sum_error;
    }

    // ****************************** YAW ctrl *********************************

    if (g_ctrl_cfg_pkt.ctrl_mode_yaw_rate == MAV_CTRL_MODE_POSITION)
    {
      float des_yaw     = g_des_pose_pkt.yaw;
      float current_yaw = g_pose_pkt.yaw;
      float yaw_rate    = LLToSIAngleRateYaw(LL_1khz_attitude_data.angvel_yaw);

      float error =  des_yaw - current_yaw;
      normalizeSIAnglePi(&error);

      g_ctrl_cmd.cmd_yaw_rate = pidCalc(&pid_yaw, error, -yaw_rate, 1, dt);
    }
    if (g_ctrl_cfg_pkt.ctrl_mode_yaw_rate == MAV_CTRL_MODE_VELOCITY)
      g_ctrl_cmd.cmd_yaw_rate = g_des_vel_pkt.yaw_rate;

    else if (g_ctrl_cfg_pkt.ctrl_mode_yaw_rate == MAV_CTRL_MODE_DIRECT)
      g_ctrl_cmd.cmd_yaw_rate = g_ctrl_input_pkt.cmd_yaw_rate;

    else if (g_ctrl_cfg_pkt.ctrl_mode_yaw_rate == MAV_CTRL_MODE_DISABLED)
      g_ctrl_cmd.cmd_yaw_rate = 0;

    // set debug info
    g_ctrl_debug_pkt.pid_yaw_i_term = pid_yaw.sum_error;

  }

  // **************************** CLAMP ********************************

  // Clamp roll command
  if      (g_ctrl_cmd.cmd_roll >  g_ctrl_cfg_pkt.cmd_roll_limit)
    g_ctrl_cmd.cmd_roll =  g_ctrl_cfg_pkt.cmd_roll_limit;
  else if (g_ctrl_cmd.cmd_roll < -g_ctrl_cfg_pkt.cmd_roll_limit)
    g_ctrl_cmd.cmd_roll = -g_ctrl_cfg_pkt.cmd_roll_limit;

  // Clamp pitch command
  if      (g_ctrl_cmd.cmd_pitch >  g_ctrl_cfg_pkt.cmd_pitch_limit)
    g_ctrl_cmd.cmd_pitch =  g_ctrl_cfg_pkt.cmd_pitch_limit;
  else if (g_ctrl_cmd.cmd_pitch < -g_ctrl_cfg_pkt.cmd_pitch_limit)
    g_ctrl_cmd.cmd_pitch = -g_ctrl_cfg_pkt.cmd_pitch_limit;

  // Clamp yaw rate command
  if      (g_ctrl_cmd.cmd_yaw_rate > g_ctrl_cfg_pkt.cmd_yaw_rate_limit)
    g_ctrl_cmd.cmd_yaw_rate =  g_ctrl_cfg_pkt.cmd_yaw_rate_limit;
  else if (g_ctrl_cmd.cmd_yaw_rate < -g_ctrl_cfg_pkt.cmd_yaw_rate_limit)
    g_ctrl_cmd.cmd_yaw_rate = -g_ctrl_cfg_pkt.cmd_yaw_rate_limit;

  // Clamp thrust command
  if      (g_ctrl_cmd.cmd_thrust > g_ctrl_cfg_pkt.cmd_thrust_limit)
    g_ctrl_cmd.cmd_thrust = g_ctrl_cfg_pkt.cmd_thrust_limit;
  else if (g_ctrl_cmd.cmd_thrust < 0)
    g_ctrl_cmd.cmd_thrust = 0;
}
