#include "linear_control.h"
#include <iostream>
#include <ros/ros.h>

LinearControl::LinearControl(Parameter_t &param) : param_(param)
{
  resetThrustMapping();
}

/* 
  compute u.thrust and u.q, controller gains and other parameters are in param_ 
*/
quadrotor_msgs::Px4ctrlDebug
LinearControl::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u)
{
  /* WRITE YOUR CODE HERE */
    double g = param_.gra;
    //compute disired acceleration
    Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
    des_acc[0] = param_.gain.Kv0 * (des.v[0] - odom.v[0]) + param_.gain.Kp0 * (des.p[0] - odom.p[0]) + des.a[0];
    des_acc[1] = param_.gain.Kv1 * (des.v[1] - odom.v[1]) + param_.gain.Kp1 * (des.p[1] - odom.p[1]) + des.a[1];
    des_acc[2] = param_.gain.Kv2 * (des.v[2] - odom.v[2]) + param_.gain.Kp2 * (des.p[2] - odom.p[2]) + des.a[2] + g;

    //supposed to be readonly, compute thrust by acc
    u.thrust = computeDesiredCollectiveThrustSignal(des_acc);

    //compute control attitude in the BODY frame
    u.q = Eigen::Quaterniond(1.0,0.0,0.0,0.0);

  /* WRITE YOUR CODE HERE */
    // Get Euler angles from odom
    Eigen::Matrix3d odomR;
    odomR = odom.q.matrix();
    Eigen::Vector3d odomEuler;
    odomEuler = RtoEulerAngle(odomR);

//    Eigen::Vector3d odomEuler = odom.q.matrix().eulerAngles(2, 1, 0);

    // Calculate control Euler angles
    double psi = odomEuler[2];
    double phi_c = 1 / g * (des_acc[0] * sin(psi) - des_acc[1] * cos(psi);      // x-roll
    double theta_c = 1 / g * (des_acc[0] * cos(psi) + des_acc[1] * sin(psi));   // y-pitch
    double psi_c = psi;                                                         // z-yaw

    // Quaternion from Euler angles
    Eigen::Quaterniond quaternion_c =
            Eigen::AngleAxisd(phi_c, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(theta_c, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(psi_c, Eigen::Vector3d::UnitZ());

    // Coordinate transformation
    u.q = imu.q * odom.q.inverse() * quaternion_c;

  //used for debug
  debug_msg_.des_p_x = des.p(0);
  debug_msg_.des_p_y = des.p(1);
  debug_msg_.des_p_z = des.p(2);
  
  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);
  
  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);
  
  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();
  
  debug_msg_.des_thr = u.thrust;
  
  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  return debug_msg_;
}

/*
  compute throttle percentage 
*/
double 
LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
{
  double throttle_percentage(0.0);
  
  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc(2) / thr2acc_;

  return throttle_percentage;
}

bool 
LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) // 45ms
    {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();
    
    /***********************************/
    /* Model: est_a(2) = thr1acc_ * thr */
    /***********************************/
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1 - K * thr) * P_ / rho2_;
    //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
    //fflush(stdout);

    debug_msg_.thr2acc = thr2acc_;
    return true;
  }
  return false;
}

void 
LinearControl::resetThrustMapping(void)
{
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
}

