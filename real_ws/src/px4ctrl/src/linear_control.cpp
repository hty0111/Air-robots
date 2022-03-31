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
	Eigen::Vector3d ddPc;
	Eigen::Vector3d Up,Ud;
	Eigen::Matrix3d imuR,nowR,desR;
	double Phi_c,Theta_c,Psi_c;
	double Psi;
	Eigen::Vector3d nowEuler,desEuler,imuEuler,transEuler;
	double g_;

	g_ = param_.gra;
	Eigen::Vector3d Gra(0,0,g_);

	Ud(0) = param_.gain.Kv0*(des.v(0)-odom.v(0));
	Ud(1) = param_.gain.Kv1*(des.v(1)-odom.v(1));
	Ud(2) = param_.gain.Kv2*(des.v(2)-odom.v(2));

	Up(0) = param_.gain.Kp0*(des.p(0)-odom.p(0));
	Up(1) = param_.gain.Kp1*(des.p(1)-odom.p(1));
	Up(2) = param_.gain.Kp2*(des.p(2)-odom.p(2));

	imuR = imu.q.matrix();
	nowR = odom.q.matrix();
	desR = des.q.matrix();

	// imuEuler = toEulerAngle(imu.q);
  	// nowEuler = toEulerAngle(odom.q);					  //yaw-pitch-roll
	// Psi = nowEuler(2);								      //yaw
	// desEuler = toEulerAngle(des.q);
	// Psi_c = desEuler(2);

	imuEuler = RtoEulerAngle(imuR);
  	nowEuler = RtoEulerAngle(nowR);					  //yaw-pitch-roll
	Psi = nowEuler(2);								      //yaw
	desEuler = RtoEulerAngle(desR);
	//Psi_c = desEuler(2);
	Psi_c = Psi;

	ddPc = des.a+Up+Ud+Gra;
	u.thrust = computeDesiredCollectiveThrustSignal(ddPc);
	Phi_c = (ddPc(0)*sin(Psi)-ddPc(1)*cos(Psi))/g_;
	Theta_c = (ddPc(0)*cos(Psi)+ddPc(1)*sin(Psi))/g_;

	Eigen::Vector3d Euler_c(Psi_c,Theta_c,Phi_c);	//yaw-pitch-roll
	Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(Euler_c(2),Eigen::Vector3d::UnitX()));
	Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(Euler_c(1),Eigen::Vector3d::UnitY()));
	Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(Euler_c(0),Eigen::Vector3d::UnitZ()));
	Eigen::Quaterniond quaternion_c;
	quaternion_c = yawAngle*rollAngle*pitchAngle;

	Eigen::AngleAxisd yawW2E(Eigen::AngleAxisd(Euler_c(0),Eigen::Vector3d::UnitZ()));
	
	u.q = imu.q*odom.q.inverse()*quaternion_c;
	
	transEuler = toEulerAngle(imu.q*odom.q.inverse());

	std::cout<<"****************************"<<std::endl;
	std::cout<<"-------imu------"<<std::endl;
	std::cout<<imuEuler<<std::endl;
	std::cout<<"-------odom------"<<std::endl;
	std::cout<<nowEuler<<std::endl;
	std::cout<<"-------trans------"<<std::endl;
	std::cout<<transEuler<<std::endl;
	std::cout<<"****************************"<<std::endl<<std::endl;

	//used for debug
	debug_msg_.des_p_x = des.p(0);
	debug_msg_.des_p_y = des.p(1);
	debug_msg_.des_p_z = des.p(2);

	debug_msg_.des_v_x = des.v(0);
	debug_msg_.des_v_y = des.v(1);
	debug_msg_.des_v_z = des.v(2);

	debug_msg_.des_a_x = ddPc(0);
	debug_msg_.des_a_y = ddPc(1);
	debug_msg_.des_a_z = ddPc(2);

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

Eigen::Vector3d toEulerAngle(Eigen::Quaterniond q)
{
	Eigen::Vector3d Rpy;
	// roll (x-axis rotation)
	double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
	double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
	Rpy(0) = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
	if (fabs(sinp) >= 1)
		Rpy(1) = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		Rpy(1) = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
	double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
	Rpy(2) = atan2(siny_cosp, cosy_cosp);

	return Rpy;
}

Eigen::Vector3d RtoEulerAngle(Eigen::Matrix3d R)
{
	Eigen::Vector3d Rpy;

	Rpy(0) = asin(R(1,2));
	Rpy(2) = atan2(-R(1,0)/cos(Rpy(0)),R(1,1)/cos(Rpy(0)));
	Rpy(1) = atan2(-R(0,2)/cos(Rpy(0)),R(2,2)/cos(Rpy(0)));

	return Rpy;
}

