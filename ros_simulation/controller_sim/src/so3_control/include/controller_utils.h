#ifndef __CONTROLLER_UTILS_H__
#define __CONTROLLER_UTILS_H__

# include <Eigen/Geometry>
# include <uav_utils/utils.h>
#include <sensor_msgs/Imu.h>


struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
    Eigen::Vector3d j;
	Eigen::Quaterniond q;

	Desired_State_t(){};
};

struct Controller_Output_t
{
	// Orientation of the body frame with respect to the world frame
	Eigen::Quaterniond q;

	// Collective mass normalized thrust
	double thrust;

};

struct Gain
{
    double Kp0, Kp1, Kp2;
    double Kv0, Kv1, Kv2;
    double Kvi0, Kvi1, Kvi2;
    double Kvd0, Kvd1, Kvd2;
    double KAngR, KAngP, KAngY;
};

class Odom_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Quaterniond q;
  Eigen::Vector3d w;

  nav_msgs::Odometry msg;
  ros::Time rcv_stamp;
  bool recv_new_msg;

  void feed(nav_msgs::OdometryConstPtr pMsg);
};

void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    recv_new_msg = true;

    uav_utils::extract_odometry(pMsg, p, v, q, w);
}

class Imu_Data_t
{
public:
  Eigen::Quaterniond q;
  Eigen::Vector3d w;
  Eigen::Vector3d a;

  sensor_msgs::Imu msg;
  ros::Time rcv_stamp;

  void feed(sensor_msgs::ImuConstPtr pMsg);
};

void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    w(0) = msg.angular_velocity.x;
    w(1) = msg.angular_velocity.y;
    w(2) = msg.angular_velocity.z;

    a(0) = msg.linear_acceleration.x;
    a(1) = msg.linear_acceleration.y;
    a(2) = msg.linear_acceleration.z;

    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;
}

#endif
