#include <linear_control.h>
#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <tf/tf.h>
#include <Eigen/Core>
#include <Eigen/Geometry>


LinearControl::LinearControl()
  : mass_(0.49)
  , g_(9.81)
{
}

void
LinearControl::setMass(const double mass)
{
  mass_ = mass;
}

void
LinearControl::setGravity(const double g)
{
  g_ = g;
}

void
LinearControl::calculateControl(const Desired_State_t &des,
                        const Odom_Data_t &odom, 
                        const Imu_Data_t &imu,
                        Controller_Output_t &u,
                        Gain gain)
{
    /* WRITE YOUR CODE HERE */

    Eigen::Vector3d a_out;
    a_out[0] = des.a[0] + gain.Kv0 * (des.v[0] - odom.v[0]) + gain.Kp0 * (des.p[0] - odom.p[0]);
    a_out[1] = des.a[1] + gain.Kv1 * (des.v[1] - odom.v[1]) + gain.Kp1 * (des.p[1] - odom.p[1]);
    a_out[2] = des.a[2] + gain.Kv2 * (des.v[2] - odom.v[2]) + gain.Kp2 * (des.p[2] - odom.p[2]);

    u.thrust = LinearControl::mass_ * (LinearControl::g_ + a_out[2]);

    Eigen::Vector3d angle = odom.q.matrix().eulerAngles(2, 0, 1);
    double phi = 1 / LinearControl::g_ * (a_out[0] * sin(angle[0]) - a_out[1] * cos(angle[0]));
    double theta = 1 / LinearControl::g_ * (a_out[0] * cos(angle[0]) + a_out[1] * sin(angle[0]));

    u.q = Eigen::AngleAxisd(angle[0], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY());
}
