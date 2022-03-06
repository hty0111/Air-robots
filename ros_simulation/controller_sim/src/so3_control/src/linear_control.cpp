#include <linear_control.h>
#include <iostream>
#include <ros/ros.h>

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
}
