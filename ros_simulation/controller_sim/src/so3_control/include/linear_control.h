#ifndef __LINEAR_CONTROL_H__
#define __LINEAR_CONTROL_H__

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <controller_utils.h>

class LinearControl
{
public:
  LinearControl();

  void setMass(const double mass);
  void setGravity(const double g);

  void calculateControl(const Desired_State_t &des,
                        const Odom_Data_t &odom, 
                        const Imu_Data_t &imu,
                        Controller_Output_t &u,
                        Gain gain);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // constants for the controller
  double          mass_;
  double          g_;
};

#endif
