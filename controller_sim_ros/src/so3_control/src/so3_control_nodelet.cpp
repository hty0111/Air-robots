#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <quadrotor_msgs/Corrections.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <linear_control.h>
#include <controller_utils.h>

class SO3ControlNodelet : public nodelet::Nodelet
{
public:
  SO3ControlNodelet()
    : position_cmd_updated_(false)
    , position_cmd_init_(false)
    , current_yaw_(0)
    , enable_motors_(true)
    , // FIXME
    use_external_yaw_(false)
  {
  }

  void onInit(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  void publishSO3Command(void);
  void position_cmd_callback(
    const quadrotor_msgs::PositionCommand::ConstPtr& cmd);
  void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
  void enable_motors_callback(const std_msgs::Bool::ConstPtr& msg);
  void corrections_callback(const quadrotor_msgs::Corrections::ConstPtr& msg);
  void imu_callback(const sensor_msgs::ImuConstPtr& imu);

  LinearControl controller_;
  ros::Publisher  so3_command_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber position_cmd_sub_;
  ros::Subscriber enable_motors_sub_;
  ros::Subscriber corrections_sub_;
  ros::Subscriber imu_sub_;

  bool        position_cmd_updated_, position_cmd_init_;
  std::string frame_id_;

  Desired_State_t des;
  Odom_Data_t odom_data;
  Imu_Data_t imu_data;
  Gain gain;

  double          current_yaw_;
  bool            enable_motors_;
  bool            use_external_yaw_;
  double          kR_[3], kOm_[3], corrections_[3];
  double          init_x_, init_y_, init_z_;
};

void
SO3ControlNodelet::publishSO3Command(void)
{
  Controller_Output_t u;
  controller_.calculateControl(des, odom_data, imu_data, u, gain);

  const Eigen::Vector3d&    force       = odom_data.q * Eigen::Vector3d(0.0, 0.0, u.thrust);
  const Eigen::Quaterniond& orientation = u.q;

  quadrotor_msgs::SO3Command::Ptr so3_command(
    new quadrotor_msgs::SO3Command); //! @note memory leak?
  so3_command->header.stamp    = ros::Time::now();
  so3_command->header.frame_id = frame_id_;
  so3_command->force.x         = force(0);
  so3_command->force.y         = force(1);
  so3_command->force.z         = force(2);
  so3_command->orientation.x   = orientation.x();
  so3_command->orientation.y   = orientation.y();
  so3_command->orientation.z   = orientation.z();
  so3_command->orientation.w   = orientation.w();
  for (int i = 0; i < 3; i++)
  {
    so3_command->kR[i]  = kR_[i];
    so3_command->kOm[i] = kOm_[i];
  }
  // so3_command->aux.current_yaw          = current_yaw_;
  so3_command->aux.kf_correction        = corrections_[0];
  so3_command->aux.angle_corrections[0] = corrections_[1];
  so3_command->aux.angle_corrections[1] = corrections_[2];
  so3_command->aux.enable_motors        = enable_motors_;
  so3_command->aux.use_external_yaw     = use_external_yaw_;
  so3_command_pub_.publish(so3_command);
}

void
SO3ControlNodelet::position_cmd_callback(
  const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
  des.p(0) = msg->position.x;
  des.p(1) = msg->position.y;
  des.p(2) = msg->position.z;

  des.v(0) = msg->velocity.x;
  des.v(1) = msg->velocity.y;
  des.v(2) = msg->velocity.z;

  des.a(0) = msg->acceleration.x;
  des.a(1) = msg->acceleration.y;
  des.a(2) = msg->acceleration.z;

  des.j(0) = msg->jerk.x;
  des.j(1) = msg->jerk.y;
  des.j(2) = msg->jerk.z;

  double yaw = uav_utils::normalize_angle(msg->yaw);
  des.q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  position_cmd_updated_ = true;
  position_cmd_init_    = true;

  publishSO3Command();
}

void
SO3ControlNodelet::odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
  odom_data.feed(odom);

  if (position_cmd_init_)
  {
    // We set position_cmd_updated_ = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the so3 command ourselves
    // TODO: Fallback to hover if position_cmd hasn't been received for some
    // time
    if (!position_cmd_updated_)   
      publishSO3Command();
    position_cmd_updated_ = false;
  }
  else if ( init_z_ > -9999.0 )
  {
    des.p = Eigen::Vector3d(init_x_, init_y_, init_z_);
    des.v = Eigen::Vector3d(0,0,0);
    des.a = Eigen::Vector3d(0,0,0);
    des.j = Eigen::Vector3d(0,0,0);
    des.q = Eigen::Quaterniond(1,0,0,0);
    publishSO3Command();
  }
  
}

void
SO3ControlNodelet::enable_motors_callback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
    ROS_INFO("Enabling motors");
  else
    ROS_INFO("Disabling motors");

  enable_motors_ = msg->data;
}

void
SO3ControlNodelet::corrections_callback(
  const quadrotor_msgs::Corrections::ConstPtr& msg)
{
  corrections_[0] = msg->kf_correction;
  corrections_[1] = msg->angle_corrections[0];
  corrections_[2] = msg->angle_corrections[1];
}

void
SO3ControlNodelet::imu_callback(const sensor_msgs::ImuConstPtr& imu)
{
  imu_data.feed(imu);
}

void
SO3ControlNodelet::onInit(void)
{
  ros::NodeHandle n(getPrivateNodeHandle());

  std::string quadrotor_name;
  n.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
  frame_id_ = "/" + quadrotor_name;

  double mass;
  n.param("mass", mass, 0.5);
  controller_.setMass(mass);

  n.param("use_external_yaw", use_external_yaw_, true);

  n.param("gains/rot/x", kR_[0], 1.5);
  n.param("gains/rot/y", kR_[1], 1.5);
  n.param("gains/rot/z", kR_[2], 1.0);
  n.param("gains/ang/x", kOm_[0], 0.13);
  n.param("gains/ang/y", kOm_[1], 0.13);
  n.param("gains/ang/z", kOm_[2], 0.1);
  
  n.param("gains/kx/x", gain.Kp0, 5.7);
  n.param("gains/kx/y", gain.Kp1, 5.7);
  n.param("gains/kx/z", gain.Kp2, 6.2);
  n.param("gains/kv/x", gain.Kv0, 3.4);
  n.param("gains/kv/y", gain.Kv1, 3.4);
  n.param("gains/kv/z", gain.Kv2, 4.0);
  n.param("gains/kAng/r", gain.KAngR, 3.4);
  n.param("gains/kAng/p", gain.KAngP, 3.4);
  n.param("gains/kAng/y", gain.KAngY, 4.0);


  n.param("corrections/z", corrections_[0], 0.0);
  n.param("corrections/r", corrections_[1], 0.0);
  n.param("corrections/p", corrections_[2], 0.0);

  n.param("so3_control/init_state_x", init_x_, 0.0);
  n.param("so3_control/init_state_y", init_y_, 0.0);
  n.param("so3_control/init_state_z", init_z_, -10000.0);

  so3_command_pub_ = n.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);

  odom_sub_ = n.subscribe("odom", 10, &SO3ControlNodelet::odom_callback, this,
                          ros::TransportHints().tcpNoDelay());
  position_cmd_sub_ =
    n.subscribe("position_cmd", 10, &SO3ControlNodelet::position_cmd_callback,
                this, ros::TransportHints().tcpNoDelay());

  enable_motors_sub_ =
    n.subscribe("motors", 2, &SO3ControlNodelet::enable_motors_callback, this,
                ros::TransportHints().tcpNoDelay());
  corrections_sub_ =
    n.subscribe("corrections", 10, &SO3ControlNodelet::corrections_callback,
                this, ros::TransportHints().tcpNoDelay());

  imu_sub_ = n.subscribe("imu", 10, &SO3ControlNodelet::imu_callback, this,
                         ros::TransportHints().tcpNoDelay());
}

#include <pluginlib/class_list_macros.h>
//PLUGINLIB_DECLARE_CLASS(so3_control, SO3ControlNodelet, SO3ControlNodelet,
//                        nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(SO3ControlNodelet, nodelet::Nodelet);
