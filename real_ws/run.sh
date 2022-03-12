roscore & sleep 5;
roslaunch vrpn_client_ros sample.launch server:=192.168.1.102 & sleep 3;
roslaunch mavros px4.launch & sleep 3;
roslaunch ekf PX4_vicon.launch & sleep 3;
roslaunch px4ctrl run_ctrl.launch 
#rosbag record /vicon_imu_ekf_odom /debugPx4ctrl
