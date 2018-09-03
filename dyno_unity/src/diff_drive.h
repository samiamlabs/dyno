/*
License: BSD
https://raw.githubusercontent.com/samiamlabs/dyno/master/LICENCE
*/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <limits>

#include <sstream>

class DiffDrive : public hardware_interface::RobotHW {
public:
  DiffDrive()
      : running_(true), start_srv_(nh_.advertiseService(
                            "start", &DiffDrive::start_callback, this)),
        stop_srv_(
            nh_.advertiseService("stop", &DiffDrive::stop_callback, this)) {
    registerJoints();
    registerJointLimits();
  }

  void registerJoints() {
    std::string left_wheel_joint_name = "wheel_left_joint";

    // Connect and register the joint state and velocity interface
    hardware_interface::JointStateHandle wheel_left_state_handle(
        left_wheel_joint_name,
        &left_wheel_joint_.position,
        &left_wheel_joint_.velocity,
        &left_wheel_joint_.effort
    );

    jnt_state_interface_.registerHandle(wheel_left_state_handle);

    hardware_interface::JointHandle left_vel_handle(wheel_left_state_handle, &left_wheel_joint_.velocity_command);
    jnt_vel_interface_.registerHandle(left_vel_handle);

    std::string right_wheel_joint_name = "wheel_right_joint";

    // Connect and register the joint state and velocity interface
    hardware_interface::JointStateHandle wheel_right_state_handle(
        right_wheel_joint_name,
        &right_wheel_joint_.position,
        &right_wheel_joint_.velocity,
        &right_wheel_joint_.effort
    );

    jnt_state_interface_.registerHandle(wheel_right_state_handle);

    hardware_interface::JointHandle right_vel_handle(wheel_right_state_handle, &right_wheel_joint_.velocity_command);
    jnt_vel_interface_.registerHandle(right_vel_handle);

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_vel_interface_);
  }

  void registerJointLimits() {
    ros::NodeHandle nh;

    joint_limits_interface::JointLimits limits;
    joint_limits_interface::SoftJointLimits soft_limits;

    if (getJointLimits("wheel_left_joint", nh, limits) == 0) {
      ROS_ERROR("Left drive wheel joint limits not specified in params. Aborting!");
      throw;
    }
    hardware_interface::JointHandle joint_handle;

    joint_handle = jnt_vel_interface_.getHandle("wheel_left_joint");
    joint_limits_interface::VelocityJointSoftLimitsHandle left_wheel_handle(
        joint_handle, limits, soft_limits);
    velocity_joint_limits_interface_.registerHandle(left_wheel_handle);

    if (getJointLimits("wheel_right_joint", nh, limits) == 0) {
      ROS_ERROR("Right drive wheel joint limits not specified in params. Aborting!");
      throw;
    }

    joint_handle = jnt_vel_interface_.getHandle("wheel_right_joint");
    joint_limits_interface::VelocityJointSoftLimitsHandle right_wheel_handle(
        joint_handle, limits, soft_limits);
    velocity_joint_limits_interface_.registerHandle(right_wheel_handle);
  }

  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(0.01); }

  void read() {}

  void write() {
    velocity_joint_limits_interface_.enforceLimits(getPeriod());

    if (running_) {
      left_wheel_joint_.position += left_wheel_joint_.velocity * getPeriod().toSec(); // update position
      left_wheel_joint_.velocity = left_wheel_joint_.velocity_command;
      right_wheel_joint_.position += right_wheel_joint_.velocity * getPeriod().toSec(); // update position
      right_wheel_joint_.velocity = right_wheel_joint_.velocity_command;
    } else {
      left_wheel_joint_.position = std::numeric_limits<double>::quiet_NaN();
      left_wheel_joint_.velocity = std::numeric_limits<double>::quiet_NaN();
      right_wheel_joint_.position = std::numeric_limits<double>::quiet_NaN();
      right_wheel_joint_.velocity = std::numeric_limits<double>::quiet_NaN();
    }
  }

  bool start_callback(std_srvs::Empty::Request & /*req*/,
                      std_srvs::Empty::Response &
                      /*res*/) {
    running_ = true;
    return true;
  }

  bool stop_callback(std_srvs::Empty::Request & /*req*/,
                     std_srvs::Empty::Response &
                     /*res*/) {
    running_ = false;
    return true;
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;

  joint_limits_interface::VelocityJointSoftLimitsInterface
      velocity_joint_limits_interface_;

  struct Joint {
    double position;
    double velocity;
    double effort;
    double velocity_command;

    Joint() : position(0), velocity(0), effort(0), velocity_command(0) {}
  };

  Joint left_wheel_joint_, right_wheel_joint_;

  bool running_;

  ros::NodeHandle nh_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;
};
