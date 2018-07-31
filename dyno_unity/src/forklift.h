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

class Forklift : public hardware_interface::RobotHW {
public:
  Forklift()
      : running_(true), start_srv_(nh_.advertiseService(
                            "start", &Forklift::start_callback, this)),
        stop_srv_(
            nh_.advertiseService("stop", &Forklift::stop_callback, this)) {
    registerJoints();
    registerJointLimits();
  }

  void registerJoints() {
    std::string wheel_joint_name = "drive_wheel_joint";

    hardware_interface::JointStateHandle state_handle;
    // Connect and register the joint state and velocity interface
    state_handle = hardware_interface::JointStateHandle(
        wheel_joint_name, &wheel_joint_.position, &wheel_joint_.velocity,
        &wheel_joint_.effort);

    jnt_state_interface_.registerHandle(state_handle);

    hardware_interface::JointHandle vel_handle(state_handle,
                                               &wheel_joint_.velocity_command);
    jnt_vel_interface_.registerHandle(vel_handle);

    std::string steer_joint_name = "steer_joint";
    // Connect and register the joint state and position interface
    state_handle = hardware_interface::JointStateHandle(
        steer_joint_name, &steer_joint_.position, &steer_joint_.velocity,
        &steer_joint_.effort);

    jnt_state_interface_.registerHandle(state_handle);

    hardware_interface::JointHandle pos_handle(state_handle,
                                               &steer_joint_.position_command);
    jnt_pos_interface_.registerHandle(pos_handle);

    std::string fork_joint_name = "fork_joint";
    // Connect and register the joint state and position interface
    state_handle = hardware_interface::JointStateHandle(
        fork_joint_name, &fork_joint_.position, &fork_joint_.velocity,
        &fork_joint_.effort);

    jnt_state_interface_.registerHandle(state_handle);

    pos_handle = hardware_interface::JointHandle(state_handle,
                                                 &fork_joint_.position_command);
    jnt_pos_interface_.registerHandle(pos_handle);

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_vel_interface_);
    registerInterface(&jnt_pos_interface_);
  }

  void registerJointLimits() {
    ros::NodeHandle nh;

    joint_limits_interface::JointLimits limits;
    joint_limits_interface::SoftJointLimits soft_limits;

    if (getJointLimits("drive_wheel_joint", nh, limits) == 0) {
      ROS_ERROR("Drive wheel joint limits not specified in params. Aborting!");
      throw;
    }
    hardware_interface::JointHandle joint_handle;

    joint_handle = jnt_vel_interface_.getHandle("drive_wheel_joint");
    joint_limits_interface::VelocityJointSoftLimitsHandle drive_wheel_handle(
        joint_handle, limits, soft_limits);
    velocity_joint_limits_interface_.registerHandle(drive_wheel_handle);

    if (getJointLimits("steer_joint", nh, limits) == 0) {
      ROS_ERROR("Steer joint limits not specified in params. Aborting!");
      throw;
    }
    joint_handle = jnt_pos_interface_.getHandle("steer_joint");
    joint_limits_interface::PositionJointSoftLimitsHandle steer_handle(
        joint_handle, limits, soft_limits);
    position_joint_limits_interface_.registerHandle(steer_handle);


    if (getJointLimits("fork_joint", nh, limits) == 0) {
      ROS_ERROR("Fork joint limits not specified in params. Aborting!");
      throw;
    }
    joint_handle = jnt_pos_interface_.getHandle("fork_joint");
    joint_limits_interface::PositionJointSoftLimitsHandle fork_handle(
        joint_handle, limits, soft_limits);
    position_joint_limits_interface_.registerHandle(fork_handle);
  }

  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(0.01); }

  double getSteerPositionCommand() { return steer_joint_.position_command; }

  void read() {}

  void write() {
    velocity_joint_limits_interface_.enforceLimits(getPeriod());
    position_joint_limits_interface_.enforceLimits(getPeriod());

    if (running_) {
      wheel_joint_.position +=
          wheel_joint_.velocity * getPeriod().toSec(); // update position
      wheel_joint_.velocity = wheel_joint_.velocity_command;
      steer_joint_.position = steer_joint_.position_command;
      fork_joint_.position = fork_joint_.position_command;
    } else {
      wheel_joint_.position = std::numeric_limits<double>::quiet_NaN();
      wheel_joint_.velocity = std::numeric_limits<double>::quiet_NaN();
      steer_joint_.position = std::numeric_limits<double>::quiet_NaN();
      steer_joint_.velocity = std::numeric_limits<double>::quiet_NaN();
      fork_joint_.position = std::numeric_limits<double>::quiet_NaN();
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
  hardware_interface::PositionJointInterface jnt_pos_interface_;

  joint_limits_interface::VelocityJointSoftLimitsInterface
      velocity_joint_limits_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface
      position_joint_limits_interface_;

  struct Joint {
    double position;
    double velocity;
    double effort;
    double velocity_command;

    Joint() : position(0), velocity(0), effort(0), velocity_command(0) {}
  } wheel_joint_;

  struct SteeringJoint {
    double position;
    double velocity;
    double effort;
    double position_command;

    SteeringJoint()
        : position(0), velocity(0), effort(0), position_command(0) {}
  } steer_joint_;

  struct ForkJoint {
    double position;
    double velocity;
    double effort;
    double position_command;

    ForkJoint() : position(0), velocity(0), effort(0), position_command(0) {}
  } fork_joint_;

  bool running_;

  ros::NodeHandle nh_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;
};
