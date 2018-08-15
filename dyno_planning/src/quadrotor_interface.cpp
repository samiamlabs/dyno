/*
License: BSD
https://raw.githubusercontent.com/samiamlabs/dyno/master/LICENCE
*/

#include <dyno_planning/quadrotor_interface.h>

namespace KCL_rosplan {

QuadrotorInterface::QuadrotorInterface(ros::NodeHandle &nh) {
  // perform setup
}

/* action dispatch callback */
bool QuadrotorInterface::concreteCallback(
    const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg) {
  // The action implementation goes here.

  // complete the action
  ROS_INFO("Dyno: (%s) Action completing.", msg->name.c_str());
  return true;
}

} // namespace KCL_rosplan

int main(int argc, char **argv) {
  ros::init(argc, argv, "quadrotor_rosplan_action",
            ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");

  // create PDDL action subscriber
  KCL_rosplan::QuadrotorInterface rpti(nh);

  rpti.runActionInterface();

  return 0;
}
