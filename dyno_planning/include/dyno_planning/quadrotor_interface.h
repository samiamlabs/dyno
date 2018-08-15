/*
License: BSD
https://raw.githubusercontent.com/samiamlabs/dyno/master/LICENCE
*/

#include <ros/ros.h>
#include <vector>

#include <rosplan_action_interface/RPActionInterface.h>

#ifndef RPQuadrotorInterface
#define RPQuadrotorInterface

/**
 * This file defines a ROSPlan action interface for the dyno quadrotor platform
 */

namespace KCL_rosplan {

class QuadrotorInterface : public RPActionInterface {

public:
  QuadrotorInterface(ros::NodeHandle &nh);

  /* listen to and process action_dispatch topic */
  bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};

} // namespace KCL_rosplan
#endif // RPQuadrotorInterface
