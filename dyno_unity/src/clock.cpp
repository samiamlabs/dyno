/*
License: BSD
https://raw.githubusercontent.com/samiamlabs/dyno/master/LICENCE
*/

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <realtime_tools/realtime_buffer.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unity_clock");
  ros::NodeHandle nh;

  // This should be set in launch files
  // as well
  nh.setParam("/use_sim_time", true);

  ros::Publisher clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time internal_time(0);

  const ros::Duration dt = ros::Duration(0.01); // 100Hz

  while(ros::ok())
  {
    usleep(dt.toSec() * 1e6);

    rosgraph_msgs::Clock clock;
    clock.clock = ros::Time(internal_time);
    clock_publisher.publish(clock);
    internal_time += dt;
  }

  spinner.stop();

  return 0;
}
