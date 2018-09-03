/*
License: BSD
https://raw.githubusercontent.com/samiamlabs/dyno/master/LICENCE
*/

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <realtime_tools/realtime_buffer.h>

ros::Time internal_time(0);
ros::Publisher clock_sync_publisher;
bool use_sim_time_;

void ClockSyncCallback(const rosgraph_msgs::Clock &clock_msg) {
    rosgraph_msgs::Clock clock;
    if(use_sim_time_)
    {
      clock.clock = ros::Time(internal_time);
    } else
    {
      clock.clock = ros::Time::now();
    }
    clock_sync_publisher.publish(clock);
    ROS_INFO("clock sync callback");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unity_clock");
  ros::NodeHandle nh;

  // This should be set in launch files as well
  // nh.setParam("/use_sim_time", true);
  nh.param<bool>("use_sim_time", use_sim_time_, false);

  ros::Publisher clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

  clock_sync_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock_sync_response", 1);
  ros::Subscriber sub_sync = nh.subscribe( "/clock_sync_request", 1, ClockSyncCallback);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  const ros::Duration dt = ros::Duration(0.01); // 100Hz

  while(ros::ok())
  {
    usleep(dt.toSec() * 1e6);

    rosgraph_msgs::Clock clock;
    if(use_sim_time_)
    {
      clock.clock = ros::Time(internal_time);
      clock_publisher.publish(clock);
    }
    //TODO: Publish ros::Time:now() on /clock when not using sim time?

    internal_time += dt;
  }

  spinner.stop();

  return 0;
}
