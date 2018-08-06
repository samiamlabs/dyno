/*
License: BSD
https://raw.githubusercontent.com/samiamlabs/dyno/master/LICENCE
*/

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <realtime_tools/realtime_buffer.h>

struct SyncMessage {
  ros::Time simulation_time;

  SyncMessage() : simulation_time(ros::Time(0)) {}
};

realtime_tools::RealtimeBuffer<SyncMessage> sync_message_;
bool sync_updated_;

void clockSyncCallback(const rosgraph_msgs::Clock &clock_msg) {
  SyncMessage sync_struct;
  sync_struct.simulation_time = clock_msg.clock;
  sync_message_.writeFromNonRT(sync_struct);

  //TODO: compensate for time until read in realtime thread
  //TODO: compensate for roundrip delay

  //FIXME: figure out better way to do this...
  sync_updated_ = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unity_clock_sync");
  ros::NodeHandle nh;

  // This should be set in launch files
  // as well
  nh.setParam("/use_sim_time", true);

  SyncMessage sync_struct;
  sync_updated_ = false;

  ros::Subscriber sub_sync = nh.subscribe( "/clock_sync", 1, clockSyncCallback);

  ros::Publisher clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  boost::chrono::system_clock::time_point wait_until   = boost::chrono::system_clock::now();

  ros::Time internal_time(0);

  const ros::Duration dt = ros::Duration(0.01); // 100Hz

  while(ros::ok() && sync_updated_ == false){
    usleep(1);
    ROS_WARN_STREAM_THROTTLE(
          5, "Waiting for sync message");
  }

  sync_struct = *(sync_message_.readFromRT());
  internal_time = sync_struct.simulation_time;

  while(ros::ok())
  {

    usleep(dt.toSec() * 1e6);

    rosgraph_msgs::Clock clock;
    clock.clock = ros::Time(internal_time);
    clock_publisher.publish(clock);

    if (sync_updated_) {

      sync_updated_ = false;

      sync_struct = *(sync_message_.readFromRT());

      if (internal_time < sync_struct.simulation_time) {
        internal_time = sync_struct.simulation_time;
      } else {
        // Moving time backwards makes everyting crash
        ros::Duration time_difference = internal_time - sync_struct.simulation_time;
        ROS_WARN_STREAM_THROTTLE( 1,

          "ROS ahead of simulation, waiting for simulation to cactch up. Simulation: "
          << sync_struct.simulation_time << " ROS: " << internal_time << " Difference: " << time_difference);

        boost::chrono::nanoseconds nsecs(time_difference.toNSec());
        wait_until = boost::chrono::system_clock::now() + nsecs;
      }

      // TODO: Allow different timescales
    }

    if (boost::chrono::system_clock::now() > wait_until) {
      internal_time += dt;
    }

  }

  spinner.stop();

  return 0;
}
