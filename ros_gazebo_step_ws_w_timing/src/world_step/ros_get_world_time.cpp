#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_world_time_test");

  ros::NodeHandle* rosnode = new ros::NodeHandle();

  // Waits for simulation time update.
  ros::Time last_ros_time_;
  bool wait = true;
  while (wait)
  {
    last_ros_time_ = ros::Time::now();
    std::cout << "\t\t\t Attempted getting sim time: " << last_ros_time_ << std::endl;
    if (last_ros_time_.toSec() > 0)
      //wait = false;
      std::cout << "Current Simulation Time: " << last_ros_time_ << std::endl;
  }

  ros::spin();

  return 0;
}
