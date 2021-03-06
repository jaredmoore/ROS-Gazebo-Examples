#include <math.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <ros/ros.h>
#include <ros/subscribe_options.h>

int main(int argc, char** argv)
{
  gazebo::setupClient(argc,argv);
  ros::init(argc, argv, "get_world_time_test");

  // Gazebo WorldControl Topic
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Gazebo ServerControl Topic
  gazebo::transport::NodePtr serverNode(new gazebo::transport::Node());
  serverNode->Init();

  // Initialize the ROSNode
  ros::NodeHandle* rosnode = new ros::NodeHandle();
    
  // Gazebo Publishers
  gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
  
  gazebo::transport::PublisherPtr serverPub = serverNode->Advertise<gazebo::msgs::ServerControl>("/gazebo/server/control");

  // Waits for simulation time update.
  ros::Time last_ros_time_;
  bool wait = true;
  while (wait)
  {
    last_ros_time_ = ros::Time::now();

    if (last_ros_time_.toSec() > 0) {
      if (last_ros_time_.toSec() > 0.02)
        wait = false;
      std::cout << "Current Simulation Time: " << last_ros_time_ << std::endl;
    }    

    // Publish the step message for the simulation.
    gazebo::msgs::WorldControl msg;
    msg.set_step(1);
    pub->Publish(msg);
    
    // Wait for 1 second and allow ROS to complete as well.
    gazebo::common::Time::MSleep(1000);
    ros::spinOnce();
  }

  // Send a server control shutdown message.
  gazebo::msgs::ServerControl serverMsg;
  serverMsg.set_stop(1);
  serverPub->Publish(serverMsg);

  //gazebo::shutdown();

  return 0;
}
