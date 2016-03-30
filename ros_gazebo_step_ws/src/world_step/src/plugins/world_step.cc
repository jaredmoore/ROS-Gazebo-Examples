/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <sdf/sdf.hh>

#include <boost/bind.hpp>

#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

/// \example examples/plugins/world_edit.cc
/// This example creates a WorldPlugin, initializes the Transport system by
/// creating a new Node, and publishes messages to alter gravity.
namespace gazebo
{
  class WorldEdit : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
      // Create a new transport node
      this->node.reset(new transport::Node());

      // Initialize the node with the world name
      this->node->Init(_parent->GetName());

      std::cout <<_parent->GetName() << std::endl;
      // Create a publisher
      this->pub = this->node->Advertise<msgs::WorldControl>("~/world_control");

      // Listen to the update event.  Event is broadcast every simulation 
      // iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateEnd(
          boost::bind(&WorldEdit::OnUpdate, this));


      // Configure the initial message to the system
      msgs::WorldControl worldControlMsg;

      // Set the world to paused
      worldControlMsg.set_pause(0);

      // Set the step flag to true
      worldControlMsg.set_step(1);

      // Publish the initial message. 
      this->pub->Publish(worldControlMsg);
      std::cout << "Publishing Load." << std::endl;
    }

    // Called by the world update start event.
    public: void OnUpdate() 
    {
        // Throttle Publication
        gazebo::common::Time::MSleep(1000);

        msgs::WorldControl msg;
        msg.set_step(1);
        this->pub->Publish(msg);
        std::cout << "Publishing OnUpdate." << std::endl;
    }

    // Pointer to the world_controller
    private: transport::NodePtr node;
    private: transport::PublisherPtr pub;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(WorldEdit)
}
