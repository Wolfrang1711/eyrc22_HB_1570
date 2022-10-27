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
 /*
  * Desc: A dynamic controller plugin that performs generic force interface.
  * Author: John Hsu
  * Date: 24 Sept 2008
  */
  
 #ifndef GAZEBO_ROS_FORCE_HH
 #define GAZEBO_ROS_FORCE_HH
  
 #include <string>
  
 // Custom Callback Queue
 #include <ros/callback_queue.h>
 #include <ros/subscribe_options.h>
 #include <geometry_msgs/Wrench.h>
  
 #include <ros/ros.h>
 #include <boost/thread.hpp>
 #include <boost/thread/mutex.hpp>
  
 #include <gazebo/physics/physics.hh>
 #include <gazebo/transport/TransportTypes.hh>
 #include <gazebo/common/Plugin.hh>
 #include <gazebo/common/Events.hh>
  
  
 namespace gazebo
 {
  
 class GazeboRosForce : public ModelPlugin
 {
   public: GazeboRosForce();
  
   public: virtual ~GazeboRosForce();
  
   // Documentation inherited
   protected: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  
   // Documentation inherited
   protected: virtual void UpdateChild();
  
   private: void UpdateObjectForce(const geometry_msgs::Wrench::ConstPtr& _msg);
  
   private: void QueueThread();
  
   private: physics::WorldPtr world_;
  
   private: physics::LinkPtr link_;
  
   private: ros::NodeHandle* rosnode_;
   private: ros::Subscriber sub_;
  
   private: boost::mutex lock_;
  
   private: std::string topic_name_;
   private: std::string link_name_;
  
   private: std::string robot_namespace_;
  
   // Custom Callback Queue
   private: ros::CallbackQueue queue_;
   private: boost::thread callback_queue_thread_;
   private: geometry_msgs::Wrench wrench_msg_;
  
   // Pointer to the update event connection
   private: event::ConnectionPtr update_connection_;
 };
 }
 #endif