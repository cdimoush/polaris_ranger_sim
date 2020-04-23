#ifndef POLARIS_CONTROL_PLUGIN_H
#define POLARIS_CONTROL_PLUGIN_H

#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <stdio.h>
#include <math.h>

// ROS 
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{

class Entity;

class ControlPlugin : public ModelPlugin
{

public:
  ControlPlugin();
  virtual ~ControlPlugin();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Init();
  void Reset();
  virtual void Update();


private:
  void write_position_data();
  void publish_odometry();
  
  physics::WorldPtr world;
  physics::LinkPtr link;
  physics::JointPtr joints[4];
  

  double wheel_distance_;
  double car_length_;
  double wheel_diam_;
  double drive_torque_;
  double steer_torque_;

  // Simulation time of the last update
  common::Time prevUpdateTime;
  
  
  bool enable_motors_;
  double odomPose[3];
  double odomVel[3];


  // ROS STUFF
  ros::NodeHandle* rosnode_;
  ros::Publisher pub_;
  tf::TransformBroadcaster *transform_broadcaster_;
  nav_msgs::Odometry odom_;
  std::string tf_prefix_;

  boost::mutex lock;

  //Names
  std::string robotNamespace;
  std::string linkName;

  // Custom Callback Queue
  ros::CallbackQueue queue_;
  boost::thread callback_queue_thread_;
  void QueueThread();
  bool alive_;

  //Steer
  void steerEffortCallback(const std_msgs::Float64::ConstPtr& cmd_msg);
  double* ackermanCalcs(double* theta, double* w);
  ros::Subscriber steer_sub_;
  ros::Publisher steer_pub_;
  std::string topic_from_steer_controller_;
  std::string topic_from_steer_plant_;
  double steer_vel_[3]; //IMPORTANT: Gazebo  joint params must be double
  double steer_angle_[3];
  double turn_radius_; 

  //Drive
  void driveEffortCallback(const std_msgs::Float64::ConstPtr& cmd_msg);
  ros::Subscriber drive_sub_;
  ros::Publisher drive_pub_;
  std::string topic_from_drive_controller_;
  std::string topic_from_drive_plant_;
  double drive_acc_;
  double drive_vel_;

  //Drive
  
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};
}

#endif