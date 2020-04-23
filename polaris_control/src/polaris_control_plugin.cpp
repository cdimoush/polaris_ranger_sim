#include <algorithm>
#include <assert.h>

#include <polaris_control_plugin.h>

using namespace gazebo;

enum
{
  RIGHT, LEFT, LSTEER, RSTEER
};

const double TAU = 6.28318530717958647693;  // 2 * pi

// Constructor
ControlPlugin::ControlPlugin()
{

}

// Destructor
ControlPlugin::~ControlPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection);
  delete transform_broadcaster_;
  rosnode_->shutdown();
  callback_queue_thread_.join();
  delete rosnode_;
}

// Load the controller
void ControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  
  ROS_INFO("ControlPlugin::Load");
  
  world = _model->GetWorld();
  robotNamespace = _model->GetName();

  //Get parameters from the sdf
  if (!_sdf->HasElement("wheelDistance"))
    wheel_distance_ = 1.2;
  else
    wheel_distance_ = _sdf->GetElement("wheelDistance")->Get<double>();

  if (!_sdf->HasElement("CarLength"))
    car_length_ = 1.88;
  else
    car_length_ = _sdf->GetElement("CarLength")->Get<double>();  

  if (!_sdf->HasElement("wheelDiameter"))
    wheel_diam_ = 0.32;
  else
    wheel_diam_ = _sdf->GetElement("wheelDiameter")->Get<double>();
  
  if (!_sdf->HasElement("driveTorque"))
    drive_torque_ = 10.0;
  else
    drive_torque_ = _sdf->GetElement("driveTorque")->Get<double>();  

  if (!_sdf->HasElement("steerTorque"))
    steer_torque_ = 10.0;
  else
    steer_torque_ = _sdf->GetElement("steerTorque")->Get<double>();


  if (!_sdf->HasElement("bodyName"))
  {
    link = _model->GetLink();
    linkName = link->GetName();
  }
  else {
    linkName = _sdf->GetElement("bodyName")->Get<std::string>();
    link = _model->GetLink(linkName);
  }

  // assert that the body by linkName exists
  if (!link)
  {
    ROS_FATAL("ControlPlugin error: bodyName: %s does not exist\n", linkName.c_str());
    return;
  }

  if (_sdf->HasElement("leftJoint"))  joints[LEFT]  = _model->GetJoint(_sdf->GetElement("leftJoint")->Get<std::string>());
  if (_sdf->HasElement("rightJoint"))  joints[RIGHT]  = _model->GetJoint(_sdf->GetElement("rightJoint")->Get<std::string>());
  if (_sdf->HasElement("leftsteerJoint"))  joints[LSTEER]  = _model->GetJoint(_sdf->GetElement("leftsteerJoint")->Get<std::string>());
  if (_sdf->HasElement("rightsteerJoint"))  joints[RSTEER]  = _model->GetJoint(_sdf->GetElement("rightsteerJoint")->Get<std::string>());
  
  if (!joints[LEFT])  ROS_FATAL("Plugin error: The controller couldn't get left joint");
  if (!joints[RIGHT]) ROS_FATAL("Plugin error: The controller couldn't get right joint");
  if (!joints[LSTEER]) ROS_FATAL("Plugin error: The controller couldn't get leftsteer joint");
  if (!joints[RSTEER]) ROS_FATAL("Plugin error: The controller couldn't get rightsteer joint");
  
  //Set initial variable values
  enable_motors_ = true;
  drive_vel_=0;
  drive_acc_=0;
  steer_vel_[0]=0;
  steer_vel_[1]=0;
  steer_vel_[2]=0;
  steer_angle_[0]=0;
  steer_angle_[1]=0;
  steer_angle_[2]=0;
  turn_radius_=0;

  prevUpdateTime = world->GetSimTime();

  //Set maximum force of joints
  joints[LEFT]->SetParam("fmax", 0, drive_torque_);
  joints[RIGHT]->SetParam("fmax", 0, drive_torque_);
  joints[LSTEER]->SetParam("fmax", 0, steer_torque_);
  joints[RSTEER]->SetParam("fmax", 0, steer_torque_);
 
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"ackermann_plugin",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  rosnode_ = new ros::NodeHandle(robotNamespace);

  tf_prefix_ = tf::getPrefixParam(*rosnode_);
  transform_broadcaster_ = new tf::TransformBroadcaster();

  // Get controller topic names
  rosnode_->param<std::string>("topic_from_steer_controller", topic_from_steer_controller_, "fuck_this");
  rosnode_->getParam("topic_from_drive_controller", topic_from_drive_controller_);
  rosnode_->getParam("topic_from_steer_plant", topic_from_steer_plant_);
  rosnode_->getParam("topic_from_drive_plant", topic_from_drive_plant_);

 
  // ROS: Subscribe to the Steer PID output
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<std_msgs::Float64>(topic_from_steer_controller_, 1,
                                                               boost::bind(&ControlPlugin::steerEffortCallback, this, _1),
                                                               ros::VoidPtr(), &queue_);
  steer_sub_ = rosnode_->subscribe(so);

  // ROS: Subscribe to the Drive PID output
  ros::SubscribeOptions s1 =
      ros::SubscribeOptions::create<std_msgs::Float64>(topic_from_drive_controller_, 1,
                                                               boost::bind(&ControlPlugin::driveEffortCallback, this, _1),
                                                               ros::VoidPtr(), &queue_);
  
  drive_sub_ = rosnode_->subscribe(s1);
  
  // ROS: Publishers
  steer_pub_ = rosnode_->advertise<std_msgs::Float64>(topic_from_steer_plant_, 1);
  drive_pub_ = rosnode_->advertise<std_msgs::Float64>(topic_from_drive_plant_, 1);
  pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);
  
  callback_queue_thread_ = boost::thread(boost::bind(&ControlPlugin::QueueThread, this));
  
  Reset();
  
  // Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ControlPlugin::Update, this));
}

// Initialize the plugin
void ControlPlugin::Init()
{
  callback_queue_thread_ = boost::thread(boost::bind(&ControlPlugin::QueueThread, this));
}


// Reset
void ControlPlugin::Reset()
{
  prevUpdateTime = world->GetSimTime();
  alive_ = true;
  joints[LEFT]->SetParam ( "fmax", 0, drive_torque_ );
  joints[RIGHT]->SetParam ( "fmax", 0, drive_torque_ );
  joints[LSTEER]->SetParam ( "fmax", 0, steer_torque_ );
  joints[RSTEER]->SetParam ( "fmax", 0, steer_torque_ );
}

// Update the polaris
void ControlPlugin::Update()
{  
  //Update time
  common::Time stepTime;

  // for ( int i = 0; i < 2; i++ ) 
  // {
  //    if ( fabs(drive_torque_ -joints[i]->GetParam ( "fmax", 0 )) > 1e-6 ) 
  //       joints[i]->SetParam ( "fmax", 0, drive_torque_);
  // }

  stepTime = world->GetSimTime() - prevUpdateTime;
  prevUpdateTime = world->GetSimTime();

  //USE LOCK TO GET ACCESS TO THE 'ACCELERATION_' AND 'STEER_VELOCITY_' VARIABLES
  lock.lock();

  drive_vel_ = drive_vel_ + drive_acc_*stepTime.Float();

  //SET SPEED LIMITS
  if (drive_vel_ > 20) drive_vel_ = 20; 
  if (drive_vel_ < -20) drive_vel_ = -20;
  
  //ACKERMAN STEER CALCS
  //1) get true tire position
  steer_angle_[1] = joints[LSTEER]->GetAngle(0).Radian();
  steer_angle_[2] = joints[RSTEER]->GetAngle(0).Radian();

  //2) calculate the pseudo center angle based on left tire position
  steer_angle_[0] = atan(pow(1/tan(steer_angle_[1]) + wheel_distance_/(2*car_length_), -1));
 
  // calculate wheel specific angular velocity
  double *w = ackermanCalcs(steer_angle_, steer_vel_);
  steer_vel_[0] = w[0];
  steer_vel_[1] = w[1];
  steer_vel_[2] = w[2];

  //IF MOTORS ARE ENABLED, SET JOINT TORQUES
  enable_motors_ = true;
  if (enable_motors_)
  {

    if(drive_vel_ > 0)
    {
      joints[LEFT]->SetForce(0, drive_torque_);
      joints[RIGHT]->SetForce(0,drive_torque_);
    }
    else if (drive_vel_ < 0)
    {
      joints[LEFT]->SetForce(0, -drive_torque_);
      joints[RIGHT]->SetForce(0,-drive_torque_);
    }

    if(steer_vel_[0]>0)
    {    
      joints[LSTEER]->SetForce(0,steer_torque_);
      joints[RSTEER]->SetForce(0,steer_torque_);
    }
    else if (steer_vel_[0]<0)
    {
      joints[LSTEER]->SetForce(0,-steer_torque_);
      joints[RSTEER]->SetForce(0,-steer_torque_);
    }

    //SET JOINT VELOCITIES
    joints[LEFT]->SetParam("vel", 0, drive_vel_ / (wheel_diam_ / 2.0));
    joints[RIGHT]->SetParam("vel", 0, drive_vel_ / (wheel_diam_ / 2.0));

    joints[LSTEER]->SetParam("vel", 0, steer_vel_[1]);
    joints[RSTEER]->SetParam("vel", 0, steer_vel_[2]);


  }

  //Publish feedback to PID controllers 
  std_msgs::Float64 steer_state_msg;
  std_msgs::Float64 drive_state_msg;

  steer_state_msg.data = steer_angle_[0];
  drive_state_msg.data = drive_vel_;

  steer_pub_.publish(steer_state_msg);
  drive_pub_.publish(drive_state_msg);

  lock.unlock();
  publish_odometry();
}

double* ControlPlugin::ackermanCalcs(double* theta, double* w)
{
  //ACKERMAN CALCULATIONS FUNCTION
  //GOAL: calculate angular velocities for the right and left wheels

  //NOTE:
  //The controller is based on feedback from left wheel joint position
  //This function calculates the theoretical position of the right wheel joint
  //
  //Since there is a possibility for wheels to become out of sync
  //The function considers right wheel error and can modifiy initial calculated velocity
  double theoretical;
  double error;
  double w_correction;
  double a = 5; //Maximum correction velocity
  //Theoretical right tire position
  theoretical = atan(2*car_length_*sin(theta[0])/(2*car_length_*cos(theta[0])+wheel_distance_*sin(theta[0])));

  if (abs(theta[0]) < 1e-3)
  {
    w[1] = w[0];
    w[2] = w[0];
  }
  else
  {
    w[1] = pow(sin(theta[1])/sin(theta[0]), 2)*w[0];
    w[2] = pow(sin(theoretical)/sin(theta[0]), 2)*w[0];
  }
  
  //RIGHT WHEEL ERROR CORRECTION
  //1) calculate error
  error = theoretical - theta[2];
  //2) adjust wheel velocity based on error
  ROS_INFO("t0: %f, t1: %f, t2: %f, error: %f", theta[0], theta[1], theta[2], error);
  if (fabs(error) > 1e-6)
  {
    w_correction = a*error / 3.1415;
    w[2] = w[2] + w_correction;
  }

  return w;
}

//update steer velocity with PID control effort
void ControlPlugin::steerEffortCallback(const std_msgs::Float64::ConstPtr& cmd_msg)
{
  lock.lock();

  steer_vel_[0] = cmd_msg->data;

  lock.unlock();
}

//update drive acceleration with PID control effort
void ControlPlugin::driveEffortCallback(const std_msgs::Float64::ConstPtr& cmd_msg)
{
  lock.lock();

  drive_acc_ = cmd_msg->data;

  lock.unlock();
}

//callback queue thread
void ControlPlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (alive_ && rosnode_->ok())
  {
    //    std::cout << "CALLING STUFF\n";
    
    queue_.callAvailable(ros::WallDuration(timeout));
  }

}

// NEW: Update this to publish odometry topic
void ControlPlugin::publish_odometry()
{	

  // get current time
  ros::Time current_time_((world->GetSimTime()).sec, (world->GetSimTime()).nsec);

  // getting data for base_footprint to odom transform
  math::Pose pose = link->GetWorldPose();
  math::Vector3 velocity = link->GetWorldLinearVel();
  math::Vector3 angular_velocity = link->GetWorldAngularVel();

  

  tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
  tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);
  tf::Transform base_footprint_to_odom(qt, vt);


  transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom,
                                                            current_time_,
                                                            "odom",
                                                            "base_footprint"));

  // publish odom topic
  odom_.pose.pose.position.x = pose.pos.x;
  odom_.pose.pose.position.y = pose.pos.y;

  odom_.pose.pose.orientation.x = pose.rot.x;
  odom_.pose.pose.orientation.y = pose.rot.y;
  odom_.pose.pose.orientation.z = pose.rot.z;
  odom_.pose.pose.orientation.w = pose.rot.w;

  odom_.twist.twist.linear.x = velocity.x;
  odom_.twist.twist.linear.y = velocity.y;
  odom_.twist.twist.angular.z = angular_velocity.z;

  odom_.header.frame_id = tf::resolve(tf_prefix_, "odom");
  odom_.child_frame_id = "base_footprint";
  odom_.header.stamp = current_time_;



  pub_.publish(odom_);


}

GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)
