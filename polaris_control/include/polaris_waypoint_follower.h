#ifndef POLARIS_WAYPOINT_FOLLOWER_H
#define POLARIS_WAYPOINT_FOLLOWER_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>


#include <stdio.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>

class WaypointFollower
{
public:
	WaypointFollower();

private:
	void loadWaypoints(std::string f);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

	std::vector<geometry_msgs::Point> waypoints_;
	nav_msgs::Odometry polaris_odom_;

	ros::NodeHandle nh_;
	ros::Subscriber odom_sub_;
	ros::Publisher steer_pub_;
	ros::Publisher drive_pub_;

	double max_steer_angle_;
	double max_drive_vel_;


};
#endif