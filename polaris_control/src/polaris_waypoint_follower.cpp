#include <polaris_waypoint_follower.h>

WaypointFollower::WaypointFollower()
{
	ros::NodeHandle node_priv("~");
	node_priv.getParam("max_steer", max_steer_angle_);
	node_priv.getParam("max_vel", max_drive_vel_);
	double accuracy = 0.75; //waypoint proximity threshold 

	ros::Rate loop_rate(10);
	ros::Time prev_time = ros::Time::now();

	odom_sub_ = nh_.subscribe("odom", 1000, &WaypointFollower::odomCallback, this);
	
	//STEER SETPOINT PUB
	std::string steer_setpoint_topic;
	node_priv.getParam("steer_setpoint_topic", steer_setpoint_topic);
	steer_pub_ = nh_.advertise<std_msgs::Float64>(steer_setpoint_topic, 1000);
	
	//DRIVE SETPOINT PUB
	std::string drive_setpoint_topic;
	node_priv.getParam("drive_setpoint_topic", drive_setpoint_topic);
	drive_pub_ = nh_.advertise<std_msgs::Float64>(drive_setpoint_topic, 1000);


	//Load waypoints from file
	std::string waypoint_data_file;
	node_priv.getParam("waypoint_data_address", waypoint_data_file);
	ROS_INFO("%s\n", waypoint_data_file.c_str());
	loadWaypoints(waypoint_data_file);

	int size = waypoints_.size();
	int index = 0;
	double old_dist;

	ROS_INFO("NEW GOAL! X: %f, Y: %f", waypoints_[index].x, waypoints_[index].y);
	while (ros::ok())
	{
		if (polaris_odom_.header.stamp.toSec() > prev_time.toSec())
		{
			prev_time = polaris_odom_.header.stamp;

			geometry_msgs::Pose polaris_pose = polaris_odom_.pose.pose;
			geometry_msgs::Point p = waypoints_[index];

			// Get dist to waypoint
			double dist;
			double pos_x;
			double pos_y;
			pos_x = polaris_pose.position.x;
			pos_y = polaris_pose.position.y;
			dist = sqrt(pow(p.x - pos_x, 2) + pow(p.y - pos_y, 2));

			// If goal get new waypoint
			if (dist < accuracy & old_dist < dist) 
			{
				index ++;
				if (index >= waypoints_.size()) index = 0;
				ROS_INFO("NEW GOAL! X: %f, Y: %f", waypoints_[index].x, waypoints_[index].y);

			}
			else
			{
				// Get angle between polaris and waypoint
				tf::Quaternion q(
				polaris_pose.orientation.x,
				polaris_pose.orientation.y,
				polaris_pose.orientation.z,
				polaris_pose.orientation.w);
				tf::Matrix3x3 m(q);
				double roll, pitch, yaw, theta;
				m.getRPY(roll, pitch, yaw);
				theta = atan2(p.y - pos_y, p.x - pos_x) - yaw;
				//Set that range to [-pi, pi]
				if (fabs(theta) > M_PI) theta = theta - 2*theta*M_PI/fabs(theta);

				// Set steer angle
				std_msgs::Float64 steer_angle;
				if (fabs(theta) > 1) steer_angle.data = theta/fabs(theta) * max_steer_angle_;
				else steer_angle.data = theta * max_steer_angle_;
				steer_pub_.publish(steer_angle);

				// Set drive velocity 
				std_msgs::Float64 drive_vel;
				drive_vel.data = max_drive_vel_;
				drive_pub_.publish(drive_vel);
			}
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

}

void WaypointFollower::loadWaypoints(std::string f)
{
	//READ WAYPOINTS FROM EXTERNAL FILE
	ROS_INFO("Loading waypoints");
	std::fstream file;
	file.open(f, std::fstream::in);
	ROS_INFO("Opening Waypoint file");
	while (file)
	{
		std::string x_str;
		std::string y_str;
		std::getline(file, x_str);
		std::getline(file, y_str);
		if (x_str.length() != 0 and y_str.length() != 0)
		{
			
			geometry_msgs::Point p;
			p.x = std::stod(x_str);
			p.y = std::stod(y_str);
			ROS_INFO("X: %f, Y: %f", p.x, p.y);
			waypoints_.push_back(p);
		}
	}

	ROS_INFO("file closed");
}

void WaypointFollower::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	polaris_odom_ =  *odom_msg;
}

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "Polaris_Waypoint_Follower");
	WaypointFollower w;

	return 0;		
}	