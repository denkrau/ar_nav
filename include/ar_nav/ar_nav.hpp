#ifndef AR_NAV_HPP
#define AR_NAV_HPP

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

class Ar_Nav {

public:
	Ar_Nav();
	ros::NodeHandle nh;
	// Functions
	void markerPoseCallback(const geometry_msgs::PoseStamped &msg);
	void sendCfPose();
	void setCfPose(const geometry_msgs::PoseStamped &msg);
	void initializeCfPose();

private:
	// Subscribers
	ros::Subscriber sub_marker_pose_;

	// Publisher
	ros::Publisher pub_cf_pose_;

	// Variables
	tf::Transform transform;
	tf::TransformBroadcaster br;
	geometry_msgs::PoseStamped cf_pose;
	std::string cf_frame;
	std::string world_frame;
};

#endif	// AR_NAV_HPP
