#ifndef AR_NAV_MULTI_HPP
#define AR_NAV_MULTI_HPP

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

class ArNavMulti {

public:
	ArNavMulti();
	ros::NodeHandle m_nh;

	// Functions
	void sendCfPose();
	void setCfPose(const geometry_msgs::TransformStamped &msg);
	void initializeCfPose();

private:
	// Functions
	void markerPoseCallback(const geometry_msgs::TransformStamped &msg);

	// Subscribers
	ros::Subscriber m_marker_pose_sub;

	// Publisher
	ros::Publisher m_cf_pose_pub;

	// Variables
	tf::Transform m_transform;
	tf::TransformBroadcaster m_br;
	geometry_msgs::PoseStamped m_cf_pose;
	std::string m_cf_frame;
	std::string m_world_frame;
	std::string m_next_waypoint;
	std::vector<std::string> m_waypoint_list;
	std::string m_waypoints;
	int m_current_waypoint_id;
	ros::Time m_next_waypoint_timeout;
};

#endif	// AR_NAV_MULTI_HPP
