#ifndef AR_NAV_MULTI_HPP
#define AR_NAV_MULTI_HPP

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

class ArNavMulti {

public:
	ArNavMulti();
	ros::NodeHandle nh;

	// Functions
	void sendCfPose();
	void setCfPose(const geometry_msgs::TransformStamped &msg);
	void initializeCfPose();
	bool onNextWaypoint(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
	bool onPrevWaypoint(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);


private:
	// Functions
	void markerPoseCallback(const geometry_msgs::TransformStamped &msg);
	void setWaypoint();
	void requestWaypoint(int waypoint_offset);

	// Subscribers
	ros::Subscriber m_marker_pose_sub;

	// Publisher
	ros::Publisher m_cf_pose_pub;

	ros::Publisher debug_pose_pub;

	// Services
	ros::ServiceServer m_next_waypoint_srv;
	ros::ServiceServer m_prev_waypoint_srv;

	// Variables
	tf::Transform m_transform;
	tf::TransformBroadcaster m_br;
	geometry_msgs::PoseStamped m_cf_pose;
	std::string m_cf_frame;
	std::string m_world_frame;
	std::string m_next_waypoint;
	std::string m_waypoint_change;
	std::vector<std::string> m_waypoint_list;
	std::string m_waypoints;
	int m_current_waypoint_id;
	int m_requested_waypoint_id;
	ros::Time m_next_waypoint_timeout;
	bool m_step_active;
	bool m_request_active;
};

#endif	// AR_NAV_MULTI_HPP
