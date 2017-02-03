#include "ar_nav/multi.hpp"

ArNavMulti::ArNavMulti() {
	// initialize topics
	ros::NodeHandle n("~");
	std::string s;
	n.param<std::string>("marker_pose_topic", s, "/marker_pose");
	n.param<std::string>("world_frame", m_world_frame, "world");
	n.param<std::string>("cf_frame", m_cf_frame, "crazyflie");
	n.param<std::string>("waypoints", m_waypoints, "board_c3po");
	n.param<std::string>("method", m_waypoint_change, "auto");
	m_marker_pose_sub = nh.subscribe(s, 1, &ArNavMulti::markerPoseCallback, this);
	m_cf_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("cf_pose", 1);
	m_next_waypoint_srv = nh.advertiseService("next_waypoint", &ArNavMulti::onNextWaypoint, this);
	m_prev_waypoint_srv = nh.advertiseService("prev_waypoint", &ArNavMulti::onPrevWaypoint, this);

	debug_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("debug_pose", 1); // DEBUG

	m_current_waypoint_id = 0;
	m_step_active = false;

	// split m_waypoints in each waypoint and store in vector 
	std::stringstream ss(m_waypoints);
	std::string tok;
	while (getline(ss, tok, '|')) {
		m_waypoint_list.push_back(tok);
	}
	
	// wait for active connections
	// TODO: replace with subscriber callback
	ros::Rate rate(10);
	while (m_cf_pose_pub.getNumSubscribers() < 1)
		rate.sleep();
}

void ArNavMulti::markerPoseCallback(const geometry_msgs::TransformStamped &bt) {
	try {
		// TODO: what happens if correct marker is not found
		// SOLUTION1: get all poses, send correct one if true, else send nearest one
		// wait for right TransformStamped
		if (!bt.child_frame_id.std::string::compare("/" + m_waypoint_list[m_current_waypoint_id])) {
			setCfPose(bt);
			debug_pose_pub.publish(m_cf_pose);
			float distance = sqrt(pow(bt.transform.translation.x, 2) + pow(bt.transform.translation.y, 2));
			if (m_step_active) {
				ROS_WARN_STREAM("Stepping to target: " << m_waypoint_list[m_current_waypoint_id]);
				// linear target change
				float step_size = 0.05;
				float step_range = 0.15;
				geometry_msgs::TransformStamped step;
				step.header = bt.header;
				step.child_frame_id = bt.child_frame_id;
				step.transform.translation.x = bt.transform.translation.x * step_size / distance;
				step.transform.translation.y = bt.transform.translation.y * step_size / distance;
				step.transform.translation.z = bt.transform.translation.z;
				step.transform.rotation = bt.transform.rotation;
				setCfPose(step);
				ROS_INFO_STREAM(step.transform.translation << "\n" << bt.transform.translation);
				if ((bt.transform.translation.x < step_range && bt.transform.translation.x > -step_range) && (bt.transform.translation.y < step_range && bt.transform.translation.y > -step_range)) {
					m_step_active = false;
				}
			} else {
				setCfPose(bt);
			}

			// broadcast pose and transformation
			m_cf_pose_pub.publish(m_cf_pose);
			sendCfPose();

			// if CF stays in range of marker, next one is targeted
			if (!m_waypoint_change.std::string::compare("auto")) {
				float timeout_range = 0.15;								// 0.05 at 0.7m height
				if ((distance < timeout_range && distance > -timeout_range) && m_next_waypoint_timeout.isValid()) {
					ros::Duration timeout(4.0);
					if (ros::Time::now() - m_next_waypoint_timeout > timeout) {
						setWaypoint(1);
					}
				}
				else
					m_next_waypoint_timeout = ros::Time::now();
			}
		}
	}
	catch (...) {
		ROS_ERROR("Failed to publish crazyflie pose");
	}
}

void ArNavMulti::sendCfPose() {
	try {
		m_transform.setOrigin(tf::Vector3(m_cf_pose.pose.position.x, m_cf_pose.pose.position.y, m_cf_pose.pose.position.z));
		m_transform.setRotation(tf::Quaternion(m_cf_pose.pose.orientation.x, m_cf_pose.pose.orientation.y, m_cf_pose.pose.orientation.z, m_cf_pose.pose.orientation.w));
		m_br.sendTransform(tf::StampedTransform(m_transform, ros::Time::now(), m_world_frame, m_cf_frame));
	}
	catch (...) {
		ROS_ERROR("Failed to update frame relation");
	}
}

void ArNavMulti::setCfPose(const geometry_msgs::TransformStamped &bt) {
	// transform TransformStamped to PoseStamped
	m_cf_pose.header.seq = bt.header.seq;
	m_cf_pose.header.stamp = bt.header.stamp;
	m_cf_pose.header.frame_id = bt.header.frame_id;
	m_cf_pose.pose.position.x = bt.transform.translation.y;
	m_cf_pose.pose.position.y = bt.transform.translation.x;
	m_cf_pose.pose.position.z = bt.transform.translation.z;
	m_cf_pose.pose.orientation.x = bt.transform.rotation.y;
	m_cf_pose.pose.orientation.y = bt.transform.rotation.x;
	m_cf_pose.pose.orientation.z = bt.transform.rotation.z;
	m_cf_pose.pose.orientation.w = bt.transform.rotation.w;

	tfScalar roll, pitch, yaw;
    	tf::Matrix3x3(tf::Quaternion(m_cf_pose.pose.orientation.x, m_cf_pose.pose.orientation.y, m_cf_pose.pose.orientation.z, m_cf_pose.pose.orientation.w)).getRPY(roll, pitch, yaw);
}

void ArNavMulti::initializeCfPose() {
	try {
		m_cf_pose.header.seq = 0;
		m_cf_pose.header.stamp = ros::Time::now();
		m_cf_pose.header.frame_id = m_cf_frame;
	      	m_cf_pose.pose.position.x = 0.0f;
	      	m_cf_pose.pose.position.y = 0.0f;
	      	m_cf_pose.pose.position.z = 0.0f;
	      	m_cf_pose.pose.orientation.x = 0.0f;
	      	m_cf_pose.pose.orientation.y = 0.0f;
	      	m_cf_pose.pose.orientation.z = 0.0f;
	      	m_cf_pose.pose.orientation.w = 1.0f;
		m_cf_pose_pub.publish(m_cf_pose);
		sendCfPose();
	}
	catch (...) {
		ROS_WARN("Could not initialize crazyflie frame");
	}
}

bool ArNavMulti::onNextWaypoint(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
	ROS_INFO_STREAM("Target next waypoint");
	setWaypoint(1);
}


bool ArNavMulti::onPrevWaypoint(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
	ROS_INFO_STREAM("Target previous waypoint");
	setWaypoint(-1);
}

void ArNavMulti::setWaypoint(int waypoint_offset) {
	m_step_active = true;
	int tmp_id = m_current_waypoint_id + waypoint_offset;
	if (tmp_id >= 0 && tmp_id < m_waypoint_list.size()) {
		m_current_waypoint_id += waypoint_offset;
	} else {
		m_current_waypoint_id = tmp_id % m_waypoint_list.size();
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "multi");
	ArNavMulti node;
	node.initializeCfPose();
	ros::spin();
	return 0;
}
