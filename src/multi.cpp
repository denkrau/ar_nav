#include "ar_nav/multi.hpp"

ArNavMulti::ArNavMulti() {
	// initialize topics
	ros::NodeHandle n("~");
	std::string s;
	n.param<std::string>("marker_pose_topic", s, "/marker_pose");
	n.param<std::string>("world_frame", m_world_frame, "world");
	n.param<std::string>("cf_frame", m_cf_frame, "crazyflie");
	n.param<std::string>("waypoints", m_waypoints, "board_c3po");
	m_marker_pose_sub = nh.subscribe(s, 1, &ArNavMulti::markerPoseCallback, this);
	m_cf_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("cf_pose", 1);
	m_next_waypoint_srv = nh.advertiseService("next_waypoint", &ArNavMulti::onNextWaypoint, this);
	m_prev_waypoint_srv = nh.advertiseService("prev_waypoint", &ArNavMulti::onPrevWaypoint, this);

	m_current_waypoint_id = 0;

	// split m_waypoints in each waypoint and store in vector 
	std::stringstream ss(m_waypoints);
	std::string tok;
	while (getline(ss, tok, '|')) {
		m_waypoint_list.push_back(tok);
	}
	
	// wait for active connections
	ros::Rate rate(10);
	while (m_cf_pose_pub.getNumSubscribers() < 1)
		rate.sleep();
}

void ArNavMulti::markerPoseCallback(const geometry_msgs::TransformStamped &bt) {
	try {
		// wait for right TransformStamped
		if (bt.child_frame_id.std::string::compare("/" + m_waypoint_list[m_current_waypoint_id])) {
			
			// create pose of crazyflie and broadcast it
			setCfPose(bt);
			m_cf_pose_pub.publish(m_cf_pose);
			sendCfPose();

			// if CF stays in range of marker, next one is targeted
			float distance = sqrt(pow(bt.transform.translation.x, 2) + pow(bt.transform.translation.y, 2));
			if (distance < 0.05 && distance > -0.05) {
				ros::Duration timeout(3.0);
				ros::Time::now() - m_next_waypoint_timeout > timeout;
				if (m_current_waypoint_id == m_waypoint_list.size() - 1)
					m_current_waypoint_id = 0;
				else
					m_current_waypoint_id++;
			}
			else
				m_next_waypoint_timeout = ros::Time::now();
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
	m_cf_pose.pose.position.x = bt.transform.translation.x;
	m_cf_pose.pose.position.y = bt.transform.translation.y;
	m_cf_pose.pose.position.z = bt.transform.translation.z;
	m_cf_pose.pose.orientation.x = bt.transform.rotation.x;
	m_cf_pose.pose.orientation.y = bt.transform.rotation.y;
	m_cf_pose.pose.orientation.z = bt.transform.rotation.z;
	m_cf_pose.pose.orientation.w = bt.transform.rotation.w;

	tfScalar roll, pitch, yaw;
    	tf::Matrix3x3(tf::Quaternion(m_cf_pose.pose.orientation.x, m_cf_pose.pose.orientation.y, m_cf_pose.pose.orientation.z, m_cf_pose.pose.orientation.w)).getRPY(roll, pitch, yaw);
	ROS_INFO_STREAM("setCfPose: " << m_cf_pose.pose.position.z /*<< "\n" << roll*180/3.141 << "\t" << pitch*180/3.141 << "\t" << yaw*180/3.141*/);

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
	setWaypoint(1);
}


bool ArNavMulti::onPrevWaypoint(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
	setWaypoint(-1);
}

void ArNavMulti::setWaypoint(int waypoint_offset) {
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
