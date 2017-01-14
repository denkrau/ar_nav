#include "ar_nav/single.hpp"

ArNavSingle::ArNavSingle() {
	ros::NodeHandle _nh("~");
	std::string s;
	_nh.param<std::string>("marker_pose_topic", s, "/marker_pose");
	_nh.param<std::string>("world_frame", world_frame, "world");
	_nh.param<std::string>("cf_frame", cf_frame, "crazyflie");
	sub_marker_pose_ = nh.subscribe(s, 1, &ArNavSingle::markerPoseCallback, this);
	pub_cf_pose_ = nh.advertise<geometry_msgs::PoseStamped>("cf_pose", 1);

	ros::Rate rate(10);
	while (pub_cf_pose_.getNumSubscribers() < 1)
		rate.sleep();
}

void ArNavSingle::markerPoseCallback(const geometry_msgs::PoseStamped &msg) {
	try {
		setCfPose(msg);
		pub_cf_pose_.publish(cf_pose);
		sendCfPose();
	}
	catch (...) {
		ROS_ERROR("Failed to publish crazyflie pose");
	}
}

void ArNavSingle::sendCfPose() {
	try {
		transform.setOrigin(tf::Vector3(cf_pose.pose.position.x, cf_pose.pose.position.y, cf_pose.pose.position.z));
		transform.setRotation(tf::Quaternion(cf_pose.pose.orientation.x, cf_pose.pose.orientation.y, cf_pose.pose.orientation.z, cf_pose.pose.orientation.w));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), world_frame, cf_frame));
	}
	catch (...) {
		ROS_ERROR("Failed to update frame relation");
	}
}

void ArNavSingle::setCfPose(const geometry_msgs::PoseStamped &msg) {
	//cf_pose(msg.header, msg.pose);
	cf_pose.pose.position.x = msg.pose.position.y;
	cf_pose.pose.position.y = msg.pose.position.x;
    	cf_pose.pose.position.z = msg.pose.position.z;
	cf_pose.pose.orientation.x = msg.pose.orientation.x;
	cf_pose.pose.orientation.y = msg.pose.orientation.y;
	cf_pose.pose.orientation.z = msg.pose.orientation.z;
	cf_pose.pose.orientation.w = msg.pose.orientation.w;

	tfScalar roll, pitch, yaw;
    	tf::Matrix3x3(tf::Quaternion(cf_pose.pose.orientation.x, cf_pose.pose.orientation.y, cf_pose.pose.orientation.z, cf_pose.pose.orientation.w)).getRPY(roll, pitch, yaw);
	ROS_INFO_STREAM("setCfPose: " << cf_pose.pose.position.z /*<< "\n" << roll*180/3.141 << "\t" << pitch*180/3.141 << "\t" << yaw*180/3.141*/);

}

void ArNavSingle::initializeCfPose() {
	try {
		cf_pose.header.seq = 0;
		cf_pose.header.stamp = ros::Time::now();
		cf_pose.header.frame_id = cf_frame;
	      	cf_pose.pose.position.x = 0.0f;
	      	cf_pose.pose.position.y = 0.0f;
	      	cf_pose.pose.position.z = 0.0f;
	      	cf_pose.pose.orientation.x = 0.0f;
	      	cf_pose.pose.orientation.y = 0.0f;
	      	cf_pose.pose.orientation.z = 0.0f;
	      	cf_pose.pose.orientation.w = 1.0f;
		pub_cf_pose_.publish(cf_pose);
		sendCfPose();
	}
	catch (...) {
		ROS_WARN("Could not initialize crazyflie frame");
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "single");
	ArNavSingle node;
	node.initializeCfPose();
	ros::spin();
	return 0;
}
