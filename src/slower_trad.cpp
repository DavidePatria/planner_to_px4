#include "planner_to_px4/slower_trad.h"

SlowDown::SlowDown(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
	:nh_{nh},
	nh_private_{nh_private},
	// this value below could be taken from a dynamic reconfigure server like
	// in px4 avoidance
	trigger_distance_{0.1},
	// initialize the mask of the message
	velocity_mask{VELOCITY2D_CONTROL}
	{
	pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 1, &SlowDown::poseCallback_, this, ros::TransportHints().tcpNoDelay());
	goal_sub_ = nh_.subscribe("/command/trajectory", 1, &SlowDown::trajCallback_, this, ros::TransportHints().tcpNoDelay());

	traj_slow_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);

	cmdloop_timer_ = nh_.createTimer(ros::Duration(0.10), &SlowDown::cmdloopCallback, this);
	// the message is always the same one
	actual_goal_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	actual_goal_.type_mask = POSITION_CONTROL;
	actual_goal_.header.frame_id = "map";
};

void SlowDown::poseCallback_(const geometry_msgs::PoseStamped& msg) {
	// update internal pose
	// position w/out orientation is enough?
	ROS_INFO_THROTTLE(5, "got pose: x=%f, y=%f. z=%f", msg.pose.position.x,
													   msg.pose.position.y,
													   msg.pose.position.z);
	pose_.position = msg.pose.position;
}

void SlowDown::trajCallback_(const trajectory_msgs::MultiDOFJointTrajectory& msg) {
	const std::lock_guard<std::mutex> lock(mutex_);

	ROS_INFO("got trajectory message");
	// copy the trajectory
	std::queue<trajectory_msgs::MultiDOFJointTrajectoryPoint> empty_q;
	std::swap(traj_q_, empty_q);

	for(auto &element:msg.points) {
		traj_q_.push(element);
	}

	// send first point
	convert_mdjt_pt(traj_q_.front(), actual_goal_);
	// actual_goal_ = traj_q_.front();

	// publicar el goal que se acaba de transladar
	traj_slow_pub_.publish(actual_goal_);
	// remove said first point
	traj_q_.pop();
}

void SlowDown::convert_mdjt_pt(const trajectory_msgs::MultiDOFJointTrajectoryPoint &point, mavros_msgs::PositionTarget &target) {
	target.position.x = point.transforms[0].translation.x;
	target.position.y = point.transforms[0].translation.y;
	target.position.z = point.transforms[0].translation.z;
	// target.yaw = point.transforms[0].rotation.
	quat_to_yaw(point.transforms[0].rotation, target.yaw);
}

// bool SlowDown::closeEnough() {
// 	float x, y, z;
// 	ROS_INFO("close enough?");
// 	// ROS_INFO("esempio di valore: %f", actual_goal_.transforms[0].translation.x);
// 	x = pose_.position.x - actual_goal_.position.x;
// 	y = pose_.position.y - actual_goal_.position.y;
// 	z = pose_.position.z - actual_goal_.position.z;
// 	// should a check on yaw be added?
//
// 	// if the drone is close enough to the actual goal by L2 norm
// 	if(std::sqrt(x*x + y*y + z*z) < trigger_distance_) {
// 		return true;
// 	} else {
// 		return false;
// 	}
// }

void SlowDown::cmdloopCallback(const ros::TimerEvent &event) {
	const std::lock_guard<std::mutex> lock(mutex_);

	// using this topic the points can be sent all at once
	if(!traj_q_.empty()) {
		ROS_INFO("pose converted");
		traj_slow_pub_.publish(actual_goal_);
		// 10hz required for offboard to be kept, slow done message publishing
		ros::Rate rate{3};
		rate.sleep();

		ROS_INFO("published traje");
		traj_q_.pop();
		ROS_INFO("popped");
	}
}

void SlowDown::quat_to_yaw(const geometry_msgs::Quaternion q, float &yaw) {
	double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
	yaw = std::atan2(siny_cosp, cosy_cosp);
}
