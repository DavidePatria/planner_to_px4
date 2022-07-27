#include <ros/ros.h>

#include <queue>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <mavros_msgs/PositionTarget.h>

// needed for the PositionTarget message
#define VELOCITY2D_CONTROL 0b011111000011
#define VELOCITY_CONTROL 0b011111000111
#define POSITION_CONTROL 0b101111111000

class SlowDown {

public: 
	SlowDown(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

private:
	unsigned short velocity_mask;
	// for now copy the trajectory points using the callback
	trajectory_msgs::MultiDOFJointTrajectory traj_;

	float trigger_distance_;
	// actual as in "ahora"
	mavros_msgs::PositionTarget actual_goal_;
	// for internal use and tracking
	geometry_msgs::Pose pose_;

	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

	ros::Subscriber pose_sub_;
	ros::Subscriber goal_sub_;

	ros::Publisher traj_slow_pub_;

	ros::Timer cmdloop_timer_;

	void poseCallback_(const geometry_msgs::PoseStamped& msg);
	void trajCallback_(const trajectory_msgs::MultiDOFJointTrajectory& msg);

	void cmdloopCallback(const ros::TimerEvent& event);

	bool closeEnough();

	// to lock when maipulating trajectory vector
	std::mutex mutex_;

	std::queue<trajectory_msgs::MultiDOFJointTrajectoryPoint> traj_q_;

	void convert_mdjt_pt(const trajectory_msgs::MultiDOFJointTrajectoryPoint &point, mavros_msgs::PositionTarget &target);

	void quat_to_yaw(const geometry_msgs::Quaternion q, float &yaw);
};
