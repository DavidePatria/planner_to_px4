#include <ros/ros.h>

#include <queue>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <geometry_msgs/PoseStamped.h>



class SlowDown {

public: 
	SlowDown(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

private:
	// for now copy the trajectory points using the callback
	trajectory_msgs::MultiDOFJointTrajectory traj_;

	float trigger_distance_;
	// actual as in "ahora"
	// geometry_msgs::Pose actual_goal_;
	trajectory_msgs::MultiDOFJointTrajectoryPoint actual_goal_;
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

	void convert_mjt_pose_(geometry_msgs::Pose& pose, const trajectory_msgs::MultiDOFJointTrajectoryPoint& traj_point);
};
