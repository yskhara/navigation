#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>

class OdomertyBroadcaster
{
private:
	ros::NodeHandle n;

	// stuff to publish
	//ros::Publisher odom_pub;
	tf::TransformBroadcaster odom_broadcaster;

	// stuff to subscribe to
	ros::Subscriber odom_pose_sub;
	//ros::Subscriber base_odom_x_sub;
	//ros::Subscriber base_odom_y_sub;
	//ros::Subscriber base_odom_yaw_sub;

	ros::Timer timer;

	// local storage

	// last pose estimate that reported by odometry unit
	geometry_msgs::PoseStamped last_pose;

	//ros::Time last_pose_time;
	bool last_pose_updated = false;

	//ros::Time last_x_time;
	//ros::Time last_y_time;
	//ros::Time last_yaw_time;

	//double _x_coeff;
	//double _y_coeff;
	//double _yaw_coeff;
	//bool _swap_xy;

	//nav_msgs::Odometry odom_msg;

	double timer_freq = 10;
	std::string odom_frame;
	std::string base_frame;

public:
	OdomertyBroadcaster(void);

	//void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose);
	void odomPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
	void timerCallback(const ros::TimerEvent &event);
};

OdomertyBroadcaster::OdomertyBroadcaster(void)
{
	//this->odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);

    this->odom_pose_sub = n.subscribe<geometry_msgs::PoseStamped>("odom_pose", 10, &OdomertyBroadcaster::odomPoseCallback, this);

	auto _nh = ros::NodeHandle("~");
	_nh.param("freq", this->timer_freq, 10.0);
	_nh.param<std::string>("odom_frame", this->odom_frame, "odom");
	_nh.param<std::string>("base_frame", this->base_frame, "base_link");

    // 10 Hz
    this->timer = n.createTimer(ros::Rate(timer_freq), &OdomertyBroadcaster::timerCallback, this);
}


void OdomertyBroadcaster::odomPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    this->last_pose = geometry_msgs::PoseStamped(*msg);
    last_pose_updated = true;
}

void OdomertyBroadcaster::timerCallback(const ros::TimerEvent &event)
{
#if 0
	double dt = event.current_real.toSec() - last_pose_time.toSec();

	if(dt > PoseTimeoutThreshold)
	{
		// too late...
		last_twist.linear.x  = 0.0;
		last_twist.linear.y  = 0.0;
		last_twist.angular.z = 0.0;
	}
	else if(dt > TimerInterval)
	{
		// something is a bit off, but still it's expected; estimate
		current_pose.x     += last_twist.linear.x  * dt;
		current_pose.y     += last_twist.linear.y  * dt;
		current_pose.theta += last_twist.angular.z * dt;
	}
#endif

	if(!last_pose_updated)
	{
	    return;
	}

	//since all odometry is 6DOF we'll need a quaternion created from yaw
	//geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(current_pose.theta);

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = event.current_real;
	odom_trans.header.frame_id = odom_frame;
	odom_trans.child_frame_id = base_frame;

	odom_trans.transform.translation.x = last_pose.pose.position.x;
	odom_trans.transform.translation.y = last_pose.pose.position.y;
	odom_trans.transform.rotation = last_pose.pose.orientation;

	//send the transform
	odom_broadcaster.sendTransform(odom_trans);

#ifdef PUBLISH_ODOM_MSG

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = event.current_real;
	odom.header.frame_id = "odom";

	//set the position
	odom.pose.pose.position.x = current_pose.x;
	odom.pose.pose.position.y = current_pose.y;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = last_twist.linear.x;
	odom.twist.twist.linear.y = last_twist.linear.y;
	odom.twist.twist.angular.z = last_twist.angular.z;

	//publish the message
	odom_pub.publish(odom);
#endif
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_broadcaster");

  ROS_INFO("odometry_broadcaster node has started.");

  OdomertyBroadcaster *_odom = new OdomertyBroadcaster();

  //ros::Rate r(1.0);
  ros::spin();

  ROS_INFO("odometry_broadcaster has been terminated.");
}




