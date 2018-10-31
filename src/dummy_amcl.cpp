/*
 * dummy_amcl.cpp
 *
 *  Created on: Mar 27, 2018
 *      Author: yusaku
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class DummyAmcl
{
private:
	ros::NodeHandle _nh;

	ros::Subscriber _initialpose_sub;

	ros::Timer _timer;

	geometry_msgs::PoseWithCovarianceStamped _initialpose_msg;

	tf::TransformListener _tflistener;
	tf::TransformBroadcaster _tfbroadcaster;
	tf::Transform _odom_tf;

public:
	DummyAmcl(void);

	void initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
	void timerCallback(const ros::TimerEvent &event);
};

DummyAmcl::DummyAmcl(void)
{
	this->_initialpose_sub = this->_nh.subscribe("initialpose", 10, &DummyAmcl::initialposeCallback, this);

	this->_timer = this->_nh.createTimer(ros::Duration(0.1), &DummyAmcl::timerCallback, this);
}

void DummyAmcl::initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
	tf::StampedTransform base_link_tf;
	try
	{
		this->_tflistener.waitForTransform("/base_link", "/odom", ros::Time(0), ros::Duration(0.1));
		this->_tflistener.lookupTransform("/base_link", "/odom", ros::Time(0), base_link_tf);
	}
	catch(...)
	{
		return;
	}

	ROS_INFO("received initial pose.")

	tf::Transform pose_tf;
	pose_tf.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
	tf::Quaternion pose_q;
	tf::quaternionMsgToTF(msg->pose.pose.orientation, pose_q);
	pose_tf.setRotation(pose_q);

	_odom_tf.mult(pose_tf, base_link_tf);

	//_odom_tf = base_link_tf.inverseTimes(pose_tf);
	//double diff_x = msg->pose.pose.position.x - base_link_tf.getOrigin().x();
	//double diff_y = msg->pose.pose.position.y - base_link_tf.getOrigin().y();
	//tf::Quaternion q_initialpose, diff_q;
	//tf::quaternionMsgToTF(msg->pose.pose.orientation, q_initialpose);
	//double diff_yaw = tf::getYaw(q_initialpose) - tf::getYaw(base_link_tf.getRotation());

	//diff_q.setRPY(0, 0, diff_yaw);

	//_odom_tf.header.frame_id = "map";
	//_odom_tf.child_frame_id = "odom";
	//_odom_tf.
	//_odom_tf.setOrigin( tf::Vector3(diff_x, diff_y, 0.0) );
	//_odom_tf.setRotation(diff_q);
	//_odom_tf = pose_tf;
}

void DummyAmcl::timerCallback(const ros::TimerEvent &event)
{
	this->_tfbroadcaster.sendTransform(tf::StampedTransform(this->_odom_tf, ros::Time::now(), "map", "odom"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dummy_odom");

  ROS_INFO("dummy_amcl node has started.");

  DummyAmcl *_amcl = new DummyAmcl();

  //ros::Rate r(1.0);
  ros::spin();

  ROS_INFO("dummy_amcl has been terminated.");
}







