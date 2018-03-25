#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

class DummyOdom
{
private:
	ros::NodeHandle n;

	ros::Subscriber cmd_vel_sub;
	ros::Publisher pose_pub;
	//ros::Publisher imu_pub;

	ros::Timer timer;

	geometry_msgs::PoseStamped pose_msg;
	//sensor_msgs::Imu imu_msg;

	ros::Time last_time;

	double yaw = 0.0;

public:
	DummyOdom(void);

	void cmd_velCallback(const geometry_msgs::Twist::ConstPtr &msg);
	void timerCallback(const ros::TimerEvent &event);
};

DummyOdom::DummyOdom(void)
{
	this->cmd_vel_sub = n.subscribe("cmd_vel", 10, &DummyOdom::cmd_velCallback, this);

	//this->twist_pub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("twist", 50);
	//this->imu_pub = n.advertise<sensor_msgs::Imu>("imu", 50);
	this->pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 50);

	this->timer = n.createTimer(ros::Duration(0.1), &DummyOdom::timerCallback, this);
}

void DummyOdom::cmd_velCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
	static bool _first = true;
	if(_first)
	{
		this->last_time = ros::Time::now();
		_first = false;
	}

	ros::Time this_time = ros::Time::now();
	double t_delta = this_time.toSec() - last_time.toSec();

	this->pose_msg.pose.position.x += ((msg->linear.x * cos(yaw)) - (msg->linear.y * sin(yaw))) * t_delta;
	this->pose_msg.pose.position.y += ((msg->linear.x * sin(yaw)) + (msg->linear.y * cos(yaw))) * t_delta;

	this->yaw += msg->angular.z * t_delta;

	last_time = this_time;
}

void DummyOdom::timerCallback(const ros::TimerEvent &event)
{
	//this->twist_msg.header.stamp = ros::Time::now();
	//this->twist_msg.header.frame_id = "base_link";
	//this->twist_msg.twist.twist.linear.x = 0.25;
	//this->twist_msg.twist.twist.linear.y = 0.25;

	this->pose_msg.header.stamp = ros::Time::now();
	this->pose_msg.header.frame_id = "odom";
	//this->imu_msg.angular_velocity.z = M_PI / 2.0;

	//[0.02,0,0,0,0.02,0,0,0,0.02];
	//double var = 0.02;
	//this->imu_msg.angular_velocity_covariance[0] = var;
	//this->imu_msg.angular_velocity_covariance[4] = var;
	//this->imu_msg.angular_velocity_covariance[8] = var;

	this->pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(this->yaw);
	//this->pose_msg.pose.orientation.x = q.x;
	//this->pose_msg.pose.orientation.y = q.y;
	//this->pose_msg.pose.orientation.z = q.z;
	//this->pose_msg.pose.orientation.w = q.w;

	this->pose_pub.publish(this->pose_msg);
	//this->imu_pub.publish(this->imu_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dummy_odom");

  ROS_INFO("dummy_odom node has started.");

  DummyOdom *_odom = new DummyOdom();

  //ros::Rate r(1.0);
  ros::spin();

  ROS_INFO("dummy_odom has been terminated.");
}




