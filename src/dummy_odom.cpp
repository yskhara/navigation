#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

class DummyOdom
{
private:
	ros::NodeHandle n;

	ros::Subscriber cmd_vel_sub;
	ros::Publisher pose_pub;

	int control_freq;
	double control_interval;

	ros::Timer timer;

	geometry_msgs::PoseStamped pose_msg;

    std::string odom_frame;

	double yaw = 0.0;

	double vx = 0.0;
	double vy = 0.0;
	double vz = 0.0;

public:
	DummyOdom(void);

	void cmd_velCallback(const geometry_msgs::Twist::ConstPtr &msg);
	void timerCallback(const ros::TimerEvent &event);
};

DummyOdom::DummyOdom(void)
{
    auto _nh = ros::NodeHandle("~");
    _nh.param<std::string>("odom_frame", odom_frame, "odom");
    _nh.param<int>("freq", control_freq, 200);

    control_interval = 1.0 / control_freq;

	this->cmd_vel_sub = n.subscribe("cmd_vel", 10, &DummyOdom::cmd_velCallback, this);
	this->pose_pub = n.advertise<geometry_msgs::PoseStamped>("odom_pose", 1);
	this->timer = n.createTimer(ros::Duration(0.1), &DummyOdom::timerCallback, this);

    this->pose_msg.pose.position.x = 0;
    this->pose_msg.pose.position.y = 0;
    this->pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
}

void DummyOdom::cmd_velCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    this->vx = msg->linear.x;
    this->vy = msg->linear.y;
    this->vz = msg->angular.z;
}

void DummyOdom::timerCallback(const ros::TimerEvent &event)
{
    this->pose_msg.header.stamp = ros::Time::now();
    this->pose_msg.header.frame_id = odom_frame;

    double t_delta = (event.current_real - event.last_real).toSec();
    this->pose_msg.pose.position.x += ((this->vx * cos(yaw)) - (this->vy * sin(yaw))) * t_delta;
    this->pose_msg.pose.position.y += ((this->vx * sin(yaw)) + (this->vy * cos(yaw))) * t_delta;

    this->yaw += this->vz * t_delta;

	this->pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(this->yaw);

	this->pose_pub.publish(this->pose_msg);
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




