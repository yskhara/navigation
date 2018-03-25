/*
 * line_mcl.cpp
 *
 *  Created on: Jan 20, 2018
 *      Author: yusaku
 */

/*
 * Monte Carlo Localization with line sensors
 */

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"

class LineMCL
{
private:
	ros::NodeHandle _nh;

	ros::Publisher map_pub;

	nav_msgs::OccupancyGrid map;

public:
	LineMCL(void);

	void OdomCallback(const nav_msgs::Odometry::ConstPtr & msg);

};

LineMCL::LineMCL(void)
{
	map.header.frame_id = "map";
	map.header.stamp = ros::Time::now();

	// meter per cell
	// 10 mm per cell
	map.info.resolution = 0.005;
	map.info.height = 14.070 / 0.005;
	map.info.width  =  8.730 / 0.005;

	//map.info.origin.position.x = 0;
	map.data
}

void LineMCL::OdomCallback(const nav_msgs::Odometry::ConstPtr & msg)
{

}



