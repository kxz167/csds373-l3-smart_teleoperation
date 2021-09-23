#include "ros/ros.h"
#include "std_msgs/String.h"

#include <cmath>
#include <vector>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

ros::Publisher commandPub;

bool shouldStop = false;

void robotCommandCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  /**
   * Run each time the robot is issued a command
   */
	geometry_msgs::Twist newMsg;
	newMsg.linear.x = msg->linear.x;
	newMsg.linear.y = msg->linear.y;
	newMsg.linear.z = msg->linear.z;
	newMsg.angular.x = msg->angular.x;
	newMsg.angular.y = msg->angular.y;
	newMsg.angular.z = msg->angular.z;
	if (shouldStop) {
		newMsg.linear.x = 0;	
	}
	commandPub.publish(newMsg);	
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  /**
   * Run each time the lidar takes in a reading.
   */
	int len = msg->ranges.size();	
	std::vector<float> currRanges = msg->ranges;
	int mid_index = std::round(len / 2);
	int collision_range = 20;
	float wall_threshold = 1;
	for (int i = mid_index - collision_range; i < mid_index + collision_range; i++) {
		if (currRanges.at(i)  <= wall_threshold) {
			shouldStop = true;		
		}
		else {
			shouldStop = false;		
		}
		printf("%f", currRanges.at(i));
	}
	
}

int main(int argc, char **argv)
{
  /**
   * Initialize ros, name the node
   */
  ros::init(argc, argv, "smart_teleoperator");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * Create a publisher
   */
  commandPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  
  /**
   * Create the subscribers for  the lidar
   */
  ros::Subscriber velocitySub = n.subscribe("des_vel", 1000, robotCommandCallback);
  ros::Subscriber laserSub = n.subscribe("laser_1", 1000, lidarCallback);

  /**
   *How often to  run through the loop
   */
  ros::Rate loop_rate(1);

  /**
   * Loop through sending messages
   */
  while (ros::ok())
  {
    /**
     * Create the geometry_msgs.
     */
    /*
    geometry_msgs::Twist msg;

    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    
    /**
     * Publish to the publisher
     */
    //commandPub.publish(msg);
    
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
