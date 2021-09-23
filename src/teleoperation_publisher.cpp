#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

void robotCommandCallback(const geometry_msgs::Twist::ConstPtr& msg)
{

}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
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
  ros::Publisher commandPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  
  /**
   *
   */
  ros::Subscriber velocitySub = n.subscribe("des_vel", 1000, robotCommandCallback);
  ros::Subscriber laserSub = n.subscribe("cmd_vel", 1000, lidarCallback);
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
    commandPub.publish(msg);

    
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
