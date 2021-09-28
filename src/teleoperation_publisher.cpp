#include "ros/ros.h"
#include "std_msgs/String.h"

#include <cmath>
#include <vector>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

// Global Variables:
ros::Publisher commandPub;
bool shouldStop = false;

//Constants:
const float STOP_DIST = 1;
const float VIEW_ANGLE = 20;
const float VIEW_PAD = (270 - VIEW_ANGLE) / 2;

/**
  * Run each time the robot is issued a command
  */
void robotCommandCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  //Create a copy of the message:
  geometry_msgs::Twist msg_copy(*msg);

  // Determine if we want to override the forward velocity.
  if (shouldStop)
  {
    msg_copy.linear.x = 0;
  }

  //Publish the message.
  commandPub.publish(msg_copy);
}


/**
   * Run each time the lidar takes in a reading.
   */
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  //Grab the set of ranges from the message.
  std::vector<float> current_ranges = msg->ranges; //Grab the ranges

  //Determine if minimum element in desired view range is less than the stopping distance.
  shouldStop = *std::min_element(current_ranges.begin() + VIEW_PAD, current_ranges.end() - VIEW_PAD) < STOP_DIST; 
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
  
  //Allow ros to read from subscribers.
  ros::spin();

  return 0;
}
