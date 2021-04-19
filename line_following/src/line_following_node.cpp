#include <ros/ros.h>
#include "line_following.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "line_following");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  lanekeeping::LaneDetection node(n, pn);
  
  ros::spin();
}
