#include <ros/ros.h>

#include "passport/passport.h"

int main(int argc, char *argv[]) {
  // Initialize ROS node
  ros::init(argc, argv, "passport");

  ROS_INFO("Hello World");

  return EXIT_SUCCESS;
}
