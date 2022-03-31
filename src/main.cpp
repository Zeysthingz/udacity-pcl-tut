#include <ros/ros.h>
#include <ros/spinner.h>
#include "UdacityTut.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "udacity_pcl_tut_node");
  ros::NodeHandle nh;
  UdacityTut udacity_tut(nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
