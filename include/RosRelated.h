#ifndef PCLDATACAMP_ROSRELATED_H
#define PCLDATACAMP_ROSRELATED_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class RosRelated {
public:
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;


  static void PublishCloud(const Cloud::ConstPtr &cloud_in,
                           const ros::Publisher &publisher);
};

#endif //PCLDATACAMP_ROSRELATED_H
