#ifndef PCL_UDACITY_TUT_H
#define PCL_UDACITY_TUT_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "PclStuff.h"
#include "RosRelated.h"

class UdacityTut {
public:
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;
  explicit UdacityTut(ros::NodeHandle &nh);

private:
  ros::Subscriber sub_velodyne_points_;

  ros::Publisher pub_cloud_raw_;
  ros::Publisher pub_cloud_downsampled_;
  ros::Publisher pub_cloud_groundless_;
//  ground point cloud publisher
  ros::Publisher pub_cloud_ground_;
  ros::Publisher pub_eucledian_clusterer_;


  ros::NodeHandle& nh_;

  void CallbackLaser(const sensor_msgs::PointCloud2ConstPtr &msg_cloud);
};


#endif //PCL_UDACITY_TUT_H
