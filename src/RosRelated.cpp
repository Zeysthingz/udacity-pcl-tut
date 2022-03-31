#include "RosRelated.h"

void RosRelated::PublishCloud(const Cloud::ConstPtr &cloud_in,
                              const ros::Publisher &publisher) {
  sensor_msgs::PointCloud2 msg_cloud;
  pcl::toROSMsg(*cloud_in, msg_cloud);
  msg_cloud.header.stamp = ros::Time::now();
  msg_cloud.header.frame_id = "velodyne";
  publisher.publish(msg_cloud);
}