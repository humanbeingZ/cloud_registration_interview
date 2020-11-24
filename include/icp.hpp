#ifndef ICP_HPP
#define ICP_HPP

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>


void correspondence_estimation(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & src,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & tgt,
  const Eigen::Matrix4f & guess=Eigen::Matrix4f::Identity());


Eigen::Matrix4f point2point_icp(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & src,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & tgt,
  const Eigen::Matrix4f & guess=Eigen::Matrix4f::Identity());

#endif
