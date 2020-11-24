#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include <string>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>


/*
 * get a random string, used for temporary file names
 */
std::string get_random_str();


void visual_correspondences_only(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & src,
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & tgt,
  const pcl::CorrespondencesPtr & correspondences,
  const Eigen::Matrix4f & extr=Eigen::Matrix4f::Identity());

#endif
