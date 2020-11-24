#include <fstream>
#include <random>
#include <cstdlib>

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>

#include "visualization.hpp"


std::string get_random_str() 
{
  std::string tmp;

  std::mt19937 rng;
  rng.seed(std::random_device()());
  std::uniform_int_distribution<std::mt19937::result_type> dist(1, 1000000);
  tmp = "temp_" + std::to_string(dist(rng));

  return tmp;
}


using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;

void visual_correspondences_only(
  const Cloud::ConstPtr & src,
  const Cloud::ConstPtr & tgt,
  const pcl::CorrespondencesPtr & correspondences,
  const Eigen::Matrix4f & extr)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_(
    new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::copyPointCloud(*src, *src_);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_(
    new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::copyPointCloud(*tgt, *tgt_);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(
    new pcl::PointCloud<pcl::PointXYZRGB>);

  bool need_transform = false;
  Eigen::Transform<float, 3, Eigen::Affine> trans(extr);
  if (extr != Eigen::Matrix4f::Identity()) {
    need_transform = true;
  }
  
  for (const auto & corr : (*correspondences)) {
    auto & s = src_->at(corr.index_query);
    s.r = 255;
    s.g = 180;
    s.b = 0;
    if (need_transform) {
      auto s_t = pcl::transformPoint(s, trans);
      output->push_back(s_t);
    } else {
      output->push_back(s);
    }

    auto & t = tgt_->at(corr.index_match);
    t.r = 0;
    t.g = 166;
    t.b = 237;
    output->push_back(t);
  }

  std::string temp_file_name = get_random_str() + "_visual_correspondences_only.ply";
  pcl::io::savePLYFileBinary(temp_file_name, *output);

  std::system(("/usr/bin/meshlab " + temp_file_name + " >/dev/null 2>&1").c_str());
  std::system(("rm " + temp_file_name).c_str());
}

