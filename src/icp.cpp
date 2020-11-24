#include <pcl/common/transforms.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>

#include "icp.hpp"
#include "visualization.hpp"


void correspondence_estimation(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & src,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & tgt,
  const Eigen::Matrix4f & guess) 
{
  // transform src to tgt's coordinate according to guess
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_t(
    new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*src, *src_t, guess);

  // correspondence estimation using closest point data association
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>::Ptr est(
    new pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>());

  est->setInputSource(src_t);
  est->setInputTarget(tgt);

  pcl::CorrespondencesPtr corrs(new pcl::Correspondences());
  est->determineCorrespondences(*corrs);

  // correspondence rejection using median distance
  pcl::registration::CorrespondenceRejectorMedianDistance rej_med;
  rej_med.setMedianFactor(15.0);
  rej_med.setInputCorrespondences(corrs);
  rej_med.getCorrespondences(*corrs);

  // correspondence rejection using one-to-one rejection
  pcl::registration::CorrespondenceRejectorOneToOne rej_one;
  rej_one.setInputCorrespondences(corrs);
  rej_one.getCorrespondences(*corrs);

  // visualization
  visual_correspondences_only(src_t, tgt, corrs);
}


Eigen::Matrix4f point2point_icp(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & src,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & tgt,
  const Eigen::Matrix4f & guess) 
{
  // transform src to tgt's coordinate according to guess
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_t(
    new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*src, *src_t, guess);

  // point-to-point icp 
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(src_t);
  icp.setInputTarget(tgt);

  // add median distance rejection method
  pcl::registration::CorrespondenceRejectorMedianDistance::Ptr rej_med(
    new pcl::registration::CorrespondenceRejectorMedianDistance);
  rej_med->setMedianFactor(15.0);
  icp.addCorrespondenceRejector(rej_med);

  // add one-to-one rejection method
  pcl::registration::CorrespondenceRejectorOneToOne::Ptr rej_one(
    new pcl::registration::CorrespondenceRejectorOneToOne);
  icp.addCorrespondenceRejector(rej_one);

  // transformation estimation using point to point error 
  pcl::PointCloud<pcl::PointXYZ> tmp;
  Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
  icp.align(tmp); 
  trans = icp.getFinalTransformation();

  return trans * guess;
}

