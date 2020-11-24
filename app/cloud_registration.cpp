#include <chrono>
#include <iostream>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>

#include "icp.hpp"
#include "visualization.hpp"


int main(int argc, char * argv[]) {
  if (argc < 3) {
    std::cout << "Usage: " << argv[0] << " <source.pcd> <target.pcd> \n";
    return EXIT_FAILURE;
  }

  // read point cloud data in
  pcl::PointCloud<pcl::PointXYZ>::Ptr src(
    new pcl::PointCloud<pcl::PointXYZ>());
  pcl::io::loadPCDFile(argv[1], *src);

  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(
    new pcl::PointCloud<pcl::PointXYZ>());
  pcl::io::loadPCDFile(argv[2], *tgt);

  // downsample the point cloud to accelerate speed
  pcl::VoxelGrid<pcl::PointXYZ> vox;
  vox.setLeafSize(0.15f, 0.15f, 0.15f);

  vox.setInputCloud(src);
  vox.filter(*src);

  vox.setInputCloud(tgt);
  vox.filter(*tgt);

  // visualize the initial correspondences
  correspondence_estimation(src, tgt);

  // iterative closest point 
  auto t1 = std::chrono::system_clock::now();

  Eigen::Matrix4f trans = point2point_icp(src, tgt);

  auto t2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = t2 - t1;
  std::cout << "icp took " << diff.count() << " seconds.\n";

  // the final result
  std::cout << "the final transformation is:\n";
  std::cout << trans << '\n';

  // visualize the final correspondences
  correspondence_estimation(src, tgt, trans);

}
