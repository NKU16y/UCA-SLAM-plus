// Declaration-only preview. Function implementations are not included.
#ifndef UCA_POINT_TYPES_H
#define UCA_POINT_TYPES_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <vector>

typedef pcl::PointXYZINormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef std::vector<PointT, Eigen::aligned_allocator<PointT>> PointVector;

#endif
