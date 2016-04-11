/*
Author: Shih-Yuan Liu
*/
#ifndef __PCL_CLUSTERING_UTILPCL_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/search/search.h>
#include <vector>

namespace utilpcl
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, double leafSize);
  pcl::PointIndices::Ptr getPlanarIndiciesRansac(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, int max_iteration, double distance_threshold, pcl::ModelCoefficients::Ptr model_coefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr extractPointCloudByIndices(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, pcl::PointIndices::Ptr indices, bool negative = false);
  std::vector<pcl::PointIndices> getEuclideanClusterIndices(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, double cluster_tolerance, int min_cluster_size, int max_cluster_size);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ProjectPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, pcl::ModelCoefficients::Ptr model_coefficients);
  pcl::PointCloud<pcl::Normal>::Ptr getNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, double search_radious, pcl::search::Search<pcl::PointXYZ>::Ptr tree_out);
  std::vector<pcl::PointIndices> getRegionGrowingClusterIndicies(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, int min_cluster_size, int max_cluster_size, int num_of_neighbors, double search_radious, double smooth_threshold, double curvature_threshold);
  pcl::PointIndices::Ptr getPassthroughIndicies(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, std::string filter_field, double value_min, double value_max);
  pcl::PointIndices::Ptr getPlaneInlierIndicies(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, pcl::ModelCoefficients::Ptr model_coefficients, double distance_threshold);
  pcl::PointCloud<pcl::PointXYZ>::Ptr removeRadiusOutlier(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, double radius, int min_neighbors);
  pcl::PointCloud<pcl::PointXYZ>::Ptr getConcaveHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr proj_cloud, double alpha);
  pcl::PointIndices::Ptr getWallIndiciesRansac(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, int max_iteration, double distance_threshold, pcl::ModelCoefficients::Ptr model_coefficients);
}

#include "utilpcl_impl.hpp"

#define __PCL_CLUSTERING_UTILPCL_HPP
#endif /* __PCL_CLUSTERING_UTILPCL_HPP */