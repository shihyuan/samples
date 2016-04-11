/*
Author: Shih-Yuan Liu
*/
#ifndef __PCL_CLUSTERING_UTILPCL_IMPL_HPP


#include <ros/ros.h>
// For using pcl
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/project_inliers.h> //For projection
#include <pcl/features/normal_3d.h> // For estimating normals
#include <pcl/segmentation/region_growing.h> //Region growing
#include <pcl/filters/passthrough.h> //For passthrough filter
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/concave_hull.h>

namespace utilpcl
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, double leafSize)
  {
    // Downsample the cloud using VoxelGrid
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (oriCloud->makeShared ());
    vg.setLeafSize (leafSize,leafSize,leafSize);
    vg.filter (*cloud_filtered);
    return cloud_filtered;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr getConcaveHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr proj_cloud, double alpha)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud(proj_cloud);
    chull.setAlpha(alpha);
    chull.reconstruct(*cloud_hull);
    return cloud_hull;
  }


  pcl::PointIndices::Ptr getPlanarIndiciesRansac(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, int max_iteration, double distance_threshold,
    pcl::ModelCoefficients::Ptr model_coefficients)
  {
    // Create the segmentation object for the planar model and set all the parameters

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_seg = oriCloud->makeShared(); //Make a copy of the input cloud
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // model_coefficients = new pcl::ModelCoefficients;

    // Sepcify segmentation parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (max_iteration);
    seg.setDistanceThreshold (distance_threshold);
    
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (oriCloud);

    seg.segment (*inliers, *model_coefficients);

    return inliers;
  }  

  pcl::PointIndices::Ptr getWallIndiciesRansac(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, int max_iteration, double distance_threshold,
    pcl::ModelCoefficients::Ptr model_coefficients)
  {
    // Create the segmentation object for the planar model and set all the parameters

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_seg = oriCloud->makeShared(); //Make a copy of the input cloud
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // model_coefficients = new pcl::ModelCoefficients;

    // Sepcify segmentation parameters
    // seg.setOptimizeCoefficients (true);
    Eigen::Vector3f axis;
    axis[0] = model_coefficients->values[0];
    axis[1] = model_coefficients->values[1];
    axis[2] = model_coefficients->values[2];

    seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
    seg.setAxis(axis);
    seg.setEpsAngle(0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (max_iteration);
    seg.setDistanceThreshold (distance_threshold);
    
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (oriCloud);

    seg.segment (*inliers, *model_coefficients);

    return inliers;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr extractPointCloudByIndices(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, pcl::PointIndices::Ptr indices, bool negative)
  {
    // Extract a point cloud from the original with the givien indices
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (oriCloud);
    extract.setIndices (indices);
    extract.setNegative (negative);
    extract.filter (*cloud_filtered);
    return cloud_filtered;
  }

  std::vector<pcl::PointIndices> getEuclideanClusterIndices(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, double cluster_tolerance, int min_cluster_size, int max_cluster_size)
  {
    // Return the indices of the Euclidean Clusters

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (oriCloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ec.setClusterTolerance (cluster_tolerance); // in meter
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (oriCloud);
    ec.extract (cluster_indices);

    return cluster_indices; 
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr ProjectPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, pcl::ModelCoefficients::Ptr model_coefficients)
  {
    // Project points onto a plane sepcified by coefficients
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (oriCloud);
    proj.setModelCoefficients (model_coefficients);
    proj.filter (*cloud_projected);
    return cloud_projected;
  }


  pcl::PointCloud<pcl::Normal>::Ptr getNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, double search_radious, pcl::search::Search<pcl::PointXYZ>::Ptr tree_out)
  {
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (oriCloud);
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    tree_out =  boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);

    ne.setSearchMethod (tree_out);
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (search_radious);
    // Compute the features
    ne.compute (*cloud_normals);
    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*

    return cloud_normals;
  }

  std::vector<pcl::PointIndices> getRegionGrowingClusterIndicies(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud,
    int min_cluster_size, int max_cluster_size, int num_of_neighbors, double search_radious, 
    double smooth_threshold, double curvature_threshold)
  {

    // Build kdtree and normals
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
    pcl::search::Search<pcl::PointXYZ>::Ptr tree;
    pcl::PointCloud<pcl::Normal>::Ptr normals = getNormals(oriCloud, search_radious, tree);

    // Apply region growing
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (min_cluster_size);
    reg.setMaxClusterSize (max_cluster_size);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (num_of_neighbors);
    reg.setInputCloud (oriCloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (smooth_threshold);
    reg.setCurvatureThreshold (curvature_threshold);
    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    
    return clusters;
  }

  pcl::PointIndices::Ptr getPassthroughIndicies(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, std::string filter_field, double value_min, double value_max)
  {
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(oriCloud);
    filter.setFilterFieldName(filter_field);
    filter.setFilterLimits(value_min, value_max);

    std::vector<int> indices;
    filter.filter(indices);

    pcl::PointIndices::Ptr pointIndices (new pcl::PointIndices());
    pointIndices->indices = indices;

    return pointIndices;
  }

  pcl::PointIndices::Ptr getPlaneInlierIndicies(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, pcl::ModelCoefficients::Ptr model_coefficients, double distance_threshold)
  {
    // Return the indicies of points in the oriCloud that is within distance threshould to the model plane specified by coefficients
    pcl::SampleConsensusModelPlane<pcl::PointXYZ> scp(oriCloud);
    Eigen::VectorXf coeff(4);
    coeff[0] = model_coefficients->values[0];
    coeff[1] = model_coefficients->values[1];
    coeff[2] = model_coefficients->values[2];
    coeff[3] = model_coefficients->values[3];
    std::vector<int> indices;
    scp.selectWithinDistance(coeff, distance_threshold, indices);
    pcl::PointIndices::Ptr pointIndices(new pcl::PointIndices());
    pointIndices->indices = indices;
    return pointIndices;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr removeRadiusOutlier(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, double radius, int min_neighbors)
  {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> rorfilter(true);
    rorfilter.setInputCloud(oriCloud);
    rorfilter.setRadiusSearch(radius);
    rorfilter.setMinNeighborsInRadius(min_neighbors);
    rorfilter.setNegative(false);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    rorfilter.filter(*cloud_out);
    return cloud_out;
  }
}


#define __PCL_CLUSTERING_UTILPCL_IMPL_HPP
#endif /* __PCL_CLUSTERING_UTILPCL_IMPL_HPP */