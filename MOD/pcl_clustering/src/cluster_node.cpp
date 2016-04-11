/*
Author: Shih-Yuan Liu
*/
#include <algorithm>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <mod_msgs/Clusters.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

// For ROS to interact with PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

// For using utilpcl
#include <utilpcl.hpp>
// For using dynmeans
#include <dynmeans.hpp>

class ClusterNode{
public:
  ros::NodeHandle nh_;
  ros::Publisher pub_marker_;
  ros::Publisher pub_clusters_;
  ros::Subscriber sub_;
  ros::ServiceServer srv_;

  DynMeans<Eigen::Vector2f>* dynm_;

  struct rgb{
    double r;
    double g;
    double b;
  };

  std::vector<rgb> colormap_;
  
  int max_marker_id_;

  ClusterNode(int argc, char** argv)
  {
    // Setup node handler
    nh_ = ros::NodeHandle("~");
    pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray> ("output/marker_array",1);
    pub_clusters_ = nh_.advertise<mod_msgs::Clusters> ("output/clusters",1);

    sub_ = nh_.subscribe("input/pointcloud", 1, &ClusterNode::cbCloud, this);
    srv_ = nh_.advertiseService("reset_cluster", &ClusterNode::srvResetCluster ,this);

    max_marker_id_ = 10000;

    // === Set Default Parameters ===
    // Euclidean
    if (!ros::param::has("~euclidean/flag")) { ros::param::set("~euclidean/flag",false);}
    if (!ros::param::has("~euclidean/dist_tolerance")) { ros::param::set("~euclidean/dist_tolerance",0.18);}
    if (!ros::param::has("~euclidean/project_to_ground")) { ros::param::set("~euclidean/project_to_ground",false);}
    if (!ros::param::has("~euclidean/min_size")) { ros::param::set("~euclidean/min_size",50);}
    if (!ros::param::has("~euclidean/max_size")) { ros::param::set("~euclidean/max_size",2000);}

    // Region Growing
    if (!ros::param::has("~region_growing/flag")) { ros::param::set("~region_growing/flag",true);}
    if (!ros::param::has("~region_growing/min_size")) { ros::param::set("~region_growing/min_size",10);}
    if (!ros::param::has("~region_growing/max_size")) { ros::param::set("~region_growing/max_size",500);}
    if (!ros::param::has("~region_growing/num_of_neighbors")) { ros::param::set("~region_growing/num_of_neighbors",10);}
    if (!ros::param::has("~region_growing/search_radious")) { ros::param::set("~region_growing/search_radious",0.5);}
    if (!ros::param::has("~region_growing/smooth_threshold")) { ros::param::set("~region_growing/smooth_threshold",0.1);}
    if (!ros::param::has("~region_growing/curvature_threshold")) { ros::param::set("~region_growing/curvature_threshold",1.0);}

    // Dynmeans
    // Set default parameters for dynmeans
    if (!ros::param::has("~dynmeans/flag")) { ros::param::set("~dynmeans/flag",false);};
    if (!ros::param::has("~dynmeans/lambda")) { ros::param::set("~dynmeans/lambda",0.7);};
    if (!ros::param::has("~dynmeans/t_q")) { ros::param::set("~dynmeans/t_q",50.0);};
    if (!ros::param::has("~dynmeans/k_tau")) { ros::param::set("~dynmeans/k_tau",1.01);};
    if (!ros::param::has("~dynmeans/n_restart")) { ros::param::set("~dynmeans/n_restart",3);}

    if (!ros::param::has("~dynmeans/use_alpha_beta")) { ros::param::set("~dynmeans/use_alpha_beta",false);}
    if (!ros::param::has("~dynmeans/alpha")) { ros::param::set("~dynmeans/alpha",1.0);}
    if (!ros::param::has("~dynmeans/beta")) { ros::param::set("~dynmeans/beta",0.0);}

    // if (!ros::param::has("~visual/min_cluster_size")) { ros::param::set("~visual/min_cluster_size",0);}
    bool dynmeans_flag; ros::param::getCached("~dynmeans/flag",dynmeans_flag);
    if(dynmeans_flag){
      double lambda; ros::param::getCached("~dynmeans/lambda",lambda);
      double T_Q; ros::param::getCached("~dynmeans/t_q",T_Q);
      double K_tau; ros::param::getCached("~dynmeans/k_tau",K_tau);
      //ROS_INFO_STREAM("lambda:" << lambda << " T_Q:" << T_Q << " K_tau:" << K_tau);
      double Q = lambda/T_Q;
      double tau = (T_Q*(K_tau-1.0)+1.0)/(T_Q-1.0);
      // Initialize DynMeans obj
      dynm_ = new DynMeans<Eigen::Vector2f>(lambda, Q, tau);
      bool use_alpha_beta; ros::param::getCached("~dynmeans/use_alpha_beta",use_alpha_beta);
      if (use_alpha_beta){
        double alpha; ros::param::getCached("~dynmeans/alpha",alpha);
        double beta; ros::param::getCached("~dynmeans/beta",beta);
        dynm_->setAlphaBeta(alpha,beta);
      }

    }
  }

  bool srvResetCluster(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
  {
    ROS_INFO_STREAM("srv_reset_cluster called");
    dynm_->reset();
    return true;
  }

  std::vector<visualization_msgs::Marker> genMarkersFromClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, const pcl::PointIndices pointIndices, int id, std::string goal_frame)
  {
    std::vector<visualization_msgs::Marker> markerVector;
    ros::Duration lifetime = ros::Duration(0.5);

    // Get 3dMinMax and mean
    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;
    pcl::getMinMax3D(*oriCloud,pointIndices.indices,minPoint,maxPoint);
    double radius = sqrt(pow(maxPoint[0]-minPoint[0],2) + pow(maxPoint[1]-minPoint[1],2))/2;
    double height = maxPoint[2] - minPoint[2];
    pcl::PointXYZ meanPoint = getMeanPointFromIndices(oriCloud, pointIndices);

    // The points
    visualization_msgs::Marker marker_points = genMarkerPointFromIndicies(oriCloud, pointIndices);
    marker_points.header.frame_id = goal_frame;
    marker_points.header.stamp = ros::Time::now();
    marker_points.ns = ros::this_node::getName();
    marker_points.action = visualization_msgs::Marker::ADD;
    marker_points.id = id%max_marker_id_;
    marker_points.lifetime = lifetime;
    marker_points.pose.orientation.w = 1.0;
    marker_points.type = visualization_msgs::Marker::POINTS;
    marker_points.scale.x = 0.05;
    marker_points.scale.y = 0.05;
    // Marker color
    
    rgb randomColor = getColorById(id);
    marker_points.color.r = randomColor.r;
    marker_points.color.g = randomColor.g;
    marker_points.color.b = randomColor.b;
    marker_points.color.a = 1.0f;
    markerVector.push_back(marker_points);

    // The text
    visualization_msgs::Marker marker_text;
    marker_text.header.frame_id = goal_frame;
    marker_text.header.stamp = ros::Time::now();
    marker_text.ns = "text";
    marker_text.action = visualization_msgs::Marker::ADD;
    marker_text.id = id%max_marker_id_;
    marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_text.lifetime = lifetime;
    marker_text.scale.z = 0.3; //Size of text
    std::stringstream ss;
    ss << "id: " << id << std::endl;
    marker_text.text = ss.str();
    marker_text.pose.position.x = meanPoint.x;
    marker_text.pose.position.y = meanPoint.y;
    marker_text.pose.position.z = maxPoint[2];
    marker_text.color.r = 1.0f;
    marker_text.color.b = 1.0f;
    marker_text.color.g = 1.0f;
    marker_text.color.a = 1.0f;
    markerVector.push_back(marker_text);


    // The box
    visualization_msgs::Marker marker_box;
    marker_box.header.frame_id = goal_frame;
    marker_box.header.stamp = ros::Time::now();
    marker_box.ns = "CUBE";
    marker_box.action = visualization_msgs::Marker::ADD;
    marker_box.id = id%max_marker_id_;
    marker_box.type = visualization_msgs::Marker::CUBE;
    marker_box.lifetime = lifetime;
    
    float min_scale = 0.01;
    marker_box.scale.x = std::max(maxPoint[0] - minPoint[0],min_scale);
    marker_box.scale.y = std::max(maxPoint[1] - minPoint[1],min_scale);
    marker_box.scale.z = std::max(maxPoint[2] - minPoint[2],min_scale);

    marker_box.pose.position.x = (maxPoint[0] + minPoint[0])/2;
    marker_box.pose.position.y = (maxPoint[1] + minPoint[1])/2;
    marker_box.pose.position.z = (maxPoint[2] + minPoint[2])/2;

    marker_box.color.r = 1.0f;
    marker_box.color.b = 0.0f;
    marker_box.color.g = 1.0f;
    marker_box.color.a = 0.3f;
    markerVector.push_back(marker_box);

    return markerVector;
  }

  visualization_msgs::Marker genMarkerPointFromIndicies(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, const pcl::PointIndices pointIndices)
  {
    // Return visualization maker from point cloud given indices
    visualization_msgs::Marker marker;
    for (std::vector<int>::const_iterator pit = pointIndices.indices.begin(); pit != pointIndices.indices.end(); pit++)
    {
      geometry_msgs::Point p;
      p.x = (float) oriCloud->points[*pit].x;
      p.y = (float) oriCloud->points[*pit].y;
      p.z = (float) oriCloud->points[*pit].z;
      marker.points.push_back(p);
    }
    return marker;
  }

  pcl::PointXYZ getMeanPointFromIndices(const pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud, const pcl::PointIndices pointIndices)
  {
    // Return visualization maker from point cloud given indices
    pcl::PointXYZ meanPoint;
    double xsum = 0; double ysum = 0; double zsum = 0;
    for (std::vector<int>::const_iterator pit = pointIndices.indices.begin(); pit != pointIndices.indices.end(); pit++)
    {
      xsum += (float) oriCloud->points[*pit].x;
      ysum += (float) oriCloud->points[*pit].y;
      zsum += (float) oriCloud->points[*pit].z;
    }
    // pcl::PointXYZ meanPoint(xsum/(float) pointIndices.indices.size(),ysum/(float) pointIndices.indices.size(), zsum/(float) pointIndices.indices.size());
    meanPoint.x = xsum/(float) pointIndices.indices.size();
    meanPoint.y = ysum/(float) pointIndices.indices.size();
    meanPoint.z = zsum/(float) pointIndices.indices.size();
    return meanPoint;
  }

  rgb getColorById(int id)
  {
    // Return color by id. If no color is defined for id. Generate a random color
    // Fill colormap_ if empty
    if ( colormap_.size() < (id + 1) ){
      rgb randomColor = { (float)(rand()%10)/10.0, (float)(rand()%10)/10.0, (float)(rand()%10)/10.0 };
      colormap_.push_back(randomColor);
    }
    return colormap_[id];
  }

  std::map<int, pcl::PointIndices> getDynMeanClusterMap(std::vector<int> learnedLabels)
  {
    // Return a vector of pointindices corresponding given the learnedLabels from dynmean
    std::map<int, pcl::PointIndices> cluster_map;
    // Iterate over the learnedLabels
    for (int i = 0; i < learnedLabels.size(); i++){
      int label = learnedLabels[i]; //label of point i
      cluster_map[label].indices.push_back(i); //Write to map
    }
    return cluster_map;
  }

  void cbCloud(const sensor_msgs::PointCloud2& input)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (input, *cloud);
    if (cloud->size() == 0){
      ROS_INFO_STREAM("cloud->size() = 0. Skipped");
      return;
    }

    // =Clustering=
    std::map<int, pcl::PointIndices> clusterMap;
    std::vector<Eigen::Vector2f> learnedParams;

    // ==Euclidean==
    // ===Read Parameters===
    bool euclidean_flag;ros::param::getCached("~euclidean/flag",euclidean_flag);
    if (euclidean_flag){
      bool project_to_ground; ros::param::getCached("~euclidean/project_to_ground",project_to_ground);
      double dist_tolerance; ros::param::getCached("~euclidean/dist_tolerance",dist_tolerance);
      int min_size; ros::param::getCached("~euclidean/min_size",min_size);
      int max_size; ros::param::getCached("~euclidean/max_size",max_size);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
      if (project_to_ground){
        pcl::ModelCoefficients::Ptr coefficient(new pcl::ModelCoefficients());
        coefficient->header = cloud->header;
        std:std::vector<float> coeff_vec;ros::param::getCached("/ground_coefficient",coeff_vec);
        coefficient->values = coeff_vec;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proj(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_proj = utilpcl::ProjectPointCloud(cloud, coefficient);
        cloud_source = cloud_proj;
      }
      else{
        cloud_source = cloud;
      }
      std::vector<pcl::PointIndices> clusterVec;
      clusterVec = utilpcl::getEuclideanClusterIndices(cloud_source, dist_tolerance, min_size, max_size);
      for(int i = 0; i < clusterVec.size(); i++){
        clusterMap[i] = clusterVec[i];
      }
    }

    // ===Region Growing===
    bool region_growing_flag;ros::param::getCached("~region_growing/flag",region_growing_flag);
    if(region_growing_flag){
      int min_size; ros::param::getCached("~region_growing/min_size",min_size);
      int max_size; ros::param::getCached("~region_growing/max_size",max_size);
      int num_of_neighbors; ros::param::getCached("~region_growing/num_of_neighbors",num_of_neighbors);
      double search_radious; ros::param::getCached("~region_growing/search_radious",search_radious);
      double smooth_threshold; ros::param::getCached("~region_growing/smooth_threshold",smooth_threshold);
      double curvature_threshold; ros::param::getCached("~region_growing/curvature_threshold",curvature_threshold);
      std::vector<pcl::PointIndices> clusterVec;
      clusterVec = utilpcl::getRegionGrowingClusterIndicies(cloud, min_size, max_size,
      num_of_neighbors, search_radious, smooth_threshold, curvature_threshold);
      for(int i = 0; i < clusterVec.size(); i++){
        clusterMap[i] = clusterVec[i];
      }
    }

    // ===Dynmean===
    bool dynmeans_flag;ros::param::getCached("~dynmeans/flag",dynmeans_flag);
    if(dynmeans_flag){
      // Fill clusterData for clustering
      std::vector<Eigen::Vector2f> clusterData;
      for( pcl::PointCloud<pcl::PointXYZ>::iterator iter = cloud->begin(); iter != cloud->end(); ++iter)
      {
        Eigen::Vector2f v2d;
        v2d[0] = (*iter).x;
        v2d[1] = (*iter).y;
        clusterData.push_back(v2d);
      }
      // Performe dynmean clustering
      std::vector<int> learnedLabels;
      double tTaken, obj;
      int n_restart; ros::param::getCached("~dynmeans/n_restart",n_restart);
      dynm_->cluster(clusterData, n_restart, learnedLabels, learnedParams, obj, tTaken, input.header.stamp.toSec());
      // Convert dymean labels to cluster indicies
      clusterMap = getDynMeanClusterMap(learnedLabels);
    }

    // Generate markerArray for visualization
    visualization_msgs::MarkerArray markerArray;
    for (std::map<int, pcl::PointIndices>::const_iterator it = clusterMap.begin(); it != clusterMap.end(); ++it)
    {
      // Generate the markers for each cluster
      std::vector<visualization_msgs::Marker> markers = genMarkersFromClusters(cloud, it->second, it->first, cloud->header.frame_id);
      //Append the markers to the marker array if the cluster has enough points
      markerArray.markers.insert(markerArray.markers.end(), markers.begin(), markers.end()); 

    }
    pub_marker_.publish(markerArray);

    // Generate the clusters msg
    mod_msgs::Clusters clusters_msg;
    // Set header
    clusters_msg.header.seq = input.header.seq;
    clusters_msg.header.stamp = input.header.stamp;
    clusters_msg.header.frame_id = input.header.frame_id;

    for (std::map<int, pcl::PointIndices>::const_iterator it = clusterMap.begin(); it != clusterMap.end(); ++it)
    {
      // Get label
      clusters_msg.labels.push_back(it->first);

      // Get counts
      clusters_msg.counts.push_back(it->second.indices.size());

      // Get mean point
      pcl::PointXYZ meanPoint = getMeanPointFromIndices(cloud, it->second);
      geometry_msgs::Point geomsg_meanPoint;
      geometry_msgs::Vector3 velocity;

      if(dynmeans_flag){
        // Find the learned parameter with the current label
        int prm_index = -1;
        // for (int index = 0; index < dynm_->oldprmlbls.size(); index++){
        for (int index = 0; index < dynm_->cluster_vec_.size(); index++){
          // if (dynm_->oldprmlbls[index] == it->first){
          if (dynm_->cluster_vec_[index].id_ == it->first){
            prm_index = index;
            break;
          }
        }
        Eigen::Vector2f prm = dynm_->cluster_vec_[prm_index].center_;
        geomsg_meanPoint.x = prm[0];
        geomsg_meanPoint.y = prm[1];
        geomsg_meanPoint.z = meanPoint.z;
        /* Populate velocities using the filtered velocity in dynmean */
        velocity.x = dynm_->cluster_vec_[prm_index].velocity_[0];
        velocity.y = dynm_->cluster_vec_[prm_index].velocity_[1];
      }
      else{
        geomsg_meanPoint.x = meanPoint.x;
        geomsg_meanPoint.y = meanPoint.y;
        geomsg_meanPoint.z = meanPoint.z;
      }
      clusters_msg.mean_points.push_back(geomsg_meanPoint);
      clusters_msg.velocities.push_back(velocity);

      // Get min and max point
      Eigen::Vector4f minPoint;
      Eigen::Vector4f maxPoint;
      pcl::getMinMax3D(*cloud,it->second.indices,minPoint,maxPoint);

      geometry_msgs::Point geomsg_minPoint;
      geomsg_minPoint.x = minPoint[0];
      geomsg_minPoint.y = minPoint[1];
      geomsg_minPoint.z = minPoint[2];
      clusters_msg.min_points.push_back(geomsg_minPoint);

      geometry_msgs::Point geomsg_maxPoint;
      geomsg_maxPoint.x = maxPoint[0];
      geomsg_maxPoint.y = maxPoint[1];
      geomsg_maxPoint.z = maxPoint[2];
      clusters_msg.max_points.push_back(geomsg_maxPoint);

      // Populate the pointclouds part of the clusters_msg
      sensor_msgs::PointCloud2 pointcloud_msg;
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointIndices::Ptr indices_ptr (new pcl::PointIndices);
      indices_ptr->indices = it->second.indices;
      pointcloud = utilpcl::extractPointCloudByIndices(cloud, indices_ptr, false);
      pcl::toROSMsg(*pointcloud, pointcloud_msg);
      pointcloud_msg.header.seq = input.header.seq;
      pointcloud_msg.header.stamp = input.header.stamp;
      pointcloud_msg.header.frame_id = input.header.frame_id;
      clusters_msg.pointclouds.push_back(pointcloud_msg);
    }
    // ROS_INFO("[ClusterNode] msg took: %.10f",(ros::Time::now() - start_time).toSec());
    pub_clusters_.publish(clusters_msg);
  }
};


int main (int argc, char**argv)
{
  ros::init (argc, argv, "cluster_node");
  ClusterNode clusterClassObj(argc, argv);
  ros::spin();
  return 0;
}
