/*
Author: Shih-Yuan Liu
*/
#include <vector>
#include <stdlib.h> 

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/octree/octree.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <utilpcl.hpp>


class CloudFilter{
public:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::ServiceServer srv_;
  tf::TransformListener* tfListener_;
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>* octree_;
  bool has_key_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr key_cloud_;
  pcl::ModelCoefficients::Ptr ground_coefficient_;

  CloudFilter(int argc, char** argv, double octree_res)
  {
    // Setup node handler
    nh_ = ros::NodeHandle("~");
    pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("output/pointcloud",10);
    sub_ = nh_.subscribe("input/pointcloud", 5, &CloudFilter::cbPointCloud, this);
    srv_ = nh_.advertiseService("reset_key_cloud", &CloudFilter::srvResetKeyCloud ,this);
    tfListener_ = new tf::TransformListener(ros::Duration(60));
    // Initialize octree
    octree_ = new pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>(octree_res);
    has_key_cloud_ = false;

    // ===Default parameters===

    // Set goal frame
    if (!ros::param::has("~goal_frame")) { ros::param::set("~goal_frame","world");}
    if (!ros::param::has("~passthough/flag")) { ros::param::set("~passthough/flag",true);}

    // Diff filter
    if (!ros::param::has("~diff_filter/flag")) { ros::param::set("~diff_filter/flag",true);}

    // Noise removal
    if (!ros::param::has("~noise_remove/flag")) { ros::param::set("~noise_remove/flag",true);}
    if (!ros::param::has("~noise_remove/radius")) { ros::param::set("~noise_remove/radius",0.1);}
    if (!ros::param::has("~noise_remove/min_neighbor")) { ros::param::set("~noise_remove/min_neighbor",5);}

    // Downsample
    if (!ros::param::has("~downsample/flag")) { ros::param::set("~downsample/flag",true);}
    if (!ros::param::has("~downsample/leaf_size")) { ros::param::set("~downsample/leaf_size",0.2);}

    // Ground removal
    if (!ros::param::has("~ground_remove/flag")) { ros::param::set("~ground_remove/flag",true); };
    if (!ros::param::has("~ground_remove/max_iteration")) { ros::param::set("~ground_remove/max_iteration",500); };
    if (!ros::param::has("~ground_remove/distance_threshold")) { ros::param::set("~ground_remove/distance_threshold",0.1); };
    if (!ros::param::has("~ground_remove/use_fixed_ground")) { ros::param::set("~ground_remove/use_fixed_ground",true); };

    // Preset ground_coefficient_
    pcl::ModelCoefficients::Ptr coefficient(new pcl::ModelCoefficients());
    ground_coefficient_ = coefficient;
    ground_coefficient_->values.push_back(-0.00627247);
    ground_coefficient_->values.push_back(-0.00825334);
    ground_coefficient_->values.push_back(0.999946);
    ground_coefficient_->values.push_back(-0.0295839);
  }

  bool srvResetKeyCloud(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
  {
    ROS_INFO_STREAM("srvResetKeyCloud called");
    has_key_cloud_ = false;
    return true;
  }

  void setKeyCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  { 
    key_cloud_ = cloud;
    has_key_cloud_ = true;
    ROS_INFO_STREAM("Key cloud set.");
  }

  void loadKeyCloud(){
    // Load the key cloud into the octree_
    octree_->deleteCurrentBuffer();
    octree_->switchBuffers();
    octree_->setInputCloud(key_cloud_);
    octree_->addPointsFromInputCloud();
    octree_->switchBuffers();
    ROS_INFO_STREAM("Key cloud loaded.");
  }

  int getDiffToKey(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
  {
    // Return the number of points in cloud_in that are not in the key cloud.
    // Put them in cloud_out
    octree_->setInputCloud (cloud_in);
    octree_->addPointsFromInputCloud ();
    // Get vector of point indices from octree voxels which did not exist in previous buffer
    std::vector<int> newPointIdxVector;
    octree_->getPointIndicesFromNewVoxels (newPointIdxVector);

    // Use extract indices to extract the differ pointcloud
    pcl::PointIndices::Ptr diffIndices (new pcl::PointIndices ());
    diffIndices->indices = newPointIdxVector;

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_in);
    extract.setIndices (diffIndices);
    extract.setNegative (false);
    extract.filter (*cloud_out);

    // Remove cloud_in from current buffer
    octree_->deleteCurrentBuffer();

    return newPointIdxVector.size();
  }

  void cbPointCloud(const sensor_msgs::PointCloud2& input)
  {
    // ===Convert the transformed cloud to pcl::PointCloud format===
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (input, *cloud_input);

    /* Remove NaNs */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<int> temp_ind;
    pcl::removeNaNFromPointCloud(*cloud_input,*cloud,temp_ind);

    bool passthrough_flag = true; ros::param::getCached("~passthough/flag",passthrough_flag);
    if (passthrough_flag){
        pcl::PointIndices::Ptr passIndices = utilpcl::getPassthroughIndicies(cloud,"y",0.0,50.0);
      cloud = utilpcl::extractPointCloudByIndices(cloud,passIndices);
    }

    // ===Diff filter===
    bool diff_filter_flag; ros::param::getCached("~diff_filter/flag",diff_filter_flag);
    if (diff_filter_flag){
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
      if (getCloudDiff(cloud, cloud_temp)){
        // Replace cloud with the diff-filtered cloud
        cloud = cloud_temp;        
        if(cloud->size()==0){ROS_INFO_STREAM("Empty cloud after diff filter.");return;}
      }
      else{
        // Key Frame reloaded. Skip this frame.
        return;
      }
    }
    // ===Transform frame=== //
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tf(new pcl::PointCloud<pcl::PointXYZ>);
    std::string goal_frame; ros::param::getCached("~goal_frame",goal_frame);
    tfListener_->waitForTransform(goal_frame, input.header.frame_id, ros::Time(0), ros::Duration(5.0));
    pcl_ros::transformPointCloud(goal_frame,*cloud, *cloud_tf, *tfListener_);
    cloud = cloud_tf;
    // ROS_INFO_STREAM("Frame id of cloud after tf: " << cloud->header.frame_id);


    // ===Downsample===
    double leaf_size; ros::param::getCached("~downsample/leaf_size",leaf_size);
    bool downsample_flag = true; ros::param::getCached("~downsample/flag",downsample_flag);
    if (downsample_flag){
      cloud = utilpcl::downsample(cloud,leaf_size);
      if(cloud->size()==0){ROS_INFO_STREAM("Empty cloud after down sample.");return;}
    }
    // ===Noise Removal===
    bool noise_remove_flag; ros::param::getCached("~noise_remove/flag",noise_remove_flag);
    if (noise_remove_flag){
      // Read parameters
      double noise_remove_radius; ros::param::getCached("~noise_remove/radius",noise_remove_radius);
      int noise_remove_min_neighbor; ros::param::getCached("~noise_remove/min_neighbor",noise_remove_min_neighbor);
      cloud = utilpcl::removeRadiusOutlier(cloud, noise_remove_radius, noise_remove_min_neighbor);
      if(cloud->size()==0){ROS_INFO_STREAM("Empty cloud after noise removal.");return;}
    }


    // ===Remove ground===
    bool ground_remove_flag; ros::param::getCached("~ground_remove/flag",ground_remove_flag);
    bool use_fixed_ground; ros::param::getCached("~ground_remove/use_fixed_ground",use_fixed_ground);
    double distance_threshold; ros::param::getCached("~ground_remove/distance_threshold",distance_threshold);
    pcl::PointIndices::Ptr ground_indices;
    pcl::ModelCoefficients::Ptr coefficient(new pcl::ModelCoefficients());
    ground_coefficient_->header = cloud->header;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground_hull (new pcl::PointCloud<pcl::PointXYZ>);
    if (ground_remove_flag){
      if (use_fixed_ground){
        // Use the preset ground coefficient 
        coefficient = ground_coefficient_;
        ground_indices = utilpcl::getPlaneInlierIndicies(cloud,ground_coefficient_,distance_threshold);
      } 
      else{
        // Use ransac to get the coefficient of the ground
        int max_iteration; ros::param::getCached("~ground_remove/max_iteration",max_iteration);
        ground_indices = utilpcl::getPlanarIndiciesRansac(cloud, max_iteration, distance_threshold, coefficient);

        // ROS_INFO_STREAM("Ground Coeff values:" << coefficient->values[0] << " " << coefficient->values[1] << " " << coefficient->values[2] << " "<< coefficient->values[3] << " " );
        // ROS_INFO_STREAM("Ground Coeff header:" << coefficient->header);
        if (coefficient->values.size() < 4){
          ROS_INFO_STREAM("coefficient extraction failed. Using preset coefficient.");
          coefficient = ground_coefficient_;
          ground_indices = utilpcl::getPlaneInlierIndicies(cloud,coefficient,distance_threshold);
        }
        ros::param::set("ground_coefficient",coefficient->values);
      }
      // Remove the ground
      cloud = utilpcl::extractPointCloudByIndices(cloud, ground_indices, true);
      if(cloud->size()==0){ROS_INFO_STREAM("Empty cloud after ground remove.");return;}
    }
    // ===Convert to ROSMsg===
    sensor_msgs::PointCloud2 cloud_pub;
    pcl::toROSMsg(*cloud, cloud_pub);    
    cloud_pub.header.frame_id = goal_frame;
    cloud_pub.header.stamp = input.header.stamp; //TODO see if this is appropreate
    
    // Publish
    pub_.publish(cloud_pub);
  }

  bool getCloudDiff(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_diff_out)
  {
    if (!has_key_cloud_){
      // No key cloud yet then set current input as key cloud
      setKeyCloud(cloud);
      // Load it to buffer
      loadKeyCloud();
      return false;
    }
    else{
      
      int size_of_diff = getDiffToKey(cloud,cloud_diff_out);

      // ROS_INFO_STREAM("Size of cloud: " << cloud->size());
      
      double change_ratio = (double)size_of_diff/(double) cloud->size();

      if (change_ratio < 0.99){
        // Acceptable change rate.
        // ROS_INFO_STREAM("cloud_diff_out size:" << cloud_diff_out->size());
        return true;
      }
      else{
        // Re-load the key cloud when change rate is too high
        loadKeyCloud();
        // Don't return cloud_diff_out
        return false;
      }
    }
  }

};


int main(int argc, char**argv)
{
  ros::init (argc, argv, "cloud_filter_node");

  // Set default octree resolution
  if (!ros::param::has("~diff_filter/octree_res")) { ros::param::set("~diff_filter/octree_res",0.2);}

  // Initializet the filter obj
  double octree_res; ros::param::getCached("~diff_filter/octree_res",octree_res);
  CloudFilter cloudFliterObj(argc, argv, octree_res);
  ros::spin();
  return 0;
}