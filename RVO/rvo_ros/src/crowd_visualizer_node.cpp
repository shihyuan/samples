/*
Author: Shih-Yuan Liu
*/

#include <ros/ros.h>
#include "CrowdVisualizer.hpp"

class CrowdVisualizerNode{
public:
  ros::NodeHandle nh_p_;
  CrowdVisualizer vis_;

  ros::Publisher pub_crowd_;
  ros::Publisher pub_pred_;
  ros::Publisher pub_info_;
  ros::Publisher pub_obs_;
  ros::Publisher pub_vo_;

  ros::Subscriber sub_crowd_;
  ros::Subscriber sub_pred_;
  ros::Subscriber sub_info_;
  ros::Subscriber sub_obs_;
  ros::Subscriber sub_vo_;

  CrowdVisualizerNode(const ros::NodeHandle& nh): nh_p_(nh), vis_(CrowdVisualizer(ros::this_node::getName()))
  {
    pub_crowd_ = nh_p_.advertise<visualization_msgs::MarkerArray>("crowd_marker",1,true);
    pub_pred_ = nh_p_.advertise<visualization_msgs::MarkerArray>("pred_marker",1);
    pub_info_ = nh_p_.advertise<visualization_msgs::MarkerArray>("info_marker",1);
    pub_obs_ = nh_p_.advertise<visualization_msgs::MarkerArray>("obs_marker",1,true);
    // pub_vo_ = nh_p_.advertise<visualization_msgs::Marker>("vo_marker",1);

    sub_crowd_ = nh_p_.subscribe("crowd",1, &CrowdVisualizerNode::cbCrowd, this);
    sub_pred_ = nh_p_.subscribe("crowd_pred",1, &CrowdVisualizerNode::cbCrowdPred, this);
    sub_info_ = nh_p_.subscribe("crowd_info",1, &CrowdVisualizerNode::cbCrowdInfo, this);
    sub_obs_ = nh_p_.subscribe("obstacles",1, &CrowdVisualizerNode::cbObstacles, this);
    // sub_vo_ = nh_p_.subscribe("agent_info",1,&CrowdVisualizerNode::cbVoInfo,this);
  }
  ~CrowdVisualizerNode(){}

  void cbCrowd(const rvo_ros::Crowd& crowd_msg)
  {
    pub_crowd_.publish(vis_.toAgentMarkerArray(crowd_msg));
  }
  void cbCrowdPred(const rvo_ros::Crowd& crowd_msg)
  {
    visualization_msgs::MarkerArray markerArray = vis_.toAgentMarkerArray(crowd_msg);
    for (std::vector<visualization_msgs::Marker>::iterator it = markerArray.markers.begin(); it != markerArray.markers.end(); ++it){
      it->color.a = it->color.a * 0.5;
    }
    pub_pred_.publish(markerArray);
  }
  void cbCrowdInfo(const rvo_ros::Crowd& crowd_msg)
  {
    pub_info_.publish(vis_.toEstimateMarkerArray(crowd_msg));
  }
  void cbObstacles(const rvo_ros::Obstacles& obs_msg)
  {
    pub_obs_.publish(vis_.toObstacleMarkerArray(obs_msg));
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crowd_visualizer");
  ros::NodeHandle nh("~");
  CrowdVisualizerNode crowd_visualizer(nh);
  ros::spin();
  return 0;
}

