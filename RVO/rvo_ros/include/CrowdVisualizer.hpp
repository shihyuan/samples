/*
Author: Shih-Yuan Liu
*/

#ifndef __CROWD_VISUALIZER_HPP
#define __CROWD_VISUALIZER_HPP

#include <ros/ros.h>
#include <vector>
#include <map>
#include <sstream>
#include <omp.h>

#include <rvo_ros/Agent.h>
#include <rvo_ros/AgentParam.h>
#include <rvo_ros/AgentState.h>
#include <rvo_ros/AgentInfo.h>
#include <rvo_ros/Obstacles.h>
#include <rvo_ros/Crowd.h>
#include <rvo_ros/CrowdTraj.h>
#include <rvo_ros/VoInfo.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

class CrowdVisualizer{
public:
  std::map<int , std_msgs::ColorRGBA > color_map_;
  std::string ns_;
  std::string frame_id_;
  ros::Time stamp_;
  ros::Duration duration_;
  // ros::Time last_draw_time_;
  bool use_random_color_;
  bool show_pref_vel_;
  bool show_vel_;

  int num_of_obs_last_;
  std::vector<int> agent_id_last_;

  CrowdVisualizer(const std::string& ns);
  ~CrowdVisualizer();
  
  visualization_msgs::MarkerArray toAgentMarkerArray(const rvo_ros::Crowd& crowd);
  visualization_msgs::MarkerArray toEstimateMarkerArray(const rvo_ros::Crowd& crowd);
  visualization_msgs::MarkerArray toObstacleMarkerArray(const rvo_ros::Obstacles& obstacles);

  std::vector<visualization_msgs::Marker> toEstimateMarkers(const rvo_ros::Agent& agentMsg);
  std::vector<visualization_msgs::Marker> toAgentMarkers(const rvo_ros::Agent& agentMsg);
  std::vector<visualization_msgs::Marker> deleteAgentMarkers(int agent_id);
  visualization_msgs::Marker toObstacleMarker(const geometry_msgs::Polygon& polygon, int id);
  visualization_msgs::Marker toVoMarker(const rvo_ros::Agent& agentMsg);

  std_msgs::ColorRGBA randomeColor();
  std_msgs::ColorRGBA getAgentColor(const rvo_ros::Agent& agentMsg);
};

/*----IMPL----*/

visualization_msgs::Marker CrowdVisualizer::toVoMarker(const rvo_ros::Agent& agentMsg)
{
  /* Extract the polygon from VoInfo using VoPolygon*/
  const geometry_msgs::Polygon& polygon = agentMsg.info.voPoly;

  /* Generate visualization::Marker */
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = stamp_;
  marker.ns = ns_ + "/vo";
  marker.id = agentMsg.id;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = duration_;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  /* Offset the center to the center of the agent */
  geometry_msgs::Point point;
  for (int i = 0; i < polygon.points.size(); ++i){
    point.x = polygon.points[i].x + agentMsg.state.pos.x;
    point.y = polygon.points[i].y + agentMsg.state.pos.y;
    point.z = 0.0;
    marker.points.push_back(point);
  }
  /* Add first point again to close the polygon. Only do so when there is more than two points */
  if (marker.points.size() > 2){
    marker.points.push_back(marker.points.front());
  }
  marker.color = getAgentColor(agentMsg);
  marker.color.a = 1.0;
  marker.scale.x = 0.05;
  return marker;
}

visualization_msgs::Marker CrowdVisualizer::toObstacleMarker(const geometry_msgs::Polygon& polygon, int id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = stamp_;
  marker.ns = ns_ + "/obs";
  marker.id = id;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(0.0); //Forever
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  geometry_msgs::Point point;
  for (int i = 0; i < polygon.points.size(); ++i){
    point.x = polygon.points[i].x;
    point.y = polygon.points[i].y;
    point.z = 0.0;
    marker.points.push_back(point);
  }
  /* Add first point again to close the polygon. Only do so when there is more than two points */
  if (marker.points.size() > 2){
    marker.points.push_back(marker.points.front());
  }
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.scale.x = 0.05;
  return marker;
}

visualization_msgs::MarkerArray CrowdVisualizer::toObstacleMarkerArray(const rvo_ros::Obstacles& obstacles)
{
  visualization_msgs::MarkerArray marker_array;
  for (int i = 0; i < std::max(num_of_obs_last_,static_cast<int>(obstacles.obstacles.size())); ++i) {
    visualization_msgs::Marker marker;
    if (i < obstacles.obstacles.size()){
      marker = toObstacleMarker(obstacles.obstacles[i],i);
    }
    else{
      marker = toObstacleMarker(geometry_msgs::Polygon(),i); //Empty marker with the right ns and id.
      marker.action = visualization_msgs::Marker::DELETE;
    }
    marker_array.markers.push_back(marker);
  }
  num_of_obs_last_ = obstacles.obstacles.size();
  return marker_array;
}
CrowdVisualizer::CrowdVisualizer(const std::string& ns): ns_(ns), use_random_color_(false), show_vel_(true), show_pref_vel_(true), duration_(ros::Duration(1.0)), num_of_obs_last_(0)
{
  frame_id_ = "world";
  stamp_ = ros::Time::now();
  std_msgs::ColorRGBA color;
  color.r = 0.0; color.g = 1.0; color.b = 0.0; color.a = 1.0;
  color_map_[rvo_ros::AgentParam::NORMAL] = color;
  color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 1.0;
  color_map_[rvo_ros::AgentParam::AGGRESSIVE] = color;
  color.r = 0.0; color.g = 0.0; color.b = 1.0; color.a = 1.0;
  color_map_[rvo_ros::AgentParam::USER] = color;
  color.r = 0.0; color.g = 0.0; color.b = 1.0; color.a = 1.0;
  color_map_[rvo_ros::AgentParam::USER_PREF] = color;
}

std_msgs::ColorRGBA CrowdVisualizer::randomeColor()
{
  std_msgs::ColorRGBA color;
  color.r = ((double) rand() / (RAND_MAX));
  color.g = ((double) rand() / (RAND_MAX));
  color.b = ((double) rand() / (RAND_MAX));
  color.a = 1.0;
  return color;
}

visualization_msgs::MarkerArray CrowdVisualizer::toAgentMarkerArray(const rvo_ros::Crowd& crowd)
{
  // ROS_INFO_STREAM("[publishMarkers] Started.");
  visualization_msgs::MarkerArray markerArray;
  frame_id_ = crowd.header.frame_id;
  stamp_ = crowd.header.stamp;

  std::vector<std::vector<visualization_msgs::Marker> > marker_vec_vec(crowd.agents.size());
  #pragma omp parallel for
  for (int i = 0; i < crowd.agents.size(); ++i){
    rvo_ros::Agent agentMsg = crowd.agents[i];
    marker_vec_vec[i] = toAgentMarkers(agentMsg);
  }

  for (int i = 0; i < marker_vec_vec.size() ;++i){
    markerArray.markers.insert(markerArray.markers.end(), marker_vec_vec[i].begin(), marker_vec_vec[i].end());
  }

  // Book keeping
  // Go through all previous agent ids. Insert delete 
  for (int i = 0; i < agent_id_last_.size(); i++){
    int agent_id = agent_id_last_[i];
    std::vector<int>::const_iterator it = std::find(crowd.agentIds.begin(),crowd.agentIds.end(),agent_id);
    if (it == crowd.agentIds.end()){
      // agent_id in agent_id_last_ not in the crowd.
      std::vector<visualization_msgs::Marker> deleteMarkers = deleteAgentMarkers(agent_id);
      markerArray.markers.insert(markerArray.markers.end(), deleteMarkers.begin(), deleteMarkers.end());
    }
  }

  // Clear and replace
  agent_id_last_.clear();
  agent_id_last_ = crowd.agentIds;
  return markerArray;
}  

visualization_msgs::MarkerArray CrowdVisualizer::toEstimateMarkerArray(const rvo_ros::Crowd& crowd)
{
  visualization_msgs::MarkerArray markerArray;
  frame_id_ = crowd.header.frame_id;
  stamp_ = crowd.header.stamp;

  // #pragma omp parallel for
  for (int i = 0; i < crowd.agents.size(); ++i){
    rvo_ros::Agent agentMsg = crowd.agents[i];
    std::vector<visualization_msgs::Marker> marker_vec = toEstimateMarkers(agentMsg);
    markerArray.markers.insert(markerArray.markers.end(), marker_vec.begin(), marker_vec.end());
  }
  return markerArray;
}

std_msgs::ColorRGBA CrowdVisualizer::getAgentColor(const rvo_ros::Agent& agentMsg){
  // std_msgs::ColorRGBA color;
  if (use_random_color_){
    std::map<int , std_msgs::ColorRGBA >::const_iterator it = color_map_.find(agentMsg.id);
    if (it == color_map_.end()){
      color_map_[agentMsg.id] = randomeColor();
    }
    return color_map_[agentMsg.id];
  }
  else{
    return color_map_[agentMsg.param.agentType];
  }
}

std::vector<visualization_msgs::Marker> CrowdVisualizer::toEstimateMarkers(const rvo_ros::Agent& agentMsg)
{
  float value_scale = 1.0;
  std::vector<visualization_msgs::Marker> marker_vec;
  /* TODO: re-implement for new AgentInfo */

  // std_msgs::ColorRGBA color = getAgentColor(agentMsg);
  // // === Lambda Estimate === //
  // visualization_msgs::Marker marker_text;
  // marker_text.header.frame_id = frame_id_;
  // marker_text.header.stamp = stamp_;
  // marker_text.ns = ns_ + "/value";
  // marker_text.id = agentMsg.id;
  // marker_text.action = visualization_msgs::Marker::ADD;
  // marker_text.lifetime = duration_;
  // marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  // marker_text.pose.position.x = agentMsg.state.pos.x;
  // marker_text.pose.position.y = agentMsg.state.pos.y;
  // marker_text.pose.position.z = value_scale*agentMsg.info.value + 0.2;
  // marker_text.pose.orientation.w = 1.0;
  // marker_text.color = color;
  // marker_text.color.a = 1.0;
  // marker_text.scale.z = 0.2;
  // std::ostringstream ss;
  // ss << agentMsg.info.value;
  // marker_text.text = ss.str();
  // marker_vec.push_back(marker_text);

  // // === Value list  === //
  // visualization_msgs::Marker marker_vector;
  // marker_vector.header.frame_id = frame_id_;
  // marker_vector.header.stamp = stamp_;
  // marker_vector.ns = ns_ + "/traj";
  // marker_vector.id = agentMsg.id;
  // marker_vector.action = visualization_msgs::Marker::ADD;
  // marker_vector.lifetime = duration_;
  // marker_vector.type = visualization_msgs::Marker::LINE_STRIP;
  // geometry_msgs::Point point;
  // for (int i = 0; i < agentMsg.info.traj.size(); ++i){
  //   point.x = agentMsg.info.traj[i].x;
  //   point.y = agentMsg.info.traj[i].y;
  //   point.z = agentMsg.info.value_list[i];
  //   marker_vector.points.push_back(point);
  // }
  // marker_vector.color = color;
  // marker_vector.color.a = 1.0;
  // marker_vector.scale.x = 0.02;
  // marker_vec.push_back(marker_vector);
  return marker_vec;
}

std::vector<visualization_msgs::Marker> CrowdVisualizer::deleteAgentMarkers(int agent_id)
{
  std::vector<visualization_msgs::Marker> marker_vec;
  visualization_msgs::Marker marker;
  marker.id = agent_id;
  marker.action = visualization_msgs::Marker::DELETE;

  marker.ns = ns_ + "/pos";
  marker_vec.push_back(marker);
  marker.ns = ns_ + "/collision";
  marker_vec.push_back(marker);
  marker.ns = ns_ + "/vel";
  marker_vec.push_back(marker);
  marker.ns = ns_ + "/pref";
  marker_vec.push_back(marker);

  return marker_vec;
}


std::vector<visualization_msgs::Marker> CrowdVisualizer::toAgentMarkers(const rvo_ros::Agent& agentMsg)
{
  std::vector<visualization_msgs::Marker> marker_vec;
  std_msgs::ColorRGBA color = getAgentColor(agentMsg);
  
  // === Pos === //
  visualization_msgs::Marker marker_pos;
  marker_pos.header.frame_id = frame_id_;
  marker_pos.header.stamp = stamp_;
  marker_pos.ns = ns_ + "/pos";
  marker_pos.id = agentMsg.id;
  marker_pos.action = visualization_msgs::Marker::ADD;
  marker_pos.lifetime = duration_;
  marker_pos.type = visualization_msgs::Marker::CYLINDER;
  marker_pos.pose.position.x = agentMsg.state.pos.x;
  marker_pos.pose.position.y = agentMsg.state.pos.y;
  marker_pos.pose.position.z = 0.0;
  marker_pos.pose.orientation.w = 1.0;
  marker_pos.color = color;
  marker_pos.color.a = 0.5;
  marker_pos.scale.x = agentMsg.param.radius*2.0;
  marker_pos.scale.y = agentMsg.param.radius*2.0;
  marker_pos.scale.z = 0.01;
  marker_vec.push_back(marker_pos);


  // === Collision Mark === //
  if (agentMsg.info.inCollisionWith.size() > 0){ 
    marker_pos.ns = ns_ + "/collision";
    marker_pos.pose.position.z = 0.5;
    marker_pos.lifetime = ros::Duration(0.2);
    marker_pos.scale.x = agentMsg.param.radius;
    marker_pos.scale.y = agentMsg.param.radius;
    marker_pos.scale.z = 1.0;
    marker_vec.push_back(marker_pos);
  }

  // === Vel === //
  float vel_ratio = 1.0;
  float arrow_length = std::pow((vel_ratio*agentMsg.state.vel.x),2) + std::pow((vel_ratio*agentMsg.state.vel.y),2);

  visualization_msgs::Marker marker_vel;
  marker_vel.header.frame_id = frame_id_;
  marker_vel.header.stamp = stamp_;
  marker_vel.ns = ns_ + "/vel";
  marker_vel.id = agentMsg.id;
  marker_vel.action = visualization_msgs::Marker::ADD;
  marker_vel.lifetime = duration_;
  marker_vel.type = visualization_msgs::Marker::ARROW;
  geometry_msgs::Point point;
  point.x = agentMsg.state.pos.x;
  point.y = agentMsg.state.pos.y;
  marker_vel.points.push_back(point);
  // avoid arrow appearing 'jumpy' when velocity is near-zero
  if (arrow_length > 0.0001){
    point.x += vel_ratio*agentMsg.state.vel.x;
    point.y += vel_ratio*agentMsg.state.vel.y;
  } else{
    point.x += 0;
    point.y += 0;
  }
  marker_vel.points.push_back(point);
  marker_vel.color = color;
  marker_vel.color.a = 1.0;
  marker_vel.scale.x = 0.1;
  marker_vel.scale.y = 0.2;
  // marker_vel.scale.z = 0.0;
  marker_vec.push_back(marker_vel);

  // === Pref Vel === //
  arrow_length = std::pow((vel_ratio*agentMsg.state.prefVelocity.x),2) + std::pow((vel_ratio*agentMsg.state.prefVelocity.y),2);
  marker_vel.ns = ns_ + "/pref";
  marker_vel.points.clear();
  point.x = agentMsg.state.pos.x;
  point.y = agentMsg.state.pos.y;
  marker_vel.points.push_back(point);
  // avoid arrow appearing 'jumpy' when velocity is near-zero
  if (arrow_length > 0.0001){
    point.x += vel_ratio*agentMsg.state.prefVelocity.x;
    point.y += vel_ratio*agentMsg.state.prefVelocity.y;
  } else{
    point.x += 0;
    point.x += 0;
  }
  marker_vel.points.push_back(point);
  marker_vel.color.a = 0.5;
  marker_vec.push_back(marker_vel);


  // ==== VO ==== //
  marker_vec.push_back(toVoMarker(agentMsg));

  return marker_vec;
}
CrowdVisualizer::~CrowdVisualizer(){}

#endif /*__CROWD_VISUALIZER_HPP*/