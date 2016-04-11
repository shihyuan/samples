/*
Author: Shih-Yuan Liu
*/


#ifndef __RVOPARAM_LOADER_HPP
#define __RVOPARAM_LOADER_HPP

#include <ros/ros.h>
#include <vector>
#include <map>
#include <rvo_ros/AgentParam.h>

class RvoParamLoader
{
public:
  ros::NodeHandle nh_;
  std::map<std::string, rvo_ros::AgentParam> rvo_param_map_;
  
  RvoParamLoader(const ros::NodeHandle nh): nh_(nh) {}
  ~RvoParamLoader(){}
  
  bool getParam(const std::string& veh_type, rvo_ros::AgentParam& param)
  {
    std::map<std::string, rvo_ros::AgentParam>::iterator it = rvo_param_map_.find(veh_type);
    if (it == rvo_param_map_.end()){
      return false;
    }
    else{
      param = it->second;
      return true;
    }
  }

  void loadVehParam(const std::vector<std::string>& veh_type_list){
    // Populate the rvo_param_map
    for (std::vector<std::string>::const_iterator it = veh_type_list.begin(); it != veh_type_list.end(); ++it){
      loadVehParam(*it);
    }    
  }

  void loadVehParam(const std::string& veh_type)
  {
    if (not nh_.hasParam(veh_type)){
      // Does not have such vehicle type defined
      return;
    }
    rvo_ros::AgentParam param;
    double neighborDist; nh_.param<double>(veh_type + "/neighborDist", neighborDist, 10);
    int maxNeighbors; nh_.param<int>(veh_type + "/maxNeighbors", maxNeighbors, 25);
    double timeHorizon; nh_.param<double>(veh_type + "/timeHorizon", timeHorizon, 1.5);
    double timeHorizonObst; nh_.param<double>(veh_type + "/timeHorizonObst", timeHorizonObst, 1.2);
    double radius; nh_.param<double>(veh_type + "/radius", radius, 0.4);
    double maxSpeed; nh_.param<double>(veh_type + "/maxSpeed", maxSpeed, 1.2);
    double maxAcc; nh_.param<double>(veh_type + "/maxAcc", maxAcc, 10.0);
    double lambdaValue; nh_.param<double>(veh_type + "/lambdaValue", lambdaValue, 0.6);
    
    param.agentType=rvo_ros::AgentParam::NORMAL;
    param.neighborDist = neighborDist;
    param.maxNeighbors = maxNeighbors;
    param.timeHorizon = timeHorizon;
    param.timeHorizonObst = timeHorizonObst;
    param.radius = radius;
    param.maxSpeed = maxSpeed;
    param.maxAcc = maxAcc;
    param.lambdaValue = lambdaValue;
    rvo_param_map_[veh_type] = param;

    // ROS_INFO_STREAM("[loadVehParam]:" << veh_type << ":" << param);
  }
};

#endif /*__RVOPARAM_LOADER_HPP*/