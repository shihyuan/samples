/*
Author: Shih-Yuan Liu
*/

#ifndef __CROWD_SIMULATOR_HPP
#define __CROWD_SIMULATOR_HPP

#include <ros/ros.h>
#include <vector>

#include <rvo_ros/Agent.h>
#include <rvo_ros/AgentParam.h>
#include <rvo_ros/AgentState.h>
#include <rvo_ros/AgentInfo.h>
#include <rvo_ros/Obstacles.h>
#include <rvo_ros/Crowd.h>
#include <rvo_ros/VoInfo.h>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point.h>

#include "RVOSimulator.h"
#include "Agent.h"
#include "Obstacle.h"

#include "VoPolygon.hpp"

class CrowdSimulator: public RVO::RVOSimulator
{
public:
  size_t max_id_;
  CrowdSimulator();
  ~CrowdSimulator();
  void addAgent(const rvo_ros::Agent& agentMsg);
  void setObstacles(const rvo_ros::Obstacles& obstacles);
  void updateAgentState(RVO::Agent* agent, const rvo_ros::AgentState& state, bool update_pos, bool update_vel, bool update_prefVel, bool update_goals);
  void updateAgentParam(RVO::Agent* agent, const rvo_ros::AgentParam& param);
  void setCrowd(const rvo_ros::Crowd& crowd_msg);
  void setAgentDefaults(const rvo_ros::AgentParam& agent_param);
  void reset();
  rvo_ros::Agent toAgentMsg(const RVO::Agent& agent) const;
  rvo_ros::AgentState toAgentStateMsg(const RVO::Agent& agent) const;
  rvo_ros::AgentParam toAgentParamMsg(const RVO::Agent& agent) const;
  rvo_ros::AgentInfo toAgentInfoMsg(const RVO::Agent& agent) const;
  rvo_ros::Crowd getCrowd() const;
  rvo_ros::Obstacles getObstacles() const;

protected:
  void addObstacle(const geometry_msgs::Polygon& polygon);
  size_t getObsPolygon(size_t obsPointId, geometry_msgs::Polygon& poly) const;
};
/*----IMPL----*/
CrowdSimulator::CrowdSimulator(): max_id_(0){}
CrowdSimulator::~CrowdSimulator(){}

void CrowdSimulator::setAgentDefaults(const rvo_ros::AgentParam& agent_param)
{
  RVO::RVOSimulator::setAgentDefaults(
    agent_param.neighborDist,
    agent_param.maxNeighbors,
    agent_param.timeHorizon,
    agent_param.timeHorizonObst,
    agent_param.radius,
    agent_param.maxSpeed,
    agent_param.maxAcc,
    agent_param.lambdaValue);
}

void CrowdSimulator::setCrowd(const rvo_ros::Crowd& crowd_msg)
{
  if (crowd_msg.agents.size() == 0){
    return;
  }
  if (defaultAgent_ == NULL){
    setAgentDefaults(crowd_msg.agents[0].param);
  }
  for (int i = 0; i < crowd_msg.agents.size(); ++i){
    addAgent(crowd_msg.agents[i]);
  }
}

void CrowdSimulator::reset()
{
  RVO::RVOSimulator::clearAgents();
  RVO::RVOSimulator::clearObstacles();
  globalTime_ = 0.0;
  max_id_ = 0;
  if (defaultAgent_ != NULL){
    delete defaultAgent_;
  }
}

void CrowdSimulator::addObstacle(const geometry_msgs::Polygon& polygon)
{
  std::vector<RVO::Vector2> obs;
  for (std::vector<geometry_msgs::Point32>::const_iterator it = polygon.points.begin(); it != polygon.points.end(); ++it){
    obs.push_back(RVO::Vector2(it->x,
      it->y));
  }
  RVO::RVOSimulator::addObstacle(obs);
}
void CrowdSimulator::setObstacles(const rvo_ros::Obstacles& obstacles)
{
  /* Clear obstacles */
  RVO::RVOSimulator::clearObstacles();
  /* Add obstacles*/
  for (std::vector<geometry_msgs::Polygon>::const_iterator it = obstacles.obstacles.begin();
    it != obstacles.obstacles.end(); ++it){
    addObstacle(*it);
  }
  /* Build the obstacles */
  RVO::RVOSimulator::processObstacles();
}

void CrowdSimulator::updateAgentState(RVO::Agent* agent, const rvo_ros::AgentState& state, bool update_pos, bool update_vel, bool update_prefVel, bool update_goals)
{
  RVO::Vector2 pos(state.pos.x,state.pos.y);
  RVO::Vector2 vel(state.vel.x,state.vel.y);
  RVO::Vector2 prefVel(state.prefVelocity.x,state.prefVelocity.y);

  if (update_pos){
    agent->position_ = pos;
  }
  if (update_vel){
    agent->velocity_ = vel;
    agent->newVelocity_ = vel;
  }
  if (update_prefVel){
    agent->prefVelocity_ = prefVel;
  }
  if (update_goals){
    agent->goals_.clear();
    for (int i = 0; i < state.goals.size(); i++){
      RVO::Vector2 vec(state.goals[i].x,state.goals[i].y);
      agent->goals_.push_back(vec);
    }
  }
}

void CrowdSimulator::updateAgentParam(RVO::Agent* agent, const rvo_ros::AgentParam& param){
  agent->maxNeighbors_ = param.maxNeighbors;
  agent->maxSpeed_ = param.maxSpeed;
  agent->maxAcc_ = param.maxAcc;
  agent->neighborDist_ = param.neighborDist;
  agent->radius_ = param.radius;
  agent->timeHorizon_ = param.timeHorizon;
  agent->timeHorizonObst_ = param.timeHorizonObst;
  agent->lambdaValue_ = param.lambdaValue;
  agent->agentType_ = (RVO::Agent::AgentType) param.agentType;
  /* Set topology */
  agent->topoMap_.clear();
  for (int i = 0; i < param.topoKeys.size(); ++i){
    agent->topoMap_[param.topoKeys[i]] = param.topoValues[i];
  }
  /* Set lambda map*/
  agent->lambdaMap_.clear();
  for (int i = 0; i < param.lambdaKeys.size(); ++i){
    agent->lambdaMap_[param.lambdaKeys[i]] = param.lambdaValues[i];
  }

}

void CrowdSimulator::addAgent(const rvo_ros::Agent& agentMsg)
{
  RVO::Vector2 pos(agentMsg.state.pos.x,agentMsg.state.pos.y);
  RVO::Vector2 vel(agentMsg.state.vel.x,agentMsg.state.vel.y);
  RVO::Vector2 prefVel(agentMsg.state.prefVelocity.x,agentMsg.state.prefVelocity.y);
  RVO::Agent* agent = RVO::RVOSimulator::addAgent(agentMsg.id,pos);
  updateAgentState(agent, agentMsg.state, true, true, true, true);
  updateAgentParam(agent, agentMsg.param);
  if (agentMsg.id >= max_id_){
    max_id_ = agentMsg.id + 1;
  }
}

rvo_ros::AgentState CrowdSimulator::toAgentStateMsg(const RVO::Agent& agent) const
{
  rvo_ros::AgentState state;
  state.pos.x = (double) agent.position_.x();
  state.pos.y = (double) agent.position_.y();
  state.vel.x = (double) agent.velocity_.x();
  state.vel.y = (double) agent.velocity_.y();
  state.prefVelocity.x = (double) agent.prefVelocity_.x();
  state.prefVelocity.y = (double) agent.prefVelocity_.y();

  // Insert goals
  for (int i = 0; i < agent.goals_.size(); i++){
    geometry_msgs::Point point;
    point.x = agent.goals_[i].x();
    point.y = agent.goals_[i].y();
    state.goals.push_back(point);
  }

  return state;
}
rvo_ros::AgentParam CrowdSimulator::toAgentParamMsg(const RVO::Agent& agent) const
{
  rvo_ros::AgentParam param;
  param.radius =  agent.radius_;
  param.maxSpeed =  agent.maxSpeed_;
  param.maxAcc =  agent.maxAcc_;
  param.maxNeighbors =  agent.maxNeighbors_;
  param.neighborDist =  agent.neighborDist_;
  param.timeHorizon =  agent.timeHorizon_;
  param.timeHorizonObst =  agent.timeHorizonObst_;
  param.lambdaValue =  agent.lambdaValue_;
  param.agentType = agent.agentType_;

  /* topology */
  for (std::map<size_t, int>::const_iterator it_topo = agent.topoMap_.begin(); it_topo != agent.topoMap_.end(); ++it_topo){
    param.topoKeys.push_back(it_topo->first);
    param.topoValues.push_back(it_topo->second);
  }

  /* lambdas */
  for (std::map<size_t, float>::const_iterator it_lambda = agent.lambdaMap_.begin(); it_lambda != agent.lambdaMap_.end(); ++it_lambda){
    param.lambdaKeys.push_back(it_lambda->first);
    param.lambdaValues.push_back(it_lambda->second);
  }

  return param;
}

rvo_ros::AgentInfo CrowdSimulator::toAgentInfoMsg(const RVO::Agent& agent) const
{
  /* Info */
  rvo_ros::AgentInfo info;
  for (int i = 0; i < agent.agentNeighbors_.size();++i){
    info.agentNeighbors.push_back(agent.agentNeighbors_[i].second->id_);
    info.agentNeighborsDistance.push_back(agent.agentNeighbors_[i].first);
  }
  for (int i = 0; i < agent.voLines_.size();++i){
    rvo_ros::VoInfo voInfo;
    voInfo.lineDirection.x = agent.voLines_[i].direction.x();
    voInfo.lineDirection.y = agent.voLines_[i].direction.y();
    voInfo.linePoint.x = agent.voLines_[i].point.x();
    voInfo.linePoint.y = agent.voLines_[i].point.y();
    voInfo.dVector.x = agent.dVectors_[i].x();
    voInfo.dVector.y = agent.dVectors_[i].y();
    voInfo.dValue = agent.voLines_[i].direction*agent.dVectors_[i];
    info.agentNeighborsVoInfo.push_back(voInfo);
  } 
  for (int i = 0; i < agent.orcaLinesObs_.size();++i){
    rvo_ros::VoInfo voInfo;
    voInfo.lineDirection.x = agent.orcaLinesObs_[i].direction.x();
    voInfo.lineDirection.y = agent.orcaLinesObs_[i].direction.y();
    voInfo.linePoint.x = agent.orcaLinesObs_[i].point.x();
    voInfo.linePoint.y = agent.orcaLinesObs_[i].point.y();
    info.obstacleNeighborsVoInfo.push_back(voInfo);
  }    



  for (int i = 0; i < agent.obstacleNeighbors_.size();++i){
    info.obstacleNeighbors.push_back(agent.obstacleNeighbors_[i].second->id_);
    info.obstacleNeighborsDistance.push_back(agent.obstacleNeighbors_[i].first);
  }

  for (int i = 0; i < agent.inCollisionWith_.size();++i){
    info.inCollisionWith.push_back(agent.inCollisionWith_[i]);
  }
  /* Compute and fill the voPoly */
  VoPolygon voPoly(info,agent.maxSpeed_);
  info.voPoly = voPoly.getPolygon();
  return info;
}

rvo_ros::Agent CrowdSimulator::toAgentMsg(const RVO::Agent& agent) const
{
  rvo_ros::Agent msg;
  msg.id = agent.id_;
  msg.state = toAgentStateMsg(agent);
  msg.param = toAgentParamMsg(agent);
  msg.info = toAgentInfoMsg(agent);
  return msg;
}

/* TODO this does not populate frame_id and stamp */
rvo_ros::Crowd CrowdSimulator::getCrowd() const
{
  rvo_ros::Crowd msg;
  for (std::map<size_t, RVO::Agent* >::const_iterator it = agents_.begin(); it != agents_.end(); ++it){
    RVO::Agent& agent = *(it->second);
    msg.agents.push_back(toAgentMsg(agent));
    msg.agentIds.push_back(agent.id_);
  }
  return msg;
}

rvo_ros::Obstacles CrowdSimulator::getObstacles() const
{
  rvo_ros::Obstacles obstacles_msg;
  /* For-loop as safe while-loop */
  size_t obsId = 0;
  for (int i = 0; i < obstacles_.size(); i++){
    geometry_msgs::Polygon poly;
    obsId = getObsPolygon(obsId,poly);
    if (poly.points.size() > 0){
      obstacles_msg.obstacles.push_back(poly);
    }
    else{
      break;
    }
  }
  return obstacles_msg;
}

size_t CrowdSimulator::getObsPolygon(size_t obsPointId, geometry_msgs::Polygon& poly) const
{
  if (obsPointId >= obstacles_.size()){
    /* Reach the last of the obstacle point */
    return obsPointId;
  }
  /* for-loop as safer while-loop */
  size_t obsPointId_temp = obsPointId; 
  size_t max_id = obsPointId;
  for (int i = 0; i < obstacles_.size(); ++i){
    geometry_msgs::Point32 point;
    RVO::Obstacle* obstacle = obstacles_[obsPointId_temp]; 
    point.x = obstacle->point_.x();
    point.y = obstacle->point_.y();
    poly.points.push_back(point);
    /* Record largest id */
    if (obsPointId_temp > max_id){
      max_id = obsPointId_temp;
    }
    /* Move on to next point */
    obsPointId_temp = obstacle->nextObstacle_->id_;
    if (obsPointId_temp == obsPointId){
      break;
    }
  }
  /* Return the possible starting id of the next obstacle*/
  return max_id+1;
}


#endif /*__CROWD_SIMULATOR_HPP*/