/*
Adapted from the RVO2 Library ( https://github.com/snape/RVO2 ) by Shih-Yuan Liu
*/

/*
 * RVOSimulator.cpp
 * RVO2 Library
 *
 * Copyright (c) 2008-2010 University of North Carolina at Chapel Hill.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for educational, research, and non-profit purposes, without
 * fee, and without a written agreement is hereby granted, provided that the
 * above copyright notice, this paragraph, and the following four paragraphs
 * appear in all copies.
 *
 * Permission to incorporate this software into commercial products may be
 * obtained by contacting the Office of Technology Development at the University
 * of North Carolina at Chapel Hill <otd@unc.edu>.
 *
 * This software program and documentation are copyrighted by the University of
 * North Carolina at Chapel Hill. The software program and documentation are
 * supplied "as is," without any accompanying services from the University of
 * North Carolina at Chapel Hill or the authors. The University of North
 * Carolina at Chapel Hill and the authors do not warrant that the operation of
 * the program will be uninterrupted or error-free. The end-user understands
 * that the program was developed for research purposes and is advised not to
 * rely exclusively on the program for any reason.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
 * AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS
 * SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT
 * CHAPEL HILL OR THE AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
 * DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY
 * STATUTORY WARRANTY OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON
 * AN "AS IS" BASIS, AND THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE
 * AUTHORS HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
 * ENHANCEMENTS, OR MODIFICATIONS.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

#include "RVOSimulator.h"

#include "Agent.h"
#include "KdTree.h"
#include "Obstacle.h"

#include <iostream>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace RVO {
  RVOSimulator::RVOSimulator() : defaultAgent_(NULL), globalTime_(0.0f), kdTree_(NULL), timeStep_(0.0f), use_max_acc_(true)
  {
    kdTree_ = new KdTree(this);
  }

  // RVOSimulator::RVOSimulator(float timeStep, float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, float maxAcc, const Vector2 &velocity) : defaultAgent_(NULL), globalTime_(0.0f), kdTree_(NULL), timeStep_(timeStep)
  // {
  //  kdTree_ = new KdTree(this);
  //  defaultAgent_ = new Agent(this);

  //  defaultAgent_->maxNeighbors_ = maxNeighbors;
  //  defaultAgent_->maxSpeed_ = maxSpeed;
  //  defaultAgent_->maxAcc_ = maxAcc;
  //  defaultAgent_->neighborDist_ = neighborDist;
  //  defaultAgent_->radius_ = radius;
  //  defaultAgent_->timeHorizon_ = timeHorizon;
  //  defaultAgent_->timeHorizonObst_ = timeHorizonObst;
  //  defaultAgent_->velocity_ = velocity;
  // }


  RVOSimulator::~RVOSimulator()
  {
    if (defaultAgent_ != NULL) {
      delete defaultAgent_;
    }

    for (size_t i = 0; i < agents_.size(); ++i) {
      delete agents_[i];
    }

    clearObstacles();
    // for (size_t i = 0; i < obstacles_.size(); ++i) {
    //  delete obstacles_[i];
    // }

    delete kdTree_;
  }
  void RVOSimulator::clearObstacles()
  {
    for (size_t i = 0; i < obstacles_.size(); ++i) {
      delete obstacles_[i];
    }
    obstacles_.clear();
  }

  Agent* RVOSimulator::addAgent(size_t agent_id, const Vector2 &position)
  {
    if (defaultAgent_ == NULL) {
    // assert(defaultAgent_ != NULL);
      return NULL;
    }

    Agent *agent;
    if (hasAgent(agent_id)){
      agent = agents_[agent_id];
    }
    else{
      agent = new Agent(this);
      agents_[agent_id] = agent;
    }

    agent->position_ = position;
    agent->maxNeighbors_ = defaultAgent_->maxNeighbors_;
    agent->maxSpeed_ = defaultAgent_->maxSpeed_;
    agent->maxAcc_ = defaultAgent_->maxAcc_;
    agent->neighborDist_ = defaultAgent_->neighborDist_;
    agent->radius_ = defaultAgent_->radius_;
    agent->timeHorizon_ = defaultAgent_->timeHorizon_;
    agent->timeHorizonObst_ = defaultAgent_->timeHorizonObst_;
    agent->velocity_ = defaultAgent_->velocity_;
    agent->id_ = agent_id;

    return agent;
  }

  void RVOSimulator::clearAgents()
  {
    for (std::map<size_t, Agent* >::iterator it = agents_.begin(); it != agents_.end();){
      delete it->second;
    }
    agents_.clear();
  }

  bool RVOSimulator::removeAgent(const size_t& agent_id)
  {
    std::map<size_t, Agent* >::iterator it = agents_.find(agent_id);
    if (it != agents_.end()){
      delete it->second;
      agents_.erase(it);
      // std::cout << "[RVOSimulator] removed agent " << agent_id << std::endl;
      return true;
    }
    else{
      return false;
    }
  }

  size_t RVOSimulator::addObstacle(const std::vector<Vector2> &vertices)
  {
    if (vertices.size() < 2) {
      return RVO_ERROR;
    }

    const size_t obstacleNo = obstacles_.size();

    for (size_t i = 0; i < vertices.size(); ++i) {
      Obstacle *obstacle = new Obstacle();
      obstacle->point_ = vertices[i];

      /* Connect to previous */
      if (i != 0) {
        obstacle->prevObstacle_ = obstacles_.back();
        obstacle->prevObstacle_->nextObstacle_ = obstacle;
      }

      /* last point, connect to the first point (of this set) */
      if (i == vertices.size() - 1) {
        obstacle->nextObstacle_ = obstacles_[obstacleNo];
        obstacle->nextObstacle_->prevObstacle_ = obstacle;
      }

      /* unitDir_ pointing to the next vertices (or the first if there is no next )*/
      obstacle->unitDir_ = normalize(vertices[(i == vertices.size() - 1 ? 0 : i + 1)] - vertices[i]);

      if (vertices.size() == 2) {
        /* Just a line segment */
        obstacle->isConvex_ = true;
      }
      else {
        /* Check if the current point is left of the previous segment? */
        obstacle->isConvex_ = (leftOf(vertices[(i == 0 ? vertices.size() - 1 : i - 1)], vertices[i], vertices[(i == vertices.size() - 1 ? 0 : i + 1)]) >= 0.0f);
      }

      obstacle->id_ = obstacles_.size();

      obstacles_.push_back(obstacle);
    }

    return obstacleNo;
  }

  void RVOSimulator::computeNewVelocity(RVO::Agent* agent)
  {
    if (agent != NULL){
      kdTree_->buildAgentTree();
      agent->computeNeighbors();
      agent->computeNewVelocity();      
    }
    /* TODO what else? */
  }


  void RVOSimulator::computeNewVelocity(){
    kdTree_->buildAgentTree();
    // std::cout << "[doStep] AgentTree built." << std::endl;
    //HACK: Build vector of Agent* for possible _OPENMP. TODO: Check if necessary 
    std::vector<Agent* > agent_vec;
    for( std::map<size_t, Agent* >::iterator it = agents_.begin(); it != agents_.end(); ++it ){
      agent_vec.push_back(it->second);
    }
    // std::cout << "[doStep] vector built." << std::endl;

#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < agent_vec.size(); ++i) {
      agent_vec[i]->computeNeighbors();
      agent_vec[i]->computeNewVelocity();
      agent_vec[i]->updateVel(use_max_acc_);
    }
  }

  void RVOSimulator::moveAgents(){
    std::vector<Agent* > agent_vec;
    for( std::map<size_t, Agent* >::iterator it = agents_.begin(); it != agents_.end(); ++it ){
      agent_vec.push_back(it->second);
    }    
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < agent_vec.size(); ++i) {
      agent_vec[i]->updatePos();
    }
    // std::cout << "[doStep] updated. " << std::endl;
    globalTime_ += timeStep_;    
  }

  void RVOSimulator::doStep()
  {
    computeNewVelocity();
    moveAgents();
  }

  bool RVOSimulator::hasAgent(size_t agent_id) const
  {
    std::map<size_t, Agent* >::const_iterator it = agents_.find(agent_id);
    return it != agents_.end();
  }

  // size_t RVOSimulator::getAgentAgentNeighbor(size_t agentNo, size_t neighborNo) const
  // {
  //  return agents_[agentNo]->agentNeighbors_[neighborNo].second->id_;
  // }

  // size_t RVOSimulator::getAgentMaxNeighbors(size_t agentNo) const
  // {
  //  return agents_[agentNo]->maxNeighbors_;
  // }

  // float RVOSimulator::getAgentMaxSpeed(size_t agentNo) const
  // {
  //  return agents_[agentNo]->maxSpeed_;
  // }

  // float RVOSimulator::getAgentNeighborDist(size_t agentNo) const
  // {
  //  return agents_[agentNo]->neighborDist_;
  // }

  // size_t RVOSimulator::getAgentNumAgentNeighbors(size_t agentNo) const
  // {
  //  return agents_[agentNo]->agentNeighbors_.size();
  // }

  // size_t RVOSimulator::getAgentNumObstacleNeighbors(size_t agentNo) const
  // {
  //  return agents_[agentNo]->obstacleNeighbors_.size();
  // }

  // size_t RVOSimulator::getAgentNumORCALines(size_t agentNo) const
  // {
  //  return agents_[agentNo]->orcaLines_.size();
  // }

  // size_t RVOSimulator::getAgentObstacleNeighbor(size_t agentNo, size_t neighborNo) const
  // {
  //  return agents_[agentNo]->obstacleNeighbors_[neighborNo].second->id_;
  // }

  // const Line &RVOSimulator::getAgentORCALine(size_t agentNo, size_t lineNo) const
  // {
  //  return agents_[agentNo]->orcaLines_[lineNo];
  // }

  // const Vector2 &RVOSimulator::getAgentPosition(size_t agentNo) const
  // {
  //  return agents_[agentNo]->position_;
  // }

  // const Vector2 &RVOSimulator::getAgentPrefVelocity(size_t agentNo) const
  // {
  //  return agents_[agentNo]->prefVelocity_;
  // }

  // float RVOSimulator::getAgentRadius(size_t agentNo) const
  // {
  //  return agents_[agentNo]->radius_;
  // }

  // float RVOSimulator::getAgentTimeHorizon(size_t agentNo) const
  // {
  //  return agents_[agentNo]->timeHorizon_;
  // }

  // float RVOSimulator::getAgentTimeHorizonObst(size_t agentNo) const
  // {
  //  return agents_[agentNo]->timeHorizonObst_;
  // }

  // const Vector2 &RVOSimulator::getAgentVelocity(size_t agentNo) const
  // {
  //  return agents_[agentNo]->velocity_;
  // }

  Agent* RVOSimulator::getAgent(const size_t& agent_key)
  {
    std::map<size_t, Agent* >::iterator it = agents_.find(agent_key);
    if (it != agents_.end()){
      return it->second;
    }
    else{
      return NULL;
    }
  }

  float RVOSimulator::getGlobalTime() const
  {
    return globalTime_;
  }

  size_t RVOSimulator::getNumAgents() const
  {
    return agents_.size();
  }

  size_t RVOSimulator::getNumObstacleVertices() const
  {
    return obstacles_.size();
  }

  const Vector2 &RVOSimulator::getObstacleVertex(size_t vertexNo) const
  {
    return obstacles_[vertexNo]->point_;
  }

  size_t RVOSimulator::getNextObstacleVertexNo(size_t vertexNo) const
  {
    return obstacles_[vertexNo]->nextObstacle_->id_;
  }

  size_t RVOSimulator::getPrevObstacleVertexNo(size_t vertexNo) const
  {
    return obstacles_[vertexNo]->prevObstacle_->id_;
  }

  float RVOSimulator::getTimeStep() const
  {
    return timeStep_;
  }

  void RVOSimulator::processObstacles()
  {
    kdTree_->buildObstacleTree();
  }

  bool RVOSimulator::queryVisibility(const Vector2 &point1, const Vector2 &point2, float radius) const
  {
    return kdTree_->queryVisibility(point1, point2, radius);
  }

  void RVOSimulator::setAgentDefaults(float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, float maxAcc, float lambdaValue)
  {
    if (defaultAgent_ == NULL) {
      defaultAgent_ = new Agent(this);
    }

    defaultAgent_->maxNeighbors_ = maxNeighbors;
    defaultAgent_->maxSpeed_ = maxSpeed;
    defaultAgent_->maxAcc_ = maxAcc;
    defaultAgent_->neighborDist_ = neighborDist;
    defaultAgent_->radius_ = radius;
    defaultAgent_->timeHorizon_ = timeHorizon;
    defaultAgent_->timeHorizonObst_ = timeHorizonObst;
    defaultAgent_->lambdaValue_ = lambdaValue;
    defaultAgent_->velocity_ = Vector2(0.0,0.0);
  }

  void RVOSimulator::setAgentMaxNeighbors(size_t agentNo, size_t maxNeighbors)
  {
    agents_[agentNo]->maxNeighbors_ = maxNeighbors;
  }

  void RVOSimulator::setAgentMaxSpeed(size_t agentNo, float maxSpeed)
  {
    agents_[agentNo]->maxSpeed_ = maxSpeed;
  }

  void RVOSimulator::setAgentNeighborDist(size_t agentNo, float neighborDist)
  {
    agents_[agentNo]->neighborDist_ = neighborDist;
  }

  void RVOSimulator::setAgentPosition(size_t agentNo, const Vector2 &position)
  {
    agents_[agentNo]->position_ = position;
  }

  void RVOSimulator::setAgentPrefVelocity(size_t agentNo, const Vector2 &prefVelocity)
  {
    agents_[agentNo]->prefVelocity_ = prefVelocity;
  }

  void RVOSimulator::setAgentRadius(size_t agentNo, float radius)
  {
    agents_[agentNo]->radius_ = radius;
  }

  void RVOSimulator::setAgentTimeHorizon(size_t agentNo, float timeHorizon)
  {
    agents_[agentNo]->timeHorizon_ = timeHorizon;
  }

  void RVOSimulator::setAgentTimeHorizonObst(size_t agentNo, float timeHorizonObst)
  {
    agents_[agentNo]->timeHorizonObst_ = timeHorizonObst;
  }
  
  void RVOSimulator::setAgentVelocity(size_t agentNo, const Vector2 &velocity)
  {
    agents_[agentNo]->velocity_ = velocity;
  }
  
  void RVOSimulator::setTimeStep(float timeStep)
  {
    timeStep_ = timeStep;
  }
}
