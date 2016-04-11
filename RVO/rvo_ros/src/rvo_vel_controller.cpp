/*
Author: Shih-Yuan Liu
*/

#include <ros/ros.h>
#include <vector>
#include <map>
#include <sstream>
#include <algorithm>

#include <rvo_ros/Agent.h>
#include <rvo_ros/AgentParam.h>
#include <rvo_ros/AgentState.h>
#include <rvo_ros/AgentInfo.h>
#include <rvo_ros/Obstacles.h>
#include <rvo_ros/Crowd.h>
#include <rvo_ros/VoInfo.h>

#include <rvo_ros/AddAgentState.h>
#include <rvo_ros/UpdateAgentState.h>
#include <rvo_ros/RemoveAgentId.h>
#include <rvo_ros/AddAgent.h>
#include <rvo_ros/SetObstacles.h>
#include <std_srvs/Empty.h>
#include <CrowdSimulator.hpp>
#include <RvoParamLoader.hpp>
#include "tf/transform_datatypes.h"

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
// #include <acl_msgs/Waypoint.h>
#include <acl_msgs/ViconState.h>
#include <acl_msgs/QuadGoal.h>
#include <acl_msgs/QuadWaypoint.h>
#include <acl_msgs/QuadWaypointError.h>
#include <acl_msgs/VehicleList.h>

class QuadAgent{
public:
  std::string quad_name_;
  int quad_id_;
  CrowdSimulator* sim_;
  ros::NodeHandle nh;
  ros::Subscriber sub_quad_vicon_;  

  ros::Subscriber sub_quad_pref_vel_;
  ros::Subscriber sub_quad_waypoint;

  ros::Publisher pub_rvo_vel_;
  ros::Publisher pub_quad_goal_;
  ros::Publisher pub_waypoint_err_;

  acl_msgs::ViconState quad_state_;
  geometry_msgs::Vector3Stamped quad_pref_vel_;
  acl_msgs::QuadWaypoint quad_waypoint_;

  rvo_ros::AgentParam param_;

  bool inserted_;

  QuadAgent(const std::string& quad_name, CrowdSimulator* sim, const rvo_ros::AgentParam& param): quad_name_(quad_name), sim_(sim), param_(param)
  {    
    inserted_ = false;
    // Initialze publisher
    pub_rvo_vel_ = nh.advertise<geometry_msgs::Vector3Stamped>(quad_name+"/rvo_vel",1);
    pub_quad_goal_ = nh.advertise<acl_msgs::QuadGoal>(quad_name+"/goal",1);
    pub_waypoint_err_ = nh.advertise<acl_msgs::QuadWaypointError>(quad_name+"/waypoint_err",1);
    // Initialze subscriber
    sub_quad_vicon_ = nh.subscribe(quad_name + "/vicon",1,&QuadAgent::cbVicon, this);
    sub_quad_pref_vel_ = nh.subscribe(quad_name_+"/pref_vel",1, &QuadAgent::cbPrefVel, this); // TODO is this subscriber needed? seems dead.
    sub_quad_waypoint = nh.subscribe(quad_name_+"/waypoint",1, &QuadAgent::cbWaypoint, this);
  }
  ~QuadAgent(){
    sim_->removeAgent(quad_id_);
  }

  void cbVicon(const acl_msgs::ViconState& vicon_state_msg){
    quad_state_ = vicon_state_msg;
    if (quad_waypoint_.header.stamp == ros::Time(0)){
      quad_waypoint_.pose = quad_state_.pose;
      quad_waypoint_.header = quad_state_.header;
      quad_waypoint_.mode = acl_msgs::QuadWaypoint::MODE_DISABLED;
    }
  }

  void cbPrefVel(const geometry_msgs::Vector3Stamped& pref_vel_msg){
    quad_pref_vel_ = pref_vel_msg;
  }

  void cbWaypoint(const acl_msgs::QuadWaypoint& quad_waypoint_msg){
    quad_waypoint_ = quad_waypoint_msg;
    // If it's a landing waypoint, overwrite z with current z.
    if (quad_waypoint_.mode == acl_msgs::QuadWaypoint::MODE_LAND){
      quad_waypoint_.pose.position.z = quad_state_.pose.position.z;
    }

  }

  bool insertAgent(){
    rvo_ros::Agent agentMsg;
    agentMsg.id = sim_->max_id_;
    agentMsg.state = getAgentStateFromMsg();
    agentMsg.param = param_;
    sim_->addAgent(agentMsg);
    quad_id_ = agentMsg.id;
    inserted_ = true;
    return true;
  }

  bool updateSim(){
    // Do nothing if no messages yet
    if (not hasMessages()){
      return false;
    } 
    // Insert 
    if (not inserted_) insertAgent();

    RVO::Agent* agent = sim_->getAgent(quad_id_);
    if (agent == NULL){
      // No agent with matching id in simulator. Update failed.
      return false;
    }
    else{
      sim_->updateAgentState(agent,getAgentStateFromMsg(),true,true,true,true);
      return true;       
    }
  }

  void publishRvoVel(){
    if (not inserted_){
      return;
    }
    rvo_ros::Agent agent = getAgentMsgFromSim();
    // ROS_INFO_STREAM(agent.state);
    geometry_msgs::Vector3Stamped rvo_vel_msg;
    rvo_vel_msg.header.stamp = quad_state_.header.stamp;
    rvo_vel_msg.vector = agent.state.vel;
    // pub_quad_goal_.publish(getQuadGoal(agent.state.vel,agent.state.pos));
    if (hasMessages()){
      if (quad_waypoint_.mode != acl_msgs::QuadWaypoint::MODE_DISABLED){
        pub_quad_goal_.publish(getQuadGoal(agent.state));
        pub_rvo_vel_.publish(rvo_vel_msg);
      }
    }
  }

  void publishWaypointError(){
    if (hasMessages()){
      acl_msgs::QuadWaypointError err_msg;
      err_msg.header.stamp = quad_state_.header.stamp;
      err_msg.header.frame_id = quad_name_ + "/trans";
      err_msg.waypoint_stamp = quad_waypoint_.header.stamp;
      err_msg.pos_error.x = quad_waypoint_.pose.position.x - quad_state_.pose.position.x;
      err_msg.pos_error.y = quad_waypoint_.pose.position.y - quad_state_.pose.position.y;
      err_msg.pos_error.z = quad_waypoint_.pose.position.z - quad_state_.pose.position.z;
      if (quad_waypoint_.mode == acl_msgs::QuadWaypoint::MODE_LAND){
        err_msg.pos_error.z = -1.0;
      }
      pub_waypoint_err_.publish(err_msg);
    }
  }

  acl_msgs::QuadGoal getQuadGoal(const rvo_ros::AgentState& agent_state){
    if (quad_waypoint_.mode == acl_msgs::QuadWaypoint::MODE_LAND){
      quad_waypoint_.pose.position.z = quad_waypoint_.pose.position.z - 0.005;
    }

    acl_msgs::QuadGoal quad_goal;
    quad_goal.header.stamp = ros::Time::now();
    quad_goal.header.frame_id = "vicon";
    quad_goal.vel = agent_state.vel;
    quad_goal.pos.x = agent_state.pos.x;
    quad_goal.pos.y = agent_state.pos.y;
    quad_goal.pos.z = quad_waypoint_.pose.position.z;


    tf::Quaternion quat;
    tf::quaternionMsgToTF(quad_waypoint_.pose.orientation, quat);
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    quad_goal.yaw = yaw;

    if (quad_waypoint_.mode == acl_msgs::QuadWaypoint::MODE_KILL){
      // quad_goal.waypointType = 2; //Kill motor
      quad_goal.cut_power = true;
    }
    else if (quad_waypoint_.mode == acl_msgs::QuadWaypoint::MODE_LAND && quad_waypoint_.pose.position.z < -0.1){
      quad_goal.cut_power = true;
    }
    else{
      quad_goal.cut_power = false;
      // quad_goal.waypointType = 1; //TAKEOFF
    }

    quad_goal.mode = acl_msgs::QuadGoal::MODE_VEL;
    return quad_goal;
  }

  geometry_msgs::Vector3Stamped getPrefVelFromWaypoint(){
    geometry_msgs::Vector3Stamped msg;
    // Compute preferred velocity based on current position and waypoint
    RVO::Vector2 waypoint(quad_waypoint_.pose.position.x,quad_waypoint_.pose.position.y);
    RVO::Vector2 pos(quad_state_.pose.position.x,quad_state_.pose.position.y);
    if (quad_waypoint_.mode != acl_msgs::QuadWaypoint::MODE_TAKEOFF){
      return msg; //Zero preferred velocity if not in takeoff mode
    }
    RVO::Vector2 pos_err_vec = waypoint - pos;
    float speed = RVO::abs(pos_err_vec);
    if (speed > param_.maxSpeed){
      speed = param_.maxSpeed;
    }
    RVO::Vector2 pref_vel = speed*RVO::normalize(pos_err_vec);
    msg.vector.x = pref_vel.x();
    msg.vector.y = pref_vel.y();
    return msg;
  }

  rvo_ros::AgentState getAgentStateFromMsg(){
    rvo_ros::AgentState state;
    state.pos = quad_state_.pose.position;
    state.vel = quad_state_.twist.linear;
    state.prefVelocity = getPrefVelFromWaypoint().vector;
    return state;
  }

  bool hasMessages(){
    return quad_state_.header.stamp > ros::Time(0) && quad_waypoint_.header.stamp > ros::Time(0);
  }

  rvo_ros::Agent getAgentMsgFromSim(){
    RVO::Agent agent = *(sim_->getAgent(quad_id_));
    // agent.velocity_ = agent.newVelocity_;
    return sim_->toAgentMsg(agent);
  }
};

class RVOVelControllerNode{
public:
  ros::NodeHandle nh_p_;
  CrowdSimulator sim_;
  RvoParamLoader param_loader_;
  std::string node_name_;

  /* Publisher */
  ros::Publisher pub_crowd_;
  ros::Publisher pub_obs_;

  /* Subscriber */
  ros::Subscriber sub_veh_list_;

  /* Srv */
  ros::ServiceServer srv_add_state_;
  ros::ServiceServer srv_add_agent_;
  ros::ServiceServer srv_update_state_;
  ros::ServiceServer srv_set_obstacles_;
  ros::ServiceServer srv_remove_;
  ros::ServiceServer srv_start_;
  ros::ServiceServer srv_pause_;

  std::map<std::string , QuadAgent*> quad_agent_map_;

  /* Timer */
  ros::Timer timer_sim_;
  ros::Time time_init_;

  /* Sim Parameters */
  float timeStep_;
  /* Default Agent Parameters*/
  float neighborDist_;
  int maxNeighbors_;
  float timeHorizon_;
  float timeHorizonObst_;
  float radius_;
  float maxSpeed_;
  float maxAcc_;
  float lambdaValue_;
  RVO::Vector2 velocity_;

  std::string frame_id_;

  RVOVelControllerNode(const ros::NodeHandle& nh):nh_p_(nh),param_loader_(nh)
  {
    setDefaultParams();
    getParams();
    node_name_ = ros::this_node::getName();
    // sim_.setAgentDefaults(neighborDist_,maxNeighbors_,timeHorizon_,timeHorizonObst_, radius_, maxSpeed_, maxAcc_, lambdaValue_);
    sim_.setAgentDefaults(getDefaultAgentParam());
    sim_.setTimeStep(timeStep_);

    /* TODO handle obstacles */
    // loadObstacles(rvo_ros::Obstacles());
    sim_.setObstacles(rvo_ros::Obstacles());

    /* Advertise Services */
    srv_add_state_ = nh_p_.advertiseService("add_agent_state", &RVOVelControllerNode::cbAddAgentState, this);
    srv_add_agent_ = nh_p_.advertiseService("add_agent", &RVOVelControllerNode::cbAddAgent, this);
    srv_update_state_ = nh_p_.advertiseService("update_agent_state", &RVOVelControllerNode::cbUpdateAgentState, this);
    srv_remove_ = nh_p_.advertiseService("remove_agent_id", &RVOVelControllerNode::cbRemoveAgentId, this);
    srv_set_obstacles_ = nh_p_.advertiseService("set_obstacles", &RVOVelControllerNode::cbSetObstacles, this);
    srv_start_ = nh_p_.advertiseService("start", &RVOVelControllerNode::cbStart, this);
    srv_pause_ = nh_p_.advertiseService("pause", &RVOVelControllerNode::cbPause, this);
    /* Advertise obstacles */
    pub_crowd_ = nh_p_.advertise<rvo_ros::Crowd>("crowd",1,true);
    pub_obs_ = nh_p_.advertise<rvo_ros::Obstacles>("obstacles",1,true);
    /* Timer */
    time_init_ = ros::Time::now();
    timer_sim_ = nh_p_.createTimer(ros::Duration(timeStep_), &RVOVelControllerNode::cbStep, this);
    // timer_sim_.stop();

    /* Subscription */
    // sub_user_agent_ = nh_p_.subscribe("user_agent", 10, &RVOVelControllerNode::cbUserAgent, this);
    sub_veh_list_ = nh_p_.subscribe("vehicle_list",1,&RVOVelControllerNode::cbVehList, this);
  }
  ~RVOVelControllerNode(){}


  void cbVehList(const acl_msgs::VehicleList& veh_list_msg)
  {
    for (int i = 0; i < veh_list_msg.vehicle_names.size(); ++i){
      std::string veh_name = veh_list_msg.vehicle_names.at(i);
      std::map<std::string, QuadAgent*>::iterator it = quad_agent_map_.find(veh_name);
      if (it == quad_agent_map_.end()){
        // Add new agent
        rvo_ros::AgentParam param;
        // Only add if veh type defined in yaml
        if (param_loader_.getParam(veh_name.substr(0,2),param))
        {
          // param = getDefaultAgentParam();
          quad_agent_map_[veh_name] = new QuadAgent(veh_name,&sim_,param);
          ROS_INFO_STREAM("[rvo_vel_controller] Add agent: " << veh_name);
        }
        // ROS_INFO_STREAM("[rvo_vel_controller]" << param);
      }
    }

    // remove agents that's no longer there
    for(std::map<std::string, QuadAgent* >::iterator it =  quad_agent_map_.begin(); it != quad_agent_map_.end();){
      std::string veh_name = it->first;
      if (std::find(veh_list_msg.vehicle_names.begin(),veh_list_msg.vehicle_names.end(),veh_name) == veh_list_msg.vehicle_names.end()){
        ROS_INFO_STREAM("[rvo_vel_controller] Remove agent: " << it->first);
        delete it->second;
        quad_agent_map_.erase(it++);
      }
      else{
        ++it;
      }
    }
    // Publish the obstacles
    pub_obs_.publish(sim_.getObstacles());

  }

  rvo_ros::AgentParam getDefaultAgentParam(){
    rvo_ros::AgentParam param;
    param_loader_.getParam("BQ",param);
    return param;
  }

  void setDefaultParams()
  {
    if (!ros::param::has("~timeStep")) { ros::param::set("~timeStep",0.02);}
    if (!ros::param::has("~frame_id")) { ros::param::set("~frame_id","/vicon");}
  }
  void getParams()
  {
    std::vector<std::string> veh_type_list;
    ros::param::get("~veh_type_list",veh_type_list);
    param_loader_.loadVehParam(veh_type_list);
    ros::param::getCached("~timeStep",timeStep_);
    ros::param::getCached("~frame_id",frame_id_);
  }

  bool cbSetObstacles(rvo_ros::SetObstacles::Request& request, rvo_ros::SetObstacles::Response& response)
  {
    rvo_ros::Obstacles obs_msg;
    obs_msg.obstacles = request.obstacles;
    sim_.setObstacles(obs_msg);
    pub_obs_.publish(sim_.getObstacles());
    /* Publish obstacle msg when it's set */
    return true;
  }

  void publishCrowd()
  {
    // ROS_INFO_STREAM("[publishCrowd]");
    rvo_ros::Crowd crowd = sim_.getCrowd();
    crowd.header.frame_id = frame_id_;
    // crowd.header.stamp = timerEvent.current_expected;
    crowd.header.stamp = time_init_ + ros::Duration(sim_.getGlobalTime());
    pub_crowd_.publish(crowd);    
  }


  void updateSimByCurrentState(){
    // for(std::vector<QuadAgent*>::iterator it = quad_agent_vec.begin(); it != quad_agent_vec.end(); ++it){
    //   (*it)->updateSim();
    // }
    for(std::map<std::string, QuadAgent* >::iterator it =  quad_agent_map_.begin(); it != quad_agent_map_.end(); ++it){
      it->second->updateSim();
    }

  }

  void publishRvo(){
    for(std::map<std::string, QuadAgent* >::iterator it =  quad_agent_map_.begin(); it != quad_agent_map_.end(); ++it){
      it->second->publishRvoVel();
      it->second->publishWaypointError();
    }
  }

  void cbStep(const ros::TimerEvent& timerEvent)
  {
    // ROS_INFO_STREAM("[cbStep]");
    updateSimByCurrentState();
    sim_.computeNewVelocity();
    publishRvo();
    sim_.moveAgents();
    publishCrowd();
  }

  bool cbAddAgent(rvo_ros::AddAgent::Request& request, rvo_ros::AddAgent::Response& response)
  {
    sim_.addAgent(request.agent);
    response.id = request.agent.id;
    // ROS_INFO_STREAM("[CrowdNode]: Add agent " << request.agent.id);
    ROS_INFO("[%s]: Add agent %d", node_name_.c_str(), request.agent.id);
    publishCrowd();
    return true;
  }

  bool cbUpdateAgentState(rvo_ros::UpdateAgentState::Request& request, rvo_ros::UpdateAgentState::Response& response){
    // ROS_INFO_STREAM("[CrowdNode] UpdateAgentState called from agent " << request.id );
    RVO::Agent* agent = sim_.getAgent(request.id);
    if (agent == NULL){
      response.has_agent = false;
    }
    else{
      response.has_agent = true;
      sim_.updateAgentState(agent,request.state,request.update_pos,request.update_vel,request.update_prefVel, request.update_goals);
      rvo_ros::Agent agent_msg = sim_.toAgentMsg(*agent);
      response.state = agent_msg.state;
    }
    // publishCrowd();
    return true;
  }

  bool cbAddAgentState(rvo_ros::AddAgentState::Request& request, rvo_ros::AddAgentState::Response& response)
  {
    rvo_ros::Agent agentMsg;
    agentMsg.id = sim_.max_id_;
    agentMsg.state = request.state;
    agentMsg.param = getDefaultAgentParam();
    sim_.addAgent(agentMsg);
    response.id = agentMsg.id;
    ROS_INFO("[%s]: Add agent %d ",node_name_.c_str(),agentMsg.id);
    publishCrowd();
    return true;
  }  

  bool cbRemoveAgentId(rvo_ros::RemoveAgentId::Request& request, rvo_ros::RemoveAgentId::Response& response)
  {
    response.flag = sim_.removeAgent(request.id);
    if (response.flag){
      // ROS_INFO_STREAM("[CrowdNode]: Remove Agent :" << request.id);
      ROS_INFO("[%s]: Remove agent %d ",node_name_.c_str(),request.id);

    }
    else{
      // ROS_INFO_STREAM("[CrowdNode]: Agent " << request.id << " does not exist.");
    }
    publishCrowd();
    return true;
  }

  bool cbStart(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
  {
    timer_sim_.start();
    return true;
  }

  bool cbPause(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
  {
    timer_sim_.stop();
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rvo_crowd");
  ros::NodeHandle nh("~");

  RVOVelControllerNode rvo_crowd_node(nh);
  ros::spin();
  return 0;
}