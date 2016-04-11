#!/usr/bin/env python
'''
Author: Shih-Yuan Liu
'''

import rospy
import random
import rospkg

from rvo_ros.srv import AddAgentState
from rvo_ros.srv import AddAgent
from rvo_ros.srv import RemoveAgentId
from rvo_ros.srv import SetObstacles
from rvo_ros.msg import AgentState
from rvo_ros.msg import AgentParam
from rvo_ros.msg import Agent
from rvo_ros.msg import Crowd
from rvo_ros.msg import Obstacles
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32

class RvoVelCrowdCommander(object):
    def __init__(self,crowd_node_name='/rvo_vel_controller'):
        self.node_name = rospy.get_name()
        self.crowd_node_name = crowd_node_name
        self.srv_add_agent_state = rospy.ServiceProxy(self.crowd_node_name+"/add_agent_state",AddAgentState)
        self.srv_add_agent = rospy.ServiceProxy(self.crowd_node_name+"/add_agent",AddAgent)
        self.srv_set_obs = rospy.ServiceProxy(self.crowd_node_name+"/set_obstacles",SetObstacles)
        self.srv_remove_agent_id = rospy.ServiceProxy(self.crowd_node_name+"/remove_agent_id",RemoveAgentId)
        self.sub_crowd = rospy.Subscriber(self.crowd_node_name+"/crowd", Crowd, self.cbCrowd)
        self.agent_id = 10;

    def defaultAgentParam(self):
        param = AgentParam()
        param.radius = 0.5
        param.maxSpeed = 1.0
        param.maxAcc = 2.0
        param.maxNeighbors = 20
        param.neighborDist = 5.0
        param.timeHorizon = 1.0
        param.timeHorizonObst = 0.5
        param.lambdaValue = 0.5
        param.agentType = AgentParam.NORMAL
        return param

    def setObstacles(self,obs_msg):
        try:
            response = self.srv_set_obs(obstacles=obs_msg.obstacles)
        except rospy.ServiceException as exc:
            rospy.loginfo("[RvoVelCrowdCommander] setObstacles service call did not response.")

    def addAgentState(self,state):
        try:
            response = self.srv_add_agent_state(state=state)
            return response.id
        except rospy.ServiceException as exc:
            rospy.loginfo("[RvoVelCrowdCommander] addAgentState service call did not response.")

    def addAgent(self,agent):
        try:
            response = self.srv_add_agent(agent=agent)
            return response.id
        except rospy.ServiceException as exc:
            rospy.loginfo("[RvoVelCrowdCommander] addAgent service call did not response.")

    def removeAgentId(self,agent_id):
        try:
            response = self.srv_remove_agent_id(id=agent_id)
        except rospy.ServiceException as exc:
            rospy.loginfo("[RvoVelCrowdCommander] RemoveAgentId service call did not response.")

    def cbCrowd(self,crowd):
        # Remove agents when reach goal region
        for agent in crowd.agents:
            if agent.id >= 10 and agent.state.pos.y > 1.9:
                self.removeAgentId(agent.id)
                # rospy.loginfo("[%s] Removing agent %s. " %(self.node_name, agent.id))

    def genRandomAgent(self):
        agent = Agent()
        agent.id = self.agent_id
        agent.param = self.defaultAgentParam()

        agent.param.radius = 0.3 + random.uniform(0,0.1)
        agent.param.timeHorizon = random.uniform(0.75,4.0);
        agent.param.lambdaValue = 0.5
        agent.param.maxSpeed = random.uniform(0.5,1.2);
        agent.state.pos.x = random.uniform(-2.0,2.0)
        agent.state.pos.y = -7.0 + random.uniform(-0.2,0.2)
        agent.state.prefVelocity.x = random.uniform(-0.1,0.1)
        agent.state.prefVelocity.y = agent.param.maxSpeed
        self.agent_id = self.agent_id + 1
        return agent

    def cbTimer(self,event):
        # Inject random agent
        agent = self.genRandomAgent()
        agent_id = self.addAgent(agent)
        rospy.loginfo("[%s] Add agent %s "%(self.node_name,agent_id))


if __name__ == '__main__':
    rospy.init_node('rvo_vel_crowd_commander',anonymous=False)
    rvo_vel_crowd_commander = RvoVelCrowdCommander()
    # Start timer
    add_timer = rospy.Timer(rospy.Duration.from_sec(1.0),rvo_vel_crowd_commander.cbTimer)
    rospy.spin()