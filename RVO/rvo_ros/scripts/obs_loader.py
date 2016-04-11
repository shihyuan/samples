#!/usr/bin/env python
'''
Author: Shih-Yuan Liu
'''

import rospy
import copy
from rvo_ros.srv import SetObstacles
from rvo_ros.msg import Obstacles
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32

if __name__ == '__main__':
    rospy.init_node('obs_loader_node',anonymous=False)
    # Get service name from param 
    crowd_node_name = rospy.get_param("~rvo_node_name","rvo_vel_controller")
    service_name = "/" + crowd_node_name + "/set_obstacles"
    # Create obstacle from param
    obstacle_list = rospy.get_param("~obstacles",[])
    # print obstacle_list
    obs_msg = Obstacles()
    for poly_list in obstacle_list:
        poly = Polygon()
        for point in poly_list:
            # print point
            poly.points.append(Point32(x=point[0],y=point[1],z=0.0))
        obs_msg.obstacles.append(poly)

    # wait for service
    rospy.wait_for_service(service_name,timeout=5.0)
    rospy.loginfo("[%s] Service %s available." %(rospy.get_name(),service_name))
    # inject obstacle using service
    set_obstacles = rospy.ServiceProxy(service_name, SetObstacles)
    try:
        response = set_obstacles(obstacles=obs_msg.obstacles)
        rospy.loginfo("[%s] %s called." %(rospy.get_name(), service_name))
    except rospy.ServiceException as exc:
        rospy.loginfo("[%s] %s did not response." %(rospy.get_name(), service_name))
    
    rospy.loginfo("[%s] Shutdown." %(rospy.get_name()))
