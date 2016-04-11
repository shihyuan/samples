/*
Author: Shih-Yuan Liu
*/

#ifndef __VOPOLYGON_HPP
#define __VOPOLYGON_HPP

#include <rvo_ros/Agent.h>
#include <rvo_ros/AgentInfo.h>
#include <rvo_ros/VoInfo.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <Vector2.h> //RVO::Vector2
#include <math.h>

class VoPolygon{
public:
  typedef RVO::Vector2 Vector2;

  std::vector<Vector2> points_;

  static float projectionDist(const Vector2& point, const Vector2& n, const float& c, Vector2& projected_point)
  {
    float t = (c - n*point)/(absSq(n));
    projected_point = point + t*n;
    return -t*abs(n);
  }
  /* Convert a VoInfo to halfplane of the format n dot u <= c */
  static void voLineToHalfPlane(const rvo_ros::VoInfo& voInfo, RVO::Vector2& n, float& c)
  {
    n = RVO::Vector2(voInfo.lineDirection.y,-voInfo.lineDirection.x);
    RVO::Vector2 point(voInfo.linePoint.x,voInfo.linePoint.y);
    c = n*point;
  }

  VoPolygon(const rvo_ros::AgentInfo& info, float max_speed)
  {
    int num_of_sides = 24;
    for (int i = 0; i < 24; ++i){
      double angle = 2*M_PI*((double) i/(double) num_of_sides);
      points_.push_back(Vector2(max_speed*cos(angle),max_speed*sin(angle)));
    }
    /* Cut polygons with agent's VoInfos */
    for (int i = 0; i < info.agentNeighborsVoInfo.size();++i){
      const rvo_ros::VoInfo& voInfo = info.agentNeighborsVoInfo.at(i);    
      RVO::Vector2 n;
      float c;
      voLineToHalfPlane(voInfo,n,c);
      cutPolygon(n,c);
    }
    /* Cut polygons with obstacle's VoInfos */
    for (int i = 0; i < info.obstacleNeighborsVoInfo.size();++i){
      const rvo_ros::VoInfo& voInfo = info.obstacleNeighborsVoInfo.at(i);    
      RVO::Vector2 n;
      float c;
      voLineToHalfPlane(voInfo,n,c);
      cutPolygon(n,c);
    }
  }
  ~VoPolygon(){}

  geometry_msgs::Polygon getPolygon()
  {
    geometry_msgs::Polygon poly_msg;
    geometry_msgs::Point32 point;
    for (int i = 0; i < points_.size(); ++i){
      point.x = points_[i].x();
      point.y = points_[i].y();
      poly_msg.points.push_back(point);
    }
    return poly_msg;
  }

  void cutPolygon(const Vector2& n, const float& c)
  {
    std::vector<Vector2> new_points;
    for(std::vector<Vector2>::const_iterator it = points_.begin(); it != points_.end(); ++it){
      Vector2 start_point = *it;
      Vector2 end_point = (it+1 == points_.end()) ? points_.front():*(it+1); //Loop to the front if at the end
      std::vector<Vector2> points = cutHelper(start_point,end_point,n,c);
      new_points.insert(new_points.end(),points.begin(),points.end());
    }
    points_ = new_points;
  }

  /* Return the new start and end point that's insdie the halfplane (excluding the end_point if end_point is insdie)*/
  std::vector<Vector2> cutHelper(const Vector2& start_point, const Vector2& end_point, const Vector2& n, const float& c)
  {
    std::vector<Vector2> inside_points;
    Vector2 proj_start;
    Vector2 proj_end;
    float dist_start = projectionDist(start_point,n,c,proj_start);
    float dist_end = projectionDist(end_point,n,c,proj_end);
    if (dist_start*dist_end > 0){
      /* On the same side. No intersection */
      if (dist_start < 0){
        /*Both inside the halfplane.*/
        inside_points.push_back(start_point);
        // inside_points.push_back(end_point);
      }
      else{
        /*Both outside. Do nothing*/
      }
    }
    else if(dist_start*dist_end < 0){
      /* On both side of the halfplane, solve for intersection */
      Vector2 proj_start_to_end = proj_end - proj_start;
      float ratio = dist_start/(dist_start - dist_end); //Alwasy positive
      Vector2 intersect_point = proj_start + ratio*proj_start_to_end;
      if (dist_start < 0){
        /* start point is insdie */
        inside_points.push_back(start_point); 
        inside_points.push_back(intersect_point); 
      }
      else{
        /* start point is outside, end point is inside */
        inside_points.push_back(intersect_point);
        // inside_points.push_back(end_point);
      }
    }
    else{
      /* Edge cases. Either one or both start and end points are exactly on the halfplane */
      if (dist_start == 0){
        inside_points.push_back(proj_start);
      }
      if (dist_end == 0){
        // inside_points.push_back(proj_end);
      }
    }
    return inside_points;
  }

};

#endif /*__VOPOLYGON_HPP*/