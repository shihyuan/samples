/*
Author: Shih-Yuan Liu
*/
#include <cmath>
#include <map>
#include <vector>
#include <stdlib.h>
#include <cstdio>
#include <ctime>
#include <algorithm>
#include <math.h>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>

#include "mod_msgs/PedTraj.h"
#include "mod_msgs/PedTrajVec.h"
#include "mod_msgs/Pose2DStamped.h"
#include "mod_msgs/Clusters.h"
#include "mod_msgs/ClusterHit.h"
#include "mod_msgs/util.hpp"

template <typename T>
std::string toString(T value)
{
    std::ostringstream os ;
    os << value ;
    return os.str() ;
}

typedef std::vector<mod_msgs::Pose2DStamped> PoseVec;

class PedData{
public:
    double pedCount_;
    mod_msgs::PedTraj pedTraj_;
    int trajIndex_;
    double maxMovedDistanceSq_;
    bool ignore_;

    size_t num_of_previous_prediction_;
    std::vector<mod_msgs::PedTraj> predictions_;

    PedData(const std_msgs::Header& header, size_t ped_id, const geometry_msgs::Point& point, const geometry_msgs::Vector3& vel, double theta=0.0){
        trajIndex_ = -1;
        pedCount_ = 0;
        num_of_previous_prediction_ = 0;
        maxMovedDistanceSq_ = 0;
        ignore_ = false;
        addData(header,ped_id,point,vel,theta);
    }
    ~PedData(){}

    void addData(const std_msgs::Header& header, size_t ped_id, const geometry_msgs::Point& point, const geometry_msgs::Vector3& vel, double theta=0.0)
    {
        pedTraj_.ped_id = ped_id;
        mod_msgs::Pose2DStamped pose2DStamped;
        pose2DStamped.header.frame_id = header.frame_id;
        pose2DStamped.header.stamp = header.stamp;
        pose2DStamped.pose.x = point.x;
        pose2DStamped.pose.y = point.y;
        pose2DStamped.pose.theta = theta;
        pose2DStamped.velocity = vel;

        if (pedTraj_.traj.size() > 0){
            mod_msgs::Pose2DStamped initPose = pedTraj_.traj.front();
            double diff_x = pose2DStamped.pose.x - initPose.pose.x;
            double diff_y = pose2DStamped.pose.y - initPose.pose.y;
            double movedDistSq = diff_x*diff_x + diff_y*diff_y;
            if (movedDistSq > maxMovedDistanceSq_){
                maxMovedDistanceSq_ = movedDistSq;
            }
        }
        pedTraj_.traj.push_back(pose2DStamped);
    }

    void setPrediciton(const std::vector<mod_msgs::PedTraj>& predictions)
    {
        if (predictions.size() > 0){
            num_of_previous_prediction_ = predictions_.size();
            predictions_ = predictions;            
        }
    }
    size_t getNumOfPrediction() const {return predictions_.size();}

    ros::Time getLastPredictionTime(){
        if (predictions_.size() > 0){
            return predictions_.front().traj.front().header.stamp;
        }
    }

    ros::Time getFinalPredictionTime(){
        /* Return the maximum prediciton time*/
        mod_msgs::PedTraj const *traj_ptr = getMostLikelyPrediciton();
        if (traj_ptr == NULL){
            return getLastUpdateTime();
        }
        else{
            return traj_ptr->traj.back().header.stamp;
        }
    }

    void addPedCount(float likelihood){
        pedCount_ += likelihood;
    }

    size_t getTrajSize() const {return pedTraj_.traj.size();}

    size_t getPedId() const
    {
        return pedTraj_.ped_id;
    }
    ros::Time getLastUpdateTime()
    {
        return pedTraj_.traj.back().header.stamp;
    }

    void updateDiffIndex(){
        trajIndex_ = pedTraj_.traj.size() - 1;
    }

    bool hasDiff() const {
        return trajIndex_ < pedTraj_.traj.size() - 1;
    }

    bool fillDiffPedTraj(bool diff_only,mod_msgs::PedTraj& pedTraj)
    {
        PoseVec poseVec;
        if (diff_only){
            poseVec = getPoseVecFrom(getTrajIter(trajIndex_));
        }
        else{
            poseVec = pedTraj_.traj; 
        }

        if (poseVec.size() > 0){
            pedTraj = pedTraj_;
            pedTraj.traj = poseVec;
            return true;             
        }
        else{
            return false;
        }
    }
    bool fillRecentPedTraj(const ros::Time& current_time,const ros::Duration& duration,mod_msgs::PedTraj& pedTraj)
    {
        PoseVec poseVec = getPoseVecFrom(getTrajIter(current_time,duration));
        if (poseVec.size() > 0){
            // pedTraj.ped_id = getPedId();
            pedTraj = pedTraj_;
            pedTraj.traj = poseVec;
            return true;             
        }
        else{
            return false;
        }
    }

    mod_msgs::PedTraj const * getMostLikelyPrediciton()
    {
        mod_msgs::PedTraj const * ptr = NULL;
        double max_value = 0;
        std::vector<mod_msgs::PedTraj>::const_iterator traj_iter;
        for(traj_iter = predictions_.begin(); traj_iter != predictions_.end(); ++traj_iter){
            if (-traj_iter->value <= max_value){
                max_value = -traj_iter->value;
                ptr = &(*traj_iter);
            }
        }
        return ptr;
    }

    visualization_msgs::Marker getCountMarker(ros::Duration lifetime, ros::Time current_time, int max_num_of_points = 8000){
        visualization_msgs::Marker marker;

        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.id = getPedId();
        marker.header.frame_id = pedTraj_.traj.back().header.frame_id;
        marker.header.stamp = pedTraj_.traj.back().header.stamp;
        marker.lifetime = lifetime;
        
        mod_msgs::Pose2DStamped pose2D = pedTraj_.traj.back();
        marker.pose.position.x = pose2D.pose.x;
        marker.pose.position.y = pose2D.pose.y;
        marker.pose.position.z = 1.0;

        marker.text = toString(pedCount_);

        marker.scale.x = .3;
        marker.scale.y = .3;
        marker.scale.z = .3;

        if(pedCount_ > 20){
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
        }

        else if(pedCount_ < -20){
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
        }

        else{
            marker.color.r = 1.0 - (pedCount_ + 20) / 40;
            marker.color.g = (pedCount_ + 20) / 40;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
        }
        return marker;
    }

    std::vector<visualization_msgs::Marker> getVisMarkers(std_msgs::ColorRGBA color, ros::Duration lifetime, ros::Time current_time, int max_num_of_points = 8000)
    {
        std::vector<visualization_msgs::Marker> marker_vec;
        int vis_id = getPedId()%10000;
        
        // Observered Trajectory
        visualization_msgs::Marker traj_marker;
        traj_marker.header.frame_id = pedTraj_.traj.back().header.frame_id;
        traj_marker.header.stamp = current_time;
        traj_marker.ns = "ped_manager/traj";
        traj_marker.id = vis_id;
        traj_marker.lifetime = lifetime;
        traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
        traj_marker.pose.orientation.w = 1.0;
        traj_marker.scale.x = 0.15;

        int count_point = 0;
        for (PoseVec::const_reverse_iterator rit = pedTraj_.traj.rbegin(); rit != pedTraj_.traj.rend(); ++rit){
            if (count_point >= max_num_of_points){
                /* Limit the size of the points vector to max_num_of_points*/
                break;
            }
            geometry_msgs::Point p;
            p.x = rit->pose.x;
            p.y = rit->pose.y;
            traj_marker.points.push_back(p);
            count_point++;
        }

        traj_marker.color = color;
        marker_vec.push_back(traj_marker);

        // Predicted Trajectory
        for (int i = 0; i < std::max(getNumOfPrediction(),num_of_previous_prediction_); ++i){
            if (i < getNumOfPrediction()){
                visualization_msgs::Marker pred_marker;
                pred_marker.header.frame_id = predictions_[i].traj.back().header.frame_id;
                pred_marker.lifetime = lifetime;
                pred_marker.type = visualization_msgs::Marker::LINE_STRIP;
                pred_marker.pose.orientation.w = 1.0;
                if (predictions_[i].type == mod_msgs::PedTraj::PREDICTION_GP && predictions_[i].value > 0){
                    pred_marker.scale.x = 0.1*predictions_[i].value;
                }
                else{
                    pred_marker.scale.x = 0.1;
                }

                for (PoseVec::const_iterator it = predictions_[i].traj.begin(); it != predictions_[i].traj.end(); ++it){
                    geometry_msgs::Point p;
                    p.x = it->pose.x;
                    p.y = it->pose.y;
                    pred_marker.points.push_back(p);
                }
                pred_marker.color = color;
                pred_marker.color.a = 0.25; //TODO: set according to value
                pred_marker.header.stamp = current_time;
                pred_marker.ns = "ped_manager/pred_" + toString(vis_id);
                pred_marker.id = i;
                marker_vec.push_back(pred_marker);
            }
            else{
                /*Send DELETE marker for predictions that no longer there*/
                visualization_msgs::Marker marker;
                marker.action = visualization_msgs::Marker::DELETE;
                marker.ns = "ped_manager/pred_" + toString(vis_id);
                marker.id = i;
                marker_vec.push_back(marker);
            }
        }

        // Current Position and Velocity
        visualization_msgs::Marker pos_marker;
        pos_marker.header.frame_id = pedTraj_.traj.back().header.frame_id;
        pos_marker.header.stamp = pedTraj_.traj.back().header.stamp;
        pos_marker.ns = "ped_manager/pos";
        pos_marker.id = vis_id;
        pos_marker.lifetime = lifetime;
        pos_marker.type = visualization_msgs::Marker::CYLINDER;
        pos_marker.pose.orientation.w = 1.0;
        pos_marker.scale.x = 0.4;
        pos_marker.scale.y = 0.4;
        pos_marker.scale.z = 1.0;
        pos_marker.color = color;
        pos_marker.pose.position.z = 0.5;

        visualization_msgs::Marker vel_marker;
        vel_marker.header.frame_id = pedTraj_.traj.back().header.frame_id;
        vel_marker.header.stamp = pedTraj_.traj.back().header.stamp;
        vel_marker.ns = "ped_manager/vel";
        vel_marker.id = vis_id;
        vel_marker.lifetime = lifetime;
        vel_marker.type = visualization_msgs::Marker::ARROW;
        // vel_marker.pose.orientation.w = 1.0;
        vel_marker.scale.x = 0.1;
        vel_marker.scale.y = 0.1;
        vel_marker.scale.z = 0.0;
        vel_marker.color = color;
        // vel_marker.color.a = 0.8;
        // vel_marker.pose.position.z = 0.0;
        double arrow_ratio = 2.0;

        if (current_time <= getLastUpdateTime()){//TODO: add tolerance
            mod_msgs::Pose2DStamped pose2D = pedTraj_.traj.back();
            pos_marker.pose.position.x = pose2D.pose.x;
            pos_marker.pose.position.y = pose2D.pose.y;
            marker_vec.push_back(pos_marker);

            geometry_msgs::Point arrow_start;
            arrow_start.x = pose2D.pose.x;
            arrow_start.y = pose2D.pose.y;
            arrow_start.z = 1.0;
            geometry_msgs::Point arrow_end;
            arrow_end.x = arrow_start.x + arrow_ratio*pose2D.velocity.x;
            arrow_end.y = arrow_start.y + arrow_ratio*pose2D.velocity.y;
            arrow_end.z = 1.0;
            vel_marker.points.push_back(arrow_start);
            vel_marker.points.push_back(arrow_end);
            marker_vec.push_back(vel_marker);
        }
        else{
            mod_msgs::PedTraj const * traj_ptr = getMostLikelyPrediciton();
            if (traj_ptr != NULL){
                // ROS_INFO_STREAM("[PedData::getVisMarkers] NOT NULL traj_ptr from getMostLikelyPrediciton()");
                mod_msgs::Pose2DStamped pose2D;
                PoseVec::const_iterator start_itr = traj_ptr->traj.begin();
                if(mod_msgs::frontInterpolate(traj_ptr->traj, current_time, start_itr, pose2D)){
                    pos_marker.pose.position.x = pose2D.pose.x;
                    pos_marker.pose.position.y = pose2D.pose.y;
                    double a_ratio = (current_time - getLastUpdateTime()).toSec()/(traj_ptr->traj.back().header.stamp - getLastUpdateTime()).toSec();
                    double min_a = 0.0;
                    double max_a = 0.8;
                    pos_marker.color.a = max_a - a_ratio*(max_a-min_a);
                    marker_vec.push_back(pos_marker);

                    geometry_msgs::Point arrow_start;
                    arrow_start.x = pose2D.pose.x;
                    arrow_start.y = pose2D.pose.y;
                    arrow_start.z = 1.0;
                    geometry_msgs::Point arrow_end;
                    arrow_end.x = arrow_start.x + arrow_ratio*pose2D.velocity.x;
                    arrow_end.y = arrow_start.y + arrow_ratio*pose2D.velocity.y;
                    arrow_end.z = 1.0;
                    vel_marker.points.push_back(arrow_start);
                    vel_marker.points.push_back(arrow_end);
                    vel_marker.color.a = max_a - a_ratio*(max_a-min_a);
                    marker_vec.push_back(vel_marker);
                }
            }
            else{
                ROS_INFO_STREAM("[PedData::getVisMarkers] NULL traj_ptr from getMostLikelyPrediciton()");
            }
        }
        return marker_vec;
    }

private:
    PoseVec getPoseVecFrom(PoseVec::iterator trajIter)
    {
        return PoseVec(trajIter,pedTraj_.traj.end());
    }
    PoseVec::iterator getTrajIter(int trajIndex)
    {   
        return pedTraj_.traj.begin() + (trajIndex_ + 1);
    }
    PoseVec::iterator getTrajIter(const ros::Time& current_time,const ros::Duration& duration)
    {   //Return the iterator that points to the first element that's recent enough
        PoseVec::reverse_iterator rit;
        for(rit = pedTraj_.traj.rbegin(); rit != pedTraj_.traj.rend(); ++rit){
            if(current_time - rit->header.stamp > duration){
                break;
            }
        }
        return rit.base();
    }
};

typedef std::map<size_t, PedData*> PedMap;

class PedManager{
public:
    ros::NodeHandle nh_p_;
    ros::Subscriber sub_clusters_;
    ros::Subscriber sub_ped_id_;
    ros::Subscriber sub_veh_pose_;
    ros::Subscriber sub_ped_pred_;

    ros::Publisher pub_ped_diff_;
    ros::Publisher pub_veh_diff_;
    ros::Publisher pub_markers_;
    ros::Publisher pub_count_markers_;
    ros::Publisher pub_ped_dump_;
    ros::Publisher pub_ped_recent_;

    ros::Timer timer_ped_diff_;
    ros::Timer timer_prune_;
    ros::Timer timer_vis_;
    ros::Timer timer_dump_;
    ros::Timer timer_recent_;

    ros::Time current_cluster_time_;

    PedMap ped_map_;

    std::map<size_t, std_msgs::ColorRGBA> color_map_;
    PedData* veh_traj_data;

    // Parameters
    double diff_pub_period_;
    double prune_period_;
    double inactive_tol_;
    double vis_period_;
    double dump_period_;
    double recent_period_;
    double recent_tol_;
    double recent_downsample_duration_;
    int ped_count_threshold_;
    bool publish_markers_;
    bool visualizer_only_;
    int static_ped_count_threshold_;
    double min_dist_threshold_;
    double min_dist_to_veh_;

    double stationary_duration_;
    double stationary_velocity_;

    PedManager()
    {
        nh_p_ = ros::NodeHandle("~");
        setDefaultParameters();
        getParameters();

        veh_traj_data = NULL;

        /* Subscriber */
        sub_clusters_ = nh_p_.subscribe("clusters",1,&PedManager::cbClusters,this);
        sub_ped_id_ = nh_p_.subscribe("ped_id",1,&PedManager::cbPedId,this);
        sub_veh_pose_ = nh_p_.subscribe("veh_pose",1,&PedManager::cbVehPose,this);
        sub_ped_pred_ = nh_p_.subscribe("ped_pred",1,&PedManager::cbPedPrediction,this);

        /* Publisher */
        if(not visualizer_only_){
            pub_ped_diff_ = nh_p_.advertise<mod_msgs::PedTrajVec>("ped_diff",1,true); //true for latching
            pub_veh_diff_ = nh_p_.advertise<mod_msgs::PedTraj>("veh_diff",1,true); //true for latchin        
            pub_ped_dump_ = nh_p_.advertise<mod_msgs::PedTrajVec>("ped_dump",1,true); //true for latching
            pub_ped_recent_ = nh_p_.advertise<mod_msgs::PedTrajVec>("ped_recent",1,true); //true for latching
        }
        if (publish_markers_){
            pub_markers_ = nh_p_.advertise<visualization_msgs::MarkerArray>("ped_markers",1,true); //true for latching
            pub_count_markers_ = nh_p_.advertise<visualization_msgs::MarkerArray>("ped_count_markers",1,true); //true for latching
        }

        /* Timer */
        if (not visualizer_only_){
            timer_ped_diff_= nh_p_.createTimer(ros::Duration(diff_pub_period_),&PedManager::cbPublishPedDiff,this);
            timer_recent_= nh_p_.createTimer(ros::Duration(recent_period_),&PedManager::cbRecent,this);            
        }
        timer_prune_= nh_p_.createTimer(ros::Duration(prune_period_),&PedManager::cbPrune,this);
        timer_dump_ = nh_p_.createTimer(ros::Duration(dump_period_),&PedManager::cbDump,this);
        
        if (publish_markers_){
            timer_vis_= nh_p_.createTimer(ros::Duration(vis_period_),&PedManager::cbVis,this);
        }
    }
    ~PedManager(){
        if (veh_traj_data != NULL){
            delete veh_traj_data;
        }
    }

    void setDefaultParameters()
    {
        if (!ros::param::has("~diff_pub_period")) { ros::param::set("~diff_pub_period",1.0);}
        if (!ros::param::has("~prune_period")) { ros::param::set("~prune_period",10.0);}
        if (!ros::param::has("~inactive_tol")) { ros::param::set("~inactive_tol",5.0);}
        if (!ros::param::has("~vis_period")) { ros::param::set("~vis_period",0.03);}
        if (!ros::param::has("~dump_period")) { ros::param::set("~dump_period",10.0);}
        if (!ros::param::has("~recent_period")) { ros::param::set("~recent_period",0.2);}
        if (!ros::param::has("~recent_tol")) { ros::param::set("~recent_tol",10.0);}
        if (!ros::param::has("~recent_downsample_duration")) { ros::param::set("~recent_downsample_duration",1.0);}
        if (!ros::param::has("~publish_markers")) { ros::param::set("~publish_markers",true);}
        if (!ros::param::has("~visualizer_only")) { ros::param::set("~visualizer_only",false);}
        if (!ros::param::has("~ped_count_threshold")) { ros::param::set("~ped_count_threshold",2);}
        if (!ros::param::has("~static_ped_count_threshold")) { ros::param::set("~static_ped_count_threshold",20);}
        if (!ros::param::has("~min_dist_threshold")) { ros::param::set("~min_dist_threshold",2);}
        if (!ros::param::has("~min_dist_to_veh")) { ros::param::set("~min_dist_to_veh",0.2);}
        if (!ros::param::has("~stationary_duration")) { ros::param::set("~stationary_duration",3.0);}
        if (!ros::param::has("~stationary_velocity")) { ros::param::set("~stationary_velocity",0.2);}
    }

    void getParameters()
    {
        ros::param::getCached("~diff_pub_period",diff_pub_period_);
        ros::param::getCached("~prune_period",prune_period_);
        ros::param::getCached("~inactive_tol",inactive_tol_);
        ros::param::getCached("~vis_period",vis_period_);
        ros::param::getCached("~dump_period",dump_period_);
        ros::param::getCached("~recent_period",recent_period_);
        ros::param::getCached("~recent_tol",recent_tol_);
        ros::param::getCached("~recent_downsample_duration",recent_downsample_duration_);
        ros::param::getCached("~ped_count_threshold",ped_count_threshold_);
        ros::param::getCached("~static_ped_count_threshold",static_ped_count_threshold_);
        ros::param::getCached("~min_dist_threshold",min_dist_threshold_);
        ros::param::getCached("~publish_markers",publish_markers_);
        ros::param::getCached("~visualizer_only",visualizer_only_);
        ros::param::getCached("~stationary_duration",stationary_duration_);
        ros::param::getCached("~stationary_velocity",stationary_velocity_);
        ros::param::getCached("~min_dist_to_veh",min_dist_to_veh_);
    }

    void cbPedPrediction(const mod_msgs::PedTrajVec& pedTrajVec)
    {   /*Save Pedestrian predictions to the pred_map*/
        if (pedTrajVec.ped_traj_vec.size() == 0){
            return;
        }
        size_t ped_id = pedTrajVec.ped_traj_vec.front().ped_id;
        PedMap::iterator itMap;
        itMap = ped_map_.find(ped_id);
        if (itMap != ped_map_.end()){ /*PedData exsit*/
            itMap->second->setPrediciton(pedTrajVec.ped_traj_vec);
        }
    }

    void cbVehPose(const geometry_msgs::PoseWithCovarianceStamped& msgc)
    {
        geometry_msgs::PoseStamped msg;
        msg.header = msgc.header;
        msg.pose = msgc.pose.pose;
        
        tf::Quaternion q;
        double roll, pitch, yaw;
        tf::quaternionMsgToTF(msg.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        if (veh_traj_data == NULL){
            veh_traj_data = new PedData(msg.header, 0, msg.pose.position, geometry_msgs::Vector3(), yaw);
        }
        else{
            veh_traj_data->addData(msg.header, 0, msg.pose.position, geometry_msgs::Vector3(), yaw);
        }
    }

    void cbClusters(const mod_msgs::Clusters& clusters){
        // Manage the clusers
        current_cluster_time_ = clusters.header.stamp; //Use the cluster time to avoid time synce problem between machines
        PedMap::iterator it;
        for (int i = 0; i < clusters.labels.size(); i++){
            it = ped_map_.find(clusters.labels[i]);
            if (it == ped_map_.end()){
                /* Compute distance to vehicle */
                ped_map_[clusters.labels[i]] = new PedData(clusters.header,clusters.labels[i],clusters.mean_points[i],clusters.velocities[i]);
                // ROS_INFO_STREAM("[PedManager::cbClusters] Add " << clusters.labels[i] << " to ped_map_");
            }
            else{
                ped_map_[clusters.labels[i]]->addData(clusters.header,clusters.labels[i],clusters.mean_points[i],clusters.velocities[i]);
            }
        }
    }

    void cbPedId(const mod_msgs::ClusterHit& cluster_hit)
    {
        PedMap::iterator it = ped_map_.find(cluster_hit.ped_id);
        if (it != ped_map_.end()){
            it->second->addPedCount(cluster_hit.likelihood);
        }
        // ROS_INFO_STREAM("[PedManager::cbPedId]:" << ped_id);
    }

    mod_msgs::PedTrajVec getPedTrajVec(bool diff_only, bool ped_only)
    {
        // Return a vector of PedTraj messages
        mod_msgs::PedTrajVec pedTrajVecMsg;
        PedMap::iterator it;
        for (it = ped_map_.begin(); it != ped_map_.end(); ++it){
            if (ped_only){
                if (not isPed(*(it->second))){
                    continue; // Skip non-pedestrian 
                }
            }

            mod_msgs::PedTraj pedTraj;
            if (it->second->fillDiffPedTraj(diff_only,pedTraj)){
                // Only pushback non-empty PedTraj
                pedTrajVecMsg.ped_traj_vec.push_back(pedTraj);
            }
        }
        return pedTrajVecMsg;
    }

    mod_msgs::PedTrajVec getRecentPedTrajVec(const ros::Duration& duration)
    {
        mod_msgs::PedTrajVec pedTrajVecMsg;
        PedMap::iterator it;
        for (it = ped_map_.begin(); it != ped_map_.end(); ++it){
            if (not isPed(*(it->second))){
                continue;
            }

            mod_msgs::PedTraj pedTraj;
            if (it->second->fillRecentPedTraj(current_cluster_time_,duration,pedTraj)){
                // Only pushback non-empty PedTraj
                pedTrajVecMsg.ped_traj_vec.push_back(pedTraj);
            }
        }
        return pedTrajVecMsg;
    }

    void cbPublishPedDiff(const ros::TimerEvent& timerEvent)
    {
        mod_msgs::PedTrajVec ped_traj_vec_msg = getPedTrajVec(true,true);

        // Publish
        pub_ped_diff_.publish(ped_traj_vec_msg);
        // Update the Diff Pointer
        PedMap::iterator it;
        for (it = ped_map_.begin(); it != ped_map_.end(); ++it){
            it->second->updateDiffIndex();
        }
        // ROS_INFO_STREAM(ped_traj_vec_msg);
        if (veh_traj_data != NULL){
            mod_msgs::PedTraj veh_traj;
            veh_traj_data->fillDiffPedTraj(true,veh_traj);
            veh_traj_data->updateDiffIndex();
            if (veh_traj.traj.size() > 0){
                pub_veh_diff_.publish(veh_traj);
            }
        }
    }

    void cbRecent(const ros::TimerEvent& timerEvent)
    {
        getParameters();
        mod_msgs::PedTrajVec pedTrajVec_recent = getRecentPedTrajVec(ros::Duration(recent_tol_)); 
        pub_ped_recent_.publish(mod_msgs::downsample(pedTrajVec_recent,ros::Duration(recent_downsample_duration_)));
    }

    void cbPrune(const ros::TimerEvent& timerEvent)
    {
        pruneInactive(ros::Duration(inactive_tol_),true);
    }

    void pruneInactive(const ros::Duration& inactive_tol, bool keep_ped)
    {
        // ROS_INFO_STREAM("[PedManager::pruneInactive] Pruning started.");
        PedMap::iterator it;
        for (it = ped_map_.begin(); it != ped_map_.end();){
            if (keep_ped){
                if (isPed(*(it->second))){ // Don't prune pedestrian data
                    ++it;
                    continue;
                }
            }
            if (current_cluster_time_ - it->second->getLastUpdateTime() > inactive_tol){
                removePed(it++);
            }
            else{
                ++it;
            }
        }
    }
    void removePed(PedMap::iterator it)
    {
        color_map_.erase(it->first);
        delete it->second;
        ped_map_.erase(it);
        // ROS_INFO_STREAM("[PedManager::removePed] Remove "<< it->first );
    }

    void cbDump(const ros::TimerEvent& timerEvent)
    {
        mod_msgs::PedTrajVec pedTrajVec = popInactivePed(ros::Duration(inactive_tol_));
        pub_ped_dump_.publish(pedTrajVec);
    }

    mod_msgs::PedTrajVec popInactivePed(const ros::Duration& inactive_tol)
    {
        // ROS_INFO_STREAM("[PedManager::popInactivePed] Popping started.");
        mod_msgs::PedTrajVec pedTrajVec;
        PedMap::iterator it;
        for (it = ped_map_.begin(); it != ped_map_.end();){
            if (isPed(*(it->second))){ //Only process if is pedestrian
                if (current_cluster_time_ - it->second->getLastUpdateTime() > inactive_tol){
                    mod_msgs::PedTraj pedTraj;
                    if (it->second->fillDiffPedTraj(false,pedTraj)){
                        pedTrajVec.ped_traj_vec.push_back(pedTraj);
                    }
                    removePed(it++);
                }
                else{ //Skip if still active
                    ++it;
                }
            }
            else{ //Skip if not a pedestrian
                ++it;
            }
        }
        return pedTrajVec;
    }

    void cbVis(const ros::TimerEvent& timerEvent)
    {
        publishMarkers();
        publishPedCountMarker();
    }

    void publishPedCountMarker()
    {
        visualization_msgs::MarkerArray countMarkers;
        PedMap::const_iterator itMap;

        for (itMap = ped_map_.begin(); itMap != ped_map_.end(); ++itMap){

            visualization_msgs::Marker marker = itMap->second->getCountMarker(ros::Duration(vis_period_ + 0.2), current_cluster_time_);
            countMarkers.markers.push_back(marker);
        }

        //countMarkers.markers.insert(markerArray.markers.end(),markers.begin(),markers.end());
        pub_count_markers_.publish(countMarkers);
    }

    void publishMarkers()
    {
        // Publish whole trajectories of pedestrians
        visualization_msgs::MarkerArray markerArray;
        // markerArray.markers = toMarkerVec(getPedTrajVec(false,true)); 
        PedMap::const_iterator itMap;
        for (itMap = ped_map_.begin();itMap != ped_map_.end(); ++itMap){
            if ( isPed(*(itMap->second)) && current_cluster_time_ <= itMap->second->getFinalPredictionTime())
            {
                // std::vector<visualization_msgs::Marker> markers = toMarkerVec(*(itMap->second));
                std::vector<visualization_msgs::Marker> markers = itMap->second->getVisMarkers(getColor(itMap->first),ros::Duration(vis_period_ + 0.2),current_cluster_time_);
                markerArray.markers.insert(markerArray.markers.end(),markers.begin(),markers.end());
            }
        }
        // Add vehicle marker 
        if (veh_traj_data != NULL){
            markerArray.markers.push_back(toVehMarker(veh_traj_data->pedTraj_));
        }
        /*TODO: Add prediction marker*/
        pub_markers_.publish(markerArray);
    }

    visualization_msgs::Marker toVehMarker(const mod_msgs::PedTraj vehTraj)
    {
        // Current Position
        visualization_msgs::Marker pos_marker;
        pos_marker.header.frame_id = vehTraj.traj.back().header.frame_id;
        pos_marker.header.stamp = vehTraj.traj.back().header.stamp;
        pos_marker.ns = "ped_manager/veh";
        pos_marker.id = vehTraj.ped_id;
        pos_marker.lifetime = ros::Duration(vis_period_ + 0.2);
        pos_marker.type = visualization_msgs::Marker::CUBE; //TODO change to model
        
        pos_marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,vehTraj.traj.back().pose.theta);

        pos_marker.pose.position.x = vehTraj.traj.back().pose.x;
        pos_marker.pose.position.y = vehTraj.traj.back().pose.y;
        pos_marker.pose.position.z = 0.5;

        pos_marker.scale.x = 2.0;
        pos_marker.scale.y = 1.0;
        pos_marker.scale.z = 1.0;
        // pos_marker.color = getColor(pedTraj.ped_id);
        pos_marker.color.r = 1.0;
        pos_marker.color.g = 1.0;
        pos_marker.color.b = 1.0;
        pos_marker.color.a = 1.0;

        return pos_marker;
    }


    bool isPed(PedData& pedTrajData){
        if (pedTrajData.ignore_){
            return false;
        }
        /* Ignore clusters too close to the host vehilce */
        if (veh_traj_data != NULL){
            mod_msgs::Pose2DStamped current_pose = pedTrajData.pedTraj_.traj.back();
            mod_msgs::Pose2DStamped current_veh = veh_traj_data->pedTraj_.traj.back();
            double diff_x = current_pose.pose.x - current_veh.pose.x;
            double diff_y = current_pose.pose.y - current_veh.pose.y;
            if (diff_x*diff_x + diff_y*diff_y <= min_dist_to_veh_*min_dist_to_veh_){
                // pedTrajData.ignore_ = true;
                return false;
            }
        }

        bool moved = pedTrajData.maxMovedDistanceSq_ > min_dist_threshold_*min_dist_threshold_;
        int count_threshold = ped_count_threshold_;
        if (not moved){
            // Requires more counts for clusters that haven't moved
            count_threshold = static_ped_count_threshold_;
        }

        if (pedTrajData.pedCount_ >= count_threshold){
            /* Prune clusters that hasn't moved for a while */
            return not isStationary(pedTrajData,ros::Duration(stationary_duration_), stationary_velocity_);
        }
        else{
            return false;
        }
    }

    bool isStationary(const PedData& pedTrajData, ros::Duration duration, double vel_thresh)
    {
        if (current_cluster_time_ - pedTrajData.pedTraj_.traj.front().header.stamp < duration){
            /* Use current velocity if not enough observations are made*/
            mod_msgs::Pose2DStamped poseStamp = pedTrajData.pedTraj_.traj.back();
            return (poseStamp.velocity.x*poseStamp.velocity.x + poseStamp.velocity.y*poseStamp.velocity.y < vel_thresh*vel_thresh);
        }
        PoseVec::const_reverse_iterator rIt;
        for (rIt = pedTrajData.pedTraj_.traj.rbegin(); rIt != pedTrajData.pedTraj_.traj.rend(); ++rIt)
        {
            double vel = rIt->velocity.x * rIt->velocity.x + rIt->velocity.y * rIt->velocity.y;
            if (vel*vel > vel_thresh*vel_thresh){
                /* Velocity above threshold */
                return false;
            }
            if (current_cluster_time_ - rIt->header.stamp >= duration){
                break;
            }
        }
        return true;
    }

    std_msgs::ColorRGBA getColor(size_t ped_id)
    {
        std::map<size_t, std_msgs::ColorRGBA>::iterator it = color_map_.find(ped_id);
        if (it == color_map_.end()){ // Insert random color for ped_id
            std_msgs::ColorRGBA color;
            color.r = ((float) rand()/(RAND_MAX));
            color.g = ((float) rand()/(RAND_MAX));
            color.b = ((float) rand()/(RAND_MAX));
            color.a = 1.0;
            color_map_[ped_id] = color;
        }
        return color_map_[ped_id];
    }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ped_manager");
    PedManager ped_manager;
    ros::spin();
    return 0;
}
