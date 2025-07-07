
.. _program_listing_file__home_ptsingaman_ros_ws_CoHAN2.0_src_hateb_local_planner_include_hateb_local_planner_hateb_local_planner_ros.h:

Program Listing for File hateb_local_planner_ros.h
==================================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ptsingaman_ros_ws_CoHAN2.0_src_hateb_local_planner_include_hateb_local_planner_hateb_local_planner_ros.h>` (``/hateb_local_planner/include/hateb_local_planner/hateb_local_planner_ros.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*********************************************************************
    *
    * Software License Agreement (BSD License)
    *
    *  Copyright (c) 2016
    *  TU Dortmund - Institute of Control Theory and Systems Engineering.
    *  All rights reserved.
    *
    *  Redistribution and use in source and binary forms, with or without
    *  modification, are permitted provided that the following conditions
    *  are met:
    *
    *   * Redistributions of source code must retain the above copyright
    *     notice, this list of conditions and the following disclaimer.
    *   * Redistributions in binary form must reproduce the above
    *     copyright notice, this list of conditions and the following
    *     disclaimer in the documentation and/or other materials provided
    *     with the distribution.
    *   * Neither the name of the institute nor the names of its
    *     contributors may be used to endorse or promote products derived
    *     from this software without specific prior written permission.
    *
    *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    *  POSSIBILITY OF SUCH DAMAGE.
    *
    * Author: Christoph Rösmann
    * Modified by: Phani Teja Singamaneni
    *********************************************************************/
   
   #ifndef HATEB_LOCAL_PLANNER_ROS_H_
   #define HATEB_LOCAL_PLANNER_ROS_H_
   
   #include <ros/ros.h>
   
   // base local planner base class and utilities
   #include <base_local_planner/costmap_model.h>
   #include <base_local_planner/goal_functions.h>
   #include <base_local_planner/odometry_helper_ros.h>
   #include <mbf_costmap_core/costmap_controller.h>
   #include <nav_core/base_local_planner.h>
   
   // timed-elastic-band related classes
   #include <hateb_local_planner/optimal_planner.h>
   #include <hateb_local_planner/recovery_behaviors.h>
   #include <hateb_local_planner/visualization.h>
   
   // message types
   #include <cohan_msgs/Optimize.h>
   #include <costmap_converter/ObstacleMsg.h>
   #include <geometry_msgs/Point.h>
   #include <geometry_msgs/PoseStamped.h>
   #include <nav_msgs/Odometry.h>
   #include <nav_msgs/Path.h>
   #include <std_msgs/String.h>
   #include <visualization_msgs/Marker.h>
   #include <visualization_msgs/MarkerArray.h>
   
   // agent data
   #include <agent_path_prediction/AgentPosePredict.h>
   #include <agent_path_prediction/PredictedGoal.h>
   #include <agent_path_prediction/agent_path_prediction.h>
   #include <cohan_msgs/StateArray.h>
   #include <std_srvs/Empty.h>
   #include <std_srvs/SetBool.h>
   #include <std_srvs/Trigger.h>
   #include <std_srvs/TriggerRequest.h>
   #include <std_srvs/TriggerResponse.h>
   
   // transforms
   #include <eigen_conversions/eigen_msg.h>
   #include <tf2/convert.h>
   #include <tf2/impl/utils.h>
   #include <tf2/utils.h>
   #include <tf2_eigen/tf2_eigen.h>
   #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
   #include <tf2_ros/buffer.h>
   #include <tf2_ros/transform_listener.h>
   
   // costmap
   #include <costmap_2d/costmap_2d_ros.h>
   #include <costmap_converter/costmap_converter_interface.h>
   
   // dynamic reconfigure
   #include <dynamic_reconfigure/server.h>
   #include <hateb_local_planner/HATebLocalPlannerReconfigureConfig.h>
   
   // boost classes
   #include <boost/bind.hpp>
   #include <boost/shared_ptr.hpp>
   #include <boost/smart_ptr/shared_ptr.hpp>
   
   // Backoff recovery
   #include <hateb_local_planner/backoff.h>
   
   // Behavior Tree and Mode Switch
   #include <hateb_local_planner/mode_switch.h>
   
   namespace hateb_local_planner {
   enum class AgentState : std::uint8_t { NO_STATE, STATIC, MOVING, STOPPED, BLOCKED };
   
   class HATebLocalPlannerROS : public nav_core::BaseLocalPlanner, public mbf_costmap_core::CostmapController {
    public:
     HATebLocalPlannerROS();
   
     ~HATebLocalPlannerROS() override;
   
     // CPP wrapper for the planner
     HATebLocalPlannerROS(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);
   
     void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros) override;
   
     bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) override;
   
     bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel) override;
   
     uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped &pose, const geometry_msgs::TwistStamped &velocity, geometry_msgs::TwistStamped &cmd_vel, std::string &message) override;
   
     bool isGoalReached() override;
   
     bool isGoalReached(double xy_tolerance, double yaw_tolerance) override { return isGoalReached(); };
   
     bool cancel() override { return false; };
   
   
     static Eigen::Vector2d tfPoseToEigenVector2dTransRot(const tf::Pose &tf_vel);
   
     static FootprintModelPtr getRobotFootprintFromParamServer(const ros::NodeHandle &nh, const HATebConfig &config);
   
     static Point2dContainer makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue &footprint_xmlrpc, const std::string &full_param_name);
   
     static double getNumberFromXMLRPC(XmlRpc::XmlRpcValue &value, const std::string &full_param_name);
   
   
    protected:
     void updateObstacleContainerWithCostmap();
   
     void updateObstacleContainerWithCostmapConverter();
   
     void updateObstacleContainerWithCustomObstacles();
   
     void updateObstacleContainerWithInvHumans();
   
     void updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped> &transformed_plan, double min_separation);
   
     void updateAgentViaPointsContainers(const AgentPlanVelMap &transformed_agent_plan_vel_map, double min_separation);
   
     void reconfigureCB(HATebLocalPlannerReconfigureConfig &config, uint32_t level);
     void customObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr &obst_msg);
   
     void InvHumansCB(const costmap_converter::ObstacleArrayMsg::ConstPtr &obst_msg);
   
     void customViaPointsCB(const nav_msgs::Path::ConstPtr &via_points_msg);
   
     static bool pruneGlobalPlan(const tf2_ros::Buffer &tf, const geometry_msgs::PoseStamped &global_pose, std::vector<geometry_msgs::PoseStamped> &global_plan, double dist_behind_robot = 1);
   
     bool transformGlobalPlan(const tf2_ros::Buffer &tf, const std::vector<geometry_msgs::PoseStamped> &global_plan, const geometry_msgs::PoseStamped &global_pose, const costmap_2d::Costmap2D &costmap,
                              const std::string &global_frame, double max_plan_length, PlanCombined &transformed_plan_combined, int *current_goal_idx = nullptr,
                              geometry_msgs::TransformStamped *tf_plan_to_global = nullptr) const;
   
     bool transformAgentPlan(const tf2_ros::Buffer &tf2, const geometry_msgs::PoseStamped &robot_pose, const costmap_2d::Costmap2D &costmap, const std::string &global_frame,
                             const std::vector<geometry_msgs::PoseWithCovarianceStamped> &agent_plan, AgentPlanCombined &transformed_agent_plan_combined,
                             geometry_msgs::TwistStamped &transformed_agent_twist, tf2::Stamped<tf2::Transform> *tf_agent_plan_to_global = nullptr) const;
   
     static double estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped> &global_plan, const geometry_msgs::PoseStamped &local_goal, int current_goal_idx,
                                                const geometry_msgs::TransformStamped &tf_plan_to_global, int moving_average_length = 3);
   
     void saturateVelocity(double &vx, double &vy, double &omega, double max_vel_x, double max_vel_y, double max_vel_theta, double max_vel_x_backwards);
   
     static double convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius = 0);
   
     void configureBackupModes(std::vector<geometry_msgs::PoseStamped> &transformed_plan, int &goal_idx);
   
     static void validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist);
   
     // Agent Prediction reset
     void resetAgentsPrediction();
   
     bool tickTreeAndUpdatePlans(const geometry_msgs::PoseStamped &robot_pose, std::vector<AgentPlanCombined> &transformed_agent_plans, AgentPlanVelMap &transformed_agent_plan_vel_map);
   
     bool optimizeStandalone(cohan_msgs::Optimize::Request &req, cohan_msgs::Optimize::Response &res);
   
     void lookupTwist(const std::string &tracking_frame, const std::string &observation_frame, const ros::Time &time, const ros::Duration &averaging_interval, geometry_msgs::Twist &twist) const {
       // ref point is origin of tracking_frame, ref_frame = obs_frame
       lookupTwist(tracking_frame, observation_frame, observation_frame, tf2::Vector3(0, 0, 0), tracking_frame, time, averaging_interval, twist);
     }
   
     void lookupTwist(const std::string &tracking_frame, const std::string &observation_frame, const std::string &reference_frame, const tf2::Vector3 &reference_point,
                      const std::string &reference_point_frame, const ros::Time &time, const ros::Duration &averaging_interval, geometry_msgs::Twist &twist) const {
       ros::Time latest_time;
       ros::Time target_time;
   
       tf2::CompactFrameID target_id = tf_->_lookupFrameNumber(tf::strip_leading_slash(tracking_frame));
       tf2::CompactFrameID source_id = tf_->_lookupFrameNumber(tf::strip_leading_slash(observation_frame));
       tf_->_getLatestCommonTime(source_id, target_id, latest_time, nullptr);
   
       if (ros::Time() == time) {
         target_time = latest_time;
       } else {
         target_time = time;
       }
   
       ros::Time end_time = std::min(target_time + averaging_interval * 0.5, latest_time);
   
       ros::Time start_time = std::max(ros::Time().fromSec(.00001) + averaging_interval, end_time) - averaging_interval;  // don't collide with zero
       ros::Duration corrected_averaging_interval = end_time - start_time;                                                // correct for the possiblity that start time was
                                                                                                                          // truncated above.
       geometry_msgs::TransformStamped start_msg;
       geometry_msgs::TransformStamped end_msg;
       start_msg = tf_->lookupTransform(observation_frame, tracking_frame, start_time);
       end_msg = tf_->lookupTransform(observation_frame, tracking_frame, end_time);
   
       tf2::Stamped<tf2::Transform> start;
       tf2::Stamped<tf2::Transform> end;
       tf2::fromMsg(start_msg, start);
       tf2::fromMsg(end_msg, end);
   
       tf2::Matrix3x3 temp = start.getBasis().inverse() * end.getBasis();
       tf2::Quaternion quat_temp;
       temp.getRotation(quat_temp);
       tf2::Vector3 o = start.getBasis() * quat_temp.getAxis();
       tfScalar ang = quat_temp.getAngle();
   
       double delta_x = end.getOrigin().getX() - start.getOrigin().getX();
       double delta_y = end.getOrigin().getY() - start.getOrigin().getY();
       double delta_z = end.getOrigin().getZ() - start.getOrigin().getZ();
   
       tf2::Vector3 twist_vel((delta_x) / corrected_averaging_interval.toSec(), (delta_y) / corrected_averaging_interval.toSec(), (delta_z) / corrected_averaging_interval.toSec());
       tf2::Vector3 twist_rot = o * (ang / corrected_averaging_interval.toSec());
   
       // This is a twist w/ reference frame in observation_frame  and reference
       // point is in the tracking_frame at the origin (at start_time)
   
       // correct for the position of the reference frame
       tf2::Stamped<tf2::Transform> inverse;
       tf2::fromMsg(tf_->lookupTransform(reference_frame, tracking_frame, target_time), inverse);
       tf2::Vector3 out_rot = inverse.getBasis() * twist_rot;
       tf2::Vector3 out_vel = inverse.getBasis() * twist_vel + inverse.getOrigin().cross(out_rot);
   
       // Rereference the twist about a new reference point
       // Start by computing the original reference point in the reference frame:
       tf2::Stamped<tf2::Vector3> rp_orig(tf2::Vector3(0, 0, 0), target_time, tracking_frame);
       geometry_msgs::TransformStamped reference_frame_trans;
       tf2::fromMsg(tf_->lookupTransform(reference_frame, rp_orig.frame_id_, rp_orig.stamp_), reference_frame_trans);
   
       geometry_msgs::PointStamped rp_orig_msg;
       tf2::toMsg(rp_orig, rp_orig_msg);
       tf2::doTransform(rp_orig_msg, rp_orig_msg, reference_frame_trans);
   
       // convert the requrested reference point into the right frame
       tf2::Stamped<tf2::Vector3> rp_desired(reference_point, target_time, reference_point_frame);
       geometry_msgs::PointStamped rp_desired_msg;
       tf2::toMsg(rp_desired, rp_desired_msg);
       tf2::doTransform(rp_desired_msg, rp_desired_msg, reference_frame_trans);
       // compute the delta
       tf2::Vector3 delta = rp_desired - rp_orig;
       // Correct for the change in reference point
       out_vel = out_vel + out_rot * delta;
       // out_rot unchanged
   
       /*
       printf("KDL: Rotation %f %f %f, Translation:%f %f %f\n",
            out_rot.x(),out_rot.y(),out_rot.z(),
            out_vel.x(),out_vel.y(),out_vel.z());
     */
   
       twist.linear.x = out_vel.x();
       twist.linear.y = out_vel.y();
       twist.linear.z = out_vel.z();
       twist.angular.x = out_rot.x();
       twist.angular.y = out_rot.y();
       twist.angular.z = out_rot.z();
     }
   
    private:
     // Definition of member variables
   
     // external objects (store weak pointers)
     costmap_2d::Costmap2DROS *costmap_ros_;  
     costmap_2d::Costmap2D *costmap_;         
     tf2_ros::Buffer *tf_;                    
   
     // internal objects (memory management owned)
     PlannerInterfacePtr planner_;                                        
     ObstContainer obstacles_;                                            
     ViaPointContainer via_points_;                                       
     std::map<uint64_t, ViaPointContainer> agents_via_points_map_;        
     TebVisualizationPtr visualization_;                                  
     boost::shared_ptr<base_local_planner::CostmapModel> costmap_model_;  
     HATebConfig cfg_;                                                    
     HATebLocalPlannerReconfigureConfig config_;                          
     FailureDetector failure_detector_;                                   
   
     std::vector<geometry_msgs::PoseStamped> global_plan_;  
   
     base_local_planner::OdometryHelperRos odom_helper_;  
   
     pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_;  
     boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_;              
   
     boost::shared_ptr<dynamic_reconfigure::Server<HATebLocalPlannerReconfigureConfig>> dynamic_recfg_;  
   
     ros::Subscriber custom_obst_sub_;                          
     ros::Subscriber inv_humans_sub_;                           
     boost::mutex custom_obst_mutex_;                           
     boost::mutex inv_human_mutex_;                             
     costmap_converter::ObstacleArrayMsg custom_obstacle_msg_;  
     costmap_converter::ObstacleArrayMsg inv_humans_msg_;       
   
     ros::Subscriber via_points_sub_;  
     bool custom_via_points_active_;   
     boost::mutex via_point_mutex_;    
   
     PoseSE2 robot_pose_;                   
     PoseSE2 robot_goal_;                   
     geometry_msgs::Twist robot_vel_;       
     bool goal_reached_;                    
     bool horizon_reduced_;                 
     ros::Time horizon_reduced_stamp_;      
     ros::Time time_last_infeasible_plan_;  
     int no_infeasible_plans_;              
     ros::Time time_last_oscillation_;      
     RotType last_preferred_rotdir_;        
     geometry_msgs::Twist last_cmd_;        
   
     std::vector<geometry_msgs::Point> footprint_spec_;  
     double robot_inscribed_radius_;                     
     double robot_circumscribed_radius_;                 
   
     std::string global_frame_;      
     std::string robot_base_frame_;  
   
     bool initialized_;  
   
     // Agent prediction services and related variables
     ros::ServiceClient predict_agents_client_;             
     ros::ServiceClient reset_agents_prediction_client_;    
     ros::ServiceClient publish_predicted_markers_client_;  
   
     std::string predict_srv_name_;           
     std::string reset_prediction_srv_name_;  
     std::string publish_makers_srv_name_;    
   
     ros::ServiceServer optimize_server_;  
     ros::Time last_call_time_;            
     ros::Time last_omega_sign_change_;    
     double last_omega_;                   
   
     // Planning control flags
     bool goal_ctrl_;     
     bool reset_states_;  
   
     int isMode_;  
   
     std::string logs_;                        
     ros::Subscriber agents_sub_;              
     ros::Publisher log_pub_;                  
     std::string ns_;                          
     std::string invisible_humans_sub_topic_;  
   
     // Helper class instances
     std::shared_ptr<agents::Agents> agents_ptr_;  
     std::shared_ptr<Backoff> backoff_ptr_;        
     ModeSwitch bt_mode_switch_;                   
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   };  // end namespace hateb_local_planner
   
   #endif  // HATEB_LOCAL_PLANNER_ROS_H_
