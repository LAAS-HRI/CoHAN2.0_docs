
.. _program_listing_file__home_ptsingaman_ros_ws_CoHAN2.0_src_hateb_local_planner_include_hateb_local_planner_planner_interface.h:

Program Listing for File planner_interface.h
============================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ptsingaman_ros_ws_CoHAN2.0_src_hateb_local_planner_include_hateb_local_planner_planner_interface.h>` (``/hateb_local_planner/include/hateb_local_planner/planner_interface.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*********************************************************************
    *
    * Software License Agreement (BSD License)
    *
    *  Copyright (c) 2016,
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
   
   #ifndef PLANNER_INTERFACE_H_
   #define PLANNER_INTERFACE_H_
   
   // boost
   #include <boost/shared_ptr.hpp>
   
   // ros
   #include <base_local_planner/costmap_model.h>
   #include <tf/transform_datatypes.h>
   
   // this package
   #include <hateb_local_planner/pose_se2.h>
   
   // messages
   #include <cohan_msgs/AgentPath.h>
   #include <cohan_msgs/Trajectory.h>
   #include <geometry_msgs/PoseArray.h>
   #include <geometry_msgs/PoseStamped.h>
   #include <geometry_msgs/TwistStamped.h>
   #include <hateb_local_planner/OptimizationCostArray.h>
   
   namespace hateb_local_planner {
   
   using PlanStartVelGoalVel = struct {
     std::vector<geometry_msgs::PoseStamped> plan;
     geometry_msgs::Twist start_vel;
     geometry_msgs::Twist goal_vel;
     double nominal_vel;
     int isMode;
   };
   
   using AgentPlanVelMap = std::map<uint64_t, PlanStartVelGoalVel>;
   
   class PlannerInterface {
    public:
     PlannerInterface() = default;
     virtual ~PlannerInterface() = default;
   
   
     virtual bool plan(const std::vector<geometry_msgs::PoseStamped>& initial_plan, const geometry_msgs::Twist* start_vel = nullptr, bool free_goal_vel = false,
                       const AgentPlanVelMap* initial_agent_plan_vels = nullptr, hateb_local_planner::OptimizationCostArray* op_costs = nullptr, double dt_ref = 0.4, double dt_hyst = 0.1,
                       int Mode = 0) = 0;
   
     virtual bool plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::Twist* start_vel = nullptr, bool free_goal_vel = false,
                       hateb_local_planner::OptimizationCostArray* op_costs = nullptr, double dt_ref = 0.4, double dt_hyst = 0.1, int Mode = 0) = 0;
   
     virtual bool plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::Twist* start_vel = nullptr, bool free_goal_vel = false, double pre_plan_time = 0.0,
                       hateb_local_planner::OptimizationCostArray* op_costs = nullptr, double dt_ref = 0.4, double dt_hyst = 0.1, int Mode = 0) = 0;
   
     virtual bool getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses, double dt_ref) const = 0;
   
   
     virtual void clearPlanner() = 0;
   
     virtual void setPreferredTurningDir(RotType dir) { ROS_WARN("setPreferredTurningDir() not implemented for this planner."); }
   
     virtual void visualize() {}
   
     virtual bool isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec, double inscribed_radius = 0.0,
                                       double circumscribed_radius = 0.0, int look_ahead_idx = -1) = 0;
   
     virtual void computeCurrentCost(std::vector<double>& cost, double obst_cost_scale = 1.0, bool alternative_time_cost = false) {}
     virtual cohan_msgs::Trajectory getFullTrajectory() const = 0;
     virtual cohan_msgs::Trajectory getFullAgentTrajectory(uint64_t agent_id) = 0;
   
     double local_weight_optimaltime_;
   };
   
   using PlannerInterfacePtr = boost::shared_ptr<PlannerInterface>;
   
   }  // namespace hateb_local_planner
   
   #endif /* PLANNER_INTERFACE_H__ */
