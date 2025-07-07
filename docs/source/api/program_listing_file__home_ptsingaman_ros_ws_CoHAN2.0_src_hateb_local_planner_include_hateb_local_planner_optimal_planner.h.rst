
.. _program_listing_file__home_ptsingaman_ros_ws_CoHAN2.0_src_hateb_local_planner_include_hateb_local_planner_optimal_planner.h:

Program Listing for File optimal_planner.h
==========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ptsingaman_ros_ws_CoHAN2.0_src_hateb_local_planner_include_hateb_local_planner_optimal_planner.h>` (``/hateb_local_planner/include/hateb_local_planner/optimal_planner.h``)

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
   
   #ifndef OPTIMAL_PLANNER_H_
   #define OPTIMAL_PLANNER_H_
   
   #include <cmath>
   
   // teb stuff
   #include <hateb_local_planner/footprint_model.h>
   #include <hateb_local_planner/hateb_config.h>
   #include <hateb_local_planner/misc.h>
   #include <hateb_local_planner/planner_interface.h>
   #include <hateb_local_planner/timed_elastic_band.h>
   #include <hateb_local_planner/visualization.h>
   
   // g2o lib stuff
   #include <g2o/core/block_solver.h>
   #include <g2o/core/factory.h>
   #include <g2o/core/optimization_algorithm_gauss_newton.h>
   #include <g2o/core/optimization_algorithm_levenberg.h>
   #include <g2o/core/sparse_optimizer.h>
   #include <g2o/solvers/cholmod/linear_solver_cholmod.h>
   #include <g2o/solvers/csparse/linear_solver_csparse.h>
   
   // g2o custom edges and vertices for the HATEB planner
   #include <hateb_local_planner/g2o_types/edge_acceleration.h>
   #include <hateb_local_planner/g2o_types/edge_agent_agent_safety.h>
   #include <hateb_local_planner/g2o_types/edge_agent_robot_rel_velocity.h>
   #include <hateb_local_planner/g2o_types/edge_agent_robot_safety.h>
   #include <hateb_local_planner/g2o_types/edge_agent_robot_visibility.h>
   #include <hateb_local_planner/g2o_types/edge_dynamic_obstacle.h>
   #include <hateb_local_planner/g2o_types/edge_invisible_human.h>
   #include <hateb_local_planner/g2o_types/edge_kinematics.h>
   #include <hateb_local_planner/g2o_types/edge_obstacle.h>
   #include <hateb_local_planner/g2o_types/edge_prefer_rotdir.h>
   #include <hateb_local_planner/g2o_types/edge_shortest_path.h>
   #include <hateb_local_planner/g2o_types/edge_static_agent_visibility.h>
   #include <hateb_local_planner/g2o_types/edge_time_optimal.h>
   #include <hateb_local_planner/g2o_types/edge_velocity.h>
   #include <hateb_local_planner/g2o_types/edge_via_point.h>
   
   // messages
   #include <cohan_msgs/AgentType.h>
   #include <cohan_msgs/Trajectory.h>
   #include <geometry_msgs/PoseStamped.h>
   #include <nav_msgs/Odometry.h>
   #include <nav_msgs/Path.h>
   #include <tf/transform_datatypes.h>
   
   #include <climits>
   
   namespace hateb_local_planner {
   
   using TEBBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
   
   using TEBLinearSolver = g2o::LinearSolverCholmod<TEBBlockSolver::PoseMatrixType>;
   
   using ViaPointContainer = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;
   
   class TebOptimalPlanner : public PlannerInterface {
    public:
     TebOptimalPlanner();
   
     explicit TebOptimalPlanner(const HATebConfig& cfg, ObstContainer* obstacles = nullptr, FootprintModelPtr robot_model = boost::make_shared<PointFootprint>(),
                                TebVisualizationPtr visual = TebVisualizationPtr(), const ViaPointContainer* via_points = nullptr,
                                CircularFootprintPtr agent_model = boost::make_shared<CircularFootprint>(), const std::map<uint64_t, ViaPointContainer>* agents_via_points_map = nullptr);
   
     ~TebOptimalPlanner() override;
   
     void initialize(const HATebConfig& cfg, ObstContainer* obstacles = nullptr, FootprintModelPtr robot_model = boost::make_shared<PointFootprint>(), TebVisualizationPtr visual = TebVisualizationPtr(),
                     const ViaPointContainer* via_points = nullptr, CircularFootprintPtr agent_model = boost::make_shared<CircularFootprint>(),
                     const std::map<uint64_t, ViaPointContainer>* agents_via_points_map = nullptr);
   
   
     bool plan(const std::vector<geometry_msgs::PoseStamped>& initial_plan, const geometry_msgs::Twist* start_vel = nullptr, bool free_goal_vel = false,
               const AgentPlanVelMap* initial_agent_plan_vel_map = nullptr, hateb_local_planner::OptimizationCostArray* op_costs = nullptr, double dt_ref = 0.4, double dt_hyst = 0.1,
               int Mode = 0) override;
   
     bool plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::Twist* start_vel = nullptr, bool free_goal_vel = false, hateb_local_planner::OptimizationCostArray* op_costs = nullptr,
               double dt_ref = 0.4, double dt_hyst = 0.1, int Mode = 0) override;
   
     bool plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::Twist* start_vel = nullptr, bool free_goal_vel = false, double pre_plan_time = 0.0,
               hateb_local_planner::OptimizationCostArray* op_costs = nullptr, double dt_ref = 0.4, double dt_hyst = 0.1, int Mode = 0) override;
   
     bool getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses, double dt_ref) const override;
   
     bool optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards = true, double obst_cost_scale = 1.0, double viapoint_cost_scale = 1.0,
                      bool alternative_time_cost = false, hateb_local_planner::OptimizationCostArray* op_costs = nullptr, double dt_ref = 0.4, double dt_hyst = 0.1);
   
     bool optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards = true, double obst_cost_scale = 1.0, double viapoint_cost_scale = 1.0,
                      bool alternative_time_cost = false, hateb_local_planner::OptimizationCostArray* op_costs = nullptr);
   
   
   
     void setVelocityStart(const geometry_msgs::Twist& vel_start);
   
     void setVelocityGoal(const geometry_msgs::Twist& vel_goal);
   
     void setVelocityGoalFree() { vel_goal_.first = false; }
   
   
   
     void setObstVector(ObstContainer* obst_vector) { obstacles_ = obst_vector; }
   
     const ObstContainer& getObstVector() const { return *obstacles_; }
   
   
   
     void setViaPoints(const ViaPointContainer* via_points) { via_points_ = via_points; }
   
     const ViaPointContainer& getViaPoints() const { return *via_points_; }
   
   
   
     void setVisualization(TebVisualizationPtr visualization);
   
     void visualize() override;
   
   
   
     void clearPlanner() override {
       clearGraph();
       teb_.clearTimedElasticBand();
       for (auto& agent_teb : agents_tebs_map_) {
         agent_teb.second.clearTimedElasticBand();
       }
     }
   
     void setPreferredTurningDir(RotType dir) override { prefer_rotdir_ = dir; }
   
     static void registerG2OTypes();
   
     TimedElasticBand& teb() { return teb_; };
   
     const TimedElasticBand& teb() const { return teb_; };
   
     boost::shared_ptr<g2o::SparseOptimizer> optimizer() { return optimizer_; };
   
     boost::shared_ptr<const g2o::SparseOptimizer> optimizer() const { return optimizer_; };
   
     bool isOptimized() const { return optimized_; };
   
     void computeCurrentCost(double obst_cost_scale = 1.0, double viapoint_cost_scale = 1.0, bool alternative_time_cost = false, hateb_local_planner::OptimizationCostArray* op_costs = NULL);
   
     virtual void computeCurrentCost(std::vector<double>& cost, double obst_cost_scale = 1.0, double viapoint_cost_scale = 1.0, bool alternative_time_cost = false) {
       computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
       cost.push_back(getCurrentCost());
     }
   
     double getCurrentCost() const { return cost_; }
   
     inline void extractVelocity(const PoseSE2& pose1, const PoseSE2& pose2, double dt, double& vx, double& vy, double& omega) const;
   
     void getVelocityProfile(std::vector<geometry_msgs::Twist>& velocity_profile) const;
   
     cohan_msgs::Trajectory getFullTrajectory() const override;
   
     cohan_msgs::Trajectory getFullAgentTrajectory(uint64_t agent_id) override;
   
     bool isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec, double inscribed_radius = 0.0, double circumscribed_radius = 0.0,
                               int look_ahead_idx = -1) override;
   
   
    protected:
   
     bool buildGraph(double weight_multiplier = 1.0);
   
     bool optimizeGraph(int no_iterations, bool clear_after = true);
   
     void clearGraph();
   
     void AddTEBVertices();
   
     void AddEdgesVelocity();
   
     void AddEdgesVelocityForAgents();
   
     void AddEdgesAcceleration();
   
     void AddEdgesAccelerationForAgents();
   
     void AddEdgesTimeOptimal();
   
     void AddEdgesTimeOptimalForAgents();
   
     void AddEdgesShortestPath();
   
     void AddEdgesObstacles(double weight_multiplier = 1.0);
   
     void AddEdgesObstaclesLegacy(double weight_multiplier = 1.0);
   
     void AddEdgesObstaclesForAgents();
   
     void AddEdgesViaPoints();
   
     void AddEdgesViaPointsForAgents();
   
     void AddEdgesDynamicObstacles(double weight_multiplier = 1.0);
   
     void AddEdgesDynamicObstaclesForAgents(double weight_multiplier = 1.0);
   
     void AddEdgesInvisibleHumans(double weight_multiplier = 1.0);
   
     void AddEdgesStaticAgentVisibility();
   
     void AddEdgesKinematicsDiffDrive();
   
     void AddEdgesKinematicsDiffDriveForAgents();
   
     void AddEdgesKinematicsCarlike();
   
     void AddEdgesKinematicsCarlikeForAgents();
     void AddEdgesPreferRotDir();
   
     void AddEdgesAgentRobotSafety();
   
     void AddEdgesAgentAgentSafety();
   
     void AddEdgesAgentRobotRelVelocity();
   
     void AddEdgesAgentRobotVisibility();
   
   
     static boost::shared_ptr<g2o::SparseOptimizer> initOptimizer();
   
     // external objects (store weak pointers)
     const HATebConfig* cfg_;                                              
     ObstContainer* obstacles_;                                            
     const ViaPointContainer* via_points_;                                 
     const std::map<uint64_t, ViaPointContainer>* agents_via_points_map_;  
   
     // internal objects (memory management owned)
     TebVisualizationPtr visualization_;                     
     TimedElasticBand teb_;                                  
     std::map<uint64_t, TimedElasticBand> agents_tebs_map_;  
     FootprintModelPtr robot_model_;                         
     CircularFootprintPtr agent_model_;                      
     boost::shared_ptr<g2o::SparseOptimizer> optimizer_;     
   
     std::map<uint64_t, std::pair<bool, geometry_msgs::Twist>> agents_vel_start_;  
     std::map<uint64_t, std::pair<bool, geometry_msgs::Twist>> agents_vel_goal_;   
     std::pair<bool, geometry_msgs::Twist> vel_start_;                             
     std::pair<bool, geometry_msgs::Twist> vel_goal_;                              
     std::vector<geometry_msgs::Pose> static_agents_;                              
     bool initialized_;                                                            
     bool optimized_;                                                              
     double agent_radius_;                                                         
     double robot_radius_;                                                         
     int isMode_;                                                                  
     std::vector<double> agent_nominal_vels_;                                      
     double current_agent_robot_min_dist_;                                         
     double cost_;                                                                 
     RotType prefer_rotdir_;  
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   using TebOptimalPlannerPtr = boost::shared_ptr<TebOptimalPlanner>;
   using TebOptimalPlannerConstPtr = boost::shared_ptr<const TebOptimalPlanner>;
   using TebOptPlannerContainer = std::vector<TebOptimalPlannerPtr>;
   
   }  // namespace hateb_local_planner
   
   #endif /* OPTIMAL_PLANNER_H_ */
