
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_obstacle.h:

Program Listing for File edge_obstacle.h
========================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_obstacle.h>` (``hateb_local_planner/include/hateb_local_planner/g2o_types/edge_obstacle.h``)

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
    * Notes:
    * The following class is derived from a class defined by the
    * g2o-framework. g2o is licensed under the terms of the BSD License.
    * Refer to the base class source for detailed licensing information.
    *
    * Author: Christoph Rösmann
    *********************************************************************/
   #ifndef EDGE_OBSTACLE_H_
   #define EDGE_OBSTACLE_H_
   
   #include <cohan_msgs/AgentType.h>
   #include <hateb_local_planner/footprint_model.h>
   #include <hateb_local_planner/g2o_types/base_teb_edges.h>
   #include <hateb_local_planner/g2o_types/penalties.h>
   #include <hateb_local_planner/g2o_types/vertex_pose.h>
   #include <hateb_local_planner/hateb_config.h>
   #include <hateb_local_planner/obstacles.h>
   
   namespace hateb_local_planner {
   
   class EdgeObstacle : public BaseTebUnaryEdge<1, const Obstacle*, VertexPose> {
    public:
     EdgeObstacle() { _measurement = nullptr; }
   
     void computeError() override {
       ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setHATebConfig() on EdgeObstacle()");
       const auto* bandpt = static_cast<const VertexPose*>(_vertices[0]);
   
       double dist = 0;
   
       if (type_ == static_cast<int>(cohan_msgs::AgentType::ROBOT)) {
         dist = cfg_->robot_model->calculateDistance(bandpt->pose(), _measurement);
       }
   
       if (type_ == static_cast<int>(cohan_msgs::AgentType::HUMAN)) {
         dist = cfg_->human_model->calculateDistance(bandpt->pose(), _measurement);
       }
   
       // Original obstacle cost.
       _error[0] = penaltyBoundFromBelow(dist, cfg_->obstacles.min_obstacle_dist, cfg_->optim.penalty_epsilon);
       // std::cout << "_error[0] obstacle cost: "<<_error[0] << '\n';
       if (cfg_->optim.obstacle_cost_exponent != 1.0 && cfg_->obstacles.min_obstacle_dist > 0.0) {
         // Optional non-linear cost. Note the max cost (before weighting) is
         // the same as the straight line version and that all other costs are
         // below the straight line (for positive exponent), so it may be
         // necessary to increase weight_obstacle and/or the inflation_weight
         // when using larger exponents.
         _error[0] = cfg_->obstacles.min_obstacle_dist * std::pow(_error[0] / cfg_->obstacles.min_obstacle_dist, cfg_->optim.obstacle_cost_exponent);
         // _error[0] = penaltyBoundFromBelowExp(dist, cfg_->obstacles.min_obstacle_dist, cfg_->optim.penalty_epsilon,cfg_->obstacles.obstacle_cost_mult);
       }
   
       ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeObstacle::computeError() _error[0]=%f\n", _error[0]);
     }
   
   #ifdef USE_ANALYTIC_JACOBI
   #if 0
   
     void linearizeOplus() {
       ROS_ASSERT_MSG(cfg_, "You must call setHATebConfig on EdgePointObstacle()");
       const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
   
       Eigen::Vector2d deltaS = *_measurement - bandpt->position();
       double angdiff = atan2(deltaS[1], deltaS[0]) - bandpt->theta();
   
       double dist_squared = deltaS.squaredNorm();
       double dist = sqrt(dist_squared);
   
       double aux0 = sin(angdiff);
       double dev_left_border = penaltyBoundFromBelowDerivative(dist * fabs(aux0), cfg_->obstacles.min_obstacle_dist, cfg_->optim.penalty_epsilon);
   
       if (dev_left_border == 0) {
         _jacobianOplusXi(0, 0) = 0;
         _jacobianOplusXi(0, 1) = 0;
         _jacobianOplusXi(0, 2) = 0;
         return;
       }
   
       double aux1 = -fabs(aux0) / dist;
       double dev_norm_x = deltaS[0] * aux1;
       double dev_norm_y = deltaS[1] * aux1;
   
       double aux2 = cos(angdiff) * g2o::sign(aux0);
       double aux3 = aux2 / dist_squared;
       double dev_proj_x = aux3 * deltaS[1] * dist;
       double dev_proj_y = -aux3 * deltaS[0] * dist;
       double dev_proj_angle = -aux2;
   
       _jacobianOplusXi(0, 0) = dev_left_border * (dev_norm_x + dev_proj_x);
       _jacobianOplusXi(0, 1) = dev_left_border * (dev_norm_y + dev_proj_y);
       _jacobianOplusXi(0, 2) = dev_left_border * dev_proj_angle;
     }
   #endif
   #endif
   
     void setParameters(const HATebConfig& cfg, const Obstacle* obstacle, const int type) {
       cfg_ = &cfg;
       _measurement = obstacle;
       type_ = type;
     }
   
    protected:
     int type_;
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   class EdgeInflatedObstacle : public BaseTebUnaryEdge<2, const Obstacle*, VertexPose> {
    public:
     EdgeInflatedObstacle() { _measurement = nullptr; }
   
     void computeError() override {
       ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setHATebConfig() on EdgeInflatedObstacle()");
       const auto* bandpt = static_cast<const VertexPose*>(_vertices[0]);
   
       double dist = 0;
   
       if (type_ == static_cast<int>(cohan_msgs::AgentType::ROBOT)) {
         dist = cfg_->robot_model->calculateDistance(bandpt->pose(), _measurement);
       }
   
       if (type_ == static_cast<int>(cohan_msgs::AgentType::HUMAN)) {
         dist = cfg_->human_model->calculateDistance(bandpt->pose(), _measurement);
       }
   
       // Original "straight line" obstacle cost. The max possible value
       // before weighting is min_obstacle_dist
       _error[0] = penaltyBoundFromBelow(dist, cfg_->obstacles.min_obstacle_dist, cfg_->optim.penalty_epsilon);
       // std::cout << "_error[0] inflated obstacle cost: "<<_error[0] << '\n';
   
       if (cfg_->optim.obstacle_cost_exponent != 1.0 && cfg_->obstacles.min_obstacle_dist > 0.0) {
         // Optional non-linear cost. Note the max cost (before weighting) is
         // the same as the straight line version and that all other costs are
         // below the straight line (for positive exponent), so it may be
         // necessary to increase weight_obstacle and/or the inflation_weight
         // when using larger exponents.
         _error[0] = cfg_->obstacles.min_obstacle_dist * std::pow(_error[0] / cfg_->obstacles.min_obstacle_dist, cfg_->optim.obstacle_cost_exponent);
       }
   
       // Additional linear inflation cost
       _error[1] = penaltyBoundFromBelow(dist, cfg_->obstacles.inflation_dist, 0.0);
   
       ROS_ASSERT_MSG(std::isfinite(_error[0]) && std::isfinite(_error[1]), "EdgeInflatedObstacle::computeError() _error[0]=%f, _error[1]=%f\n", _error[0], _error[1]);
     }
   
     void setParameters(const HATebConfig& cfg, const Obstacle* obstacle, const int type) {
       cfg_ = &cfg;
       _measurement = obstacle;
       type_ = type;
     }
   
    protected:
     int type_;
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   }  // namespace hateb_local_planner
   
   #endif
