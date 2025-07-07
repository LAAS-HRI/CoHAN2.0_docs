
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_acceleration.h:

Program Listing for File edge_acceleration.h
============================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_acceleration.h>` (``hateb_local_planner/include/hateb_local_planner/g2o_types/edge_acceleration.h``)

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
   
   #ifndef EDGE_ACCELERATION_H_
   #define EDGE_ACCELERATION_H_
   
   #include <geometry_msgs/Twist.h>
   #include <hateb_local_planner/g2o_types/base_teb_edges.h>
   #include <hateb_local_planner/g2o_types/penalties.h>
   #include <hateb_local_planner/g2o_types/vertex_pose.h>
   #include <hateb_local_planner/g2o_types/vertex_timediff.h>
   #include <hateb_local_planner/hateb_config.h>
   
   namespace hateb_local_planner {
   
   class EdgeAcceleration : public BaseTebMultiEdge<2, double> {
    public:
     EdgeAcceleration() { this->resize(5); }
   
     void computeError() override {
       ROS_ASSERT_MSG(cfg_, "You must call setHATebConfig on EdgeAcceleration()");
       const auto* pose1 = static_cast<const VertexPose*>(_vertices[0]);
       const auto* pose2 = static_cast<const VertexPose*>(_vertices[1]);
       const auto* pose3 = static_cast<const VertexPose*>(_vertices[2]);
       const auto* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
       const auto* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);
   
       // VELOCITY & ACCELERATION
       const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
       const Eigen::Vector2d diff2 = pose3->position() - pose2->position();
   
       double dist1 = diff1.norm();
       double dist2 = diff2.norm();
       const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
       const double angle_diff2 = g2o::normalize_theta(pose3->theta() - pose2->theta());
   
       if (cfg_->trajectory.exact_arc_length)  // use exact arc length instead of Euclidean approximation
       {
         if (angle_diff1 != 0) {
           const double radius = dist1 / (2 * sin(angle_diff1 / 2));
           dist1 = fabs(angle_diff1 * radius);  // actual arg length!
         }
         if (angle_diff2 != 0) {
           const double radius = dist2 / (2 * sin(angle_diff2 / 2));
           dist2 = fabs(angle_diff2 * radius);  // actual arg length!
         }
       }
   
       double vel1 = dist1 / dt1->dt();
       double vel2 = dist2 / dt2->dt();
   
       // consider directions
       vel1 *= fast_sigmoid(100 * (diff1.x() * cos(pose1->theta()) + diff1.y() * sin(pose1->theta())));
       vel2 *= fast_sigmoid(100 * (diff2.x() * cos(pose2->theta()) + diff2.y() * sin(pose2->theta())));
   
       const double acc_lin = (vel2 - vel1) * 2 / (dt1->dt() + dt2->dt());
   
       _error[0] = penaltyBoundToInterval(acc_lin, cfg_->robot.acc_lim_x, cfg_->optim.penalty_epsilon);
   
       // ANGULAR ACCELERATION
       const double omega1 = angle_diff1 / dt1->dt();
       const double omega2 = angle_diff2 / dt2->dt();
       const double acc_rot = (omega2 - omega1) * 2 / (dt1->dt() + dt2->dt());
   
       _error[1] = penaltyBoundToInterval(acc_rot, cfg_->robot.acc_lim_theta, cfg_->optim.penalty_epsilon);
   
       ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAcceleration::computeError() translational: _error[0]=%f\n", _error[0]);
       ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAcceleration::computeError() rotational: _error[1]=%f\n", _error[1]);
     }
   
   #ifdef USE_ANALYTIC_JACOBI
   #if 0
     /*
      * @brief Jacobi matrix of the cost function specified in computeError().
      */
     void linearizeOplus() {
       ROS_ASSERT_MSG(cfg_, "You must call setHATebConfig on EdgeAcceleration()");
       const VertexPointXY* conf1 = static_cast<const VertexPointXY*>(_vertices[0]);
       const VertexPointXY* conf2 = static_cast<const VertexPointXY*>(_vertices[1]);
       const VertexPointXY* conf3 = static_cast<const VertexPointXY*>(_vertices[2]);
       const VertexTimeDiff* deltaT1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
       const VertexTimeDiff* deltaT2 = static_cast<const VertexTimeDiff*>(_vertices[4]);
       const VertexOrientation* angle1 = static_cast<const VertexOrientation*>(_vertices[5]);
       const VertexOrientation* angle2 = static_cast<const VertexOrientation*>(_vertices[6]);
       const VertexOrientation* angle3 = static_cast<const VertexOrientation*>(_vertices[7]);
   
       Eigen::Vector2d deltaS1 = conf2->estimate() - conf1->estimate();
       Eigen::Vector2d deltaS2 = conf3->estimate() - conf2->estimate();
       double dist1 = deltaS1.norm();
       double dist2 = deltaS2.norm();
   
       double sum_time = deltaT1->estimate() + deltaT2->estimate();
       double sum_time_inv = 1 / sum_time;
       double dt1_inv = 1 / deltaT1->estimate();
       double dt2_inv = 1 / deltaT2->estimate();
       double aux0 = 2 / sum_time_inv;
       double aux1 = dist1 * deltaT1->estimate();
       double aux2 = dist2 * deltaT2->estimate();
   
       double vel1 = dist1 * dt1_inv;
       double vel2 = dist2 * dt2_inv;
       double omega1 = g2o::normalize_theta(angle2->estimate() - angle1->estimate()) * dt1_inv;
       double omega2 = g2o::normalize_theta(angle3->estimate() - angle2->estimate()) * dt2_inv;
       double acc = (vel2 - vel1) * aux0;
       double omegadot = (omega2 - omega1) * aux0;
       double aux3 = -acc / 2;
       double aux4 = -omegadot / 2;
   
       double dev_border_acc = penaltyBoundToIntervalDerivative(acc, HATebConfig.robot_acceleration_max_trans, optimizationConfig.optimization_boundaries_epsilon,
                                                                optimizationConfig.optimization_boundaries_scale, optimizationConfig.optimization_boundaries_order);
       double dev_border_omegadot = penaltyBoundToIntervalDerivative(omegadot, HATebConfig.robot_acceleration_max_rot, optimizationConfig.optimization_boundaries_epsilon,
                                                                     optimizationConfig.optimization_boundaries_scale, optimizationConfig.optimization_boundaries_order);
   
       _jacobianOplus[0].resize(2, 2);  // conf1
       _jacobianOplus[1].resize(2, 2);  // conf2
       _jacobianOplus[2].resize(2, 2);  // conf3
       _jacobianOplus[3].resize(2, 1);  // deltaT1
       _jacobianOplus[4].resize(2, 1);  // deltaT2
       _jacobianOplus[5].resize(2, 1);  // angle1
       _jacobianOplus[6].resize(2, 1);  // angle2
       _jacobianOplus[7].resize(2, 1);  // angle3
   
       if (aux1 == 0) aux1 = 1e-20;
       if (aux2 == 0) aux2 = 1e-20;
   
       if (dev_border_acc != 0) {
         // TODO: double aux = aux0 * dev_border_acc;
         // double aux123 = aux / aux1;
         _jacobianOplus[0](0, 0) = aux0 * deltaS1[0] / aux1 * dev_border_acc;                         // acc x1
         _jacobianOplus[0](0, 1) = aux0 * deltaS1[1] / aux1 * dev_border_acc;                         // acc y1
         _jacobianOplus[1](0, 0) = -aux0 * (deltaS1[0] / aux1 + deltaS2[0] / aux2) * dev_border_acc;  // acc x2
         _jacobianOplus[1](0, 1) = -aux0 * (deltaS1[1] / aux1 + deltaS2[1] / aux2) * dev_border_acc;  // acc y2
         _jacobianOplus[2](0, 0) = aux0 * deltaS2[0] / aux2 * dev_border_acc;                         // acc x3
         _jacobianOplus[2](0, 1) = aux0 * deltaS2[1] / aux2 * dev_border_acc;                         // acc y3
         _jacobianOplus[2](0, 0) = 0;
         _jacobianOplus[2](0, 1) = 0;
         _jacobianOplus[3](0, 0) = aux0 * (aux3 + vel1 * dt1_inv) * dev_border_acc;  // acc deltaT1
         _jacobianOplus[4](0, 0) = aux0 * (aux3 - vel2 * dt2_inv) * dev_border_acc;  // acc deltaT2
       } else {
         _jacobianOplus[0](0, 0) = 0;  // acc x1
         _jacobianOplus[0](0, 1) = 0;  // acc y1
         _jacobianOplus[1](0, 0) = 0;  // acc x2
         _jacobianOplus[1](0, 1) = 0;  // acc y2
         _jacobianOplus[2](0, 0) = 0;  // acc x3
         _jacobianOplus[2](0, 1) = 0;  // acc y3
         _jacobianOplus[3](0, 0) = 0;  // acc deltaT1
         _jacobianOplus[4](0, 0) = 0;  // acc deltaT2
       }
   
       if (dev_border_omegadot != 0) {
         _jacobianOplus[3](1, 0) = aux0 * (aux4 + omega1 * dt1_inv) * dev_border_omegadot;  // omegadot deltaT1
         _jacobianOplus[4](1, 0) = aux0 * (aux4 - omega2 * dt2_inv) * dev_border_omegadot;  // omegadot deltaT2
         _jacobianOplus[5](1, 0) = aux0 * dt1_inv * dev_border_omegadot;                    // omegadot angle1
         _jacobianOplus[6](1, 0) = -aux0 * (dt1_inv + dt2_inv) * dev_border_omegadot;       // omegadot angle2
         _jacobianOplus[7](1, 0) = aux0 * dt2_inv * dev_border_omegadot;                    // omegadot angle3
       } else {
         _jacobianOplus[3](1, 0) = 0;  // omegadot deltaT1
         _jacobianOplus[4](1, 0) = 0;  // omegadot deltaT2
         _jacobianOplus[5](1, 0) = 0;  // omegadot angle1
         _jacobianOplus[6](1, 0) = 0;  // omegadot angle2
         _jacobianOplus[7](1, 0) = 0;  // omegadot angle3
       }
   
       _jacobianOplus[0](1, 0) = 0;  // omegadot x1
       _jacobianOplus[0](1, 1) = 0;  // omegadot y1
       _jacobianOplus[1](1, 0) = 0;  // omegadot x2
       _jacobianOplus[1](1, 1) = 0;  // omegadot y2
       _jacobianOplus[2](1, 0) = 0;  // omegadot x3
       _jacobianOplus[2](1, 1) = 0;  // omegadot y3
       _jacobianOplus[5](0, 0) = 0;  // acc angle1
       _jacobianOplus[6](0, 0) = 0;  // acc angle2
       _jacobianOplus[7](0, 0) = 0;  // acc angle3
     }
   #endif
   #endif
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   class EdgeAccelerationStart : public BaseTebMultiEdge<2, const geometry_msgs::Twist*> {
    public:
     EdgeAccelerationStart() {
       _measurement = nullptr;
       this->resize(3);
     }
   
     void computeError() override {
       ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setHATebConfig() and setStartVelocity() on EdgeAccelerationStart()");
       const auto* pose1 = static_cast<const VertexPose*>(_vertices[0]);
       const auto* pose2 = static_cast<const VertexPose*>(_vertices[1]);
       const auto* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);
   
       // VELOCITY & ACCELERATION
       const Eigen::Vector2d diff = pose2->position() - pose1->position();
       double dist = diff.norm();
       const double angle_diff = g2o::normalize_theta(pose2->theta() - pose1->theta());
       if (cfg_->trajectory.exact_arc_length && angle_diff != 0) {
         const double radius = dist / (2 * sin(angle_diff / 2));
         dist = fabs(angle_diff * radius);  // actual arg length!
       }
   
       const double vel1 = _measurement->linear.x;
       double vel2 = dist / dt->dt();
   
       // consider directions
       vel2 *= fast_sigmoid(100 * (diff.x() * cos(pose1->theta()) + diff.y() * sin(pose1->theta())));
   
       const double acc_lin = (vel2 - vel1) / dt->dt();
   
       _error[0] = penaltyBoundToInterval(acc_lin, cfg_->robot.acc_lim_x, cfg_->optim.penalty_epsilon);
   
       // ANGULAR ACCELERATION
       const double omega1 = _measurement->angular.z;
       const double omega2 = angle_diff / dt->dt();
       const double acc_rot = (omega2 - omega1) / dt->dt();
   
       _error[1] = penaltyBoundToInterval(acc_rot, cfg_->robot.acc_lim_theta, cfg_->optim.penalty_epsilon);
   
       ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationStart::computeError() translational: _error[0]=%f\n", _error[0]);
       ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationStart::computeError() rotational: _error[1]=%f\n", _error[1]);
     }
   
     void setInitialVelocity(const geometry_msgs::Twist& vel_start) { _measurement = &vel_start; }
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   class EdgeAccelerationGoal : public BaseTebMultiEdge<2, const geometry_msgs::Twist*> {
    public:
     EdgeAccelerationGoal() {
       _measurement = nullptr;
       this->resize(3);
     }
   
     void computeError() override {
       ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setHATebConfig() and setGoalVelocity() on EdgeAccelerationGoal()");
       const auto* pose_pre_goal = static_cast<const VertexPose*>(_vertices[0]);
       const auto* pose_goal = static_cast<const VertexPose*>(_vertices[1]);
       const auto* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);
   
       // VELOCITY & ACCELERATION
   
       const Eigen::Vector2d diff = pose_goal->position() - pose_pre_goal->position();
       double dist = diff.norm();
       const double angle_diff = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta());
       if (cfg_->trajectory.exact_arc_length && angle_diff != 0) {
         double radius = dist / (2 * sin(angle_diff / 2));
         dist = fabs(angle_diff * radius);  // actual arg length!
       }
   
       double vel1 = dist / dt->dt();
       const double vel2 = _measurement->linear.x;
   
       // consider directions
       vel1 *= fast_sigmoid(100 * (diff.x() * cos(pose_pre_goal->theta()) + diff.y() * sin(pose_pre_goal->theta())));
   
       const double acc_lin = (vel2 - vel1) / dt->dt();
   
       _error[0] = penaltyBoundToInterval(acc_lin, cfg_->robot.acc_lim_x, cfg_->optim.penalty_epsilon);
   
       // ANGULAR ACCELERATION
       const double omega1 = angle_diff / dt->dt();
       const double omega2 = _measurement->angular.z;
       const double acc_rot = (omega2 - omega1) / dt->dt();
   
       _error[1] = penaltyBoundToInterval(acc_rot, cfg_->robot.acc_lim_theta, cfg_->optim.penalty_epsilon);
   
       ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationGoal::computeError() translational: _error[0]=%f\n", _error[0]);
       ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationGoal::computeError() rotational: _error[1]=%f\n", _error[1]);
     }
   
     void setGoalVelocity(const geometry_msgs::Twist& vel_goal) { _measurement = &vel_goal; }
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   class EdgeAccelerationHolonomic : public BaseTebMultiEdge<3, double> {
    public:
     EdgeAccelerationHolonomic() { this->resize(5); }
   
     void computeError() override {
       ROS_ASSERT_MSG(cfg_, "You must call setHATebConfig on EdgeAcceleration()");
       const auto* pose1 = static_cast<const VertexPose*>(_vertices[0]);
       const auto* pose2 = static_cast<const VertexPose*>(_vertices[1]);
       const auto* pose3 = static_cast<const VertexPose*>(_vertices[2]);
       const auto* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
       const auto* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);
   
       // VELOCITY & ACCELERATION
       Eigen::Vector2d diff1 = pose2->position() - pose1->position();
       Eigen::Vector2d diff2 = pose3->position() - pose2->position();
   
       double cos_theta1 = std::cos(pose1->theta());
       double sin_theta1 = std::sin(pose1->theta());
       double cos_theta2 = std::cos(pose2->theta());
       double sin_theta2 = std::sin(pose2->theta());
   
       // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
       double p1_dx = (cos_theta1 * diff1.x()) + (sin_theta1 * diff1.y());
       double p1_dy = (-sin_theta1 * diff1.x()) + (cos_theta1 * diff1.y());
       // transform pose3 into robot frame pose2 (inverse 2d rotation matrix)
       double p2_dx = (cos_theta2 * diff2.x()) + (sin_theta2 * diff2.y());
       double p2_dy = (-sin_theta2 * diff2.x()) + (cos_theta2 * diff2.y());
   
       double vel1_x = p1_dx / dt1->dt();
       double vel1_y = p1_dy / dt1->dt();
       double vel2_x = p2_dx / dt2->dt();
       double vel2_y = p2_dy / dt2->dt();
   
       double dt12 = dt1->dt() + dt2->dt();
   
       double acc_x = (vel2_x - vel1_x) * 2 / dt12;
       double acc_y = (vel2_y - vel1_y) * 2 / dt12;
   
       _error[0] = penaltyBoundToInterval(acc_x, cfg_->robot.acc_lim_x, cfg_->optim.penalty_epsilon);
       _error[1] = penaltyBoundToInterval(acc_y, cfg_->robot.acc_lim_y, cfg_->optim.penalty_epsilon);
   
       // ANGULAR ACCELERATION
       double omega1 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt1->dt();
       double omega2 = g2o::normalize_theta(pose3->theta() - pose2->theta()) / dt2->dt();
       double acc_rot = (omega2 - omega1) * 2 / dt12;
   
       _error[2] = penaltyBoundToInterval(acc_rot, cfg_->robot.acc_lim_theta, cfg_->optim.penalty_epsilon);
   
       ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAcceleration::computeError() translational: _error[0]=%f\n", _error[0]);
       ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAcceleration::computeError() strafing: _error[1]=%f\n", _error[1]);
       ROS_ASSERT_MSG(std::isfinite(_error[2]), "EdgeAcceleration::computeError() rotational: _error[2]=%f\n", _error[2]);
     }
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   class EdgeAccelerationHolonomicStart : public BaseTebMultiEdge<3, const geometry_msgs::Twist*> {
    public:
     EdgeAccelerationHolonomicStart() {
       this->resize(3);
       _measurement = nullptr;
     }
   
     void computeError() override {
       ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setHATebConfig() and setStartVelocity() on EdgeAccelerationStart()");
       const auto* pose1 = static_cast<const VertexPose*>(_vertices[0]);
       const auto* pose2 = static_cast<const VertexPose*>(_vertices[1]);
       const auto* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);
   
       // VELOCITY & ACCELERATION
       Eigen::Vector2d diff = pose2->position() - pose1->position();
   
       double cos_theta1 = std::cos(pose1->theta());
       double sin_theta1 = std::sin(pose1->theta());
   
       // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
       double p1_dx = (cos_theta1 * diff.x()) + (sin_theta1 * diff.y());
       double p1_dy = (-sin_theta1 * diff.x()) + (cos_theta1 * diff.y());
   
       double vel1_x = _measurement->linear.x;
       double vel1_y = _measurement->linear.y;
       double vel2_x = p1_dx / dt->dt();
       double vel2_y = p1_dy / dt->dt();
   
       double acc_lin_x = (vel2_x - vel1_x) / dt->dt();
       double acc_lin_y = (vel2_y - vel1_y) / dt->dt();
   
       _error[0] = penaltyBoundToInterval(acc_lin_x, cfg_->robot.acc_lim_x, cfg_->optim.penalty_epsilon);
       _error[1] = penaltyBoundToInterval(acc_lin_y, cfg_->robot.acc_lim_y, cfg_->optim.penalty_epsilon);
   
       // ANGULAR ACCELERATION
       double omega1 = _measurement->angular.z;
       double omega2 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt->dt();
       double acc_rot = (omega2 - omega1) / dt->dt();
   
       _error[2] = penaltyBoundToInterval(acc_rot, cfg_->robot.acc_lim_theta, cfg_->optim.penalty_epsilon);
   
       ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationStart::computeError() translational: _error[0]=%f\n", _error[0]);
       ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationStart::computeError() strafing: _error[1]=%f\n", _error[1]);
       ROS_ASSERT_MSG(std::isfinite(_error[2]), "EdgeAccelerationStart::computeError() rotational: _error[2]=%f\n", _error[2]);
     }
   
     void setInitialVelocity(const geometry_msgs::Twist& vel_start) { _measurement = &vel_start; }
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   class EdgeAccelerationHolonomicGoal : public BaseTebMultiEdge<3, const geometry_msgs::Twist*> {
    public:
     EdgeAccelerationHolonomicGoal() {
       _measurement = nullptr;
       this->resize(3);
     }
   
     void computeError() override {
       ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setHATebConfig() and setGoalVelocity() on EdgeAccelerationGoal()");
       const auto* pose_pre_goal = static_cast<const VertexPose*>(_vertices[0]);
       const auto* pose_goal = static_cast<const VertexPose*>(_vertices[1]);
       const auto* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);
   
       // VELOCITY & ACCELERATION
   
       Eigen::Vector2d diff = pose_goal->position() - pose_pre_goal->position();
   
       double cos_theta1 = std::cos(pose_pre_goal->theta());
       double sin_theta1 = std::sin(pose_pre_goal->theta());
   
       // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
       double p1_dx = (cos_theta1 * diff.x()) + (sin_theta1 * diff.y());
       double p1_dy = (-sin_theta1 * diff.x()) + (cos_theta1 * diff.y());
   
       double vel1_x = p1_dx / dt->dt();
       double vel1_y = p1_dy / dt->dt();
       double vel2_x = _measurement->linear.x;
       double vel2_y = _measurement->linear.y;
   
       double acc_lin_x = (vel2_x - vel1_x) / dt->dt();
       double acc_lin_y = (vel2_y - vel1_y) / dt->dt();
   
       _error[0] = penaltyBoundToInterval(acc_lin_x, cfg_->robot.acc_lim_x, cfg_->optim.penalty_epsilon);
       _error[1] = penaltyBoundToInterval(acc_lin_y, cfg_->robot.acc_lim_y, cfg_->optim.penalty_epsilon);
   
       // ANGULAR ACCELERATION
       double omega1 = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta()) / dt->dt();
       double omega2 = _measurement->angular.z;
       double acc_rot = (omega2 - omega1) / dt->dt();
   
       _error[2] = penaltyBoundToInterval(acc_rot, cfg_->robot.acc_lim_theta, cfg_->optim.penalty_epsilon);
   
       ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationGoal::computeError() translational: _error[0]=%f\n", _error[0]);
       ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationGoal::computeError() strafing: _error[1]=%f\n", _error[1]);
       ROS_ASSERT_MSG(std::isfinite(_error[2]), "EdgeAccelerationGoal::computeError() rotational: _error[2]=%f\n", _error[2]);
     }
   
     void setGoalVelocity(const geometry_msgs::Twist& vel_goal) { _measurement = &vel_goal; }
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   };  // namespace hateb_local_planner
   
   #endif /* EDGE_ACCELERATION_H_ */
