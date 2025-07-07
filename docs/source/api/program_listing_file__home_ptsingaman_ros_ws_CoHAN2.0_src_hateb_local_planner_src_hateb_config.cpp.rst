
.. _program_listing_file__home_ptsingaman_ros_ws_CoHAN2.0_src_hateb_local_planner_src_hateb_config.cpp:

Program Listing for File hateb_config.cpp
=========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ptsingaman_ros_ws_CoHAN2.0_src_hateb_local_planner_src_hateb_config.cpp>` (``/home/ptsingaman/ros_ws/CoHAN2.0/src/hateb_local_planner/src/hateb_config.cpp``)

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
    * Copyright (c) 2020 LAAS/CNRS
    * All rights reserved.
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
   
   #include <hateb_local_planner/hateb_config.h>
   
   namespace hateb_local_planner {
   
   void HATebConfig::loadRosParamFromNodeHandle(const ros::NodeHandle& nh) {
     nh.param("ns", ns_, std::string(""));
     nh.param("odom_topic", odom_topic, odom_topic);
     nh.param("map_frame", map_frame, map_frame);
     nh.param("planning_mode", planning_mode, planning_mode);
   
     // Trajectory
     nh.param("teb_autosize", trajectory.teb_autosize, trajectory.teb_autosize);
     nh.param("dt_ref", trajectory.dt_ref, trajectory.dt_ref);
     nh.param("dt_hysteresis", trajectory.dt_hysteresis, trajectory.dt_hysteresis);
     nh.param("min_samples", trajectory.min_samples, trajectory.min_samples);
     nh.param("max_samples", trajectory.max_samples, trajectory.max_samples);
     nh.param("agent_min_samples", trajectory.agent_min_samples, trajectory.agent_min_samples);
     nh.param("global_plan_overwrite_orientation", trajectory.global_plan_overwrite_orientation, trajectory.global_plan_overwrite_orientation);
     nh.param("allow_init_with_backwards_motion", trajectory.allow_init_with_backwards_motion, trajectory.allow_init_with_backwards_motion);
     nh.getParam("global_plan_via_point_sep", trajectory.global_plan_viapoint_sep);  // deprecated, see checkDeprecated()
     if (!nh.param("global_plan_viapoint_sep", trajectory.global_plan_viapoint_sep, trajectory.global_plan_viapoint_sep)) {
       nh.setParam("global_plan_viapoint_sep", trajectory.global_plan_viapoint_sep);  // write deprecated value to param server
     }
     nh.param("via_points_ordered", trajectory.via_points_ordered, trajectory.via_points_ordered);
     nh.param("max_global_plan_lookahead_dist", trajectory.max_global_plan_lookahead_dist, trajectory.max_global_plan_lookahead_dist);
     nh.param("global_plan_prune_distance", trajectory.global_plan_prune_distance, trajectory.global_plan_prune_distance);
     nh.param("exact_arc_length", trajectory.exact_arc_length, trajectory.exact_arc_length);
     nh.param("force_reinit_new_goal_dist", trajectory.force_reinit_new_goal_dist, trajectory.force_reinit_new_goal_dist);
     nh.param("force_reinit_new_goal_angular", trajectory.force_reinit_new_goal_angular, trajectory.force_reinit_new_goal_angular);
     nh.param("feasibility_check_no_poses", trajectory.feasibility_check_no_poses, trajectory.feasibility_check_no_poses);
     nh.param("publish_feedback", trajectory.publish_feedback, trajectory.publish_feedback);
     nh.param("min_resolution_collision_check_angular", trajectory.min_resolution_collision_check_angular, trajectory.min_resolution_collision_check_angular);
     nh.param("control_look_ahead_poses", trajectory.control_look_ahead_poses, trajectory.control_look_ahead_poses);
     nh.param("teb_init_skip_dist", trajectory.teb_init_skip_dist, trajectory.teb_init_skip_dist);
     // Robot
     nh.param("type", robot.type, robot.type);
     nh.param("max_vel_x", robot.max_vel_x, robot.max_vel_x);
     nh.param("max_vel_x_backwards", robot.max_vel_x_backwards, robot.max_vel_x_backwards);
     nh.param("max_vel_y", robot.max_vel_y, robot.max_vel_y);
     nh.param("max_vel_theta", robot.max_vel_theta, robot.max_vel_theta);
     nh.param("acc_lim_x", robot.acc_lim_x, robot.acc_lim_x);
     nh.param("acc_lim_y", robot.acc_lim_y, robot.acc_lim_y);
     nh.param("acc_lim_theta", robot.acc_lim_theta, robot.acc_lim_theta);
     nh.param("min_turning_radius", robot.min_turning_radius, robot.min_turning_radius);
     nh.param("wheelbase", robot.wheelbase, robot.wheelbase);
     nh.param("cmd_angle_instead_rotvel", robot.cmd_angle_instead_rotvel, robot.cmd_angle_instead_rotvel);
     nh.param("is_footprint_dynamic", robot.is_footprint_dynamic, robot.is_footprint_dynamic);
     nh.param("is_real", robot.is_real, robot.is_real);
   
     // Agent
     nh.param("agent_radius", agent.radius, agent.radius);
     nh.param("max_agent_vel_x", agent.max_vel_x, agent.max_vel_x);
     nh.param("max_agent_vel_y", agent.max_vel_y, agent.max_vel_y);
     nh.param("max_agent_vel_x_backwards", agent.max_vel_x_backwards, agent.max_vel_x_backwards);
     nh.param("max_agent_vel_theta", agent.max_vel_theta, agent.max_vel_theta);
     nh.param("agent_acc_lim_x", agent.acc_lim_x, agent.acc_lim_x);
     nh.param("agent_acc_lim_y", agent.acc_lim_y, agent.acc_lim_y);
     nh.param("agent_acc_lim_theta", agent.acc_lim_theta, agent.acc_lim_theta);
     nh.param("num_moving_avg", agent.num_moving_avg, agent.num_moving_avg);
   
     // GoalTolerance
     nh.param("xy_goal_tolerance", goal_tolerance.xy_goal_tolerance, goal_tolerance.xy_goal_tolerance);
     nh.param("yaw_goal_tolerance", goal_tolerance.yaw_goal_tolerance, goal_tolerance.yaw_goal_tolerance);
     nh.param("free_goal_vel", goal_tolerance.free_goal_vel, goal_tolerance.free_goal_vel);
     nh.param("complete_global_plan", goal_tolerance.complete_global_plan, goal_tolerance.complete_global_plan);
   
     // Obstacles
     nh.param("min_obstacle_dist", obstacles.min_obstacle_dist, obstacles.min_obstacle_dist);
     nh.param("inflation_dist", obstacles.inflation_dist, obstacles.inflation_dist);
     nh.param("dynamic_obstacle_inflation_dist", obstacles.dynamic_obstacle_inflation_dist, obstacles.dynamic_obstacle_inflation_dist);
     nh.param("include_dynamic_obstacles", obstacles.include_dynamic_obstacles, obstacles.include_dynamic_obstacles);
     nh.param("include_costmap_obstacles", obstacles.include_costmap_obstacles, obstacles.include_costmap_obstacles);
     nh.param("costmap_obstacles_behind_robot_dist", obstacles.costmap_obstacles_behind_robot_dist, obstacles.costmap_obstacles_behind_robot_dist);
     nh.param("obstacle_poses_affected", obstacles.obstacle_poses_affected, obstacles.obstacle_poses_affected);
     nh.param("legacy_obstacle_association", obstacles.legacy_obstacle_association, obstacles.legacy_obstacle_association);
     nh.param("obstacle_association_force_inclusion_factor", obstacles.obstacle_association_force_inclusion_factor, obstacles.obstacle_association_force_inclusion_factor);
     nh.param("obstacle_association_cutoff_factor", obstacles.obstacle_association_cutoff_factor, obstacles.obstacle_association_cutoff_factor);
     nh.param("costmap_converter_plugin", obstacles.costmap_converter_plugin, obstacles.costmap_converter_plugin);
     nh.param("costmap_converter_spin_thread", obstacles.costmap_converter_spin_thread, obstacles.costmap_converter_spin_thread);
   
     // Optimization
     nh.param("no_inner_iterations", optim.no_inner_iterations, optim.no_inner_iterations);
     nh.param("no_outer_iterations", optim.no_outer_iterations, optim.no_outer_iterations);
     nh.param("optimization_activate", optim.optimization_activate, optim.optimization_activate);
     nh.param("optimization_verbose", optim.optimization_verbose, optim.optimization_verbose);
     nh.param("penalty_epsilon", optim.penalty_epsilon, optim.penalty_epsilon);
     nh.param("time_penalty_epsilon", optim.time_penalty_epsilon, optim.time_penalty_epsilon);
     nh.param("cap_optimaltime_penalty", optim.cap_optimaltime_penalty, optim.cap_optimaltime_penalty);
     nh.param("weight_max_vel_x", optim.weight_max_vel_x, optim.weight_max_vel_x);
     nh.param("weight_max_vel_y", optim.weight_max_vel_y, optim.weight_max_vel_y);
     nh.param("weight_max_vel_theta", optim.weight_max_vel_theta, optim.weight_max_vel_theta);
     nh.param("weight_acc_lim_x", optim.weight_acc_lim_x, optim.weight_acc_lim_x);
     nh.param("weight_acc_lim_y", optim.weight_acc_lim_y, optim.weight_acc_lim_y);
     nh.param("weight_acc_lim_theta", optim.weight_acc_lim_theta, optim.weight_acc_lim_theta);
     nh.param("weight_kinematics_nh", optim.weight_kinematics_nh, optim.weight_kinematics_nh);
     nh.param("weight_kinematics_forward_drive", optim.weight_kinematics_forward_drive, optim.weight_kinematics_forward_drive);
     nh.param("weight_kinematics_turning_radius", optim.weight_kinematics_turning_radius, optim.weight_kinematics_turning_radius);
     nh.param("weight_optimaltime", optim.weight_optimaltime, optim.weight_optimaltime);
     nh.param("weight_shortest_path", optim.weight_shortest_path, optim.weight_shortest_path);
     nh.param("weight_obstacle", optim.weight_obstacle, optim.weight_obstacle);
     nh.param("weight_inflation", optim.weight_inflation, optim.weight_inflation);
     nh.param("weight_dynamic_obstacle", optim.weight_dynamic_obstacle, optim.weight_dynamic_obstacle);
     nh.param("weight_dynamic_obstacle_inflation", optim.weight_dynamic_obstacle_inflation, optim.weight_dynamic_obstacle_inflation);
     nh.param("weight_viapoint", optim.weight_viapoint, optim.weight_viapoint);
     nh.param("weight_prefer_rotdir", optim.weight_prefer_rotdir, optim.weight_prefer_rotdir);
     nh.param("weight_adapt_factor", optim.weight_adapt_factor, optim.weight_adapt_factor);
     nh.param("obstacle_cost_exponent", optim.obstacle_cost_exponent, optim.obstacle_cost_exponent);
   
     nh.param("weight_max_agent_vel_x", optim.weight_max_agent_vel_x, optim.weight_max_agent_vel_x);
     nh.param("weight_max_agent_vel_y", optim.weight_max_agent_vel_y, optim.weight_max_agent_vel_y);
     nh.param("weight_nominal_agent_vel_x", optim.weight_nominal_agent_vel_x, optim.weight_nominal_agent_vel_x);
     nh.param("weight_max_agent_vel_theta", optim.weight_max_agent_vel_theta, optim.weight_max_agent_vel_theta);
     nh.param("weight_agent_acc_lim_x", optim.weight_acc_lim_x, optim.weight_agent_acc_lim_x);
     nh.param("weight_agent_acc_lim_y", optim.weight_acc_lim_y, optim.weight_agent_acc_lim_y);
     nh.param("weight_agent_acc_lim_theta", optim.weight_acc_lim_theta, optim.weight_agent_acc_lim_theta);
     nh.param("weight_agent_optimaltime", optim.weight_agent_optimaltime, optim.weight_agent_optimaltime);
     nh.param("weight_agent_viapoint", optim.weight_agent_viapoint, optim.weight_agent_viapoint);
     nh.param("weight_agent_robot_safety", optim.weight_agent_robot_safety, optim.weight_agent_robot_safety);
     nh.param("weight_agent_agent_safety", optim.weight_agent_agent_safety, optim.weight_agent_agent_safety);
     nh.param("weight_agent_robot_rel_vel", optim.weight_agent_robot_rel_vel, optim.weight_agent_robot_rel_vel);
     nh.param("weight_agent_robot_visibility", optim.weight_agent_robot_visibility, optim.weight_agent_robot_visibility);
     nh.param("disable_warm_start", optim.disable_warm_start, optim.disable_warm_start);
     nh.param("disable_rapid_omega_chage", optim.disable_rapid_omega_chage, optim.disable_rapid_omega_chage);
     nh.param("omega_chage_time_seperation", optim.omega_chage_time_seperation, optim.omega_chage_time_seperation);
   
     // Hateb
     nh.param("use_agent_robot_safety_c", hateb.use_agent_robot_safety_c, hateb.use_agent_robot_safety_c);
     nh.param("use_agent_agent_safety_c", hateb.use_agent_agent_safety_c, hateb.use_agent_agent_safety_c);
     nh.param("use_agent_robot_rel_vel_c", hateb.use_agent_robot_rel_vel_c, hateb.use_agent_robot_rel_vel_c);
     nh.param("add_invisible_humans", hateb.add_invisible_humans, hateb.add_invisible_humans);
     nh.param("use_agent_robot_visi_c", hateb.use_agent_robot_visi_c, hateb.use_agent_robot_visi_c);
     nh.param("use_agent_elastic_vel", hateb.use_agent_elastic_vel, hateb.use_agent_elastic_vel);
     nh.param("min_agent_robot_dist", hateb.min_agent_robot_dist, hateb.min_agent_robot_dist);
     nh.param("min_agent_agent_dist", hateb.min_agent_agent_dist, hateb.min_agent_agent_dist);
     nh.param("agent_pose_prediction_reset_time", hateb.pose_prediction_reset_time, hateb.pose_prediction_reset_time);
     nh.param("rel_vel_cost_threshold", hateb.rel_vel_cost_threshold, hateb.rel_vel_cost_threshold);
     nh.param("invisible_human_threshold", hateb.invisible_human_threshold, hateb.invisible_human_threshold);
   
     // Recovery
     nh.param("shrink_horizon_backup", recovery.shrink_horizon_backup, recovery.shrink_horizon_backup);
     nh.param("shrink_horizon_min_duration", recovery.shrink_horizon_min_duration, recovery.shrink_horizon_min_duration);
     nh.param("oscillation_recovery", recovery.oscillation_recovery, recovery.oscillation_recovery);
     nh.param("oscillation_v_eps", recovery.oscillation_v_eps, recovery.oscillation_v_eps);
     nh.param("oscillation_omega_eps", recovery.oscillation_omega_eps, recovery.oscillation_omega_eps);
     nh.param("oscillation_recovery_min_duration", recovery.oscillation_recovery_min_duration, recovery.oscillation_recovery_min_duration);
     nh.param("oscillation_filter_duration", recovery.oscillation_filter_duration, recovery.oscillation_filter_duration);
   
     // Visualization
     nh.param("publish_robot_global_plan", visualization.publish_robot_global_plan, visualization.publish_robot_global_plan);
     nh.param("publish_robot_local_plan", visualization.publish_robot_local_plan, visualization.publish_robot_local_plan);
     nh.param("publish_robot_local_plan_poses", visualization.publish_robot_local_plan_poses, visualization.publish_robot_local_plan_poses);
     nh.param("publish_robot_local_plan_fp_poses", visualization.publish_robot_local_plan_fp_poses, visualization.publish_robot_local_plan_fp_poses);
     nh.param("publish_agents_global_plans", visualization.publish_agents_global_plans, visualization.publish_agents_global_plans);
     nh.param("publish_agents_local_plans", visualization.publish_agents_local_plans, visualization.publish_agents_local_plans);
     nh.param("publish_agents_local_plan_poses", visualization.publish_agents_local_plan_poses, visualization.publish_agents_local_plan_poses);
     nh.param("publish_agents_local_plan_fp_poses", visualization.publish_agents_local_plan_fp_poses, visualization.publish_agents_local_plan_fp_poses);
     nh.param("pose_array_z_scale", visualization.pose_array_z_scale, visualization.pose_array_z_scale);
   
     checkParameters();
     checkDeprecated(nh);
   }
   
   void HATebConfig::reconfigure(HATebLocalPlannerReconfigureConfig& cfg) {
     boost::mutex::scoped_lock l(config_mutex_);
   
     planning_mode = cfg.planning_mode;
   
     // Trajectory
     trajectory.teb_autosize = cfg.teb_autosize;
     trajectory.dt_ref = cfg.dt_ref;
     trajectory.dt_hysteresis = cfg.dt_hysteresis;
     trajectory.global_plan_overwrite_orientation = cfg.global_plan_overwrite_orientation;
     trajectory.allow_init_with_backwards_motion = cfg.allow_init_with_backwards_motion;
     trajectory.global_plan_viapoint_sep = cfg.global_plan_viapoint_sep;
     trajectory.via_points_ordered = cfg.via_points_ordered;
     trajectory.max_global_plan_lookahead_dist = cfg.max_global_plan_lookahead_dist;
     trajectory.exact_arc_length = cfg.exact_arc_length;
     trajectory.force_reinit_new_goal_dist = cfg.force_reinit_new_goal_dist;
     trajectory.force_reinit_new_goal_angular = cfg.force_reinit_new_goal_angular;
     trajectory.feasibility_check_no_poses = cfg.feasibility_check_no_poses;
     trajectory.publish_feedback = cfg.publish_feedback;
     trajectory.teb_init_skip_dist = cfg.teb_init_skip_dist;
   
     // Robot
     robot.is_real = cfg.is_real;
     robot.max_vel_x = cfg.max_vel_x;
     robot.max_vel_x_backwards = cfg.max_vel_x_backwards;
     robot.max_vel_y = cfg.max_vel_y;
     robot.max_vel_theta = cfg.max_vel_theta;
     robot.acc_lim_x = cfg.acc_lim_x;
     robot.acc_lim_y = cfg.acc_lim_y;
     robot.acc_lim_theta = cfg.acc_lim_theta;
     robot.min_turning_radius = cfg.min_turning_radius;
     robot.wheelbase = cfg.wheelbase;
     robot.cmd_angle_instead_rotvel = cfg.cmd_angle_instead_rotvel;
   
     // Agent
     agent.max_vel_x = cfg.max_agent_vel_x;
     agent.max_vel_y = cfg.max_agent_vel_y;
     agent.max_vel_x_backwards = cfg.max_agent_vel_x_backwards;
     agent.max_vel_theta = cfg.max_agent_vel_theta;
     agent.acc_lim_x = cfg.agent_acc_lim_x;
     agent.acc_lim_y = cfg.agent_acc_lim_y;
     agent.acc_lim_theta = cfg.agent_acc_lim_theta;
     agent.fov = cfg.fov;
     agent.num_moving_avg = cfg.num_moving_avg;
   
     // GoalTolerance
     goal_tolerance.xy_goal_tolerance = cfg.xy_goal_tolerance;
     goal_tolerance.yaw_goal_tolerance = cfg.yaw_goal_tolerance;
     goal_tolerance.complete_global_plan = cfg.complete_global_plan;
     goal_tolerance.free_goal_vel = cfg.free_goal_vel;
   
     // Obstacles
     obstacles.min_obstacle_dist = cfg.min_obstacle_dist;
     obstacles.inflation_dist = cfg.inflation_dist;
     obstacles.legacy_obstacle_association = cfg.legacy_obstacle_association;
     obstacles.obstacle_association_force_inclusion_factor = cfg.obstacle_association_force_inclusion_factor;
     obstacles.obstacle_association_cutoff_factor = cfg.obstacle_association_cutoff_factor;
     obstacles.use_nonlinear_obstacle_penalty = cfg.use_nonlinear_obstacle_penalty;
     obstacles.obstacle_cost_mult = cfg.obstacle_cost_mult;
     obstacles.dynamic_obstacle_inflation_dist = cfg.dynamic_obstacle_inflation_dist;
     obstacles.include_dynamic_obstacles = cfg.include_dynamic_obstacles;
     obstacles.include_costmap_obstacles = cfg.include_costmap_obstacles;
     obstacles.costmap_obstacles_behind_robot_dist = cfg.costmap_obstacles_behind_robot_dist;
     obstacles.obstacle_poses_affected = cfg.obstacle_poses_affected;
   
     // Optimization
     optim.no_inner_iterations = cfg.no_inner_iterations;
     optim.no_outer_iterations = cfg.no_outer_iterations;
     optim.optimization_activate = cfg.optimization_activate;
     optim.optimization_verbose = cfg.optimization_verbose;
     optim.penalty_epsilon = cfg.penalty_epsilon;
     optim.time_penalty_epsilon = cfg.time_penalty_epsilon;
     optim.cap_optimaltime_penalty = cfg.cap_optimaltime_penalty;
     optim.weight_max_vel_x = cfg.weight_max_vel_x;
     optim.weight_max_vel_y = cfg.weight_max_vel_y;
     optim.weight_max_vel_theta = cfg.weight_max_vel_theta;
     optim.weight_acc_lim_x = cfg.weight_acc_lim_x;
     optim.weight_acc_lim_y = cfg.weight_acc_lim_y;
     optim.weight_acc_lim_theta = cfg.weight_acc_lim_theta;
     optim.weight_kinematics_nh = cfg.weight_kinematics_nh;
     optim.weight_kinematics_forward_drive = cfg.weight_kinematics_forward_drive;
     optim.weight_kinematics_turning_radius = cfg.weight_kinematics_turning_radius;
     optim.weight_optimaltime = cfg.weight_optimaltime;
     optim.weight_shortest_path = cfg.weight_shortest_path;
     optim.weight_obstacle = cfg.weight_obstacle;
     optim.weight_inflation = cfg.weight_inflation;
     optim.weight_dynamic_obstacle = cfg.weight_dynamic_obstacle;
     optim.weight_dynamic_obstacle_inflation = cfg.weight_dynamic_obstacle_inflation;
     optim.weight_viapoint = cfg.weight_viapoint;
     optim.weight_adapt_factor = cfg.weight_adapt_factor;
     optim.obstacle_cost_exponent = cfg.obstacle_cost_exponent;
     optim.weight_max_agent_vel_x = cfg.weight_max_agent_vel_x;
     optim.weight_max_agent_vel_y = cfg.weight_max_agent_vel_y;
     optim.weight_nominal_agent_vel_x = cfg.weight_nominal_agent_vel_x;
     optim.weight_max_agent_vel_theta = cfg.weight_max_agent_vel_theta;
     optim.weight_agent_acc_lim_x = cfg.weight_agent_acc_lim_x;
     optim.weight_agent_acc_lim_y = cfg.weight_agent_acc_lim_y;
     optim.weight_agent_acc_lim_theta = cfg.weight_agent_acc_lim_theta;
     optim.weight_agent_optimaltime = cfg.weight_agent_optimaltime;
     optim.weight_agent_viapoint = cfg.weight_agent_viapoint;
     optim.weight_agent_robot_safety = cfg.weight_agent_robot_safety;
     optim.weight_agent_agent_safety = cfg.weight_agent_agent_safety;
     optim.weight_agent_robot_rel_vel = cfg.weight_agent_robot_rel_vel;
     optim.weight_invisible_human = cfg.weight_invisible_human;
     optim.weight_agent_robot_visibility = cfg.weight_agent_robot_visibility;
     optim.disable_warm_start = cfg.disable_warm_start;
     optim.disable_rapid_omega_chage = cfg.disable_rapid_omega_chage;
     optim.omega_chage_time_seperation = cfg.omega_chage_time_seperation;
   
     // Hateb
     hateb.use_agent_robot_safety_c = cfg.use_agent_robot_safety_c;
     hateb.use_agent_agent_safety_c = cfg.use_agent_agent_safety_c;
     hateb.use_agent_robot_rel_vel_c = cfg.use_agent_robot_rel_vel_c;
     hateb.add_invisible_humans = cfg.add_invisible_humans;
     hateb.use_agent_robot_visi_c = cfg.use_agent_robot_visi_c;
     hateb.use_agent_elastic_vel = cfg.use_agent_elastic_vel;
     hateb.min_agent_robot_dist = cfg.min_agent_robot_dist;
     hateb.min_agent_agent_dist = cfg.min_agent_agent_dist;
     hateb.rel_vel_cost_threshold = cfg.rel_vel_cost_threshold;
     hateb.invisible_human_threshold = cfg.invisible_human_threshold;
     hateb.visibility_cost_threshold = cfg.visibility_cost_threshold;
     hateb.pose_prediction_reset_time = cfg.agent_pose_prediction_reset_time;
   
     // Recovery
     recovery.shrink_horizon_backup = cfg.shrink_horizon_backup;
     recovery.oscillation_recovery = cfg.oscillation_recovery;
   
     // Visualization
     visualization.publish_robot_global_plan = cfg.publish_robot_global_plan;
     visualization.publish_robot_local_plan = cfg.publish_robot_local_plan;
     visualization.publish_robot_local_plan_poses = cfg.publish_robot_local_plan_poses;
     visualization.publish_robot_local_plan_fp_poses = cfg.publish_robot_local_plan_fp_poses;
     visualization.publish_agents_global_plans = cfg.publish_agents_global_plans;
     visualization.publish_agents_local_plans = cfg.publish_agents_local_plans;
     visualization.publish_agents_local_plan_poses = cfg.publish_agents_local_plan_poses;
     visualization.publish_agents_local_plan_fp_poses = cfg.publish_agents_local_plan_fp_poses;
     visualization.pose_array_z_scale = cfg.pose_array_z_scale;
   
     checkParameters();
   }
   
   void HATebConfig::checkParameters() const {
     // positive backward velocity?
     if (robot.max_vel_x_backwards <= 0) {
       ROS_WARN("HATebLocalPlannerROS() Param Warning: Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
     }
     // bounds smaller than penalty epsilon
     if (robot.max_vel_x <= optim.penalty_epsilon) {
       ROS_WARN("HATebLocalPlannerROS() Param Warning: max_vel_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
     }
     if (robot.max_vel_x_backwards <= optim.penalty_epsilon) {
       ROS_WARN("HATebLocalPlannerROS() Param Warning: max_vel_x_backwards <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
     }
     if (robot.max_vel_theta <= optim.penalty_epsilon) {
       ROS_WARN("HATebLocalPlannerROS() Param Warning: max_vel_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
     }
     if (robot.acc_lim_x <= optim.penalty_epsilon) {
       ROS_WARN("HATebLocalPlannerROS() Param Warning: acc_lim_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
     }
     if (robot.acc_lim_theta <= optim.penalty_epsilon) {
       ROS_WARN("HATebLocalPlannerROS() Param Warning: acc_lim_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
     }
     // dt_ref and dt_hyst
     if (trajectory.dt_ref <= trajectory.dt_hysteresis) {
       ROS_WARN("HATebLocalPlannerROS() Param Warning: dt_ref <= dt_hysteresis. The hysteresis is not allowed to be greater or equal!. Undefined behavior... Change at least one of them!");
     }
     // min number of samples
     if (trajectory.min_samples < 3) {
       ROS_WARN("HATebLocalPlannerROS() Param Warning: parameter min_samples is smaller than 3! Sorry, I haven't enough degrees of freedom to plan a trajectory for you. Please increase ...");
     }
     // costmap obstacle behind robot
     if (obstacles.costmap_obstacles_behind_robot_dist < 0) {
       ROS_WARN("HATebLocalPlannerROS() Param Warning: parameter 'costmap_obstacles_behind_robot_dist' should be positive or zero.");
     }
   
     // carlike
     if (robot.cmd_angle_instead_rotvel && robot.wheelbase == 0) {
       ROS_WARN("HATebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but wheelbase is set to zero: undesired behavior.");
     }
     if (robot.cmd_angle_instead_rotvel && robot.min_turning_radius == 0) {
       ROS_WARN(
           "HATebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but min_turning_radius is set to zero: undesired behavior. You are mixing a carlike and a diffdrive "
           "robot");
     }
   
     // positive weight_adapt_factor
     if (optim.weight_adapt_factor < 1.0) {
       ROS_WARN("HATebLocalPlannerROS() Param Warning: parameter weight_adapt_factor shoud be >= 1.0");
     }
   
     if (recovery.oscillation_filter_duration < 0) {
       ROS_WARN("HATebLocalPlannerROS() Param Warning: parameter oscillation_filter_duration must be >= 0");
     }
   
     // weights
     if (optim.weight_optimaltime <= 0) {
       ROS_WARN("HATebLocalPlannerROS() Param Warning: parameter weight_optimaltime shoud be > 0 (even if weight_shortest_path is in use)");
     }
   }
   
   void HATebConfig::checkDeprecated(const ros::NodeHandle& nh) const {
     if (nh.hasParam("line_obstacle_poses_affected") || nh.hasParam("polygon_obstacle_poses_affected")) {
       ROS_WARN(
           "HATebLocalPlannerROS() Param Warning: 'line_obstacle_poses_affected' and 'polygon_obstacle_poses_affected' are deprecated. They share now the common parameter 'obstacle_poses_affected'.");
     }
     if (nh.hasParam("weight_point_obstacle") || nh.hasParam("weight_line_obstacle") || nh.hasParam("weight_poly_obstacle")) {
       ROS_WARN(
           "HATebLocalPlannerROS() Param Warning: 'weight_point_obstacle', 'weight_line_obstacle' and 'weight_poly_obstacle' are deprecated. They are replaced by the single param 'weight_obstacle'.");
     }
     if (nh.hasParam("costmap_obstacles_front_only")) {
       ROS_WARN(
           "HATebLocalPlannerROS() Param Warning: 'costmap_obstacles_front_only' is deprecated. It is replaced by 'costmap_obstacles_behind_robot_dist' to define the actual area taken into account.");
     }
     if (nh.hasParam("costmap_emergency_stop_dist")) {
       ROS_WARN("HATebLocalPlannerROS() Param Warning: 'costmap_emergency_stop_dist' is deprecated. You can safely remove it from your parameter config.");
     }
     if (nh.hasParam("alternative_time_cost")) {
       ROS_WARN("HATebLocalPlannerROS() Param Warning: 'alternative_time_cost' is deprecated. It has been replaced by 'selection_alternative_time_cost'.");
     }
     if (nh.hasParam("global_plan_via_point_sep")) {
       ROS_WARN("HATebLocalPlannerROS() Param Warning: 'global_plan_via_point_sep' is deprecated. It has been replaced by 'global_plan_viapoint_sep' due to consistency reasons.");
     }
   }
   
   }  // namespace hateb_local_planner
