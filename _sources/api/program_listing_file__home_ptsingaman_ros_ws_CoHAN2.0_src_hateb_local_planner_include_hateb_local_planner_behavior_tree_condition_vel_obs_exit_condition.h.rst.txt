
.. _program_listing_file__home_ptsingaman_ros_ws_CoHAN2.0_src_hateb_local_planner_include_hateb_local_planner_behavior_tree_condition_vel_obs_exit_condition.h:

Program Listing for File vel_obs_exit_condition.h
=================================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ptsingaman_ros_ws_CoHAN2.0_src_hateb_local_planner_include_hateb_local_planner_behavior_tree_condition_vel_obs_exit_condition.h>` (``/hateb_local_planner/include/hateb_local_planner/behavior_tree/condition/vel_obs_exit_condition.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*********************************************************************
    *
    * Software License Agreement (BSD License)
    *
    * Copyright (c) 2025 LAAS/CNRS
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
    * Author: Phani Teja Singamaneni (email:ptsingaman@laas.fr)
    *********************************************************************/
   
   #include <ros/ros.h>
   #include <tf2/utils.h>
   
   // New
   #include <agent_path_prediction/AgentsInfo.h>
   #include <agent_path_prediction/agents_class.h>
   #include <hateb_local_planner/behavior_tree/bt_core.h>
   
   #include <mutex>
   
   namespace hateb_local_planner {
   
   class VelObsExitCondition : public BT::ConditionNode {
    public:
     VelObsExitCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);
   
     VelObsExitCondition() = delete;
   
     ~VelObsExitCondition() override;
   
     BT::NodeStatus tick() override;
   
     static BT::PortsList providedPorts() {
       // This action has a single input port called "agents_info"
       return {BT::InputPort<agent_path_prediction::AgentsInfo>("agents_info"), BT::InputPort<std::shared_ptr<agents::Agents>>("agents_ptr"), BT::OutputPort<int>("stuck_agent")};
     }
   
    private:
     bool hasHumanStopped();
   
     // bool isHumanPlaying(); // Add this later
   
     // Blackboard entries
     agent_path_prediction::AgentsInfo agents_info_;  
     std::shared_ptr<agents::Agents> agents_ptr_;     
   
     std::string name_;         
     int nearest_human_id_;     
     int t_stuck_;              
     std::mutex agents_mutex_;  
   };
   };  // namespace hateb_local_planner
